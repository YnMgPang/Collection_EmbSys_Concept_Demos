//ref: https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-9-hardware-interrupts/3ae7a68462584e1eb408e1638002e9ed
//****************************************************** operations
//concepts: double buffer, semaphore, queue, FreeRTOS task notif.

//timer ISR writes to buffer A/B 10 times/1sec
//-> check buffer overrun
//-> send notif./give semaphore when written 10 times

//calcAvg task wait for available notif./semaphore
//-> write to avg
//-> check buffer overrun msg
//-> give done_reading semaphore

//serial task handles serial terminal I/O

//*** swap() only when done_writing(idx >= BUF_LEN check) && done_reading(semaphore check), not just done reading
//****************************************************** includes
// #include semphr.h                                   // vanilla
//****************************************************** only 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//****************************************************** settings
static const char command[] = "avg";                  // command
static const uint32_t cli_delay = 20;                 // ms delay
enum { CMD_BUF_LEN = 255};                            // number of characters in command buffer

static const uint16_t timer_divider = 8;              // divide 80 MHz by this
static const uint64_t timer_max_count = 1000000;      // timer counts to this value -> 0.1s = 1 million / (80 Mhz / 8)

enum { BUF_LEN = 10 };                                // number of elements in sample buffer
enum { MSG_LEN = 100 };                               // max characters in message body
enum { MSG_QUEUE_LEN = 5 };                           // number of slots in message queue
//------------------------------------------------------ pins
static const int adc_pin = A0;
//------------------------------------------------------ message struct to wrap strings for queue
typedef struct Message {
  char body[MSG_LEN];
} Message;
//****************************************************** var.
//------------------------------------------------------ interrupt + hw. timer
static hw_timer_t *timer = NULL;
//------------------------------------------------------ access control & signaling
static TaskHandle_t processing_task = NULL;           // for notification
static SemaphoreHandle_t sem_done_reading = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
//------------------------------------------------------ serial
static QueueHandle_t msg_queue;

static volatile uint16_t buf_0[BUF_LEN];              // one buffer in the pair
static volatile uint16_t buf_1[BUF_LEN];              // the other buffer in the pair
static volatile uint16_t* write_to = buf_0;           // double buffer write pointer
static volatile uint16_t* read_from = buf_1;          // double buffer read pointer
static volatile uint8_t buf_overrun = 0;              // = 1 when can't take sem_done_reading
                                                      // double buffer overrun flag -> true: write is too fast
                                                      // only for serial error msg signaling -> not necessary feature?
static float adc_avg;                                             
//****************************************************** functions that can be called from anywhere (in this file)
                                                      // swap the write_to and read_from pointers in the double buffer
                                                      // only ISR calls this at the moment, so no need to make it thread-safe
void IRAM_ATTR swap() {
  volatile uint16_t* temp_ptr = write_to;
  write_to = read_from;
  read_from = temp_ptr;
}
//****************************************************** ISR
void IRAM_ATTR onTimer() {                            // this function executes when timer reaches max (and resets)

  static uint16_t idx = 0;                            // ***STATIC ->  idx only initializes once
  BaseType_t task_woken = pdFALSE;
                                                      // if buffer is not overrun, read ADC to next buffer element. 
                                                      // if buffer is overrun, drop the sample.
  if ((idx < BUF_LEN) && (buf_overrun == 0)) {
    //write_to[idx] = analogRead(adc_pin);
    write_to[idx] = hallRead();
    idx++;
  }
 
  if (idx >= BUF_LEN) {                               // check if the buffer is full

                                                      // if reading is not done, set overrun flag. We don't need to set this
                                                      // as a critical section, as nothing can interrupt and change either value.
                                                      // (unlikely) task_woken needed -> tasks blocked on it waiting to give the semaphore
                                                      // (if overrun) drop current adc val.
                                                      // + next check is one timer period away
    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdFALSE) {
      buf_overrun = 1;                                // write buffer filled up + read buffer still going
    }
                                                      
    if (buf_overrun == 0) {                           // only swap buffers and notify task if overrun flag is cleared

      idx = 0;                                        // reset index 
      swap();                                         // swap buffer pointers
  
      vTaskNotifyGiveFromISR(processing_task, &task_woken);     // task notification works like a binary semaphore but faster
      //****** Why not use flag here? ******
      //ISR -> flag = 1
      //task -> flag == 1?
      //task -> flag = 0
      //if flag == 1
      //>>>>>>>>> interrupt can happen here <<<<<<<<<
      //flag = 0
      //-> if flag is 1 when calcAvg checks
      //-> ISR interrupts and "changes"/"sets" flag to 1
      //-> calcAvg sets flag to 0 at end of processing
      //-> loss of data
    }

//    //-------------------------------------------------- TEST: use only semaphore
//    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdTRUE) {
//      
//    }
  }
  
  //portYIELD_FROM_ISR(task_woken);                   // exit to unblocked higher priority task (vanilla) -> not nec. the current task

  if (task_woken) {                                   // exit to unblocked higher priority task (ESP-IDF) -> not nec. the current task
    portYIELD_FROM_ISR();
  }
}
//****************************************************** tasks
//------------------------------------------------------ serial terminal task
void doCLI(void *parameters) {
  //-------------- process queue
  //-------------- process serial
  //--------------1)check w/ buffer length limit
  //--------------2)echo
  //--------------3)check w/ cmd && print avg

  Message rcv_msg;                                    // for queue
  
  char c;
  char cmd_buf[CMD_BUF_LEN];
  uint8_t idx = 0;
  uint8_t cmd_len = strlen(command);

  memset(cmd_buf, 0, CMD_BUF_LEN);                    // clear whole buffer

  while (1) {
                                                                        // ****** process msg from other tasks ******
    if (xQueueReceive(msg_queue, (void *)&rcv_msg, 0) == pdTRUE) {      // ****** look for any error messages that need to be printed
      Serial.println(rcv_msg.body);
    }

    if (Serial.available() > 0) {                     // read characters from serial
      c = Serial.read();
  
      if (idx < CMD_BUF_LEN - 1) {                    // store received character to buffer if not over buffer limit
        cmd_buf[idx] = c;
        idx++;
      }
  
      if ((c == '\n') || (c == '\r')) {               // print newline and check input on 'enter'
        
        Serial.print("\r\n");

        //---------------------------------------------- print average value if command given is "avg"
        cmd_buf[idx - 1] = '\0';
        if (strcmp(cmd_buf, command) == 0) {
          Serial.print("Average: ");
          Serial.println(adc_avg);
        }

        //---------------------------------------------- reset receive buffer and index counter
        memset(cmd_buf, 0, CMD_BUF_LEN);
        idx = 0;

      } else {                                        // otherwise, echo character
        Serial.print(c);
      }
    }

    vTaskDelay(cli_delay / portTICK_PERIOD_MS);       // don't hog the CPU -> yield to other tasks
  }
}
//------------------------------------------------------ task 2 -> Wait for semaphore and calculate average of ADC values
void calcAverage(void *parameters) {

  Message msg;
  float avg;

  while (1) {                                         // loop forever, wait for semaphore, and print value

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          // wait for notification from ISR (similar to binary semaphore)
    
    //-------------------------------------------------- calculate average (as floating point value)
    avg = 0.0;
    for (int i = 0; i < BUF_LEN; i++) {
      avg += (float)read_from[i];
      //vTaskDelay(105 / portTICK_PERIOD_MS);         // uncomment to test overrun flag
    }
    avg /= BUF_LEN;
                                                      // updating the shared float may or may not take multiple isntructions, so
                                                      // we protect it with a mutex or critical section. The ESP-IDF critical
                                                      // section is the easiest for this application.
    portENTER_CRITICAL(&spinlock);
    adc_avg = avg;
    portEXIT_CRITICAL(&spinlock);
                                                      // if we took too long to process, buffer writing will have overrun. So,
                                                      // we send a message to be printed out to the serial terminal.
    if (buf_overrun == 1) {
      strcpy(msg.body, "Error: Buffer overrun. Samples have been dropped.");
      xQueueSend(msg_queue, (void *)&msg, 10);
    }
                                                      // clearing the overrun flag and giving the "done reading" semaphore must
                                                      // be done together without being interrupted.
    portENTER_CRITICAL(&spinlock);
    buf_overrun = 0;
    xSemaphoreGive(sem_done_reading);
    portEXIT_CRITICAL(&spinlock);
  }
}
//****************************************************** main (runs as its own task with priority 1 on core 1)
void setup() {

  //---------------------------------------------------- serial
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);              // wait a moment to start (so we don't miss Serial output)
  Serial.println();
  Serial.println("---FreeRTOS Sample and Process Demo---");
  //---------------------------------------------------- semaphore
  sem_done_reading = xSemaphoreCreateBinary();        // create semaphore before it is used (in task or ISR)
  if (sem_done_reading == NULL) {                     // force reboot if we can't create the semaphore
    Serial.println("Could not create one or more semaphores");
    ESP.restart();
  }

  xSemaphoreGive(sem_done_reading);                   // we want the done reading semaphore to initialize to 1
  //---------------------------------------------------- queue
  msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(Message));   // create message queue before it is used

                                                      // start task to handle command line interface events
                                                      // set it at a higher priority but only run it once every 20 ms
  xTaskCreatePinnedToCore(doCLI,
                          "Do CLI",
                          1024,
                          NULL,
                          2,
                          NULL,
                          app_cpu);
                                                      // start task to calculate average. Save handle for use with notifications.
  xTaskCreatePinnedToCore(calcAverage,
                          "Calculate average",
                          1024,
                          NULL,
                          1,
                          &processing_task,
                          app_cpu);

  //---------------------------------------------------- start a timer to run ISR every 100 ms
  timer = timerBegin(0, timer_divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timer_max_count, true);
  timerAlarmEnable(timer);

  vTaskDelete(NULL);                                  // delete "setup and loop" task
}

void loop() {
  // Execution should never get here
}
