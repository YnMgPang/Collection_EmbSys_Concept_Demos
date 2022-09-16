//ref: https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-9-hardware-interrupts/3ae7a68462584e1eb408e1638002e9ed
//****************************************************** introduction
// this program borrows design from the given answer (check modified answer in Answers folder)
// instead of double buffer, a circular buffer is used here
// not using FreeRTOS task notification
//****************************************************** operation
//concepts: circular buffer, semaphore, queue, spinlock
//timer ISR writes to buffer 10 times/1sec
//-> check for overwritting
//-> send notif./give semaphore when written 10 times

//calcAvg task wait for available /semaphore
//-> wait for semaphore
//-> "wake up" whenever available
//-> calc. average
//-> write to avg
//-> check for overwritting
//-> give done_reading semaphore

//serial task handles serial terminal I/O

// special: buffer overriting? -> write position goes too fast and wants to overwrite read position
//****************************************************** notes
// PuTTY: press Ctrl + J to send linefeed
// extra: use enum to replace static const
// critical section vs mutex???
//****************************************************** includes
// #include semphr.h                                   // vanilla
//****************************************************** only 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//****************************************************** settings
// static const int adc_pin = A0;                      // no need -> use hallRead()
uint8_t maxSerialReadBytes = 32;
//------------------------------------------------------ interrupt + HW timer
static const uint16_t timer_divider = 8;             // ESP32 base clock = 80MHz -> divided clock = 1MHz
static const uint64_t timer_max_count = 1000000;
//------------------------------------------------------ circular buffer
static const uint16_t BUF_LEN = 50;
static const uint16_t SAMPLE_SIZE = 10;
//------------------------------------------------------ access control & signaling -> semaphore/mutex/spinlock
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
//------------------------------------------------------ queue
enum { MSG_LEN = 100 };                               // max characters in message body
enum { MSG_QUEUE_LEN = 5 };                           // number of slots in message queue
typedef struct Message{
  char body[MSG_LEN];
}Message;
//------------------------------------------------------ task A -> avg calc
//------------------------------------------------------ task B -> serial
static const char command[] = "avg";                  // command to compare w/ user input
//static const int cmdLength;                         // no need -> strcmp() checks until first NULL
static const uint32_t cli_delay = 20;                 // serial task delay -> ms
enum { maxSerialBytes = 255};                            // number of characters in buffer
//****************************************************** var.
static volatile float avg = 99;
//------------------------------------------------------ circular buffer
static volatile uint16_t circBuffer[BUF_LEN];
//------------------------------------------------------ interrupt + HW timer
static hw_timer_t *timer = NULL;                      // ESP32 HAL timer -> comes w/ Arduino lib.
//static volatile int val = 0;                          // protect -> val. might change outside the scope of current task???
//static SemaphoreHandle_t bin_sem = NULL;
//------------------------------------------------------ access control & signaling -> semaphore/mutex/spinlock
static TaskHandle_t processing_task = NULL;
static SemaphoreHandle_t hndl_sem_done_writing = NULL;
static SemaphoreHandle_t hndl_sem_done_reading = NULL;
//------------------------------------------------------ queue
static QueueHandle_t hndl_msg_queue;
//****************************************************** ISR
void IRAM_ATTR onTimer(){
  //-------------- take hndl_sem_done_reading
  //-------------- if yes, take adc, put into [idx]

  //Serial.println("______^^^^^^^^^^^^^^^^^^^________");

  static uint16_t idx = 0;                            // static -> initialize only once + val. persists
  static uint8_t sampling = 0;                        // as both enable and count
  BaseType_t task_woken = pdFALSE;

  if (sampling == 0 && xSemaphoreTakeFromISR(hndl_sem_done_reading, &task_woken) == pdTRUE){
    sampling = 1;                                     // can only take once -> need a flag for 10X sampling
  }
  
  if (sampling > 0 && sampling <= SAMPLE_SIZE){       // do 10X sampling until sampling < SAMPLE_SIZE
    circBuffer[idx] = hallRead();
    idx = (idx + 1) % BUF_LEN; 
    sampling++;    
  }else{
    xSemaphoreGiveFromISR(hndl_sem_done_writing, &task_woken);
    sampling = 0;
  }
  
  if (task_woken) {                                   // exit to unblocked higher priority task (ESP-IDF) -> not nec. the current task
    portYIELD_FROM_ISR();
  }  
}
//****************************************************** task
//------------------------------------------------------ task A f()
void calcAvgVal(void *parameter){
  //-------------- take hndl_sem_done_writing
  //-------------- if yes -> calc average from circ. buffer
  //-------------- put into avg
  //-------------- give hndl_sem_done_reading semaphore
  Message msg;
  static uint16_t idx = 0;
  float average = 0;
  
  float sum = 0;
  int i = 0;
  while(1){
    //-------------------------------------------------- wake-up detection
    xSemaphoreTake(hndl_sem_done_writing, portMAX_DELAY);
    //-------------------------------------------------- calc. average
    average = 0.0;
    for (i = 0; i < SAMPLE_SIZE; i++){
      average = average + (float) circBuffer[i];
    }
    average = average / 10;
    //-------------------------------------------------- put into avg
    portENTER_CRITICAL(&spinlock);                    // updating the shared float may or may not take multiple isntructions
    //Serial.println("______******************________");
    avg = average;
    portEXIT_CRITICAL(&spinlock);
    //-------------------------------------------------- give hndl_sem_done_reading semaphore
    portENTER_CRITICAL(&spinlock);
    xSemaphoreGive(hndl_sem_done_reading);
    portEXIT_CRITICAL(&spinlock);

    strcpy(msg.body, "-------------Testing Comm. With Serial Handler---------------");
    xQueueSend(hndl_msg_queue, (void *)&msg, 10);
  }
}
//------------------------------------------------------ task B f()
// keep checking serial availability
// if avail. -> print to terminal
// when typing finished -> check w/ "avg" -> if true -> print float Average
// ***for simplicity -> assume no buffer overflow
void cmdLI(void *parameters){
  //-------------- process queue
  //-------------- process serial
  //--------------1)check w/ buffer length limit
  //--------------2)echo
  //--------------3)check w/ cmd && print avg

  Message rcv_msg;                                    // for queue

  char charRead;
  char charSerial[maxSerialBytes];
  int i = 0;

  memset(charSerial, 0, maxSerialBytes);
  
  while(1){                                          
    //-------------------------------------------------------------------- process queue
    if (xQueueReceive(hndl_msg_queue, (void *)&rcv_msg, 0) == pdTRUE){  // (handle, buffer, block time)
      Serial.println(rcv_msg.body);
    }
    //-------------------------------------------------------------------- process serial
    if(Serial.available()){
      //if buffer not full -> read once
      //if CR or LF -> insert termination -> done
      //if not, put in [i] -> i++
      if (i < maxSerialBytes - 1){                                      // last index for termination
        charSerial[i] = Serial.read();                                  // read once
        
        if ( (charSerial[i] == '\n') || (charSerial[i] == '\r') ){      // check for CR LF -> finish reading?
          Serial.print("\r\n");                                         // "echo"
          charSerial[i] = '\0';                                         // insert termination                                 
          //-------------------------------------------------------------- 3)check w/ cmd && print avg
          if (strcmp(charSerial, command) == 0){
          //if (strncmp(charSerial, "avg",3) == 0){  
            Serial.println("_________________________");
            Serial.println(avg);
          }
          memset(charSerial, 0, maxSerialBytes);                        // reset buffer    
          i = 0;                                                        // reset index
        }else{         
          Serial.print(charSerial[i]);                                  // echo
          i++;                                                          // ++i & continue reading
        }
      }                       
        //--------------------------------------------- ASCII alt.
//        Serial.write(13);
//        Serial.write(10);
    } 
    vTaskDelay(cli_delay / portTICK_PERIOD_MS);       // don't hog the CPU -> yield to other tasks
  }
}
//****************************************************** main
void setup() {
  //--------------------------------------------------- serial
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("-----------------------------");
  //--------------------------------------------------- semaphore
  hndl_sem_done_writing = xSemaphoreCreateCounting((int)BUF_LEN/SAMPLE_SIZE, 0); //(max, init)
  hndl_sem_done_reading = xSemaphoreCreateCounting((int)BUF_LEN/SAMPLE_SIZE, 5);
  //****** add NULL creation check ******
  //--------------------------------------------------- queue
  hndl_msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(Message));
  //--------------------------------------------------- start task
  xTaskCreatePinnedToCore(calcAvgVal,
                          "Calculate Average",
                          1024,
                          NULL,
                          1,
                          &processing_task,          // save handle for use with notifications
                          app_cpu);  
  xTaskCreatePinnedToCore(cmdLI,
                          "Command Line Interface",
                          1024,
                          NULL,
                          2,                         // ****** why priority 2? ******
                          NULL,
                          app_cpu
                          );                                                  
  //--------------------------------------------------- start timer
  timer = timerBegin(0, timer_divider, true);        // (timer num, divider, count up?)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timer_max_count, true);
  timerAlarmEnable(timer);
                           
  vTaskDelete(NULL);                                 // task for setup() & loop() no longer needed      
}

void loop() {

}


//--- check w/ readIdx
//1)if writeIdx+10 >= readIdx
//2)if tail+10 > bufferSize - 1
//-> check if [+10 - (bufferSize - writeIdx)] >= readIdx
//
//-> i++ incremental smapling for 10 times
//-> each time -> if ((wIdx + 1)%(size - 1)) >= rIdx -> retracment
//-> retracement: wIdx = wIdx_new - (i+1)-> if wIdx < 0 -> wIdx = size + wIdx
//
//workflow
//*<-> head, setup, loop
//*-> serial task
//-> ISR
//-> calc task
