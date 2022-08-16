//-------------------------------------------------- operations
// * One task performs basic echo on Serial. If it sees "delay" followed by a
// * number, it sends the number (in a queue) to the second task. If it receives
// * a message in a second queue, it prints it to the console. The second task
// * blinks an LED. When it gets a message from the first queue (number), it
// * updates the blink delay to that number. Whenever the LED blinks 100 times,
// * the second task sends a message to the first task to be printed.
//2 tasks, 2 queues
//task A -> print queue 2, *read serial, echo serial, check and send delay to queue 1
//task B -> update t w/info from queue 1, blinks, send to queue 2 if blinked 100
//-------------------------------------------------- structure
//queue 1 -> 5 * int
//queue 2 -> 5 * string + each 15 char
//-------------------------------------------------- findings
//...
//-------------------------------------------------- only 1 core
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif
//-------------------------------------------------- settings
static const uint8_t len_queue1 = 5;       //number of items in queue
static const uint8_t len_queue2 = 5;
int itemLen_queue1 = 15;
int itemLen_queue2 = 15;
uint8_t maxSerialReadBytes = 32;
//-------------------------------------------------- pin def
static const int led_pin = 17;
//-------------------------------------------------- var.
static QueueHandle_t hndl_queue1;               //global -> accessed by all tasks
static QueueHandle_t hndl_queue2;
int t = 500;                                    //LED blink rate
//-------------------------------------------------- task A
void function_serial(void *parameters){
  //================================================================= var. -> queue 2
  char stringReadFromQueue[itemLen_queue2];
  //================================================================= var. -> serial
  char stringReadFromSerial[maxSerialReadBytes];
  int i = 0;
  
  while(1){
    //=============================================================== print queue 2
    if(xQueueReceive(hndl_queue2, (void *)&stringReadFromQueue[0], 0) == pdTRUE){
      Serial.println("*************************");
      Serial.println(stringReadFromQueue);     
    }
    //=============================================================== read serial + echo serial + check and send delay
    while (Serial.available()){
      
      if (i < maxSerialReadBytes - 1){                              //if [5] -> 4 is reserved -> i up to 3 < 5 - 1
        stringReadFromSerial[i] = Serial.read();
        if (stringReadFromSerial[i] != '\n'){
          i++;
        }else{                                                      //msg shorter than print buffer -> reading done
          stringReadFromSerial[i] = '\0';

          //Serial.println("__________________________");
          Serial.print("Echo: ");                                   //echo back use input
          Serial.println(stringReadFromSerial);
          //========================================================= send to queue 1 
          if (strncmp(stringReadFromSerial, "delay ", 6) == 0){
            char stringDelayLED[((int)strlen(stringReadFromSerial)) - 6];   //num. of char = total - first 6
            
            for (int j = 6; j < (int)strlen(stringReadFromSerial); j++){    //substring extraction
              stringDelayLED[j-6] = stringReadFromSerial[j];
            }
            //****** BUG ****** atoi() returns 0 even if input invalid           
            int numDelayLED = atoi(stringDelayLED);                         //convert to int -> (!)returns 0 if invalid
            xQueueSend(hndl_queue1, (void *)&numDelayLED, 10);              //send an int to queue 1
          }
          i = 0;                              
        }
      }      
    }
  }
}
//-------------------------------------------------- task B
void function_LED(void *parameters){
  //================================================================= 
  int toggleDelay = 500;
  int numberBlinked = 0;  
  
  while(1){
    xQueueReceive(hndl_queue1, (void *)&toggleDelay, 0);
    
    digitalWrite(led_pin, HIGH);                           
    vTaskDelay(toggleDelay / portTICK_PERIOD_MS);          
    digitalWrite(led_pin, LOW);
    vTaskDelay(toggleDelay / portTICK_PERIOD_MS);

    numberBlinked++;
    if (numberBlinked >= 100){
      //============================================================= assemble string
      char msgBlinked[] = "Blinked";
      
      xQueueSend(hndl_queue2, (void *)&msgBlinked, 0);              //send to queue 2
      
      numberBlinked = 0;
    }
  }
}

void setup() {
  //------------------------------------------------ serial setup
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);                  //wait for serial conn. to finish setup
  //------------------------------------------------ pin config
  pinMode(led_pin, OUTPUT);
  //------------------------------------------------ create queue
  hndl_queue1 = xQueueCreate(len_queue1, sizeof(int));      //queue1 -> delay for LED blink rate
  hndl_queue2 = xQueueCreate(len_queue2, sizeof(char) * itemLen_queue2);      //queue2 -> char???
  //------------------------------------------------ start task
  xTaskCreatePinnedToCore(function_serial,
                          "Serial Handler",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                          );
  xTaskCreatePinnedToCore(function_LED,
                          "LED Handler",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                          );
  //------------------------------------------------ task for setup() & loop() no longer needed
  vTaskDelete(NULL);                                                     
}

void loop() {
}
