//ref:https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-4-memory-management/6d4dfcaa1ff84f57a2098da8e6401d9c
//-------------------------------------------------- results
//how does Serial.available() & Serial.read() work?
//-------------------------------------------------- operation
//*two tasks
//(1)listen for serial msg
//store in heap
//(2)print msg to serial monitor
//free up heap
//*priority
//round-robin -> both 1
//-------------------------------------------------- only 1 core
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif
//---------------------------------------- global var.
//int message = 100;
//char *string_message = (char *)malloc(sizeof(char) * numberMessageBytes_max);
char *ptrString = NULL;
uint8_t maxPrintBytes = 5;
volatile uint8_t msgReady = 0;
//----------------------------------------------------- Task A -> read
void task_serialListener(void *ptr) {
  
  char charRead[maxPrintBytes];
  int i = 0;

  //memset(charRead, 0, maxPrintBytes);

  while (1) {   
    //------------------------------ listen for input
    while (Serial.available()) {
          
      if (i < maxPrintBytes - 1){     //if [5] -> 4 is reserved -> i up to 3 < 5 - 1
        charRead[i] = Serial.read();
        if (charRead[i] != '\n'){
          i++;
        }else{                        //msg shorter than print buffer  
          charRead[i] = '\0';
                   
          if (msgReady == 0){         //move to heap
            ptrString = (char *)pvPortMalloc((i+1) * sizeof(char));
            memcpy(ptrString, charRead, (i+1) * sizeof(char));
            msgReady = 1;
          }

          i = 0;                              
        }
      }else{                          //msg longer than print buffer -> close now and throw away unread 
        charRead[i] = '\0';
        
        ptrString = (char *)pvPortMalloc((i+1) * sizeof(char));
        memcpy(ptrString, charRead, (i+1) * sizeof(char));
        
        //Serial.begin(115200);       //clear buffer? -> (here) too complicated to implement continuous read
        while(Serial.available() > 0) {
          char t = Serial.read();
          vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        i = 0; 
        msgReady = 1;       
      }            
    }
  }
}
//----------------------------------------------------- Task B -> print
void task_printMsg(void *ptr) {
  while (1) {
    //--------------------------------- check if available
    if (msgReady == 1) {
      //--------------------------------- print msg.
      Serial.println("This program echoes.");
      Serial.println(ptrString);
      msgReady = 0;
      //--------------------------------- free up heap
      vPortFree(ptrString);
    }
  }
}

void setup() {
  //------------------------------------------------ serial setup
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //------------------------------------------------ task setup
  xTaskCreatePinnedToCore(task_serialListener,
                          "Task Serial Listener",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                         );
  xTaskCreatePinnedToCore(task_printMsg,
                          "Task Print Msg",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                         );
  //------------------------------------------------ setup() and loop() not needed
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}
