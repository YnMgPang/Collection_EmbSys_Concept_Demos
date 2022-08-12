//ref:https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-3-task-scheduling/8fbb9e0b0eed4279a2dd698f02ce125f
//-------------------------------------------------- operation
//2 functions
//1 reads serial input and update blinking rate
//1 blinks LED
//---------------------------------------- includes
//#include <stdlib.h>                                         //for atoi()
//---------------------------------------- only 1 core
#if CONFIG_FREERTOS_UNICORE                                   //check FreeRTOSConfig.h in ESP32 board package
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif
//---------------------------------------- pin def
static const int led_pin = 17;
//---------------------------------------- global var.
int delay_blink = 100;
//---------------------------------------- task 1 -> read serial
void task_readSerial(void *parameter){
  uint8_t numberMessageBytes_max = 5;
  char string_delay_blink[numberMessageBytes_max];
  char c;
  int i = 0;
    
  while(1){
    
    while(Serial.available()){
      if (i < numberMessageBytes_max - 1){
        string_delay_blink[i] = Serial.read();        //read after condition check -> not dumping last char
        if (string_delay_blink[i] != '\n'){
          i++;
        }else{
          string_delay_blink[i] = '\0';
          
          delay_blink = atoi(string_delay_blink);
          
          i = 0;
          Serial.println(delay_blink);
        }
      }else{                                          //msg longer than print buffer -> clear serial buffer
                                                      //not critical in this application
        string_delay_blink[i] = '\0';
        
        delay_blink = atoi(string_delay_blink);
        
        i = 0;
        Serial.println(delay_blink);
      }         
    }
  } 
}
//---------------------------------------- task 2 -> blink LED
void task_blinkLED(void *parameter){
  while(1){
    digitalWrite(led_pin, HIGH);                           
    vTaskDelay(delay_blink / portTICK_PERIOD_MS);          
    digitalWrite(led_pin, LOW);
    vTaskDelay(delay_blink / portTICK_PERIOD_MS);
    }
}

void setup() {
  //-------------------------------------- serial setup
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);                  //wait for serial conn. to finish setup
  //------------------------------------------- pin config
  pinMode(led_pin, OUTPUT);
  //-------------------------------------- create task
  xTaskCreatePinnedToCore(task_readSerial,
                          "Task Read Serial",
                          1024,
                          NULL,                           //using global, not passing
                          1,
                          NULL,      
                          app_cpu
                          );
  xTaskCreatePinnedToCore(task_blinkLED,
                          "Task Blink LED",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                          );
  //------------------------------------------- task for setup() & loop() no longer needed
  vTaskDelete(NULL);        
}

void loop() {
  // put your main code here, to run repeatedly:

}
