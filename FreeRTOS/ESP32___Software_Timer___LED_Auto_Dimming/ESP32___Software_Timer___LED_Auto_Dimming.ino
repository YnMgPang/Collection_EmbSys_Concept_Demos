//ref:
//https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-8-software-timers/0f64cf758da440a29476165a5b2e577e
//***************************************************** operations
//pretend onboard LED is the backlight for LCD
//use serial terminal to enter char. -> e.g. PuTTY
//has serial input -> LED turns on + stays on
//if serial input stops -> 5s -> LED off
//task A: echo serial + turn on LED
//SW timer: turn off LED
//***use xTimerStart() to restart counter
//***************************************************** findings
//***************************************************** includes
//#include timers.h                                  // vanilla
//***************************************************** only 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//***************************************************** settings & var.
uint8_t maxSerialReadBytes = 32;
static const int led_pin = 17;                       // pin def
//----------------------------------------------------- SW timer
static TimerHandle_t handle_timer = NULL;
//***************************************************** function
void timerCallbackA (TimerHandle_t xTimer){
  digitalWrite(led_pin, LOW);                        // turn off LED
}
//***************************************************** task
//----------------------------------------------------- task A
//keep checking serial
//if avail. -> write HIGH to LED
//read and echo back to terminal
//if not avail. -> start timer
void serialListener(void *parameters){
  char charRead;
  int i = 0;
  
  //xTimerStart(handle_timer, portMAX_DELAY);        // use with while-loop
  while(1){                                          // keep checking serial
    //while(Serial.available()){
    if(Serial.available()){  
      //xTimerReset(handle_timer, portMAX_DELAY);    // use with while-loop                    
      digitalWrite(led_pin, HIGH);                   // if avail. -> write HIGH to LED
      //----------------------------------------------- read and echo back to terminal
      charRead = Serial.read();
      if (charRead  == '\n'){
        Serial.println("\r\n");                      // ***enable implicit LF in PuTTY -> '\n' not working
      }else{
        Serial.print(charRead);
      }
    xTimerStart(handle_timer, portMAX_DELAY);        // use with if-loop
    }
    
  }
}

//***************************************************** main
void setup() {
  //--------------------------------------------------- serial
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //--------------------------------------------------- pin config
  pinMode(led_pin, OUTPUT);
  //--------------------------------------------------- create timer
  handle_timer = xTimerCreate("Serial Timer",             // timer name
                              5000 / portTICK_PERIOD_MS,  // period in ticks -> serial inactive time until dimming
                              pdFALSE,                    // auto-reload toggle
                              (void *)0,                  // timer ID
                              timerCallbackA              // callback f()
                              );
                                                          // heap used to create structures
                                                          // can fail -> check
  if (handle_timer == NULL){
    Serial.println("********* Timer Creation Error *********");                                                                                    
  }else{
    Serial.println("********* Time Created *********");
  }
  //--------------------------------------------------- start task
  xTaskCreatePinnedToCore(serialListener,
                          "Serial Listener",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu
                          );
  //--------------------------------------------------- task for setup() & loop() no longer needed                        
  vTaskDelete(NULL);                          
}

void loop() {
  // put your main code here, to run repeatedly:

}
