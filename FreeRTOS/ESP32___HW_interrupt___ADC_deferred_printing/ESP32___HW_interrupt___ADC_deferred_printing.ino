//ref:
//https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-9-hardware-interrupts/3ae7a68462584e1eb408e1638002e9ed
//****************************************************** operation
//timer ISR reads adc pin
//timer signals task using semaphore
//task prints adc values
//****************************************************** findings
//BUG -> check below
//****************************************************** includes
//#include semphr.h                                   // vanilla
//****************************************************** only 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//****************************************************** settings
static const int adc_pin = A0;                        // pin def
//------------------------------------------------------ HW timer interrupt
static const uint16_t timer_divider = 80;             // ESP32 base clock = 80MHz -> divided clock = 1MHz
static const uint64_t timer_max_count = 1000000;
//****************************************************** var.
static hw_timer_t *timer = NULL;                      // ESP32 HAL timer -> comes w/ Arduino lib.
static volatile uint16_t val = 0;                     // protect -> val. might change outside the scope of current task???
static volatile int valTwo = 0;
static SemaphoreHandle_t bin_sem = NULL;
//static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;  //for critical section in ISR
//****************************************************** ISR
void IRAM_ATTR onTimer(){                             // attribute qualifier -> put in RAM -> faster

  BaseType_t task_woken = pdFALSE;
  //val = analogRead(adc_pin);
  //val = hallRead();
  valTwo = hallRead();                                // ****** BUG ******
                                                      // no. mostly -ve, does not react
  xSemaphoreGiveFromISR(bin_sem, &task_woken);

  if (task_woken){                                    // ESP-IDF yield() takes no arg.
    portYIELD_FROM_ISR();
  }
  
  //---------------------------------------------------- FreeRTOS -> DO NOT call other FreeRTOS API f() from within
  //portENTER_CRITICAL_ISR(&spinlock);
  ///...
  //portEXIT_CRITICAL_ISR(&spinlock);
  //---------------------------------------------------- vanilla ISR critical section
  //UBaseType_t saved_int_status;
  //save_int_status = taskENTER_CRITICAL_FROM_ISR();
  //...
  //taskEXIT_CRITICAL_FROM_ISR(saved_int_status);

}
//****************************************************** task
//------------------------------------------------------ task A
void printValues(void *parameters){
  while(1){
    xSemaphoreTake(bin_sem, portMAX_DELAY);          // wait indef. for go-ahead
    Serial.println(valTwo);
  }
}
//****************************************************** main
void setup() {

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("-----------------------------");

  bin_sem = xSemaphoreCreateBinary();
  if (bin_sem == NULL){
    Serial.println("Could not create semaphore");
    ESP.restart();
  }

  xTaskCreatePinnedToCore(printValues,
                          "Print Values",
                          1024,
                          NULL,
                          2,                          // priority 2
                          NULL,
                          app_cpu
                         );
  
  //pinMode(adc_pin, OUTPUT);                           // pin config
  //---------------------------------------------------- timer
  timer = timerBegin(0, timer_divider, true);         // create + start -> (num, divider, countUp?)
  timerAttachInterrupt(timer, &onTimer, true);        // provide ISR -> (..., ..., edgeTrig?)
  timerAlarmWrite(timer, timer_max_count, true);      // def. count -> (timer, max. count, autoReload?)
  timerAlarmEnable(timer);                            // allow ISR to trigger
}

void loop() {
  // put your main code here, to run repeatedly:

}
