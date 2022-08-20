//----------------------------------------------------- operation
//1 task
//setup() passes address of a local var. to task using xTaskCreatePinnedToCore()
//original code -> setup() ends before task gets to read the var. local to setup()
//goal: use a mutex to prevent pre-mature exiting of stup() --> not conventional use of mutex
//----------------------------------------------------- knowledge
//Semaphore and mutex are implemented very similarly in FreeRTOS.
//----------------------------------------------------- needed include for vanilla FreeRTOS
//#include semphr.h
//----------------------------------------------------- only 1 core -> config found in a FreeRTOS file
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif
//----------------------------------------------------- pin def
static const int led_pin = 17;
//static const int led_pin = LED_BUILTIN;
//----------------------------------------------------- settings
static SemaphoreHandle_t handle_mutex;
//----------------------------------------------------- task
void blinkLED(void *parameters) {                     //blink LED based on rate passed by parameter

  int num = *(int *)parameters;                       //copy the parameter into a local variable

  xSemaphoreGive(handle_mutex);                       //release mutex after job
  
  Serial.print("Received: ");
  Serial.println(num);

  // Configure the LED pin
  pinMode(led_pin, OUTPUT);

  // Blink forever and ever
  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(num / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(num / portTICK_PERIOD_MS);
  }
}
//----------------------------------------------------- main (its own task with priority 1 on core 1)
void setup() {

  long int delay_arg;

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  Serial.println();
  Serial.println("---FreeRTOS Mutex Challenge---");
  Serial.println("Enter a number for delay (milliseconds)");

  while (Serial.available() <= 0);                    //Wait for input from Serial

  delay_arg = Serial.parseInt();                      //Read integer value
  Serial.print("Sending: ");
  Serial.println(delay_arg);

  //=================================================== create mutex
  handle_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(handle_mutex, 0);                    //take to set mutex to unavail. + 0 delay
                                                      //task will set mutex to avail. -> enable setup() progression
  xTaskCreatePinnedToCore(blinkLED,
                          "Blink LED",
                          1024,
                          (void *)&delay_arg,
                          1,
                          NULL,
                          app_cpu);
  
  //while (xSemaphoreTake(handle_mutex, 100) == pdTRUE);//waiting until mutex is avail.
  xSemaphoreTake(handle_mutex, portMAX_DELAY);
  
  Serial.println("Done!");                            //show -> accomplished passing the stack-based argument
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);              //Do nothing but allow yielding to lower-priority tasks
                                                      //use??
}
