//***************************************************** operations
//shared resource -> circular buffer
//5 tasks add val. to circular buffer -> producers
//2 tasks read -> consumers
//producers write task number 3 times
//goal: protect buffer using mutexes and semaphores
//need 2 separate semaphores + 1 mutex
//2 modes -> semaphores & queues
//***************************************************** includes
//#include <semphr.h>                                 //vanilla FreeRTOS
//***************************************************** only 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//***************************************************** settings & var.
enum {BUF_SIZE = 5};                                  //size of buffer array
static const int num_prod_tasks = 5;                  //number of producer tasks
static const int num_cons_tasks = 2;                  //number of consumer tasks
static const int num_writes = 3;                      //num times each producer writes to buf
//----------------------------------------------------- globals
static int buf[BUF_SIZE];                             //shared buffer
static int head = 0;                                  //writing index to buffer
static int tail = 0;                                  //reading index to buffer
static SemaphoreHandle_t bin_sem;                     //waits for parameter to be read
//----------------------------------------------------- semaphore & mutex
static SemaphoreHandle_t handle_semaphore_filled;
static SemaphoreHandle_t handle_semaphore_empty;
static SemaphoreHandle_t handle_mutex;                //protect code involving buffer
//***************************************************** task
//----------------------------------------------------- producer
void producer(void *parameters) {

  int num = *(int *)parameters;                       //copy the parameters into a local variable

  xSemaphoreGive(bin_sem);                            //release the binary semaphore

  //--------------------------------------------------- semaphore protection, okay if "interrupted" here
  for (int i = 0; i < num_writes; i++) {              //fill shared buffer with task number 
    
    xSemaphoreTake(handle_semaphore_empty, portMAX_DELAY);      //one for each write -> put inside for loop
    
    xSemaphoreTake(handle_mutex, portMAX_DELAY);
    //------------------------------------------------- critical section (accessing shared buffer)
    buf[head] = num;                                  //write task number to buffer
    head = (head + 1) % BUF_SIZE;                     //if buffer size is 5
                                                      //all smaller heads(0,1,2,3,4) return the same
                                                      //5 returns 0 -> eff. go to beginning if buffer full                                                     
    xSemaphoreGive(handle_mutex);
    
    xSemaphoreGive(handle_semaphore_filled);          //can't do it before critical section
                                                      //-> once signaled -> could falsely enable consumer
                                                      //***behavior -> if keep giving, circle back to 1st?    
  }
  vTaskDelete(NULL);
}
//----------------------------------------------------- consumer
void consumer(void *parameters) {

  int val;
 
  while (1) {                                         //read from buffer
    xSemaphoreTake(handle_semaphore_filled, portMAX_DELAY);       //take -> i.e. will do below instructions once
                                                                  //can't do more than avail. semaphore count     
    xSemaphoreTake(handle_mutex, portMAX_DELAY); 
    //------------------------------------------------- critical section (accessing shared buffer and Serial)
    val = buf[tail];
    tail = (tail + 1) % BUF_SIZE;  
    Serial.println(val);    
    xSemaphoreGive(handle_mutex);
    //------------------------------------------------- "interrupt" here okay -> assume 0 count of empty
                                                      //producer cannot take empty -> can't update buffer
                                                      //->can't update filled counts
                                                      //->consumer tasks only take as many as available
    xSemaphoreGive(handle_semaphore_empty);
  }
}
//***************************************************** main (its own task with priority 1 on core 1)
void setup() {

  char task_name[12];
  
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  //--------------------------------------------------- create mutexes and semaphores before starting tasks
  bin_sem = xSemaphoreCreateBinary();                 //"created in the empty state"      
  handle_semaphore_filled = xSemaphoreCreateCounting(BUF_SIZE, 0);  //initially 0 counts
  handle_semaphore_empty = xSemaphoreCreateCounting(BUF_SIZE, 5);
  handle_mutex = xSemaphoreCreateMutex();             //"The mutex is initialized to 1 by default."
  //--------------------------------------------------- start producer tasks (wait for each to read argument)
  for (int i = 0; i < num_prod_tasks; i++) {
    sprintf(task_name, "Producer %i", i);
    xTaskCreatePinnedToCore(producer,
                            task_name,
                            1024,
                            (void *)&i,
                            1,
                            NULL,
                            app_cpu
                            );
    xSemaphoreTake(bin_sem, portMAX_DELAY);           //******???don't ++i before i is read by the just started task
  }
  //--------------------------------------------------- start consumer tasks
  for (int i = 0; i < num_cons_tasks; i++) {
    sprintf(task_name, "Consumer %i", i);
    xTaskCreatePinnedToCore(consumer,
                            task_name,
                            1024,
                            NULL,
                            1,
                            NULL,
                            app_cpu
                            );
  }
  Serial.println("----------------All tasks created----------------");//notify that all tasks have been created
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);              //do nothing but allow yielding to lower-priority tasks
}
