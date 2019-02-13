/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "vl53l0x_api.h"
#include "stdlib.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern VL53L0X_Dev_t	myDevStruct[2];
extern VL53L0X_DEV myDev;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
xSemaphoreHandle xBinarySemaphore1;
xSemaphoreHandle xBinarySemaphore2;
xSemaphoreHandle xBinarySemaphore3;
xSemaphoreHandle xBinarySemaphoreStop;
xSemaphoreHandle xBinarySemaphoreStart;
xQueueHandle xQueue;
xQueueHandle xQueueStart;
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct struct_conn_t {
  uint32_t conn;
  uint32_t buf;
} struct_conn;
struct_conn conn01;
//-----------------------------------------------------------------------------------------------------------------------------------------------

osThreadId Task01Handle, Task02Handle, Task03Handle,task_startHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void vHandlerTask1( void *pvParameters );
static void vHandlerTask2( void *pvParameters );
static void tcp_thread( void *pvParameters );
static void task_start(void *pvParameters);
static void task_stop(void *pvParameters);
static void task_servo(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
//-----------------------------------------------------------------------------------------------------------------------------------------------

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
vSemaphoreCreateBinary( xBinarySemaphore1 );
vSemaphoreCreateBinary( xBinarySemaphore2 );
vSemaphoreCreateBinary( xBinarySemaphore3 );
vSemaphoreCreateBinary( xBinarySemaphoreStart );
vSemaphoreCreateBinary( xBinarySemaphoreStop );
xQueue = xQueueCreate(1, sizeof( int ) );
xQueueStart = xQueueCreate(1, sizeof( int ) );

if( xBinarySemaphore1 != NULL && xBinarySemaphore2 != NULL && xBinarySemaphore3 != NULL && xBinarySemaphoreStart != NULL && xBinarySemaphoreStop != NULL)
{
    osThreadDef(task_start, task_start, 2, 0, 400);
    task_startHandle = osThreadCreate(osThread(task_start), NULL);
}
vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
//-----------------------------------------------------------------------------------------------------------------------------------------------

void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
struct netconn *conn;
err_t err;
ip_addr_t ServerIPaddr;
IP4_ADDR(&ServerIPaddr, 192, 168, 0, 189);
conn = netconn_new(NETCONN_TCP);
	if(conn!=NULL)
	{
		err = netconn_bind(conn, NULL, 4555);
	  	if (err == ERR_OK)
	  	{
	  		err = netconn_connect(conn, &ServerIPaddr, 5000);
	  		while(err!=ERR_OK)
	  		{
	  			netconn_delete(conn);
	  		    conn = netconn_new(NETCONN_TCP);
  		    	if(conn!=NULL)
  		    	{
	  		    	err = netconn_bind(conn, NULL, 4555);
  		    		if (err == ERR_OK)
	  		    	{
	  		    		err = netconn_connect(conn, &ServerIPaddr, 5000);
	  		    	}
  		    	}
	  		    vTaskDelay(10);
	  		}
	  		if (err==ERR_OK)
	  		{
				HAL_UART_Transmit(&huart2, "port listening\r\n", strlen("port listening\r\n"), 0x100);
				conn01.conn = conn;
				sys_thread_new("tcp_thread", tcp_thread, (void*)&conn01, 1500, osPriorityHigh );
			}
	  	}
		else
		{
			netconn_delete(conn);
		}
	}

  /* Infinite loop */
  for(;;)
  {
  	  osDelay(1);
  }

  /* USER CODE END StartDefaultTask */
}
//-----------------------------------------------------------------------------------------------------------------------------------------------


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


static void tcp_thread(void *arg)
{
HAL_UART_Transmit(&huart2, "TCP_THREAD\r\n", strlen("TCP_THREAD\r\n"), 0x100);
struct_conn *arg_conn;
struct netconn *conn;
err_t sent_err;
err_t ent_err;
arg_conn = (struct_conn*) arg;
conn = (void*)arg_conn->conn;
portBASE_TYPE xStatus;
int *buf;
int lReceivedValue;
static int i=1;
	HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); //go to sleep mode
    HAL_ResumeTick();
  	xStatus = xQueueReceive( xQueueStart, &lReceivedValue, portMAX_DELAY);
	sent_err=netconn_write(conn, "start", 5, NETCONN_COPY);
	sent_err=netconn_write(conn, "go!!!", 5, NETCONN_COPY);
		if (sent_err!=ERR_OK)
    	{
			netconn_delete(conn);
    	}
		else
		{
			sys_thread_new("task_stop", task_stop, (void*)conn, 2048, 3 );
			sys_thread_new("task_servo", task_servo, (void*)conn, 512, 1 );
			for(;;)
			{
				if (i==1)
				{
		    		xStatus = xQueueReceive( xQueue, &lReceivedValue, portMAX_DELAY);
	  		    	taskENTER_CRITICAL();
	  		    	HAL_UART_Transmit(&huart2, "can`t move\r\n", strlen("can`t move\r\n"), 0x100);
	  		    	buf=&lReceivedValue;
   					sent_err=netconn_write(conn, "stop!", 5, NETCONN_COPY);
   					if (sent_err!=ERR_OK)
	  		    	{
   						HAL_UART_Transmit(&huart2, "Send data fail\r\n", strlen("Send data fail\r\n"), 0x100);
	  		    	  	netconn_close(conn);
   				    }
   					taskEXIT_CRITICAL();
	  		    	i=2;
	  		    	}
				else if (i==2)
				{
					xStatus = xQueueReceive( xQueue, &lReceivedValue, 100/portTICK_RATE_MS);
	  		    	if (xStatus==errQUEUE_EMPTY)
	  		    	{
	  		    		taskENTER_CRITICAL();
	  		    		HAL_UART_Transmit(&huart2, "Let`s go\r\n", strlen("Let`s go\r\n"), 0x100);
	  		    		buf=&lReceivedValue;
	  		    		ent_err=netconn_write(conn, "go!!!", 5, NETCONN_COPY);
	  		    		if (sent_err!=ERR_OK)
	  		    		{
	  		    			HAL_UART_Transmit(&huart2, "Send data fail\r\n", strlen("Send data fail\r\n"), 0x100);
	  		    			netconn_close(conn);
	  		    		}
	  		    			i=1;
	  		  		    	taskEXIT_CRITICAL();
	  		    	}
	  		    	else if (xStatus==pdPASS)
	  		    	{
	  		    		continue;
	  		    	}
				}
			}
		}

}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void vHandlerTask1( void *pvParameters )
{
myDev=&myDevStruct[0];
uint8_t status;
static int j=0;
int lValueToSend;
portBASE_TYPE xStatus;
VL53L0X_RangingMeasurementData_t myRangingData;
	for( ;; )
  {
		myDev=&myDevStruct[0];
		xSemaphoreTake( xBinarySemaphore1, portMAX_DELAY );
		taskENTER_CRITICAL();
		myDev=&myDevStruct[0];
		if(j>=1)
		{
			VL53L0X_GetRangingMeasurementData(myDev, &myRangingData);
			lValueToSend=(int)myRangingData.RangeMilliMeter;
			xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );
			j=2;
		}
		j++;
		VL53L0X_ClearInterruptMask(myDev, -1);
		taskEXIT_CRITICAL();
   }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------


static void vHandlerTask2( void *pvParameters )
{
myDev=&myDevStruct[1];
uint8_t status;
static int j=0;
int lValueToSend;
portBASE_TYPE xStatus;
VL53L0X_RangingMeasurementData_t myRangingData;
	for( ;; )
  {
		myDev=&myDevStruct[1];
		xSemaphoreTake( xBinarySemaphore2, portMAX_DELAY);
		taskENTER_CRITICAL();
		myDev=&myDevStruct[1];
		if(j>=1)
		{
			VL53L0X_GetRangingMeasurementData(myDev, &myRangingData);
			lValueToSend=(int)myRangingData.RangeMilliMeter;
			xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );
			j=2;
		}
		j++;
		VL53L0X_ClearInterruptMask(myDev, -1);
		taskEXIT_CRITICAL();
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void vHandlerTask3( void *pvParameters )
{
myDev=&myDevStruct[2];
uint8_t status;
static int j=0;
int lValueToSend;
portBASE_TYPE xStatus;
VL53L0X_RangingMeasurementData_t myRangingData;
	for( ;; )
	{
		myDev=&myDevStruct[2];
		xSemaphoreTake( xBinarySemaphore3, portMAX_DELAY );
		taskENTER_CRITICAL();
		myDev=&myDevStruct[2];
		if(j>=1)
		{
			VL53L0X_GetRangingMeasurementData(myDev, &myRangingData);
			lValueToSend=(int)myRangingData.RangeMilliMeter;
			xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );
			j=2;
		}
		j++;
		VL53L0X_ClearInterruptMask(myDev, -1);
		taskEXIT_CRITICAL();
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void task_start(void *pVparametrs)
{
	int i=1;
	xSemaphoreTake(xBinarySemaphoreStart, 100);
	xSemaphoreTake(xBinarySemaphoreStart, portMAX_DELAY );
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_UART_Transmit(&huart2, "enter to Start\r\n", strlen("enter to Start\r\n"), 0x100);
	osThreadDef(tsk01, vHandlerTask1, 1, 0, 350);
	Task01Handle = osThreadCreate(osThread(tsk01), NULL);
	osThreadDef(tsk02, vHandlerTask2, 1, 0, 350);
	Task02Handle = osThreadCreate(osThread(tsk02), NULL);
	osThreadDef(tsk03, vHandlerTask3, 1, 0, 350);
	Task02Handle = osThreadCreate(osThread(tsk03), NULL);
	xQueueSendToBack( xQueueStart, &i, 0 );
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	osThreadTerminate(NULL);
		for(;;)
		{
		}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void task_stop(void *pVparametrs)

{
err_t err;
struct netconn *conn;
conn=(struct netconn *) pVparametrs;
xSemaphoreTake(xBinarySemaphoreStop, 100);
HAL_UART_Transmit(&huart2, "enter to Stop\r\n", strlen("enter to Stop\r\n"), 0x100);
xSemaphoreTake(xBinarySemaphoreStop, portMAX_DELAY );
err=netconn_write(conn, "emerg", 5, NETCONN_COPY);
netconn_close(conn);
netconn_delete(conn);
HAL_NVIC_DisableIRQ(EXTI4_IRQn);
HAL_NVIC_DisableIRQ(EXTI2_IRQn);
HAL_NVIC_DisableIRQ(EXTI3_IRQn);
osThreadTerminate(Task01Handle);
osThreadTerminate(Task02Handle);
osThreadTerminate(Task03Handle);
osThreadTerminate(tcp_thread);
osThreadTerminate(defaultTaskHandle);
osThreadTerminate(task_servo);
HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
HAL_SuspendTick();
HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  		      for(;;)
  		      {
  		      }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void task_servo(void *pVparametrs)
{
err_t recv_err;
struct netbuf *inbuf;
struct netconn *conn;
uint8_t* buf;
u16_t buflen;
conn=(struct netconn *) pVparametrs;
HAL_UART_Transmit(&huart2, "enter to servo\r\n", strlen("enter to servo\r\n"), 0x100);
		for(;;)
			{
				recv_err = netconn_recv(conn, &inbuf);
				if (recv_err == ERR_OK)
				{
				  if (netconn_err(conn) == ERR_OK)
				  {
				    netbuf_data(inbuf, (void**)&buf, &buflen);
				    if(buflen>1)
				    {
			    	HAL_UART_Transmit(&huart2, (uint8_t*)buf, 5, 0x100);
			    	TIM1->CCR1=1600;
			    	TIM2->CCR4=1600;
					vTaskDelay(2000);
					TIM1->CCR1=800;
					TIM2->CCR4=800;
					}
				  netbuf_delete(inbuf);
				  }
				}
			}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
