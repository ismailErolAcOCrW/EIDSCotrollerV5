/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *180 derece faz farkı alınabılıyor. simdilik artımsal deger otomatık fakat bunu bır encoder
  *baglanacak
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <nextion.h>
#include <flash.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO ITStatus UartReady = RESET;    
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_FINISHED            (0x01)
#define EVENT_LAUNCHED          (0x88)
#define EVENT_UPGRADED          (0x89)
#define EVENT_TOUCH_HEAD            (0x65)
#define EVENT_POSITION_HEAD         (0x67)
#define EVENT_SLEEP_POSITION_HEAD   (0x68)
#define CURRENT_PAGE_ID_HEAD        (0x66)
#define STRING_HEAD                 (0x70)
#define NUMBER_HEAD                 (0x71)
#define INVALID_CMD             (0x00)
#define INVALID_COMPONENT_ID    (0x02)
#define INVALID_PAGE_ID         (0x03)
#define INVALID_PICTURE_ID      (0x04)
#define INVALID_FONT_ID         (0x05)
#define INVALID_VARIABLE        (0x1A)
#define INVALID_OPERATION       (0x1B)

 //------------------ PAGE --------------------
#define INIT_PAGE       (0x00)


// eıds projesi

#define MAIN_PAGE (0x13)  //19. SAYFA
#define TORQUE_PAGE (0x14)  //20. SAYFA
#define RPM_PAGE (0x15)  //21. SAYFA
#define PARAMETERS_PAGE (0x04)


//------------------ BUTTON --------------------
#define MENU_BUTTON       (0x01)
// AUTOMATIC MANUAL SAYFASI BUTONLARI
#define START_BUTTON      (0x02)
#define STOP_BUTTON       (0x17)
// #define RESET_BUTTON       (0x18)

 // PARAMETERS SAYFASI BUTONLARI
#define DEFAULT_BUTTON    (0x02)//
#define SAVE_BUTTON       (0x03)//
#define LOGOUT_BUTTON       (0x0C)//									
//#define CANCEL_BUTTON       (0x02)//


#define RELAY1_BUTTON       (0x08)
#define RELAY2_BUTTON       (0x09)
#define RELAY3_BUTTON       (21)
#define RELAY4_BUTTON       (23)								

#define OFFSET_BUTTON       (0x12)//18

// FACTORY_SETTINGS SAYFASI BUTONLARI
#define YES_BUTTON       (0x04)
#define NO_BUTTON       (0x05)

// START SAYFASI

#define PARAMETERS_BUTTON (0x08)

#define MAIN_BUTTON (0x08) // MAIN BUTON EKLENECEK GEREK YOK
#define TORQUE_BUTTON (0x01)
#define RPM_BUTTON (0x02)


#define LOGIN_BUTTON (0x02)						   
// KOMUTLAR
#define COMMAND_QUIT  (0x01)
#define COMMAND_START (0x02)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//------------------------------ flash adresleri -------------------

uint32_t RPM_Unit_address = 0x08010000;// 1
uint32_t Torque_Unit_address= 0x08010010;// 2
uint32_t Record_address = 0x08010020;//  1



// Flash kayıt parametreleri
uint32_t Fac_RPM_Unit = 1;
uint32_t Fac_Torque_Unit = 2;
uint32_t Fac_Record = 1;//


uint32_t RPM_Unit;
uint32_t Torque_Unit;
uint32_t Record;//

int countReceive;
char CurrentPage;
char CurrentButton;
char CurrentCommand;

char rx_buffer[50];
char reading_buffer[10];

int usartReceiveITEnable;
uint32_t Next_Number_Value = 0 ;

uint16_t torque;
uint16_t rpm;
uint16_t tit;
uint16_t ff;
uint16_t gop;
uint16_t eop;
uint16_t eot;
uint16_t eoq;

int count = 0;
bool nextion_command_ready = false;
bool numberHead = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Flash_Read_All(void);
void Flash_Write_All(void);
void Factory_Settings_Load(void);

void user_pwm_setValue(uint16_t value);

void Clear_rx_buffer(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void nextion_command_control(void);

void FN_MAIN_PAGE(void);
void FN_TORQUE_PAGE(void);
void FN_RPM_PAGE(void);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pulse_value =0,periodValue=9999,periodValue2=3333,prescalerValue=3,prescalerValueOld=2;
/*
 * prescalerValue degerim ile 120derecelik açıyı sabit tutuyor ve frekansı gezdiriyorum
 * frekans araalığımı 3-200 hz arasında prescalerValue degerini ise 1-100 aralıgında gezdiriyorum
 *
 */


uint8_t break_falg =0;
uint16_t dead_value =0;
uint16_t pulse2=0,geriSayim=0;
uint32_t deneme=0;

void startPWM(void)
{
	HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_2);			//OC2			enable
	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_2);	//OC2N		enable

	HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_3);			//OC3			enable
	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_3);	//OC3N		enable
}

void stopPWM(void)
{
	HAL_TIM_OC_Stop(&htim1,TIM_CHANNEL_2);		//stop pwm output	or user function
	HAL_TIMEx_OCN_Stop(&htim1,TIM_CHANNEL_2);

	HAL_TIM_OC_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_OCN_Stop(&htim1,TIM_CHANNEL_3);
}

void pulsePWM(uint16_t pulse)
{

	HAL_Delay(500);

	if(geriSayim==0)
	{
	pulse2++;
	if(pulse2>580)
		{
		geriSayim=1;
		}
	}

	if(geriSayim==1)
	{
		pulse2-=1;//600 oluyor 700de faz takla atıyor.}
		if(pulse2<2){geriSayim=0;}
	}

	//prescalerValue=map(pulse2,0,580,1,100);
	//htim2.Init.Prescaler=prescalerValue;
	//htim3.Init.Prescaler = prescalerValue;
	//htim4.Init.Prescaler = prescalerValue;//64

	//TIM2->PSC=prescalerValue;
	//TIM3->PSC=prescalerValue;
	//TIM4->PSC=prescalerValue;
	//__HAL_TIM_SET_PRESCALER(&htim2,prescalerValue);
	//__HAL_TIM_SET_PRESCALER(&htim3,prescalerValue);
	//__HAL_TIM_SET_PRESCALER(&htim4,prescalerValue);

 if(prescalerValueOld!=prescalerValue)
 {


	  prescalerValueOld=prescalerValue;

	 // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 // HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
	 // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	 // HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

	 //TIM2->PSC &= 0xFF00;
	// TIM3->PSC &= 0xFF00;
	 //TIM4->PSC &= 0xFF00;

	 // MX_TIM2_Init();
	 // MX_TIM3_Init();
	 // MX_TIM4_Init();

	// TIM2->PSC=prescalerValue;
	// TIM3->PSC=prescalerValue;
	// TIM4->PSC=prescalerValue;

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);

	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);

	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
 }
	 //map(x, in_min, in_max, out_min, out_max)


	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse2);//pulse fazın kaydırılgı yer.
}

void deadTimePWM(uint16_t dtime)
{
	/* Set TIMx_BDTR	->	DTG data	*/
	TIM1->BDTR &= 0xFF00;							//clear	DTG data
	TIM1->BDTR |= (dtime & 0x00FF);	//dtime       	//set	DTG data
}


uint16_t pwm_value=0,step=1;
uint16_t arrValue=0;
uint16_t preScalar=0,prePulse=0;
long  freqValue=8000000;
long map(long x,long in_min,long in_max,long out_min,long out_max)
{
return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
//map(value,fromLow,fromHigh,toLow,toHigh);//yapıyı burda actım.
}
uint32_t Constrain(uint32_t au32_IN, uint32_t au32_MIN, uint32_t au32_MAX)
{
	if(au32_IN < au32_MIN)
	{
		return au32_MIN;
	}
	else if (au32_IN > au32_MAX)
	{
		return au32_MAX;
	}
	else
	{
		return au32_IN;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_ENABLE(&htim1);										//TIM1		enable
  	startPWM();
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);	//break interrupt enable
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  
   Flash_Read_All();
  
   HAL_Delay(100);
	CurrentPage = INIT_PAGE;					  
  	Nextion_Page(INIT_PAGE);
  	HAL_Delay(2000);
  	//BAsLANGIÇ SAYFASINA YÖNLENDİR
	HAL_Delay(100);
  	CurrentPage = MAIN_PAGE;
  
  
	HAL_UART_Receive_IT (&huart6, (uint8_t *)reading_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if (CurrentPage == MAIN_PAGE) {
			Nextion_Page(MAIN_PAGE);
			FN_MAIN_PAGE();//5
		} else if (CurrentPage == TORQUE_PAGE) {
			Nextion_Page(TORQUE_PAGE);
			FN_TORQUE_PAGE();//5
		} else if (CurrentPage == RPM_PAGE) {
			Nextion_Page(RPM_PAGE);
			FN_RPM_PAGE();//5
		}
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 590;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 127;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 62499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 31250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void Flash_Write_All()
{
	Flash_Unlock();
	Flash_Erase();

	Flash_Write(RPM_Unit_address, RPM_Unit);
	Flash_Write(Torque_Unit_address, Torque_Unit);
	Flash_Write(Record_address, 1);


	Flash_Lock();
}

void Factory_Settings_Load()
{
	 RPM_Unit = Fac_RPM_Unit;
	 Torque_Unit = Fac_Torque_Unit;
	 Record = Fac_Record;//

}

void Nextion_Settings_Load()
{
}

void Flash_Read_All()
{



	RPM_Unit = Flash_Read(RPM_Unit_address);
	Torque_Unit = Flash_Read(Torque_Unit_address);
	Record = Flash_Read(Record_address);

}


void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)		//need modify
	{
		stopPWM();									//stop pwm output	or user function
		break_falg = 1;
	}
}


void FN_MAIN_PAGE(void)
{
	while (CurrentPage == MAIN_PAGE) {
		pulsePWM(pulse_value);	//user function set
			//dead_value++;
			deadTimePWM(dead_value);

			if(break_falg == 1)			//user function set
			{
				break_falg = 0;				//clear falg
				startPWM();
			}

		   HAL_Delay(1000);
		   step++;//hz degerimi belirleyecek olan degişken
		   step=Constrain(step, 1, 5000);
		   arrValue=((freqValue/step)/128);

		   //arrValue=map(arrValue, 0, 31250, 0, 62499);
		   prePulse=map(arrValue, 0, 62499, 0, 31250);

		   TIM4->ARR=arrValue;
		   TIM4->CCR1=prePulse;




		   Nextion_Set_Value("torque", torque);
		   Nextion_Set_Value("rpm", rpm);
		   Nextion_Set_Value("tit", tit);
		   Nextion_Set_Value("ff", ff);
		   Nextion_Set_Value("gop", gop);
		   Nextion_Set_Value("eop", eop);
		   Nextion_Set_Value("eot", eot);
		   Nextion_Set_Value("eoq", eoq);

	}

}
void FN_TORQUE_PAGE(void)
{
	if (CurrentPage == TORQUE_PAGE)
		{
			HAL_Delay(500);

			if(Torque_Unit == 0)
			 {
				Nextion_Set_Value("c0", 1);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 0);

			 }else if(Torque_Unit == 1)
			 {
				Nextion_Set_Value("c0", 0);
				Nextion_Set_Value("c1", 1);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 0);

			 }else if(Torque_Unit == 2)
			 {
				Nextion_Set_Value("c0", 0);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 1);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 0);
			 }

			  if(Torque_Unit == 3)
			 {
				Nextion_Set_Value("c0", 0);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 1);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 0);

			 }else if(Torque_Unit == 4)
			 {
				Nextion_Set_Value("c0", 0);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 1);
				Nextion_Set_Value("c5", 0);

			 }else if(Torque_Unit == 5)
			 {
				Nextion_Set_Value("c0", 0);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 1);
			 }

			  while(CurrentPage == TORQUE_PAGE){

				HAL_Delay(300);

				if (CurrentButton == DEFAULT_BUTTON) //
				{

				Nextion_Set_Value("c0", 1);
				Nextion_Set_Value("c1", 0);
				Nextion_Set_Value("c2", 0);
				Nextion_Set_Value("c3", 0);
				Nextion_Set_Value("c4", 0);
				Nextion_Set_Value("c5", 0);

				Torque_Unit = Fac_Torque_Unit;

				Flash_Write_All();
				HAL_Delay(100);
				Flash_Read_All();
				CurrentButton = 0;

				}
				else if (CurrentButton == SAVE_BUTTON) //
				{

				Nextion_Send_Command("tsw 255,0");
				HAL_Delay(200);
				Nextion_Get_Value("c0");
				HAL_Delay(200);
				uint8_t c0 = Next_Number_Value;


				Nextion_Get_Value("c1");
				HAL_Delay(200);
				uint8_t c1 = Next_Number_Value;


				Nextion_Get_Value("c2");
				HAL_Delay(200);
				uint8_t c2 = Next_Number_Value;


				Nextion_Get_Value("c3");
				HAL_Delay(200);
				uint8_t c3 = Next_Number_Value;


				Nextion_Get_Value("c4");
				HAL_Delay(200);
				uint8_t c4 = Next_Number_Value;


				Nextion_Get_Value("c5");
				HAL_Delay(200);
				uint8_t c5 = Next_Number_Value;


				if(c0 == 1)
				{
					Torque_Unit = 0;
				}
				else if(c1 == 1)
				{
					Torque_Unit = 1;
				}
				else if(c2 == 1)
				{
					Torque_Unit = 2;
				}

				if(c3 == 1)
				{
					Torque_Unit = 3;
				}
				else if(c4 == 1)
				{
					Torque_Unit = 4;
				}
				else if(c5 == 1)
				{
					Torque_Unit = 5;
				}
				Flash_Write_All();
				HAL_Delay(100);
				Flash_Read_All();
				Nextion_Send_Command("tsw 255,255");
				CurrentButton = 0;

				}
				else if (CurrentButton == 0)
				{
				}
			  }
		}else {
		}

}
void FN_RPM_PAGE(void)
{

	if (CurrentPage == RPM_PAGE)
			{
				HAL_Delay(500);

				if(RPM_Unit == 0)
				 {
					Nextion_Set_Value("c0", 1);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 0);

				 }else if(RPM_Unit == 1)
				 {
					Nextion_Set_Value("c0", 0);
					Nextion_Set_Value("c1", 1);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 0);

				 }else if(RPM_Unit == 2)
				 {
					Nextion_Set_Value("c0", 0);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 1);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 0);
				 }

				  if(RPM_Unit == 3)
				 {
					Nextion_Set_Value("c0", 0);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 1);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 0);

				 }else if(RPM_Unit == 4)
				 {
					Nextion_Set_Value("c0", 0);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 1);
					Nextion_Set_Value("c5", 0);

				 }else if(RPM_Unit == 5)
				 {
					Nextion_Set_Value("c0", 0);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 1);
				 }

				  while(CurrentPage == RPM_PAGE){

					HAL_Delay(300);

					if (CurrentButton == DEFAULT_BUTTON) //
					{

					Nextion_Set_Value("c0", 1);
					Nextion_Set_Value("c1", 0);
					Nextion_Set_Value("c2", 0);
					Nextion_Set_Value("c3", 0);
					Nextion_Set_Value("c4", 0);
					Nextion_Set_Value("c5", 0);

					RPM_Unit = Fac_RPM_Unit;

					Flash_Write_All();
					HAL_Delay(100);
					Flash_Read_All();
					CurrentButton = 0;

					}
					else if (CurrentButton == SAVE_BUTTON) //
					{
					Nextion_Send_Command("tsw 255,0");
					HAL_Delay(200);
					Nextion_Get_Value("c0");
					HAL_Delay(200);
					uint8_t c0 = Next_Number_Value;

					Nextion_Get_Value("c1");
					HAL_Delay(200);
					uint8_t c1 = Next_Number_Value;

					Nextion_Get_Value("c2");
					HAL_Delay(200);
					uint8_t c2 = Next_Number_Value;

					Nextion_Get_Value("c3");
					HAL_Delay(200);
					uint8_t c3 = Next_Number_Value;

					Nextion_Get_Value("c4");
					HAL_Delay(200);
					uint8_t c4 = Next_Number_Value;

					Nextion_Get_Value("c5");
					HAL_Delay(200);
					uint8_t c5 = Next_Number_Value;

					if(c0 == 1)
					{
						RPM_Unit = 0;
					}
					else if(c1 == 1)
					{
						RPM_Unit = 1;
					}
					else if(c2 == 1)
					{
						RPM_Unit = 2;
					}

					if(c3 == 1)
					{
						RPM_Unit = 3;
					}
					else if(c4 == 1)
					{
						RPM_Unit = 4;
					}
					else if(c5 == 1)
					{
						RPM_Unit = 5;
					}
					Flash_Write_All();
					HAL_Delay(100);
					Flash_Read_All();
					Nextion_Send_Command("tsw 255,255");
					CurrentButton = 0;

					}
					else if (CurrentButton == 0)
					{
					}
				  }
			}else {
			}



}


void Clear_rx_buffer(void)
{
	for(int i = 0; i < 50; i++)
	  	  rx_buffer[i] = '\0';
}

void nextion_command_control()
{
	if (rx_buffer[0] == EVENT_TOUCH_HEAD) { // 0X65 BUTON


		if (rx_buffer[1] == INIT_PAGE) {
		}else if (rx_buffer[1] == MAIN_PAGE) {
			if (rx_buffer[2] == TORQUE_BUTTON) {
				CurrentPage = TORQUE_PAGE;
				CurrentButton = TORQUE_BUTTON;
			}else if (rx_buffer[2] == RPM_BUTTON) {
				CurrentPage = RPM_PAGE;
				CurrentButton = RPM_BUTTON;
			}
		}else if (rx_buffer[1] == TORQUE_PAGE) {
			if (rx_buffer[2] == MENU_BUTTON) {
				 CurrentPage = MAIN_PAGE;
			}else if (rx_buffer[2] == DEFAULT_BUTTON) {
				 CurrentButton = DEFAULT_BUTTON;
			}else if (rx_buffer[2] == SAVE_BUTTON) {
				 CurrentButton = SAVE_BUTTON;
			}

		}else if (rx_buffer[1] == RPM_PAGE) {
			if (rx_buffer[2] == MENU_BUTTON) {
				CurrentPage = MAIN_PAGE;
			}else if (rx_buffer[2] == DEFAULT_BUTTON) {
				 CurrentButton = DEFAULT_BUTTON;
			}else if (rx_buffer[2] == SAVE_BUTTON) {
				 CurrentButton = SAVE_BUTTON;
			}
		}




	}else if (rx_buffer[0] == STRING_HEAD) {
	}else if (rx_buffer[0] == NUMBER_HEAD) {
		numberHead=true;
		if (rx_buffer[0] == NUMBER_HEAD
				&& rx_buffer[5] == 0xFF
				&& rx_buffer[6] == 0xFF
				&& rx_buffer[7] == 0xFF
		){
			Next_Number_Value = (rx_buffer[4] << 24) | (rx_buffer[3] << 16) | (rx_buffer[2] << 8) | (rx_buffer[1]);
		}
	}else if (rx_buffer[0] == CMD_FINISHED) {
	}else if (rx_buffer[0] == INVALID_CMD) {
	}else if (rx_buffer[0] == INVALID_VARIABLE) {
	}else {  
	}
	
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	//HAL_UART_Receive_IT (&huart6, (uint8_t *)reading_buffer, 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6 ){
		rx_buffer[count] = reading_buffer[0];
		if (rx_buffer[count] == 0xff) {
			if (rx_buffer[count-1] == 0xff) {
				if (rx_buffer[count-2] == 0xff) {
					count=0;
					countReceive=0;
					nextion_command_ready = true;
				}else {
					count++;
				}
			}else {
				count++;
			}
		}else {
			count++;
		}
	}

	if(huart->Instance == USART6 && nextion_command_ready == true){
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
			UartReady = RESET;
			nextion_command_control();
		usartReceiveITEnable = 0;
		nextion_command_ready = false;
		Clear_rx_buffer();

		//__NOP();
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
		UartReady = SET;
	}

	/*
	* buffer 7 den fazla ise ne yapılacak
	*  buffer 7 önemli gelen verinin boyutunu veriyor
	* nextion buton verileri 7 hex ten oluşuyor
	* 65 01 08 00 FF FF FF
	* 65 buton
	* 01 sayfa
	* 08 component id
	* 00 bilgi
	* ff bitiş kodları
	* ff bitiş kodları
	* ff bitiş kodları			  
	*/
	//Clear_rx_buffer();
	//HAL_UART_Receive_IT (&huart3, rx_buffer, 7);
	HAL_UART_Receive_IT (&huart6, (uint8_t *)reading_buffer, 1);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
