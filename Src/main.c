/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataType.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256
#define M_PI 3.141592

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

//Serial 
uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_size = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_len;

volatile unsigned char g_tail_pos = 255;
volatile unsigned char g_head_pos;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID=1;
volatile unsigned char checkSize;
volatile unsigned char g_checksum;


volatile Packet_t g_Tx_Packet;
volatile unsigned char g_Tx_checksum;
volatile int g_SendFlag;

// Motor L
volatile uint16_t Motor_CCR_Y = 0;
//position
volatile double g_Pdes_Y = 0., g_Ppre_Y;
volatile double g_Pcur_Y, g_Pvcur_Y;
volatile double g_Perr_Y;
volatile double g_Pderr_Y;

volatile double Kp_p_Y = 4.5;
volatile double Kd_p_Y = 0.15;

//speed
volatile double g_Vcur_Y, g_Vpre_Y;
volatile double g_Vdes_Y = 0.2;
volatile double g_Verr_Y;
volatile double g_Vlimit_Y = 10.;
volatile double g_Verr_sum_Y;

volatile double Kp_s_Y = 1.5042 ;
volatile double Ki_s_Y = 46.660104;

//curreent
volatile double g_Climit = 0.1;		//상위 단에서의 제한 

volatile int g_Pdes_Y_display; 
volatile int g_Vlimit_Y_display;
volatile int g_Climit_display;



volatile double g_vel_control_Y;
volatile double g_pos_control_Y;

//Motor2
volatile uint16_t Motor_CCR_P = 0;
//position
volatile double g_Pdes_P = 0., g_Ppre_P;
volatile double g_Pcur_P, g_Pvcur_P;
volatile double g_Perr_P;
volatile double g_Pderr_P;

volatile double Kp_p_P = 4.5;
volatile double Kd_p_P = 0.15;

//speed
volatile double g_Vcur_P, g_Vpre_P;
volatile double g_Vdes_P = 0.2;
volatile double g_Verr_P;
volatile double g_Vlimit_P = 10.;
volatile double g_Verr_sum_P;

volatile double Kp_s_P = 1.5042 ;
volatile double Ki_s_P = 46.660104;


volatile double g_vel_control_P;
volatile double g_pos_control_P;


volatile int g_Pdes_P_display; 
volatile int g_Vlimit_P_display;
volatile int g_Climit_display;

//TImer4 관련
volatile uint16_t g_TimerCnt;
volatile uint16_t cnt;
volatile uint16_t timer;

//encoder
volatile uint16_t   encoder_cnt_P;
volatile uint16_t   encoder_cnt_Y;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Uart_rx_dma_handler() {
  uint8_t cnt;
  uint8_t i;

  rx_size = 0;
  g_head_pos = huart6.RxXferSize - huart6.hdmarx->Instance->NDTR -1;
  while(g_tail_pos != g_head_pos) {
//    sprintf(tx_buffer,"%c\n",rx_dma_buffer[g_tail_pos]);
//    HAL_UART_Transmit(&huart6,tx_buffer,sizeof(tx_buffer), 10);
//    
    switch (g_PacketMode) {    			//mode0 : 패킷의 들어오는지 확인
      case 0:
            if (rx_dma_buffer[g_tail_pos] == 0xFE) {			//Header check
                    checkSize++;
                    if (checkSize == 4) { // header 4개가 다 들어왔으면 1번 모드로 변경
                        g_PacketMode = 1;
                    }
            }
            else {
                    checkSize = 0;	//초기화 
            }
      break;

      case 1:
        
        g_PacketBuffer.buffer[checkSize++] = rx_dma_buffer[g_tail_pos]; //checkSize 8될때 까지 g_PacketBuffer.buffer에 받아온 값 담기(size,mode,g_checksum)
        if (checkSize == 8){
               g_PacketMode = 2;
        }
      break;
     
     case 2:
       g_PacketBuffer.buffer[checkSize++] = rx_dma_buffer[g_tail_pos];
       g_checksum += rx_dma_buffer[g_tail_pos];
       
       if (checkSize == g_PacketBuffer.data.size){					//data size만큼 버퍼읽음
               if (g_checksum == g_PacketBuffer.data.check){					// checksum check
                 
                    switch(g_PacketBuffer.data.mode){				// 모든게 올바르면, Get target position, Velocity limit Current Limit
                                 case 2:
                                   g_Pdes_Y_display = g_PacketBuffer.data.pos_Y ;
                                   g_Vlimit_Y_display = g_PacketBuffer.data.velo_Y ;
                                   
                                 g_Pdes_Y = g_PacketBuffer.data.pos_Y / 1000.;
                                 g_Vlimit_Y = g_PacketBuffer.data.velo_Y / 1000.;
                                 g_Pdes_P = g_PacketBuffer.data.pos_P / 1000.;
                                 g_Vlimit_P = g_PacketBuffer.data.velo_P / 1000.;
                                 break;
                         }
                   
                       
               }
               //초기화
               g_checksum = 0;						
               g_PacketMode = 0;
               checkSize = 0;
               
       }
       else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){		//오류시 초기화
               g_checksum = 0;
               g_PacketMode = 0;
               checkSize = 0;
       }
    }
    
     if(g_SendFlag >49) {			
            g_SendFlag = 0;
            g_Tx_Packet.data.id	= g_ID;							//motor id 다만 이번과제에선 사용하지 않음
            g_Tx_Packet.data.size = sizeof(Packet_data_t);
            g_Tx_Packet.data.mode = 3;						//mode 3
            g_Tx_Packet.data.check = 0;
            
            g_Tx_Packet.data.pos_Y = g_Pcur_Y*1000;
            g_Tx_Packet.data.velo_Y = g_Vcur_Y* 1000;
            g_Tx_Packet.data.pos_P= 0.1 * 1000;
            g_Tx_Packet.data.velo_P= 0.1 * 1000;
            
            for(int i = 8; i< sizeof(Packet_t);i++)		// checksum 제작
            g_Tx_Packet.data.check += g_Tx_Packet.buffer[i];
            
            for(int i =0; i<g_Tx_Packet.data.size;i++){		//송신
              tx_buffer[i] = g_Tx_Packet.buffer[i];
            }
            HAL_UART_Transmit(&huart6, tx_buffer, g_Tx_Packet.data.size,10);
    }
    
    
    g_tail_pos++;
  }
  
 //     HAL_UART_Transmit(&huart6,"a",2, 10);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim == &htim4) {                                                          //20Khz
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);             //motor1 REF

    	if((g_TimerCnt % 100) == 0){		// position control     200Hz(5ms)

		g_TimerCnt = 0;
                
		//Yaw관련 제어기
		if(g_Pdes_Y < 0) g_Pdes_Y + 2*M_PI;	// 음수 target시 0~360으로 표현
		
		g_Perr_Y = g_Pdes_Y - (g_Pcur_Y);		//position error
		g_Pderr_Y = g_Perr_Y - g_Ppre_Y;		//position error_dot
		
		// 효율적인 도는 방향 정하기
		if(g_Perr_Y > M_PI) {
			g_Perr_Y -= 2*M_PI;
		}
		else if(g_Perr_Y < -M_PI) {
			g_Perr_Y += 2*M_PI;
		}
		
		g_pos_control_Y = (double) g_Perr_Y * Kp_p_Y +  g_Pderr_Y* Kd_p_Y;		//pd controller

		//gear가 적용된 motor의 최대 속도는 161rpm (16.8599 rad/sec)
		if(g_pos_control_Y > 16.8599){
			g_pos_control_Y = 16.8599;
		}
		else if(g_pos_control_Y < -16.8599){
			g_pos_control_Y = -16.8599;
		}
		
                //pitch관련 제어기 
		if(g_Pdes_P < 0) g_Pdes_P + 2*M_PI;	// 음수 target시 0~360으로 표현
		
		g_Perr_P = g_Pdes_P - (g_Pcur_P);		//position error
		g_Pderr_P = g_Perr_P - g_Ppre_P;		//position error_dot
		
		// 효율적인 도는 방향 정하기
		if(g_Perr_P > M_PI) {
			g_Perr_P -= 2*M_PI;
		}
		else if(g_Perr_P < -M_PI) {
			g_Perr_P += 2*M_PI;
		}
		
		g_pos_control_P = (double) g_Perr_P * Kp_p_P +  g_Pderr_P* Kd_p_P;		//pd controller

		//gear가 적용된 motor의 최대 속도는 161rpm (16.8599 rad/sec)
		if(g_pos_control_P > 16.8599){
			g_pos_control_P = 16.8599;
		}
		else if(g_pos_control_P < -16.8599){
			g_pos_control_P = -16.8599;
		}

		g_Ppre_Y = g_Perr_Y;
                g_Ppre_P = g_Perr_P;
		
	}
	if((g_TimerCnt % 10) == 0){			// speed control     2KHz 0.5ms   0.0005초
                
                encoder_cnt_P = TIM3->CNT;
                encoder_cnt_Y = TIM2->CNT;
                g_Pcur_Y = (encoder_cnt_Y / (2048. * 26.)) * 2 * M_PI;   //  encoder counter -> radian/sec
                g_Pcur_P = (encoder_cnt_P / (2048. * 26.)) * 2 * M_PI;   //  encoder counter -> radian/sec
                 
		//speed limit -Vlimit ~ +Vlimit
		if(g_pos_control_Y > g_Vlimit_Y){	
			g_pos_control_Y = g_Vlimit_Y;
		}
		else if(g_pos_control_Y < -g_Vlimit_Y){
			g_pos_control_Y = -g_Vlimit_Y;
		}
		
		g_Vdes_Y = g_pos_control_Y; // rad/sec
			
		g_Vcur_Y = (g_Pcur_Y - g_Pvcur_Y) / 0.0005;	// (현재 엔코더값 - 예전 엔코더값)/0.0005초 -> 현재속도 [rad/sec]
		g_Pvcur_Y = g_Pcur_Y;						// 에전 엔코더값 저장
		
		g_Verr_Y = g_Vdes_Y - g_Vcur_Y;  //speed error
		 
		g_vel_control_Y = g_Verr_Y * Kp_s_Y + g_Verr_sum_Y * Ki_s_Y * 0.0005;		// PI제어기
		
		
		g_Verr_sum_Y += g_Verr_Y;	// spped error sum
		
		//최대 허용 전류 saturation & anti-windup
		if(g_vel_control_Y > 1.6){								
			g_Verr_sum_Y -= (g_vel_control_Y - 1.6) * 1. / 2.5042;	//  anti windup gain은 1/Kps
			g_vel_control_Y = 1.6;
		}
		else if(g_vel_control_Y < -1.6){
			g_Verr_sum_Y -= (g_vel_control_Y + 1.6) * 1. / 2.5042; //  anti windup gain은 1/Kps
			g_vel_control_Y = -1.6;
		}

                //speed limit -Vlimit ~ +Vlimit
		if(g_pos_control_P > g_Vlimit_P){	
			g_pos_control_P = g_Vlimit_P;
		}
		else if(g_pos_control_P < -g_Vlimit_P){
			g_pos_control_P = -g_Vlimit_P;
		}
		
		g_Vdes_P = g_pos_control_P; // rad/sec
			
		g_Vcur_P = (g_Pcur_P - g_Pvcur_P) / 0.0005;	// (현재 엔코더값 - 예전 엔코더값)/0.0005초 -> 현재속도 [rad/sec]
		g_Pvcur_P = g_Pcur_P;						// 에전 엔코더값 저장
		
		g_Verr_P = g_Vdes_P - g_Vcur_P;  //speed error
		 
		g_vel_control_P = g_Verr_P * Kp_s_P + g_Verr_sum_P * Ki_s_P * 0.0005;		// PI제어기
		
		
		g_Verr_sum_P += g_Verr_P;	// spped error sum
		
		//최대 허용 전류 saturation & anti-windup
		if(g_vel_control_P > 1.6){								
			g_Verr_sum_P -= (g_vel_control_P - 1.6) * 1. / 2.5042;	//  anti windup gain은 1/Kps
			g_vel_control_P = 1.6;
		}
		else if(g_vel_control_P < -1.6){
			g_Verr_sum_P -= (g_vel_control_P + 1.6) * 1. / 2.5042; //  anti windup gain은 1/Kps
			g_vel_control_P = -1.6;
		}

		
                Motor_CCR_Y = g_vel_control_Y * (2250. / 1.6) - 1;
                if(Motor_CCR_Y>0) {
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR
                  TIM4->CCR1 = Motor_CCR_Y;
                }
                else{
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);             //motor1 DIR
                  TIM4->CCR1 = -Motor_CCR_Y;
                }
                
                
                Motor_CCR_P = g_vel_control_P * (2250. / 1.6) - 1;                
                if(Motor_CCR_P>0) {
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR
                  TIM4->CCR2 = Motor_CCR_P;
                }
                else{
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);             //motor1 DIR
                  TIM4->CCR2 = -Motor_CCR_P;
                }
	}

        g_SendFlag++;
        g_TimerCnt++;      
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);             //motor1 REF


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
  g_Tx_Packet.data.header[0] = g_Tx_Packet.data.header[1] = g_Tx_Packet.data.header[2] = g_Tx_Packet.data.header[3] = 0xfe;
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart6, rx_dma_buffer, RX_BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);                          //motor1 pwm
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);             //motor1 REF
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);           // motor1 MODE
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);            // motor1 COAST
  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);             //motor2 DIR
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);             //motor2 MODE
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);           // motor2 COAST

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    
    
    Uart_rx_dma_handler();
   
  //  HAL_Delay(10);`
    
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2249;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  huart6.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10 
                           PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
