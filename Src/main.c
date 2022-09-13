/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Odometry.h"
#include "Motors.h"
#include "Robot_Navi_Euro20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float current_x,current_y,current_phi_deg;
extern float total_right,total_left;
extern float** matrix;
int taille;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	set_dimentions(39.9858,39.9196,243,139);//291.28//291.35//39.98,39.3740.8:40.2
	
	set_motors(&htim1,4499,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_1,TIM_CHANNEL_2);
  set_right_encoder(&htim3,TIM3,400,4,1);
	set_left_encoder(&htim4,TIM4,400,4,-1);
	
	HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	set_PWM_min(900,900,900,900);
	
//taille=6;
//allocation(taille);


//matrix[0][0]=300;
//matrix[0][1]=-300;
//matrix[0][2]=-90;


//matrix[1][0]=0;
//matrix[1][1]=-600;
//matrix[1][2]=-180;

//matrix[2][0]=-300;
//matrix[2][1]=-900;
//matrix[2][2]=-90;

//matrix[3][0]=0;
//matrix[3][1]=-1200;
//matrix[3][2]=0;

//matrix[4][0]=300;
//matrix[4][1]=-1500;
//matrix[4][2]=-90;

//matrix[5][0]=0;
//matrix[5][1]=-1800;
//matrix[5][2]=-180;

//Robot_Locate_Multi_Curv(matrix ,taille,300);
//free(matrix);

//HAL_Delay(3000);

//uint8_t s[3] ;
//HAL_UART_Transmit(&huart4,"c",1,10);
//orientate(30,500);
//move_distance(500,700);
//HAL_UART_Transmit(&huart4,"bgm",3,10);
//HAL_Delay(750);
//HAL_UART_Transmit(&huart4,"hnda",4,10);
//Robot_Locate(0,1000,700);
//HAL_UART_Transmit(&huart4,"ek",2,10);
//HAL_Delay(500);
//HAL_UART_Transmit(&huart4,"fl",2,10);
//HAL_Delay(500);
//HAL_UART_Transmit(&huart4,"ek",2,10);
//HAL_Delay(500);
//HAL_UART_Transmit(&huart4,"fl",2,10);
//HAL_Delay(500);
//Robot_Locate(0,0,700);



//orientate(180,200);
//orientate(90,100);
//rotate(3600,200);

/*HAL_Delay(2000);

HAL_UART_Transmit(&huart4,"cb",2,10);

//////

allocation(2);
matrix[0][0]=683.46;
matrix[0][1]=-139.46;
matrix[0][2]=-51.28;

matrix[1][0]=520;
matrix[1][1]=-480;
matrix[1][2]=-180;

Robot_Locate_Multi_Curv(matrix,2,700);
HAL_UART_Transmit(&huart4,"bgm",3,10);
HAL_Delay(750);
HAL_UART_Transmit(&huart4,"hndacek",7,10);
HAL_Delay(1000);

Robot_locateCurv(189,-312.27,126.25,700);
HAL_UART_Transmit(&huart4,"flbgm",5,10);
HAL_Delay(750);
HAL_UART_Transmit(&huart4,"ekhndflac",9,10);
HAL_Delay(500);

HAL_UART_Transmit(&huart4,"b",1,10);
matrix[0][0]=819.52;
matrix[0][1]=-54.06;
matrix[0][2]=54.95;
matrix[1][0]=926.53;
matrix[1][1]=404.4;
matrix[1][2]=98.77;

Robot_Locate_Multi_Curv(matrix,2,700);
HAL_UART_Transmit(&huart4,"ekbgm",5,10);
HAL_Delay(750);
HAL_UART_Transmit(&huart4,"flhndac",7,10);
HAL_Delay(1000);*/

//rotate(720,100);


/*Robot_Locate(650.86,0.0,700);//640
Robot_Locate(650.86,160.0,700);
HAL_UART_Transmit(&huart4,"a",7,10);
HAL_Delay(8000);
move_distance(-160 ,200);
orientate(90,300);

HAL_UART_Transmit(&huart4,"c",1,10);

Robot_Locate(-265.14,0.0,700);
HAL_UART_Transmit(&huart4,"b",1,10);
HAL_Delay(4000);
move_distance(-600,200);
orientate(180,300);

HAL_UART_Transmit(&huart4,"d",1,10);
HAL_Delay(2000);

move_distance(-150,200);*/





/*Robot_locateCurv(-128.66,-422.93,180,200);
Robot_locateCurv(-780.26,-18.56,90,200);
orientate(180,300);
move_distance(-50,200);
Robot_Locate(-760.26,-547.0,700);
orientate(180,300);*///pour l'activation de la phare

/*Robot_locateCurv(145.19,-589.0,180,400);
Robot_Locate(-812.0,-587.0,700);
HAL_UART_Transmit(&huart4,"a",7,10);
HAL_Delay(8000);
move_distance(-700,200);
Robot_Locate(-50,-50,700);
HAL_UART_Transmit(&huart4,"d",1,10);
HAL_Delay(2000);*/

//Robot_Locate_Multi_Curv(matrix,2,300);

/*HAL_UART_Transmit(&huart4,"c",1,10);
HAL_Delay(1000);*/
//move_distance(1100.0,300);//640

//move_distance(500.0,300);//640
//HAL_Delay(1500);
//rotate(1800,100);
/*HAL_Delay(1500);*/
//move_distance(800.0,300);
/*HAL_Delay(1500);//640*/
//rotate(3600,100);

//HAL_UART_Transmit(&huart4,"z",1,10);
Robot_Locate(660,0,700);
Robot_Locate(660.0,155,700);//640
HAL_UART_Transmit(&huart4,"a",1,10);
HAL_Delay(4500);
move_distance(-160,700);//640
Robot_Locate(-250.0,30,700);//640
HAL_UART_Transmit(&huart4,"b",1,10);
HAL_Delay(3000);
move_distance(-545,700);//640
HAL_UART_Transmit(&huart4,"c",1,10);
HAL_Delay(1000);
move_distance(-400,700);
//Robot_locateCurv(0,250,90,200);
//Robot_locateCurv(-7,-18.56,90,200);





	while (1)                     
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//run_forward(2000,2000);
		//HAL_Delay(3000);
		//run_backward(2000,2000);
		//HAL_Delay(3000);
		
		
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

/* USER CODE BEGIN 4 */
/*void bhema()
{
	HAL_UART_Transmit(&huart4,"c",1,10);
Robot_Locate(137.58,76.44,700);
HAL_UART_Transmit(&huart4,"bgm",3,10);
HAL_Delay(750);
HAL_UART_Transmit(&huart4,"hndac",5,10);
//////

allocation(2);
matrix[0][0]=500;
matrix[0][1]=300;
matrix[0][2]=90;


matrix[1][0]=407.31;
matrix[1][1]=656.5;
matrix[1][2]=119.15;

Robot_Locate_Multi_Curv(matrix,2,700);

HAL_UART_Transmit(&huart4,"ekbgm",5,10);
HAL_Delay(1000);
HAL_UART_Transmit(&huart4,"flhndac",7,10);
HAL_Delay(500);

matrix[0][0]=53.16;
matrix[0][1]=786.63;
matrix[0][2]=-159.5;


matrix[1][0]=-377.14;
matrix[1][1]=956.83;
matrix[1][2]=116.34;

Robot_Locate_Multi_Curv(matrix,2,700);
HAL_UART_Transmit(&huart4,"bgm",3,10);
HAL_Delay(500);
HAL_UART_Transmit(&huart4,"hnda",4,10);
Robot_locateCurv(-757.87,1550,129.05,700);
orientate(-180,700);
HAL_UART_Transmit(&huart4,"c",1,10);
Robot_Locate(-1050,1550,700);
HAL_UART_Transmit(&huart4,"d",1,10);
move_distance(-100,700);
orientate(0,700);
move_distance(-100,700);
HAL_UART_Transmit(&huart4,"i",1,10);
move_distance(100,700);
HAL_UART_Transmit(&huart4,"jek",3,10);
HAL_Delay(1000);
HAL_UART_Transmit(&huart4,"ifl",3,10);
move_distance(100,700);
HAL_UART_Transmit(&huart4,"j",1,10);
Robot_Locate(0,0,700);
orientate(0,150);
}*/
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
