/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * The 7 bit slave address of the MPU6050 is b110100X, with X depending on AD0 (allows two MPU60X0s to be connected to the same I2C bus).
 * HAL needs 8 bit address - b1101000 << 1 == 0xD0.
*/
#define MPU6050_ADDR 0xD0
/* MPU6050 registers. */
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_READ 0x3B
#define MPU6050_GYRO_READ 0x43
#define MPU6050_SELF_TEST 0x0D
#define CALIBRATION_SAMPLES 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

int16_t Accel_X_offset = 0;
int16_t Accel_Y_offset = 0;
int16_t Accel_Z_offset = 0;

int16_t Gyro_X_offset = 0;
int16_t Gyro_Y_offset = 0;
int16_t Gyro_Z_offset = 0;

float Ax;
float Ay;
float Az;
float Gx;
float Gy;
float Gz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*Redirecting printf to SWO, needs syscall.*/
int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return(ch);
}

void MPU6050_Init(void)
{
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 1000);
	if(status == HAL_OK)
	{
	    printf("I2C device is ready.\n");
	}
	else
	{
	    printf("Error connecting to the I2C device.\n");
	}
	uint8_t check;
	/*
	 * This register is used to verify the identity of the device.
	 * The default value of the register for MPU6050 is 0x68.
	 */
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 1000);
	if (check == 0x68)
	{
	    printf("MPU6050 is ready!\n");
	    uint8_t Data;
	    /*
	     * PWR_MGMT_1: Wakes up MPU6050. Turns off temperature sensor.
	     * SMPRT_DIV: Specifies the divider from the gyroscope output rate used to generate the Sample Rate.
	     * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). -> 8KHZ / (1 +7) = 1KHz
	     * ACCEL_CONFIG: Sets full scale range, triggers self test.
	     * Accelometer range set to: 2g, sensibility: 16384.
	     * GYRO_CONFIG: Sets full scale range, triggers self test.
	     * Gyroscope range set to: 250 °/s, sensibility 16384.
	     */
	    Data = 0b00001000;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &Data, 1, 1000);
	    Data = 0x0;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, 1, &Data, 1, 1000);
	    Data = 0x10;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPRT_DIV, 1, &Data, 1,  1000);
	    Data = 0x0;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &Data, 1, 1000);
	    Data = 0x0;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &Data, 1, 1000);
	    HAL_Delay(300);
	}
}

void MPU6050_Reset(void) {
    uint8_t reset_command = 0x80;  // Wartość resetująca urządzenie
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &reset_command, 1, 1000);
    HAL_Delay(100);  // Czekamy chwilę na zakończenie resetu
}

void MPU6050_Calibrate(void)
{
    int32_t Accel_X_sum = 0;
    int32_t Accel_Y_sum = 0;
    int32_t Accel_Z_sum = 0;

    int32_t Gyro_X_sum = 0;
    int32_t Gyro_Y_sum = 0;
    int32_t Gyro_Z_sum = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Odczytujemy dane akcelerometru i żyroskopu
        MPU6050_Read_Accel();
        MPU6050_Read_Gyro();

        // Sumujemy wartości
        Accel_X_sum += Accel_X_RAW;
        Accel_Y_sum += Accel_Y_RAW;
        Accel_Z_sum += Accel_Z_RAW;

        Gyro_X_sum += Gyro_X_RAW;
        Gyro_Y_sum += Gyro_Y_RAW;
        Gyro_Z_sum += Gyro_Z_RAW;

        // Czekamy chwilę, aby zebrać dane
        HAL_Delay(10);
    }

    // Obliczamy średnie wartości
    Accel_X_offset = Accel_X_sum / CALIBRATION_SAMPLES;
    Accel_Y_offset = Accel_Y_sum / CALIBRATION_SAMPLES;
    Accel_Z_offset = Accel_Z_sum / CALIBRATION_SAMPLES;

    Gyro_X_offset = Gyro_X_sum / CALIBRATION_SAMPLES;
    Gyro_Y_offset = Gyro_Y_sum / CALIBRATION_SAMPLES;
    Gyro_Z_offset = Gyro_Z_sum / CALIBRATION_SAMPLES;

    printf("Calibration Complete!\n");
    printf("Accel Offsets: X = %d, Y = %d, Z = %d\n", Accel_X_offset, Accel_Y_offset, Accel_Z_offset);
    printf("Gyro Offsets: X = %d, Y = %d, Z = %d\n", Gyro_X_offset, Gyro_Y_offset, Gyro_Z_offset);
}

void MPU6050_Read_Accel(void)
{
	/*
	 * Read from 6 registers all the Accel values.
	 */
	uint8_t RecData[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_READ, 1, RecData, 6, 1000);

	/*
	 * Raw value is 16 bit - merge every two registers.
	 */
	 Accel_X_RAW = (int16_t)(RecData[0] << 8 | RecData[1]) - Accel_X_offset;
	 Accel_Y_RAW = (int16_t)(RecData[2] << 8 | RecData[3]) - Accel_Y_offset;
	 Accel_Z_RAW = (int16_t)(RecData[4] << 8 | RecData[5]) - Accel_Z_offset;

	/*
	 * Normalization: raw data to degrees per second.
	 * Analog data -> digital data.
	 * LSB Sensitivity defines physical value corresponds to one unit in the digital scale. (?)
	 * Divide by the set sensibility for chosen range.
	 */
	float AccelSensitivity = 16384.0;
	Ax = (float)Accel_X_RAW/AccelSensitivity;
	Ay = (float)Accel_Y_RAW/AccelSensitivity;
	Az = (float)Accel_Z_RAW/AccelSensitivity;
}

void MPU6050_Read_Gyro(void)
{
	/*
  	 * Read from 6 registers all the Gyro values.
  	 */
	uint8_t RecData[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, MPU6050_GYRO_READ, 1, RecData, 6, 1000);

	/*
	* Raw value is 16 bit - merge every two registers.
	*/
	Gyro_X_RAW = (int16_t)(RecData[0] << 8 | RecData[1]) - Gyro_X_offset;
	Gyro_Y_RAW = (int16_t)(RecData[2] << 8 | RecData[3]) - Gyro_Y_offset;
	Gyro_Z_RAW = (int16_t)(RecData[4] << 8 | RecData[5]) - Gyro_Z_offset;;

	/*
	 * Normalization: raw data to degrees per second.
	 * Divide by the set sensibility for chosen range.
	 */
	float GyroSensitivity = 131.0;
	Gx = (float)Gyro_X_RAW/GyroSensitivity;
	Gy = (float)Gyro_Y_RAW/GyroSensitivity;
	Gz = (float)Gyro_Z_RAW/GyroSensitivity;
}

void MPU6050_Self_Test(void)
{
	/*
	 * Self-test enable for X, Y and Z.
	 * When performing accelerometer self test, the full-scale range should be set to ±8g
	 * Accelerator: full range: ± 8g -> sensitivity: 16384 LSB/g.
	 * When performing self test for the gyroscope, the full-scale range should be set to ±250dps.
	 * Gyroscope: full range: ± 250 °/s -> sensitivity: 131 LSB/°/s.
	 */

	/*
	 * Sets needed range and enables self-test.
	 */
	uint8_t Data = 0b11110000;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &Data, 1, 1000);
	Data = 0b11100000;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &Data, 1, 1000);
	/* Wait for config update. (?)*/
	HAL_Delay(300);

	/* Reading raw data from SELF_TEST registers. */
	uint8_t RawData[4];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_SELF_TEST, 1, RawData, 4, 1000);

	uint8_t SelfTestData[6];
	/* Accelerometer Self Test Data   0b000yyxxx*/
	SelfTestData[0] = ((RawData[3] &  0b00110000) >> 1) | (RawData[0] >> 5);
	SelfTestData[1] = ((RawData[3] &  0b00001100) << 1) | (RawData[1] >> 5);
	SelfTestData[2] = ((RawData[3] &  0b00000011) << 3) | (RawData[2] >> 5);

	/* Gyroscope Self Test Data */
	SelfTestData[3] = RawData[0] & 0b00011111;
	SelfTestData[4] = RawData[1] & 0b00011111;
	SelfTestData[5] = RawData[2] & 0b00011111;

	float FactoryTrim[6];
	/* The factory trim value for accelerometer. */
	FactoryTrim[0] = 4096.0 * 0.34 * pow((0.92/0.34), (((float)SelfTestData[0] - 1))/(pow(2,5)-2));
	FactoryTrim[1] = 4096.0 * 0.34 * pow((0.92/0.34), (((float)SelfTestData[1] - 1))/(pow(2,5)-2));
	FactoryTrim[2] = 4096.0 * 0.34 * pow((0.92/0.34), (((float)SelfTestData[2] - 1))/(pow(2,5)-2));

	/* The factory trim value for gyroscope. */
	FactoryTrim[3] = 25.0 * 131.0 * pow(1.046, (float)SelfTestData[3]-1);
	FactoryTrim[4] = -25.0 * 131.0 * pow(1.046, (float)SelfTestData[4]-1);
	FactoryTrim[5] = 25.0 * 131.0 * pow(1.046, (float)SelfTestData[5]-1);

	float Results[6];

	char Labels[6][3] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};

	for(int i = 0; i < 6; i++)
	{
	     Results[i] = ((float)SelfTestData[i] - FactoryTrim[i]) / FactoryTrim[i];
	     /*
	      * Self-test response must be within the specified limits to pass.
	      * The limit is: 14%, for both accelerometer and gyroscope.
	      */
	     printf("%s: %.2f, test passed: %s\n", Labels[i], Results[i], fabs(Results[i]) < 14.0 ? "true" : "false");
	}

}

int Read_Button(void)
{
	int buttonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	return buttonState;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  MPU6050_Calibrate();
  /*MPU6050_Self_Test();*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */
      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();
      printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", Ax, Ay, Az, Gx, Gy, Gz);
      HAL_Delay(10);
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

  /** Initializes the CPU, AHB and APB buses clocks
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
