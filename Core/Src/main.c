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
#include "stdio.h"
#include "string.h"
#include "i2c-lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LIS302SDL_WHO_AM_I           0x0F    // Device ID
#define LIS302SDL_CTRL_REG1          0x20    // Main settings
#define LIS302SDL_CTRL_REG2          0x21    // Filter settings
#define LIS302SDL_CTRL_REG3          0x22    // Interrupt settings
#define LIS302SDL_HP_FILTER_RESET    0x23    // Reset filter
#define LIS302SDL_STATUS_REG         0x27    // Status info
#define LIS302SDL_OUT_X              0x29    // X-axis data
#define LIS302SDL_OUT_Y              0x2B    // Y-axis data
#define LIS302SDL_OUT_Z              0x2D    // Z-axis data
#define LIS302SDL_FF_WU_CFG_1        0x30    // FF/WU config 1
#define LIS302SDL_FF_WU_SRC_1        0x31    // FF/WU status 1
#define LIS302SDL_FF_WU_THS_1        0x32    // FF/WU threshold 1
#define LIS302SDL_FF_WU_DURATION_1   0x33    // FF/WU duration 1
#define LIS302SDL_FF_WU_CFG_2        0x34    // FF/WU config 2
#define LIS302SDL_FF_WU_SRC_2        0x35    // FF/WU status 2
#define LIS302SDL_FF_WU_THS_2        0x36    // FF/WU threshold 2
#define LIS302SDL_FF_WU_DURATION_2   0x37    // FF/WU duration 2
#define LIS302SDL_CLICK_CFG          0x38    // Click config
#define LIS302SDL_CLICK_SRC          0x39    // Click status
#define LIS302SDL_CLICK_THSY_X       0x3B    // Y/X click threshold
#define LIS302SDL_CLICK_THSZ         0x3C    // Z click threshold
#define LIS302SDL_CLICK_TIMELIMIT    0x3D    // Click time limit
#define LIS302SDL_CLICK_LATENCY      0x3E    // Click latency
#define LIS302SDL_CLICK_WINDOW       0x3F    // Double-click window

// Macros for controlling GPIOE Pin 3 (CS pin)
#define GPIOE_CS_LOW()      (GPIOE->ODR &= ~(1 << 3))	// CS LOW (select device)
#define GPIOE_CS_HIGH()     (GPIOE->ODR |= (1 << 3))	// CS HIGH (deselect device)

#define Orange_LED_Pin       GPIO_PIN_13   // Orange LED pin
#define Orange_LED_GPIO_Port GPIOD         // Orange LED port
#define Red_LED_Pin          GPIO_PIN_14   // Red LED pin
#define Red_LED_GPIO_Port    GPIOD         // Red LED port
#define Blue_LED_Pin         GPIO_PIN_15   // Blue LED pin
#define Blue_LED_GPIO_Port   GPIOD         // Blue LED port

#define LCD_ID 0x4E 						// device I2C address

//=====================================================
void LIS302SDL_write_RX(uint8_t reg_addr, uint8_t data);
void LIS302SDL_read_RX(uint8_t reg_addr, uint8_t *data);
uint16_t SPI_RX_FUNC(uint8_t reg_addr);
uint16_t SPI_TX_FUNC(uint8_t TX_val);
void LIS302SDL_read_axis();

void mdelay(uint32_t delayvalue);
//=====================================================

uint16_t TX_data;
uint16_t RX_data;
uint16_t x_axis, y_axis, z_axis;
int16_t x_val, y_val, z_val;
int8_t row = 0;
int8_t col = 0;

char disp[32] = { };

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// function to write printf
int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

// transmit data via SPI and receive response
uint16_t SPI_TX_FUNC(uint8_t data) {
	while (SPI1->SR & (1 << 7))
		;		// wait until SPI is not busy

	// load data into SPI transmit buffer
	while (!(SPI1->SR & (1 << 1)))
		;	//wait till transmit buffer is empty
	SPI1->DR = data;

	// wait for data reception
	while (!(SPI1->SR & (1 << 0)))
		;
	TX_data = SPI1->DR;

	return TX_data;
}

// read data from a specified register over SPI
uint16_t SPI_RX_FUNC(uint8_t reg_addr) {

	GPIOE_CS_LOW();

	reg_addr |= 0x80;			// set MSB for read operation (MSB=1)

	SPI_TX_FUNC(reg_addr);
	RX_data = SPI_TX_FUNC(0);

	GPIOE_CS_HIGH();

	return RX_data;
}

// write data to a specified register over SPI
void LIS302SDL_write(uint8_t reg_addr, uint8_t data) {

	GPIOE_CS_LOW();

	SPI_TX_FUNC(reg_addr);          // send register address
	SPI_TX_FUNC(data);              // send data to register

	GPIOE_CS_HIGH();
}

// read acceleration data for X, Y, and Z axes
void LIS302SDL_read_axis() {
	x_axis = SPI_RX_FUNC(LIS302SDL_OUT_X);
	y_axis = SPI_RX_FUNC(LIS302SDL_OUT_Y);
	z_axis = SPI_RX_FUNC(LIS302SDL_OUT_Z);
}

// convert raw data to calibrated acceleration value
int16_t Calibration(uint16_t val) {
	// check if val is negative (if MSB=1 then negative)
	if ((val & 0x80) == 0x80) {
		// 2's compliment
		val = ~val;
		val += 1;

		val &= 0x00FF; // mask to keep the last 8bits (output data registers are 8bit)

		val = (val * 2300) / 127;//measurement range: ±2.3g (between ±2g and ±8g) (*1000 for convenience)
								 // devide by 127 to normalize the outpout (8bit range: -127 to -0 and +0 to +127)
		return (-1 * val);
	} else
		return ((val * 2300) / 127);
}

// Simple delay function
void mdelay(uint32_t delayvalue) {
	uint32_t i;
	for (i = 0; i < delayvalue * 4000; ++i) {
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	lcd_init();
	lcd_put_cur(0, 0);
	lcd_send_string("LiS302DL_low_lvl");
	lcd_put_cur(1, 0);
	lcd_send_string("By: Najd Elaoud");
	mdelay(1000);
	lcd_clear();

	// initialize acceleromter
	LIS302SDL_write(LIS302SDL_CTRL_REG1, 0x47);	// data rate = 400Hz + (x,y,z) axis enabled
	GPIOE_CS_LOW();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		LIS302SDL_read_axis();
		x_val = Calibration(x_axis);
		y_val = Calibration(y_axis);
		z_val = Calibration(z_axis);

		lcd_put_cur(0, 0);
		sprintf(disp, "x=%d", x_val);
		lcd_send_string(disp);
		lcd_put_cur(0, 9);
		sprintf(disp, "y=%d", y_val);
		lcd_send_string(disp);
		lcd_put_cur(1, 0);
		sprintf(disp, "z=%d", z_val);
		lcd_send_string(disp);
		lcd_put_cur(1, 8);
		lcd_send_string("Dir:");
		mdelay(50);

		// toggle LEDs based on acceleration value
		if ((x_val != 0) && (y_val != 0)) {
			if (x_val > 90) {
				GPIOD->ODR |= (1 << 14);
				GPIOD->ODR &= ~((1 << 12) | (1 << 13) | (1 << 15));
				lcd_put_cur(1, 12);
				lcd_send_string("Left");
			}
			if (x_val < -90) {
				GPIOD->ODR |= (1 << 12);
				GPIOD->ODR &= ~((1 << 13) | (1 << 14) | (1 << 15));
				lcd_put_cur(1, 12);
				lcd_send_string("Right");
			}
			if (y_val > 90) {
				GPIOD->ODR |= (1 << 13);
				GPIOD->ODR &= ~((1 << 12) | (1 << 14) | (1 << 15));
				lcd_put_cur(1, 12);
				lcd_send_string("Up  ");
			}
			if (y_val < -90) {
				GPIOD->ODR |= (1 << 15);
				GPIOD->ODR &= ~((1 << 12) | (1 << 13) | (1 << 14));
				lcd_put_cur(1, 12);
				lcd_send_string("down");
			}
		} else {
			GPIOD->ODR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
		}

		mdelay(20);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 *//*
	 RCC->APB1ENR |= (1 << 21);		// Enable I2C1 clock

	 // set & reset I2C (!! I2C won't work without this)
	 I2C1->CR1 |= (1 << 15);
	 I2C1->CR1 &= ~(1 << 15);

	 I2C1->CR2 |= (0b001010 << 0);	// 10MHz peripheral clock
	 I2C1->CCR |= 0x50;
	 I2C1->TRISE |= (11 << 0);

	 I2C1->CR1 |= (1 << 0); 			// enable I2C1
	 */
	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */
	RCC->APB2ENR |= (1 << 12);             // Enable SPI1 clock

	SPI1->CR1 |= (1 << 2);                 // Set SPI1 as master
	SPI1->CR1 &= ~(1 << 15);   // Enable 2-line unidirectional mode (BIDIMODE=0)
	SPI1->CR1 &= ~(1 << 10);               // Enable full duplex (RXONLY=0)

	SPI1->CR1 &= ~(1 << 11);               // Set 8-bit data format (DFF=0)
	SPI1->CR1 |= ((1 << 8) | (1 << 9)); // Enable software slave management (SSI=1, SSM=1)

	SPI1->CR1 &= ~(0b111 << 3);            // Clear baud rate bits
	SPI1->CR1 |= (0b010 << 3);             // Set baud rate to fPCLK/8

	SPI1->CR1 &= ~(1 << 7);                // Set MSB first (LSBFIRST=0)
	SPI1->CR1 &= ~(1 << 0);                // Set to first clock edge (CPHA=0)
	SPI1->CR1 &= ~(1 << 1);           // Set clock polarity to idle low (CPOL=0)

	SPI1->CR1 |= (1 << 13);                // Enable CRC calculation (CRCEN=1)

	SPI1->CR2 &= ~(1 << 4);              // Set frame format to Motorola (FRF=0)

	// Enable SPI peripheral
	SPI1->CR1 |= (1 << 6);                 // Enable SPI1 (SPE=1)

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */
	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/

	/* USER CODE BEGIN SPI1_Init 2 */
	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	RCC->AHB1ENR |= (1 << 0);	// enable clock for Port A
	RCC->AHB1ENR |= (1 << 1);	// enable clock for Port B
	RCC->AHB1ENR |= (1 << 3);	// enable clock for Port D
	RCC->AHB1ENR |= (1 << 4);	// enable clock for Port E

	// CS PE3 as output
	GPIOE->MODER |= (1 << (3 << 1));

	// set PA5, PA6, PA7 as alternate function
	GPIOA->MODER |= ((2 << (5 << 1)) | (2 << (6 << 1)) | (2 << (7 << 1)));
	// select AF5 for PA5, PA6, PA7 (AFRL)
	GPIOA->AFR[0] |= (0b0101 << 20);	// PA5
	GPIOA->AFR[0] |= (0b0101 << 24);	// PA6
	GPIOA->AFR[0] |= (0b0101 << 28);	// PA7
	// set port speed (HIGH-speed)
	GPIOA->OSPEEDR |= ((2 << (5 << 1)) | (2 << (6 << 1)) | (2 << (7 << 1)));
	// set PA5, PA6, PA7 OUT mode as pull up
	GPIOA->PUPDR |= ((2 << (5 << 1)) | (2 << (6 << 1)) | (2 << (7 << 1)));

	// set LEDs as output
	GPIOD->MODER |= ((1 << (12 << 1)) | (1 << (13 << 1)) | (1 << (14 << 1))
			| (1 << (15 << 1)));

	// I2C setup
	// set PB6, PB7 as alternate function
	GPIOB->MODER |= ((2 << (6 << 1)) | (2 << (7 << 1)));
	// select AF4 for PB6, PB7 (AFRL)
	GPIOB->AFR[0] |= (0b0100 << 24);
	GPIOB->AFR[0] |= (0b0100 << 28);
	// set output as open drain
	GPIOB->OTYPER |= (1 << 6);
	GPIOB->OTYPER |= (1 << 7);
	// set port speed (HIGH-speed)
	GPIOB->OSPEEDR |= ((2 << (5 << 1)) | (2 << (6 << 1)) | (2 << (7 << 1)));

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
