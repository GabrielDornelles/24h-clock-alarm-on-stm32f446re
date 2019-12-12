/* USER CODE BEGIN Header */



/*
Author: Gabriel Dornelles Monteiro
Turma :4423
Sistemas Microprocessados II
Professor: Raul Faviero de Mesquita
Trabalho: Relógio digital ajustável com carga
*/








/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
_Bool ok=0;
_Bool hajime=0;

int segundos=0;
int minutos=0;
int horas=0;
int minselect=0;
int horselect=0;
int minselectend=0;
int horselectend=0;
int minselected=1;
int hrselected=0;
int actualposition=0x80 - 1;
int Repique=0;
int clockstart=0;
int time_select=0;
int time_set=0;
int minstart=61;
int minend=61;
int hourstart=61;
int hourend=61;
int load_on=1;
int notchecking=0;
int horarioacertado=0;
int traderepeat=0;
int funciona=0;
int displayinfo=1;
int time_set_not=0;
char undyne1[]={ // era para ser um caractere especial pra escrever no display
		0b01110,
		0b01110,
		0b00100,
		0b01110,
		0b10101,
		0b00100,
		0b01010,
		0b01010 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
  * lcd.c
  *
  *  Created on: 18 de mar de 2019
  *      Author: Prof. Raul Faviero de Mesquita
  */

 /* Includes ------------------------------------------------------------------*/
 #include "stdio.h"
 #include "main.h"

 /*
  *  PINO D4 DO LCD -> Pino PB14
  *  PINO D5 DO LCD -> Pino PB15
  *  PINO D6 DO LCD -> Pino PB1
  *  PINO D7 DO LCD -> Pino PB2
  *
  *  PINO LCD_RS ->  Pino PB5
  *  PINO LCD_EN ->  Pino PB13
  *
  */
 void tempo_us(uint16_t t)
 {
 	for(uint16_t i=0; i<=t*15; i++)
 	{
 		__NOP();
 		__NOP();
 	}
 }

 //**** PULSO DE ENABLE  *****

 void enable() {
 	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,1);
 	tempo_us(4);
 	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);
 }

 //****  ESCRITA NO LCD EM 04 BITS  ******

 void lcd_write(unsigned char c) {
 	unsigned char d = (c >> 4) & 0x0F;
 	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (d & 0x01));
 	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (d & 0x02));
 	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (d & 0x04));
 	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (d & 0x08));
 	enable();

 	d = (c & 0x0F);
 	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (d & 0x01));
 	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (d & 0x02));
 	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (d & 0x04));
 	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (d & 0x08));
 	enable();

 	tempo_us(40);
 }

 //**** ESCRITA DE UMA STRING (NOME)  ****

 void lcd_puts(const char *s) {
 	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,1);
 	while (*s) {
 		lcd_write(*s++);
 	}
 }

 //**** ESCRITA DE UMA VARIÁVEL NUMÉRICA DECIMAL (Número)  ****

 void lcd_printd(uint16_t c){
 	char buf[10];
 	sprintf(buf,"%.2d",c); // 2 digitos, a unidade do numero fica no endereço de memoria seguinte ao indicado na função goto
 	lcd_puts(buf);
 }

 /*
 void lcd_printf(float c){
 	char buf[10];
 	sprintf(buf,"%f",c);
 	lcd_puts(buf);
 }
 */

 //*** ESCRITA DE UM CARACTERE  ****



 void lcd_putc(char c) {
 	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,1);
 	lcd_write(c);
 }

 //**** POSICIONAMENTO DO CURSOR  *****

 void lcd_goto(unsigned char pos) {
 	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_RS_Pin,0);
 	lcd_write(pos);
 }


/////////// Funções para caracter especial adaptadas do PIC, não são confiáveis no momento.
 void Lcd_SetBit(char data_bit) //Based on the Hex value Set the Bits of the Data Lines
 {
     if(data_bit& 1)
    	 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
     else
    	 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);

     if(data_bit& 2)
    	 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);
     else
    	 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);

     if(data_bit& 4)
    	 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET);
     else
    	 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);

     if(data_bit& 8)
    	 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET);
     else
    	 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET);
 }



 void Lcd_Print_Char(char data)  //Send 8-bits through 4-bit mode
 {
    char Lower_Nibble,Upper_Nibble;
    Lower_Nibble = data&0x0F;
    Upper_Nibble = data&0xF0;
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_RS_Pin,1);
    Lcd_SetBit(Upper_Nibble>>4);             //Send upper half by shifting by 4
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,1);
    for(int i=2130483; i<=0; i--){}
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);
    Lcd_SetBit(Lower_Nibble); //Send Lower half
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,1);
    for(int i=2130483; i<=0; i--){}
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);
 }


 void Lcd_Cmd(char a)
 { HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);

     Lcd_SetBit(a); //Incoming Hex value
     HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,1);
         HAL_Delay(4);
         HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);
 }

////////////////////////////////







 //*** LIMPEZA DA MEMÓRIA DDRAM (Tela) *****

 void lcd_clear() {
 	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_RS_Pin,0);
 	lcd_write(0x01);
 	HAL_Delay(2);
 }
 void lcd_init() {

 	HAL_Delay(15);
 	HAL_GPIO_WritePin(D4_GPIO_Port,
 			D4_Pin | D5_Pin | D6_Pin | D7_Pin | LCD_RS_Pin | LCD_EN_Pin,
 			GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin | D5_Pin, GPIO_PIN_SET);

 	enable();
 	HAL_Delay(5);

 	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin | D5_Pin, GPIO_PIN_SET);
 	enable();
 	tempo_us(100);

 	HAL_GPIO_WritePin(D4_GPIO_Port,
 			D4_Pin | D5_Pin | D6_Pin | D7_Pin | LCD_RS_Pin | LCD_EN_Pin,
 			GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
 	enable();
 	tempo_us(40);

 	lcd_write(0x28);
 	lcd_write(0x06);
 	lcd_write(0x0C);
 	lcd_clear();
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
  MX_TIM10_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();

  /* lcd_goto(0x80);
   lcd_puts("Kawashita");
   lcd_goto(0xc0);
   lcd_puts("yakusoku");
   HAL_Delay(2000);

   lcd_goto(0x80);
   lcd_puts("wasurenai");
   lcd_goto(0xc0);
   lcd_puts("yo me       ");
   HAL_Delay(2000);

   lcd_goto(0x80);
   lcd_puts("wo toji    ");
   lcd_goto(0xc0);
   lcd_puts("tashikameru       ");
   HAL_Delay(2000);

   lcd_goto(0x80);
   lcd_puts("Oshiyoseta");
   lcd_goto(0xc0);
   lcd_puts("yami       ");
   HAL_Delay(2000);

   lcd_goto(0x80);
   lcd_puts("Furiharatte");
   lcd_goto(0xc0);
   lcd_puts("susumu yo      ");*/

  //lcd_goto(0xc4);
  // lcd_puts("(~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~ (~=w=)~");

   //HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, RESET);





  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

			if(time_set==1)
			{
				minstart=minselect;
				minend=minselectend;

				hourstart=horselect;
				hourend=horselectend;
			}
			if(time_set_not==1)
			{
				minstart=0;
minend=0;
hourstart=0;
hourend=0;
			}




     	 if(Repique==1)
    		 {
    	  HAL_Delay(100);
    	  lcd_write(0x0D);// pisca na posição da variavel actualposition, que é chamada sempre ao fim da interrupção do timer, posição real, visivel ao usuario
          if(horarioacertado==1)
          {
        	  lcd_write(0x0C); //desliga o cursor após ter acertado todos os horarios
          }

    	  HAL_Delay(100);
    	  Repique=0;
    		 }else __NOP();


     	 if(minutos==minstart && horas==hourstart) //liga carga
     	 {
    	 notchecking=1;
    	 HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, SET);
    	 lcd_goto(0x8d);
    	 HAL_Delay(10);
    	 lcd_puts("ON ");

         }

      if(minutos == minend && horas==hourend) //desliga carga
      {
    	  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, RESET);
    	  load_on=1;

    	  if(traderepeat==1) //1 é para não repetir no dia seguinte
    	      	  {
    	      		  displayinfo=0;
    	      		  time_set_not=1;
    	      	  }

    	  if(traderepeat==0) //0 é para repetir no dia seguinte
    	  {
    		  displayinfo=1;
    	  }

    	  //funciona=1;
    	  lcd_goto(0x8d);
    	  HAL_Delay(10);
    	  lcd_puts("OFF");
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 41999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 41199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_Pin|D7_Pin|LCD_EN_Pin|D4_Pin 
                          |D5_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INT0_Pin INT1_Pin */
  GPIO_InitStruct.Pin = INT0_Pin|INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D7_Pin LCD_EN_Pin D4_Pin 
                           D5_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D7_Pin|LCD_EN_Pin|D4_Pin 
                          |D5_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(Repique==0)
	{

		if( GPIO_Pin == INT1_Pin) // pc1 como interrupção
		{
			if(actualposition==0x80)
			{
				horas+=10;
			}
			else if(actualposition==0x81)
			{
				horas++;
			}
			else if(actualposition==0x83)
			{
				minutos+=10;
			}

			else if(actualposition==0x84)
			{
				minutos++;
			}
			else if(actualposition==0xc2)
			{
				horselect+=10;
			}
			else if(actualposition==0xc3)
			{
				 horselect++;
			}
			else if(actualposition==0xc5)
			{
				minselect+=10;
			}
			else if(actualposition==0xc6)
			{
				minselect++;
			}
			else if(actualposition==0xcb)
			{
				horselectend+=10;
			}
			else if(actualposition==0xcc)
			{
				 horselectend++;
			}
			else if(actualposition==0xce)
			{
				minselectend+=10;
			}
			else if(actualposition==0xcf)
			{
				minselectend++;
			}
			else if(actualposition==0x8a)
			{
				traderepeat^=1;
			}


		}


				if( GPIO_Pin == INT0_Pin) //pc0 como interrupção
				{
				 if(actualposition==0x89+0x1)
			     {
					 clockstart=1;
					 time_select=1;
					 actualposition = 0xc1;
				}

				if(actualposition==0x81)
				{
					actualposition+=1;
				}
				if(actualposition==0x84)
				{
					actualposition=0x8a;
				}

				else if(actualposition<0x85)
					actualposition+=0x1;




				if(time_select==1 ) // seleção do horario de ligar/desligar carga
				{

					actualposition+=0x01;
					if(actualposition==0xc4)
					{
					actualposition+=0x01;
					}
				}

				if(actualposition==0xc7)
				{
					actualposition=0xcb;
				}

				if(actualposition==0xcf-0x2)
				{
					actualposition=0xcf-0x1;
				}

				if(actualposition==0xcf+0x1)
				{
					horarioacertado=1;
					time_set=1;
				}







			}
			Repique=1;
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1)
{
	/*if(hajime==1)
	{
		ok=1;
		HAL_TIM_Base_Stop_IT(&htim1);
	}
	else


		hajime=1;*/

if(clockstart==1)
{
	segundos++;
}

if(horas>23)
{
	horas=0;
}

if(segundos>=60)
{
	segundos=0;
	minutos++;
}

if(minutos>=60)
{
	minutos=0;
	horas++;
}

if(horselectend>23)
{
	horselectend=0;
}

if(horselect>23)
{
	horselect=0;
}

if(minselect>=60)
{
	minselect=0;
}

if(minselectend>=60)
{
	minselectend=0;
}

          //Posições e escrita do relógio no display
		  lcd_goto(0x80);
		  lcd_printd(horas);
		  lcd_goto(0x82);
		  lcd_puts(":");
		  lcd_goto(0x83);
		  lcd_printd(minutos);
		  lcd_goto(0x85);
		  lcd_puts(":");
		  lcd_goto(0x86);
	      lcd_printd(segundos);
	      lcd_goto(0x89);
	      lcd_printd(traderepeat);


    //Posições e escrita dos tempos de liga e desliga da carga no display

if(displayinfo==1)
{
		    lcd_goto(0xc0);
			lcd_puts("L:");
			lcd_goto(0xc0 + 0x2);
			lcd_printd(horselect);
			lcd_goto(0xc0 + 0x4);
			lcd_puts(":");
			lcd_goto(0xc0 + 0x5);
			lcd_printd(minselect);
			lcd_goto(0xc9);
			lcd_puts("D:");
			lcd_goto(0xc8);
			lcd_puts(" ");
			lcd_goto(0xc9+ 0x2);
			lcd_printd(horselectend);
			lcd_goto(0xc9+0x4);
			lcd_puts(":");
			lcd_goto(0xc9+0x5);
			lcd_printd(minselectend);
}

else if(displayinfo==0)
{
	lcd_goto(0xc0);
	lcd_puts("                 ");
}

    if(notchecking==0)
    {
    	  lcd_goto(actualposition);
    }


	//HAL_TIM_Base_Stop_IT(&htim1)
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
