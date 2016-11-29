/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

#include "nrf24l01_lib.h"

/* CSN Pin*/
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PORT			GPIOC
#define NRF24L01_CSN_PIN			GPIO_PIN_4
#endif

/* CE Pin */
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PORT			GPIOB
#define NRF24L01_CE_PIN				GPIO_PIN_0
#endif

/* Configurazioni dei Pin CE e CSN */
#define NRF24L01_CE_LOW				HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH			HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)
#define NRF24L01_CSN_LOW			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET)
#define NRF24L01_CSN_HIGH			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Esprime lo stato della trasmissione delle variabili */

typedef enum _TM_NRF24L01_Transmit_Status_t {
	TM_NRF24L01_Transmit_Status_Lost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
	TM_NRF24L01_Transmit_Status_Ok = 0x01,     /*!< Message sent successfully */
	TM_NRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} TM_NRF24L01_Transmit_Status_t;

typedef union _TM_NRF24L01_IRQ_t {
	struct {
		uint8_t reserved0:4;
		uint8_t MaxRT:1;     /*!< Set to 1 if MAX retransmissions flag is set */
		uint8_t DataSent:1;  /*!< Set to 1 if last transmission is OK */
		uint8_t DataReady:1; /*!< Set to 1 if data are ready to be read */
		uint8_t reserved1:1;
	} F;
	uint8_t Status;          /*!< NRF status register value */
} TM_NRF24L01_IRQ_t;

/* Datarate di trasmissione possibili */

typedef enum _TM_NRF24L01_DataRate_t {
	TM_NRF24L01_DataRate_2M = 0x00, /*!< Data rate set to 2Mbps */
	TM_NRF24L01_DataRate_1M,        /*!< Data rate set to 1Mbps */
	TM_NRF24L01_DataRate_250k       /*!< Data rate set to 250kbps */
} TM_NRF24L01_DataRate_t;

/* Potenza del segnale del TRX */

typedef enum _TM_NRF24L01_OutputPower_t {
	TM_NRF24L01_OutputPower_M18dBm = 0x00,/*!< Output power set to -18dBm */
	TM_NRF24L01_OutputPower_M12dBm,       /*!< Output power set to -12dBm */
	TM_NRF24L01_OutputPower_M6dBm,        /*!< Output power set to -6dBm */
	TM_NRF24L01_OutputPower_0dBm          /*!< Output power set to 0dBm */
} TM_NRF24L01_OutputPower_t;

/* Struct delle caratteristiche del collegamento */

typedef struct {
	uint8_t PayloadSize;				//Payload size
	uint8_t Channel;					//Channel selected
	TM_NRF24L01_OutputPower_t OutPwr;	//Output power
	TM_NRF24L01_DataRate_t DataRate;	//Data rate
} TM_NRF24L01_t;

static TM_NRF24L01_t TM_NRF24L01_Struct;

/* Indirizzo della NUCLEO F3*/

uint8_t MyAddress[] = {
	0xAA,
	0x00,
	0x00,
	0x00,
	0x00
};

/* Indirizzo della SENSORBOARD F0 */

uint8_t TxAddress[] = {
	0xAA,
	0x00,
	0x00,
	0x00,
	0x01
};

/* Istanze delle struct dichiarate precedentemente */

TM_NRF24L01_Transmit_Status_t transmissionStatus;
TM_NRF24L01_IRQ_t NRF_IRQ;


/* Array dei dati da trasmettere */
uint8_t comando1[6] = {0x01, 0x01};              //GET TIM
uint8_t comando2[6] = {0x01, 0x02};              //GET INC
uint8_t comando3[6] = {0x03, 0x03, 0x01};        //SET LED val = 0xFFFF 
uint8_t comando4[6] = {0x03, 0x03, 0x02};        //SET LED val = 0x0000
int flag = 0;           /* Permette di eseguire le trasmissioni dei comandi*/
int led_flag = 1;       /* Indica se bisogna settare i LED come stato alto (=1) o basso (=0) */

/* Array dei dati da ricevere */
uint8_t dataIn[6];

int usart1_flag = 0;             /* flag usato per individuare eventuali dati ricevuti su usart1 */
uint8_t rx_buff[200];           /*buffer di ricezione per la UART1*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* Funzioni di supporto alla comunicazione con TRX wireless */

void TM_NRF24L01_SetChannel(uint8_t);
uint8_t TM_NRF24L01_Init(uint8_t, uint8_t);
void TM_NRF24L01_SoftwareReset(void);
void TM_NRF24L01_WriteRegister(uint8_t reg, uint8_t valore);
void TM_NRF24L01_Clear_Interrupts(void);
void TM_NRF24L01_PowerUpRx(void);
void TM_NRF24L01_WriteRegisterMulti(uint8_t, uint8_t*, uint8_t);
void TM_NRF24L01_SetMyAddress(uint8_t*);
void TM_NRF24L01_SetTxAddress(uint8_t*);
void TM_NRF24L01_PowerUpTx(void);
void TM_NRF24L01_Transmit();
TM_NRF24L01_Transmit_Status_t TM_NRF24L01_GetTransmissionStatus(void);
uint8_t TM_NRF24L01_GetStatus(void);
uint8_t TM_NRF24L01_ReadRegister(uint8_t reg);
void TM_NRF24L01_GetData(uint8_t*);
uint8_t TM_NRF24L01_DataReady(void);
uint8_t TM_NRF24L01_RxFifoEmpty(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  uint8_t printed = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  
  HAL_UART_Receive_IT(&huart1, rx_buff, sizeof(rx_buff));
    
  /* Inizializzazione dei registri */
  TM_NRF24L01_Init(0x050, 0x06);
  
  /* Inizializzazione indirizzo cui trasmettere */
  TM_NRF24L01_SetTxAddress(TxAddress);
  /* Inizializzazione indirizzo del trasmettitore */
  TM_NRF24L01_SetMyAddress(MyAddress);
  HAL_Delay(500);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    
    if(usart1_flag == 1){
      HAL_UART_Transmit(&huart2, rx_buff, sizeof(rx_buff), 100);
    }

    /* Transmit data, goes automatically to TX mode */
    //dataOut[1] = 0x00;
    
    switch (flag){
    case 0:
      TM_NRF24L01_Transmit(comando1);           //GET TIM
      flag++;
      HAL_Delay(500);
      break;
    
    case 1:                                     //GET INC
      TM_NRF24L01_Transmit(comando2);
      flag++;
      HAL_Delay(500);
      break;
      
    case 2:                                     
      if(led_flag == 1){
        TM_NRF24L01_Transmit(comando3);         //SET LED 1
        led_flag = 0;
      } else if(led_flag == 0){
        TM_NRF24L01_Transmit(comando4);         //SET LED 2
        led_flag = 1;
      }
      flag=0;
      HAL_Delay(1000);
      break;
    }

    do {
      /* Restituisce lo stato della trasmissione */
      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
    } while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
        
    /* Il TRX torna nello stato RX */
    TM_NRF24L01_PowerUpRx();
    
    /* Si attende la ricezione dei dati dalla SENSORBOARD F0 */
    while (!TM_NRF24L01_DataReady());
    
    /* Ottieni i dati trasmessi dalla SENSORBOARD F0*/
    TM_NRF24L01_GetData(dataIn);
    /* Si stampano i dati ricevuti sulla USART2*/
    HAL_UART_Transmit(&huart2, dataIn, sizeof(dataIn), 100);
    
    /* Stato della comunicazione non stampato sulla USART1 */
    printed = 0;

    /* Controlla se lo stato è cambiato */
    if (transmissionStatus != TM_NRF24L01_Transmit_Status_Sending && 
	!printed)                                                    
	{
          /* Transmissione OK */
          if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) {
            uint8_t service_txt[] = "\nMessaggio consegnato!\n";
            //HAL_UART_Transmit(&huart2, service_txt, sizeof(service_txt), 100);
          }
          
          /* Message LOST */
          if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) {
            uint8_t service_txt2[] = "\nMessaggio perso!\n";
            //HAL_UART_Transmit(&huart2, service_txt, sizeof(service_txt), 100);
          }
          
          /* Set del flag Stampato */
          printed = 1;
	}
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCN_Pin */
  GPIO_InitStruct.Pin = SCN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCN_GPIO_Port, SCN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/*Rx_IT Callback*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  /*Flush della Uart*/
  usart1_flag = 1;
  __HAL_UART_FLUSH_DRREGISTER(huart);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF); 

  HAL_UART_Receive_IT(&huart1, rx_buff, sizeof(rx_buff));
}

void TM_NRF24L01_WriteBit(uint8_t reg, uint8_t bit, uint8_t value) {
	uint8_t tmp;
	/* Read register */
	tmp = TM_NRF24L01_ReadRegister(reg);
	/* Make operation */
	if (value) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}
	/* Write back */
	TM_NRF24L01_WriteRegister(reg, tmp);
}

uint8_t TM_NRF24L01_ReadBit(uint8_t reg, uint8_t bit) {
	uint8_t tmp;
	tmp = TM_NRF24L01_ReadRegister(reg);
	if (!NRF24L01_CHECK_BIT(tmp, bit)) {
		return 0;
	}
	return 1;
}

uint8_t TM_NRF24L01_ReadRegister(uint8_t reg) {
	reg = NRF24L01_READ_REGISTER_MASK(reg); //ha i riferimenti
        uint8_t nop = NRF24L01_NOP_MASK;
        uint8_t value;
	NRF24L01_CSN_LOW;
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK(reg));
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10); 
	//value = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
         HAL_SPI_TransmitReceive(&hspi1, &nop, &value, 1 ,10);
	NRF24L01_CSN_HIGH;
	
	return value;
}

void TM_NRF24L01_WriteRegister(uint8_t reg, uint8_t valore){
    uint8_t comando =  (0x20|(0x1F & reg));    
    //CSN low
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    NRF24L01_CSN_LOW;
    HAL_SPI_Transmit(&hspi1,&comando,1,10);
    HAL_SPI_Transmit(&hspi1,&valore,1,10);
    //CSN high
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    NRF24L01_CSN_HIGH;
 
}

void TM_NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t *vett, uint8_t dim){
    uint8_t comando =  (0x20|(0x1F & reg));     
    //CSN low
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    NRF24L01_CSN_LOW;  
    HAL_SPI_Transmit(&hspi1, &comando ,1, 10);               
    HAL_SPI_Transmit(&hspi1, vett, 5, 10);
    //CSN high
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    NRF24L01_CSN_HIGH;
}

uint8_t TM_NRF24L01_GetStatus(void) {
	uint8_t status;
	uint8_t nop=NRF24L01_NOP_MASK;
	NRF24L01_CSN_LOW;
	/* First received byte is always status register */
	HAL_SPI_TransmitReceive(&hspi1,&nop,&status,1,10);
	/* Pull up chip select */
	NRF24L01_CSN_HIGH;
	
	return status;
}

TM_NRF24L01_Transmit_Status_t TM_NRF24L01_GetTransmissionStatus(void) {
	uint8_t status = TM_NRF24L01_GetStatus();
	if (NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS)) {
		/* Successfully sent */
		return TM_NRF24L01_Transmit_Status_Ok;
	} else if (NRF24L01_CHECK_BIT(status, NRF24L01_MAX_RT)) {
		/* Message lost */
		return TM_NRF24L01_Transmit_Status_Lost;
	}
	
	/* Still sending */
	return TM_NRF24L01_Transmit_Status_Sending;
}

void TM_NRF24L01_SetMyAddress(uint8_t *adr) {
	NRF24L01_CE_LOW;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, adr, 5);      
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_SetTxAddress(uint8_t *adr) {
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, adr, 5);        //La pipeline P0 viene usata per la trasmissione dell'ACK (vedi pag 72 nrf24l01+.pdf
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, adr, 5);
}

void TM_NRF24L01_PowerUpRx(void) {
	/* Disable RX/TX mode */
	NRF24L01_CE_LOW;
	/* Clear RX buffer */
        NRF24L01_FLUSH_RX;
	/* Clear interrupts */
	TM_NRF24L01_Clear_Interrupts();
	/* Setup RX mode */
	//TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, 0x3F);
        /* Start listening */
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_Transmit(uint8_t * data) {
  /* Chip enable put to low, disable it */
  NRF24L01_CE_LOW;
  
  /* Entra in modalità Trasmissione */	
  /*TM_NRF24L01_PowerUpTxPowerUpTx()*/
  
  TM_NRF24L01_Clear_Interrupts();
  TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, 0x0E);
  
  /* Clear TX FIFO from NRF24L01+ */

  NRF24L01_FLUSH_TX;

  /* Send payload to nRF24L01+ */
  NRF24L01_CSN_LOW;
        
  /* Send write payload command */
  uint8_t payloadTxMask = NRF24L01_W_TX_PAYLOAD_MASK;
  HAL_SPI_Transmit(&hspi1, &payloadTxMask, 1, 100);
  
  /* Fill payload with data*/
  HAL_SPI_Transmit(&hspi1, data, TM_NRF24L01_Struct.PayloadSize, 200);
         
  /* Disable SPI */
  NRF24L01_CSN_HIGH;
	
  /* Send data! */
  NRF24L01_CE_HIGH;
}

void TM_NRF24L01_SetChannel(uint8_t channel) {
	if (channel <= 125 && channel != TM_NRF24L01_Struct.Channel) {
		/* Store new channel setting */
		TM_NRF24L01_Struct.Channel = channel;
		/* Write channel */
		TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, channel);
	}
}

uint8_t TM_NRF24L01_Init(uint8_t channel, uint8_t payload_size) {
        //questo
        NRF24L01_CSN_HIGH;
        NRF24L01_CE_LOW;
	/* Max payload is 32bytes */
	if (payload_size > 32) {
		payload_size = 32;
	}
	
	/* Fill structure */
	TM_NRF24L01_Struct.Channel = !channel; /* Set channel to some different value for TM_NRF24L01_SetChannel() function */
	TM_NRF24L01_Struct.PayloadSize = payload_size;
	TM_NRF24L01_Struct.OutPwr = TM_NRF24L01_OutputPower_0dBm;
	TM_NRF24L01_Struct.DataRate = TM_NRF24L01_DataRate_1M;

        /*Abilita l'Auto-ACK per la Pipe 0*/
        //TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_AA,0x01);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_AA,0x03);
        /* Abilita la pipe 0 per la ricezione */ 
        //TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 0x01);
        TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 0x03);
	/* Impostazione del canale */
	TM_NRF24L01_SetChannel(channel);
        
	/* Impostazioni del collegamento: 1Mbps, 0dBm, LNA OFF  */
        TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, 0x06);
        
        /* Impostazione della dimensione degli indirizzi dRX (5 byte) */
        TM_NRF24L01_WriteRegister(NRF24L01_REG_SETUP_AW, 0x03);
        
        /* Config register */
	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, 0x3F);
        
	/* Set pipeline to max possible 32 bytes */
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, TM_NRF24L01_Struct.PayloadSize); // Auto-ACK pipe
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, TM_NRF24L01_Struct.PayloadSize); // Data payload pipe
		
	/* Abilita il payload dinamico */
        //TM_NRF24L01_WriteRegister(NRF24L01_REG_FEATURE,0x05);
        TM_NRF24L01_WriteRegister(NRF24L01_REG_FEATURE,0x06);
        //TM_NRF24L01_WriteRegister(NRF24L01_REG_DYNPD,0x01); // Payload Dinamico
        TM_NRF24L01_WriteRegister(NRF24L01_REG_DYNPD,0x03); // Payload Dinamico
	
	/* Clear FIFOs */
	NRF24L01_FLUSH_TX;
        /*
        NRF24L01_CSN_LOW;
        HAL_SPI_Transmit(&hspi1, &flushTxMask, 1, 10); 
        NRF24L01_CSN_HIGH;
        */
        
	NRF24L01_FLUSH_RX;
        /*
        NRF24L01_CSN_LOW;
        HAL_SPI_Transmit(&hspi1, &flushRxMask, 1, 10); 
        NRF24L01_CSN_HIGH;
	*/
        
	/* Clear interrupts */
        TM_NRF24L01_Clear_Interrupts();
	
	/* Go to RX mode */
	TM_NRF24L01_PowerUpRx();
	
	/* Return OK */
	return 1;
}

void TM_NRF24L01_Clear_Interrupts(void) {
	TM_NRF24L01_WriteRegister(0x07, 0x70);
}

/* Funzioni necessarie per la ricezione */

void TM_NRF24L01_GetData(uint8_t* dataOut) {
        uint8_t payloadRxMask = NRF24L01_R_RX_PAYLOAD_MASK;
        
	/* Pull down chip select */
	NRF24L01_CSN_LOW;
	/* Send read payload command*/
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_R_RX_PAYLOAD_MASK);
	HAL_SPI_Transmit(&hspi1, &payloadRxMask, 1, 10);
	/* Read payload */
	//TM_SPI_SendMulti(NRF24L01_SPI, data, data, TM_NRF24L01_Struct.PayloadSize);
	HAL_SPI_TransmitReceive(&hspi1, dataOut, dataOut, TM_NRF24L01_Struct.PayloadSize, 10);
	/* Pull up chip select */
	NRF24L01_CSN_HIGH;
	
	/* Reset status register, clear RX_DR interrupt flag */
	TM_NRF24L01_WriteRegister(NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

uint8_t TM_NRF24L01_DataReady(void) {
	uint8_t status = TM_NRF24L01_GetStatus();
	
	if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR)) {
		return 1;
	}
	return !TM_NRF24L01_RxFifoEmpty();
}

uint8_t TM_NRF24L01_RxFifoEmpty(void) {
	uint8_t reg = TM_NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS);
	return NRF24L01_CHECK_BIT(reg, NRF24L01_RX_EMPTY);
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
