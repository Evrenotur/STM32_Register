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
#include "canard.h"
#include <stdio.h>
#include <string.h>
CanardInstance canard;  // UAVCAN instance
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// UAVCAN node ID and message subject ID
uint8_t some_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};  // 8 byte'lık veri
uint8_t data_length = 8;  // Veri boyutu 8 byte
char tx_buffer[100];  // UART ile gönderilecek mesajlar için yeterli boyutta bir buffer
#define UAVCAN_NODE_ID           10
#define UAVCAN_MESSAGE_SUBJECT_ID 1000
CanardTxQueue tx_queue;  // CAN mesajları için iletim kuyruğu
CanardInstance canard;  // UAVCAN instance

static uint8_t transfer_idd = 0;  // Transfer ID her gönderimde artırılmalı

#define CANARD_MTU_CAN_CLASSIC 8  // Klasik CAN için maksimum mesaj boyutu


uint32_t error;
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef pTXHeader;
CAN_TxHeaderTypeDef pTXHeader2;
CAN_TxHeaderTypeDef pTXHeader3;
CAN_TxHeaderTypeDef pTXHeader4;
CAN_TxHeaderTypeDef pTXHeader5;
CAN_TxHeaderTypeDef pTXHeader6;
CAN_TxHeaderTypeDef pTXHeader7;
CAN_TxHeaderTypeDef pTXHeader8;
CAN_TxHeaderTypeDef pTXHeader9;
CAN_TxHeaderTypeDef pTXHeader10;

CAN_TxHeaderTypeDef pTXHeaderBV;
CAN_RxHeaderTypeDef pRXHeader;
CAN_FilterTypeDef sfilterconfig;
uint32_t pTxMailbox;
uint8_t count;
uint16_t rcount,rcount2,rcount3;
uint8_t adc_value[1]={0};
uint8_t deger=0;
int durum;
int durum2;

char uart_buffer[100];  // UART ile mesajları göstermek için buffer

uint16_t adcValue = 0;
uint8_t adcValueHigh = 0;
uint8_t adcValueLow = 0;
uint8_t data[8]; // CAN mesajı veri alanı
uint8_t data2[8];
uint8_t data23[8];
uint8_t led_set=1;
uint8_t led_reset=0;
char rx_buffer[50];
volatile uint8_t message_pending = 0;  // Mesaj geldiğinde set edilecek bayrak
CAN_RxHeaderTypeDef RxHeader;
 uint8_t sayac=0;
 uint8_t rx_data[8];  // Gelen CAN mesajını depolamak için 8 byte'lık bir buffer
 CanardInstance canard;  // UAVCAN instance

 CanardRxTransfer transfer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_PWM_Duty(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR1 = pulse_length; //değer setlendi
}
void Set_PWM_Duty2(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR2 = pulse_length; //değer setlendi
}
void Set_PWM_Duty3(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR3 = pulse_length; //değer setlendi
}


// Bellek tahsisi fonksiyonları
void* canardAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    return malloc(amount);  // Standart malloc kullanıyoruz
}

void canardFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    free(pointer);  // Standart free kullanıyoruz
}
/*
// Bu fonksiyon CAN mesajı geldiğinde 4 numaralı LED'i yakar
void receiveCANMessage()
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_data[8];  // Gelen CAN mesajını depolamak için 8 byte'lık bir buffer
    char uart_buffer[100];  // UART ile mesajları göstermek için buffer

    // CAN mesajını al
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
    {
        // Gelen mesajı UART ile yazdır (isteğe bağlı)
        sprintf(uart_buffer, "Gelen Veri: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

        // Eğer bir mesaj alındıysa, 4 numaralı LED'i (GPIO_PIN_4) yak
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // LED'i yak

        // LED'i belli bir süre açık tutup sonra söndür (örneğin 500 ms)
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // LED'i söndür
    }
    else
    {
        // Eğer mesaj alınamazsa, hata mesajı gönder
        sprintf(uart_buffer, "Mesaj alınamadı!\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    }
}
*/
/// CAN mesajı alma ve UART'a yazma fonksiyonu
void receiveMessage(CanardInstance* ins)
{

}



// CAN iletim fonksiyonu
void sendMessage(CanardInstance* ins, uint32_t subject_id, uint8_t* data, uint8_t data_len)
{
     // CanardTransferMetadata yapısı ile mesajın meta verileri tanımlanıyor
    CanardTransferMetadata metadata = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = subject_id,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id    = transfer_idd  // Transfer ID her gönderimden sonra artırılmalı
    };

    // CANARD_MTU_CAN_CLASSIC: 8 byte'lık CAN paketleri için maksimum MTU
    int32_t result = canardTxPush(&tx_queue, ins, HAL_GetTick() * 1000ULL, &metadata, data_len, data);

    // Mesaj gönderimi başarısız olursa
    if (result < 0)
    {
        // Hata durumunu bildiriyoruz
        sprintf(tx_buffer, "Mesaj Gitmedi, hata kodu: %ld\n", result);
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {
        // Başarılı mesaj gönderimi
        sprintf(tx_buffer, "Data Gitti. Transfer ID: %d\n", transfer_idd);
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    }

    // Transfer ID'yi artırıyoruz
    transfer_idd++;
}

// CAN iletim kuyruğunu temizleme fonksiyonu
void cleanTxQueue(void)
{
	 const CanardTxQueueItem* item = canardTxPeek(&tx_queue);  // Kuyruktaki ilk mesajı al
	    uint32_t TxMailbox;  // CAN posta kutusu

	    while (item != NULL)
	    {
	        // Posta kutusunda boş yer olup olmadığını kontrol edin
	        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)  // Boş posta kutusu varsa
	        {
	            // CAN iletim fonksiyonunu çağır ve sıradaki mesajı gönder
	            CAN_TxHeaderTypeDef TxHeader;
	            TxHeader.DLC = item->frame.payload_size;  // Data Length Code
	            TxHeader.IDE = CAN_ID_STD;  // Standart ID
	            TxHeader.RTR = CAN_RTR_DATA;  // Veri mesajı
	            TxHeader.StdId = 0x123;  // Standart CAN ID

	            // CAN mesajını iletim için sıraya ekleyin
	            if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, item->frame.payload, &TxMailbox) == HAL_OK)
	            {
	                // Mesaj başarıyla gönderildi, kuyruğun başından çıkar
	                item = canardTxPop(&tx_queue, item);
	            }
	            else
	            {
	                // Mesaj gönderimi başarısız olduysa hata mesajı gönder
	                sprintf(tx_buffer, "Mesaj kuyruğa eklenemedi.\n");
	                HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
	                break;
	            }
	        }
	        else
	        {
	            // Eğer boş posta kutusu yoksa, daha fazla mesaj eklemeyin ve döngüden çıkın
	            sprintf(tx_buffer, "Posta kutusu dolu, mesaj gönderilemedi.\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
	            break;
	        }

	        // Gönderilen mesajın hala işlenip işlenmediğini kontrol edin
	        if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
	        {
	            // Mesaj gönderildiyse kuyruktan çıkarın
	            item = canardTxPop(&tx_queue, item);
	        }
	        else
	        {
	            // Mesaj hala bekleniyorsa, döngüyü durdurun
	            break;
	        }
	    }
}
// CAN mesajı gönder
void sendUAVCANMessage()
{
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};  // Örnek veri
    uint8_t hello_message[] = "Hello";  // Gönderilecek mesaj
       uint8_t data_len = sizeof(hello_message) - 1;  // Mesaj uzunluğu (null karakter hariç)

    sendMessage(&canard, UAVCAN_MESSAGE_SUBJECT_ID, hello_message, data_len);
}

























void USB_LP_CAN1_RX0_IRQHandler(void)
{
/* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

/* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
HAL_CAN_IRQHandler(&hcan);
/* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);

   uint8_t rx_data[8];  // CAN veri çerçevesi için buffer



   // CAN RX FIFO'dan gelen mesajı alalım
      if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
      {

             sprintf(uart_buffer, "Mesaj alındı %d\n", rx_data);
             HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    }
      else   if (RxHeader.DLC == 5 && memcmp(rx_data, "Hello", 5) == 0)
      {
          // Eğer mesaj "Hello" ise, UART'a yazdır
          sprintf(uart_buffer, "Alınan mesaj: Hello\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
      }
      else
      {
      	  sprintf(uart_buffer, "Mesaj alınamadı, hata kodu: %d\n", 0);
      	              HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
      }

/* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */





	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

  //Can baslatildi
  HAL_CAN_Start(&hcan);
  //interrupt için aktif edilme
 // HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //mesaj geldi mesaj bekleniyor


  pTXHeader.DLC=1;  //1byte lık değer geldiğini ifade eder
  pTXHeader.IDE=CAN_ID_STD; //standart ID kullanılıcağı belirtir
  pTXHeader.RTR=CAN_RTR_DATA; //data gönderilmesini transmit yapıldığını belirtir
  pTXHeader.StdId=0x0156; //mesage ID numarasıdır



  sfilterconfig.FilterActivation=ENABLE;
  sfilterconfig.FilterBank=0;
  sfilterconfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  sfilterconfig.FilterIdHigh=0x0000;
  sfilterconfig.FilterIdLow=0x0000;
  sfilterconfig.FilterMaskIdHigh=0x0000;
  sfilterconfig.FilterMaskIdLow=0x0000;
  sfilterconfig.FilterMode=CAN_FILTERMODE_IDMASK;
  sfilterconfig.FilterScale=CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan, &sfilterconfig);


//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // UAVCAN başlatma
     canard = canardInit(canardAllocate, canardFree);
     // Node ID'yi doğrudan ayarlayın
     canard.node_id = UAVCAN_NODE_ID;

     // CAN kuyruğunu başlat
     tx_queue = canardTxInit(100, CANARD_MTU_CAN_CLASSIC);  // Kuyruk kapasitesi: 8, MTU: 8 byte

     // Abonelik (UAVCAN mesajlarını almayı kabul etme)
     canardRxSubscribe(&canard, CanardTransferKindMessage, UAVCAN_MESSAGE_SUBJECT_ID, 8, 1000, NULL);

     // CAN RX mesajı bekleme fonksiyonunu aktif et
     if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
     {
         // Hata oluşursa hata işlemi
         sprintf(uart_buffer, "CAN Bildirimleri Aktive Edilemedi! Hata kodu: %ld\n", HAL_CAN_GetError(&hcan));
         HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
         Error_Handler();
     }

     sprintf(uart_buffer, "CAN Bildirimleri Aktif.\n");
     HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 // sprintf(uart_buffer, "Mesaj alındı %d\n", rx_data);
	  //            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);




	 sendUAVCANMessage();  // Her 1 saniyede bir mesaj gönder
	 	  		    cleanTxQueue();

	 	  		// receiveUAVCANMessage();
	 	  		    HAL_Delay(1000);  // 100ms bekleyin









	  	  ////////////////////////////uart_end///////////////////////////////////////////////////


     Set_PWM_Duty(rcount); //pwm fonksiyonuna bu değeri gönderiyor
     Set_PWM_Duty2(rcount2);
     Set_PWM_Duty3(rcount3);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
	/*
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;  // Prescaler, clock frekansını 1/9 oranında böler
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;  // Sync Jump Width: 1 Time Quantum
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;      // Time Segment 1: 12 Time Quantum
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;       // Time Segment 2: 4 Time Quantum
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
