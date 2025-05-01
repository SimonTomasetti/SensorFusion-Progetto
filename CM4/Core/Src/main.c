/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix.h"
#include "kalman.h"
#include <stdio.h>
#include <bno055.h>
#include "dwm_high_precision.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_STATES 6
#define NUM_INPUTS 6
#define NUM_MEASUREMENTS 6
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t flag_100ms = 0;
int32_t counts = 0;
double delta_angle = 0.0;
double old_delta_angle = 0.0;
double diff_angle = 0.0;
double speed = 0.0;
double speed_degsec=0.0;
uint8_t dir = 0;
uint8_t old_dir = 0;
const double dt = 0.1; // Intervallo di tempo in secondi (100ms)
const int ENCODER_PPR = 1000; // Impulsi per giro dell'encoder
const int ENCODER_COUNTING_MODE = 4; // Modalità di conteggio (x4)
const int GEARBOX_RATIO = 1; // Rapporto di riduzione del gearbox
#define ALPHA 0.3  // Fattore di filtro (0.1-0.5)
#define MAX_EXPECTED_RPM 15  // Soglia massima RPM
double filtered_speed=0;
DistanceMeasurement measurements[10];  // Array per memorizzare fino a 10 misurazioni
uint8_t anchor_count;
UART_HandleTypeDef* dwm_huart = &huart2;  // USART2 per DWM1001
static matrix_data_t y_data[NUM_MEASUREMENTS];
static matrix_data_t S_data[NUM_MEASUREMENTS][NUM_MEASUREMENTS];
static matrix_data_t K_data[NUM_STATES][NUM_MEASUREMENTS];
static matrix_data_t S_inv_data[NUM_MEASUREMENTS][NUM_MEASUREMENTS];
static matrix_data_t temp_HP_data[NUM_MEASUREMENTS][NUM_STATES];
static matrix_data_t temp_PHt_data[NUM_STATES][NUM_MEASUREMENTS];
static matrix_data_t temp_KHP_data[NUM_STATES][NUM_STATES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Print_Distances(void);
//static matrix_data_t A_data[NUM_STATES][NUM_STATES];  // Matrice di transizione 6x6
static matrix_data_t x_data[NUM_STATES] = {0};        // Stato iniziale [posX, posY, posZ, velX, velY, velZ]
//static matrix_data_t B_data[NUM_STATES][NUM_INPUTS] = {0}; // Matrice controllo (se utilizzata)
static matrix_data_t u_data[NUM_INPUTS] = {0};        // Input di controllo
static matrix_data_t P_data[NUM_STATES][NUM_STATES];  // Covarianza iniziale
static matrix_data_t Q_data[NUM_INPUTS][NUM_INPUTS];  // Rumore del processo

// Buffer temporanei
static matrix_data_t aux_buffer[NUM_STATES];
static matrix_data_t predictedX[NUM_STATES];
static matrix_data_t temp_P[NUM_STATES][NUM_STATES];
static matrix_data_t temp_BQ[NUM_STATES][NUM_INPUTS];
static matrix_data_t z_data[NUM_MEASUREMENTS];//matrice z solo una colonna
// Strutture del filtro
static kalman_t kf;
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
	float pitch, roll, yaw;
		float vel_filtrata;
		float pos_x, pos_y;
		z_data[0] = pitch;
		z_data[1] = roll;
		z_data[2] = yaw;
		z_data[3] = vel_filtrata;
		z_data[4] = pos_x;
		z_data[5] = pos_y;
  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Funzione di inizializzazione del filtro
  void init_kalman_filter() {
      // Inizializzazione matrice A (esempio per sistema cinematico 3D)
      // Struttura: [Posizione X  Velocità X]
      //            [Posizione Y  Velocità Y]
      //            [Posizione Z  Velocità Z]

      const float dt = 0.1f; // Time step

      matrix_data_t A_data[6][6] ={
    		  {1, 0, dt, 0,  0,  0},
    		  {0, 1, 0,  dt, 0,  0},
    		  {0, 0, 1,  0,  0,  0},
    		  {0, 0, 0,  1,  0,  0},
    		  {0, 0, 0,  0,  1,  0},
    		  {0, 0, 0,  0,  0,  1}
      };
      matrix_data_t B_data[6][6]={
          		  {0.5*dt*dt, 0,        0,        0,        0,        0},
          		  {0,         0.5*dt*dt, 0,        0,        0,        0},
          		  {dt,        0,        0,        0,        0,        0},
          		  {0,         dt,       0,        0,        0,        0},
          		  {0,         0,        dt,       0,        0,        0},
          		  {0,         0,        0,        dt,       0,        0}
            };
      // Inizializzazione matrice P (incertezza iniziale)
            for(int i=0; i<NUM_STATES; i++) {
                for(int j=0; j<NUM_STATES; j++) {
                    P_data[i][j] = (i == j) ? 1000.0f : 0.0f;
                }
            }
            // Inizializzazione matrice Q (rumore del processo)
                  for(int i=0; i<NUM_INPUTS; i++) {
                      for(int j=0; j<NUM_INPUTS; j++) {
                          Q_data[i][j] = (i == j) ? 0.1f : 0.0f;
                      }
                  }
                  kalman_filter_initialize(
                           &kf,
                           NUM_STATES,
                           NUM_INPUTS,
                           (matrix_data_t*)A_data,
                           x_data,
                           (matrix_data_t*)B_data,
                           u_data,
                           (matrix_data_t*)P_data,
                           (matrix_data_t*)Q_data,
                           aux_buffer,
                           predictedX,
                           (matrix_data_t*)temp_P,
                           (matrix_data_t*)temp_BQ
                       );
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      TIM1->CCR1=10;
      HAL_TIM_Base_Start_IT(&htim6);

      HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
      bno055_assignI2C(&hi2c1);  // Assegna l'interfaccia I2C all'IMU
      bno055_setup();            // Configura l'IMU
      bno055_setOperationModeNDOF(); // Imposta la modalità NDOF (Fusion Mode)
      printf("Inizializzazione DWM1001...\r\n");
           if (DWM_Init(&huart2) != HAL_OK) {
                printf("ERRORE: DWM1001 non collegato o inizializzazione fallita!\r\n");
            } else {
                printf("DWM1001 pronto per misurazioni ad alta precisione (1cm)\r\n");
            }
           printf("Test UART...\r\n");
               uint8_t test_data[] = {0x55, 0xAA};  // Byte di test (0x55 = 'U', 0xAA = simbolo speciale)
               HAL_UART_Transmit(&huart2, test_data, sizeof(test_data), 100);

               // Opzionale: Verifica ricezione (se il DWM1001 dovrebbe rispondere)
               uint8_t rx_data;
               if(HAL_UART_Receive(&huart2, &rx_data, 1, 100) == HAL_OK) {
                   printf("Ricevuto byte: 0x%02X\r\n", rx_data);
               } else {
                   printf("Nessun dato ricevuto dal DWM1001!\r\n");

               // === FINE CODICE DI DEBUG ===
           }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Fase di predizione
	  kalman_predict_x(&kf);
	  kalman_predict_Q(&kf);
	  // Riempimento di z_data
	        z_data[0] = pitch;
	        z_data[1] = roll;
	        z_data[2] = yaw;
	        z_data[3] = vel_filtrata;
	        z_data[4] = pos_x;
	        z_data[5] = pos_y;
	        matrix_data_t H_data[6][6] = {
	            		  {1, 0, 0, 0, 0, 0},
	            		  {0, 1, 0, 0, 0, 0},
	            		  {0, 0, 1, 0, 0, 0},
	            		  {0, 0, 0, 1, 0, 0},
	            		  {0, 0, 0, 0, 1, 0},
	            		  {0, 0, 0, 0, 0, 1}
	              };
	        matrix_data_t R_data[6][6] = {
	                    {0.1, 0,   0,   0,   0,   0},  // Rumore posizione X
	                    {0,   0.1, 0,   0,   0,   0},  // Rumore posizione Y
	                    {0,   0,   0.1, 0,   0,   0},  // Rumore posizione Z
	                    {0,   0,   0,   0.5, 0,   0},  // Rumore velocità X
	                    {0,   0,   0,   0,   0.5, 0},  // Rumore velocità Y
	                    {0,   0,   0,   0,   0,   0.5} // Rumore velocità Z
	                };
	        matrix_data_t z_data[NUM_MEASUREMENTS] = {0};
	              // Correzione
	              kalman_measurement_t kfm;
	              kalman_measurement_initialize(
	                  &kfm,
	                  NUM_STATES,
	                  NUM_MEASUREMENTS,
	                  (matrix_data_t*)H_data,
	                  (matrix_data_t*)z_data,
	                  (matrix_data_t*)R_data,
	                  (matrix_data_t*)y_data,
	                  (matrix_data_t*)S_data,
	                  (matrix_data_t*)K_data,
	                  aux_buffer,
	                  (matrix_data_t*)S_inv_data,
	                  (matrix_data_t*)temp_HP_data,
	                  (matrix_data_t*)temp_PHt_data,
	                  (matrix_data_t*)temp_KHP_data
	              );
	              kalman_correct(&kf, &kfm);
	              // Stampa lo stato ogni 10 cicli
	                    static uint8_t counter = 0;
	                    if(++counter >= 10) {
	                        print_state(&kf);
	                        counter = 0;
	                    }

	                    HAL_Delay(10); // 10ms loop

	  readImu();
	  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
	  	  	  HAL_Delay(300);
	  	  	if (1) {
	  	  	    flag_100ms = 0;
	  	  	    counts = TIM4->CNT;
	  	  	    dir = (TIM4->CR1 & TIM_CR1_DIR) ? 1 : 0;
	  	  	    delta_angle = (double)counts * 360 / 2651;

	  	  	    // Calcolo differenza angolo con correzione wrap-around
	  	  	    diff_angle = -(delta_angle - old_delta_angle);
	  	  	    if(fabs(diff_angle) > 180) {
	  	  	        diff_angle = diff_angle - copysign(360.0, diff_angle);
	  	  	    }

	  	  	    // Gestione cambio direzione
	  	  	    if(dir != old_dir) {
	  	  	        if(dir == 1 && diff_angle > 0) {
	  	  	            diff_angle -= 360.0;
	  	  	        } else if(dir == 0 && diff_angle < 0) {
	  	  	            diff_angle += 360.0;
	  	  	        }
	  	  	    }

	  	  	    // Calcolo velocità
	  	  	    speed_degsec = diff_angle / dt;
	  	  	    speed = speed_degsec / 6.0;

	  	  	    // Filtraggio e validazione velocità
	  	  	    if(fabs(speed) < MAX_EXPECTED_RPM) {
	  	  	        filtered_speed = ALPHA * speed + (1-ALPHA) * filtered_speed;
	  	  	    } else {
	  	  	        // Usa l'ultimo valore valido se supera la soglia
	  	  	        speed = filtered_speed;
	  	  	    }

	  	  	    old_delta_angle = delta_angle;
	  	  	    old_dir = dir;

	  	  	    // Stampa dati
	  	  	    printf("Angolo: %.2f° | Velocità: %.2f RPM | Velocità filtrata: %.2f RPM \r\n",
	  	  	           delta_angle, speed, filtered_speed);
	  	  	}
	  	  HAL_StatusTypeDef status = DWM_GetDistances(measurements, &anchor_count);
	  	  	  	     printf("Stato: %d (HAL_OK=0)\r\n", status);  // Debug

	  	  	  	     if (status == HAL_OK) {
	  	  	  	         if (anchor_count == 0) {
	  	  	  	             printf("DWM1001 connesso, ma nessun anchor rilevato!\r\n");
	  	  	  	         } else {
	  	  	  	             printf("DWM1001 risponde correttamente. Anchor rilevati: %d\r\n", anchor_count);
	  	  	  	             Print_Distances();
	  	  	  	         }
	  	  	  	     } else {
	  	  	  	         printf("ERRORE: Comunicazione UART fallita (codice: %d)!\r\n", status);
	  	  	  	     }

	  	  	  	     HAL_Delay(500);  // Misurazioni ogni 500ms
  }
  /* USER CODE END 3 */
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1001-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2652-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 64-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //configurazione pin
    // PA3 -> USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PD5 -> USART2_TX
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Funzione per calcolare il segno (usata da copysign)

int __io_putchar(int ch){
	HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        flag_100ms = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); // Cambia la direzione del motore
    }
}
static inline double custom_copysign(double x, double y) {
    return (y < 0) ? -fabs(x) : fabs(x);
}
void readImu() {
    bno055_vector_t v = bno055_getVectorEuler();  // Legge i dati di orientamento (roll, pitch, yaw)
    float roll = v.y;  // Roll (rotazione sull'asse X)
    float pitch = +v.z; //Pitch (rotazione sull'assey)
    float yaw = v.x;  // Yaw (rotazione sull'asse Z)
    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", roll, pitch, yaw);
    }
void Print_Distances(void) {
    char buffer[128];
    printf("--- Distanze rilevate ---\r\n");
    for(uint8_t i = 0; i < anchor_count; i++) {
        // Formattazione con 2 decimali (precisione cm)
        snprintf(buffer, sizeof(buffer),
                "Anchor %04X: %.2f m (QF: %d)\r\n",
                measurements[i].anchor_id,
                measurements[i].distance_m,
                measurements[i].quality);

        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
    }

}
void print_state(const kalman_t *kf) {
    printf("Stato stimato:\n");
    printf("PosX: %7.3f  VelX: %7.3f\n", kf->x.data[0], kf->x.data[3]);
    printf("PosY: %7.3f  VelY: %7.3f\n", kf->x.data[1], kf->x.data[4]);
    printf("PosZ: %7.3f  VelZ: %7.3f\n", kf->x.data[2], kf->x.data[5]);
    printf("----------------------------\n");
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
