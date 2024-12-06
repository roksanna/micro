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
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define TOGGLE_INTERVAL 1000  // Czas w milisekundach między zmianami stanu GPIO
#define MAX_FRAME_SIZE 256    // Maksymalny rozmiar ramki

/* Private variables ---------------------------------------------------------*/
uint32_t last_toggle_time = 0;  // Zmienna do przechowywania ostatniego czasu zmiany stanu
uint32_t test_data[] = {0x12345678, 0xABCDEF01, 0x98765432}; // Dane testowe do obliczeń CRC
uint32_t expected_crc = 0xDEADBEEF; // Przykładowa oczekiwana wartość CRC
uint32_t computed_crc = 0; // Wynik obliczonego CRC

uint32_t pwm_values[] = {0, 50, 100, 150, 200, 255};



uint8_t message[] = "Hello from USART2!";  // Wiadomość do wysłania przez USART2
uint8_t buffer[MAX_FRAME_SIZE];  // Bufor do odbioru danych przez USART2

// Zmienne dla sterowania prędkością PWM
uint32_t max_speed = 1000;  // Maksymalna prędkość (częstotliwość PWM)
uint32_t accel_time = 1000;  // Czas rozpędzania (ms)
uint32_t decel_time = 1000;  // Czas hamowania (ms)
uint32_t current_steps = 0;  // Liczba wykonanych kroków
uint32_t total_steps = 200;  // Łączna liczba kroków do wykonania
uint8_t buffer[MAX_FRAME_SIZE];
uint32_t last_step_time = 0;  // Zmienna do przechowywania czasu ostatniego kroku
uint32_t step_delay = 100;  // Czas opóźnienia między krokami (w milisekundach)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void UART_Transmit(uint8_t *data, uint16_t size); // Prototyp funkcji do transmisji danych przez USART
void UART_Receive(uint8_t *buffer, uint16_t size); // Prototyp funkcji do odbioru danych przez USART
void Update_PWM_Speed(uint32_t current_time); // Funkcja do zmiany prędkości PWM
void Process_Frame(uint8_t *frame, uint16_t frame_len); // Funkcja do przetwarzania odebranej ramki
void Execute_Command(char *command, char *data);  // Funkcja do wykonania komendy

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, pwm_values, sizeof(pwm_values)/sizeof(pwm_values[0]));  // Start PWM with DMA

  // Obliczenie CRC dla danych testowych
  computed_crc = HAL_CRC_Calculate(&hcrc, test_data, sizeof(test_data) / sizeof(test_data[0]));

  // Weryfikacja CRC i sterowanie GPIO PA9
  if (computed_crc == expected_crc)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // Jeśli CRC jest zgodne, ustaw PA9
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Jeśli CRC nie pasuje, wyłącz PA9
  }

  UART_Transmit(message, sizeof(message)); // Wysyłanie danych przez USART2
  HAL_UART_Receive_IT(&huart2, buffer, MAX_FRAME_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();  // Czas od uruchomienia

    // Zaktualizowanie prędkości PWM w zależności od czasu
    Update_PWM_Speed(current_time);

    if (current_time - last_step_time >= step_delay && current_steps < total_steps)
        {
            // Wykonaj krok, np. zmiana stanu GPIO, jak w poprzednich przykładach
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);  // Symulacja kroku - np. przełączanie diody

            current_steps++;  // Zwiększ liczbę wykonanych kroków

            // Wyświetl status co 10 kroków
            if (current_steps % 10 == 0)
            {
                char step_message[100];
                snprintf(step_message, sizeof(step_message), "Step: %lu/%lu\n", current_steps, total_steps);
                UART_Transmit((uint8_t *)step_message, strlen(step_message));
            }

            last_step_time = current_time;  // Zaktualizuj czas ostatniego kroku
        }
        else if (current_steps >= total_steps)
        {
            // Jeśli wszystkie kroki zostały wykonane, wyślij komunikat o zakończeniu
            char finished_message[] = "All steps completed\n";
            UART_Transmit((uint8_t *)finished_message, sizeof(finished_message) - 1);

            // Poczekaj na kolejne komendy
            HAL_UART_Receive_IT(&huart2, buffer, MAX_FRAME_SIZE);  // Zainicjuj odbiór nowych danych
        }

    // Przetwarzanie odbieranej ramki
    Process_Frame(buffer, MAX_FRAME_SIZE);

  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief Transmit data over USART2.
  * @param data: Pointer to the data buffer
  * @param size: Size of the data to be transmitted
  * @retval None
  */
void UART_Transmit(uint8_t *data, uint16_t size)
{
  HAL_UART_Transmit(&huart2, data, size, HAL_MAX_DELAY);  // Wysyłanie danych przez USART2
}

/**
  * @brief Receive data over USART2.
  * @param buffer: Pointer to the buffer where received data will be stored
  * @param size: Number of bytes to be received
  * @retval None
  */
void UART_Receive(uint8_t *buffer, uint16_t size)
{
  HAL_UART_Receive(&huart2, buffer, size, HAL_MAX_DELAY);  // Odbieranie danych przez USART2
}

/**
  * @brief Process the received frame.
  * @param frame: Pointer to the received frame
  * @param frame_len: Length of the frame
  * @retval None
  */
void Process_Frame(uint8_t *frame, uint16_t frame_len)
{
    // Assuming frame contains a command and corresponding data
    char command[20], data[100];
    sscanf((char *)frame, "%s %s", command, data);

    Execute_Command(command, data);  // Execute command with its data
}

/**
  * @brief Execute the given command.
  * @param command: The command to execute
  * @param data: The data associated with the command
  * @retval None
  */
void Execute_Command(char *command, char *data)
{
    if (strcmp(command, "maxspeed") == 0)
    {
        max_speed = atoi(data);
    }
    else if (strcmp(command, "accel") == 0)
    {
        accel_time = atoi(data);
    }
    else if (strcmp(command, "decel") == 0)
    {
        decel_time = atoi(data);
    }
    else if (strcmp(command, "steps") == 0)
    {
        total_steps = atoi(data);
    }
    else if (strcmp(command, "start") == 0)
    {
        current_steps = 0; // Start the motion
    }
    else if (strcmp(command, "stop") == 0)
    {
        current_steps = total_steps;  // Stop at total steps
    }
    else if (strcmp(command, "status") == 0)
    {
        // Send current status over USART
        char status_message[100];
        snprintf(status_message, sizeof(status_message), "Speed: %lu, Steps: %lu/%lu", max_speed, current_steps, total_steps);
        UART_Transmit((uint8_t *)status_message, strlen(status_message));
    }
    else if (strcmp(command, "w") == 0)  // Komenda 'w' - włącz diodę
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Ustawienie PA5 na wysoki stan (włączenie diody)
            UART_Transmit((uint8_t *)"LED ON\n", 7);  // Potwierdzenie włączenia diody
        }
        else if (strcmp(command, "e") == 0)  // Komenda 'e' - wyłącz diodę
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Ustawienie PA5 na niski stan (wyłączenie diody)
            UART_Transmit((uint8_t *)"LED OFF\n", 8);  // Potwierdzenie wyłączenia diody
        }
}

/**
  * @brief Update the PWM speed based on the current time.
  * @param current_time: The current time in milliseconds
  * @retval None
  */
void Update_PWM_Speed(uint32_t current_time)
{
    uint32_t speed = 0;

    if (current_time < accel_time) {
        speed = (max_speed * current_time) / accel_time;  // Rozpędzanie
    } else if (current_time < (accel_time + decel_time)) {
        speed = max_speed - ((max_speed * (current_time - accel_time)) / decel_time);  // Hamowanie
    } else {
        speed = max_speed;  // Maksymalna prędkość
    }

    // Ustawienie wartości PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // Sprawdź, czy przerwanie dotyczy USART2
    {
        // Obsłuż odebrane dane
        Process_Frame(buffer, strlen((char *)buffer));

        // Restart odbioru danych
        HAL_UART_Receive_IT(&huart2, buffer, MAX_FRAME_SIZE); // Zainicjuj ponownie odbiór
    }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
