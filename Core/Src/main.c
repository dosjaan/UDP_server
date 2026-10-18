/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <limits.h>
#include <string.h>

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static struct udp_pcb *udp_server_pcb = NULL;

#define UDP_SERVER_PORT       5000U
#define CMD_TIMEOUT_MS        500U
#define MAX_PACKET_LEN        95U

#define SPEED_MIN_MM_S       (-500L)
#define SPEED_MAX_MM_S         500L

#define STEER_MIN_MDEG      (-45000L)
#define STEER_MAX_MDEG        45000L

volatile int32_t speed_setpoint_mm_s = 0;
volatile int32_t steering_setpoint_mdeg = 0;

volatile uint32_t last_command_tick = 0;
volatile uint32_t last_command_seq = 0;
volatile uint8_t command_valid = 0;
volatile uint8_t command_timeout = 1;

static uint8_t have_sequence = 0;
static int32_t last_accepted_speed = 0;
static int32_t last_accepted_steering = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static struct udp_pcb *udp_server_pcb;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int parse_number(char **cursor,
                        long long *value,
                        char delimiter)
{
    char *start = *cursor;
    char *end = NULL;

    if (*start == '-')
        start++;

    if (*start < '0' || *start > '9')
        return 0;

    errno = 0;
    long long parsed = strtoll(*cursor, &end, 10);

    if (errno == ERANGE || end == *cursor)
        return 0;

    if (delimiter == '\0')
    {
        if (*end != '\0')
            return 0;
    }
    else
    {
        if (*end != delimiter)
            return 0;

        end++;
    }

    *value = parsed;
    *cursor = end;

    return 1;
}


static int parse_command(char *buf,
                         uint32_t *seq,
                         int32_t *speed,
                         int32_t *steering)
{
    char *cursor = buf;
    long long s, v, d;

    if (strncmp(cursor, "CMD,", 4) != 0)
        return 0;

    cursor += 4;

    if (!parse_number(&cursor, &s, ','))
        return 0;

    if (!parse_number(&cursor, &v, ','))
        return 0;

    if (!parse_number(&cursor, &d, '\0'))
        return 0;

    if (s < 0 || (unsigned long long)s > UINT32_MAX)
        return 0;

    if (v < INT32_MIN || v > INT32_MAX)
        return 0;

    if (d < INT32_MIN || d > INT32_MAX)
        return 0;

    *seq = (uint32_t)s;
    *speed = (int32_t)v;
    *steering = (int32_t)d;

    return 1;
}

static void udp_send_text(struct udp_pcb *pcb,
                          const ip_addr_t *addr,
                          u16_t port,
                          const char *message)
{
    size_t len = strlen(message);

    if (len == 0U || len > 255U)
        return;

    struct pbuf *reply =
        pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);

    if (reply == NULL)
        return;

    if (pbuf_take(reply, message, (u16_t)len) == ERR_OK)
    {
        err_t err = udp_sendto(pcb, reply, addr, port);
        (void)err;
    }

    pbuf_free(reply);
}
static int sequence_is_newer(uint32_t seq, uint32_t previous)
{
    uint32_t difference = seq - previous;

    return (difference != 0U) &&
           (difference < 0x80000000U);
}

static void udp_server_recv(void *arg,
                            struct udp_pcb *pcb,
                            struct pbuf *p,
                            const ip_addr_t *addr,
                            u16_t port)
{
    char rx[MAX_PACKET_LEN + 1U];
    char tx[64];

    uint32_t seq;
    int32_t speed;
    int32_t steering;

    (void)arg;

    if (p == NULL)
        return;

    /* Reject empty and oversized packets. */
    if (p->tot_len == 0U || p->tot_len > MAX_PACKET_LEN)
    {
        udp_send_text(pcb, addr, port, "ERR,LENGTH");
        goto cleanup;
    }

    u16_t packet_len = p->tot_len;

    if (pbuf_copy_partial(p, rx, packet_len, 0) != packet_len)
    {
        udp_send_text(pcb, addr, port, "ERR,COPY");
        goto cleanup;
    }

    rx[packet_len] = '\0';

    /* Embedded NUL bytes are invalid. */
    if (strlen(rx) != packet_len)
    {
        udp_send_text(pcb, addr, port, "ERR,FORMAT");
        goto cleanup;
    }

    /* Permit a single CR/LF ending from text clients. */
    while (packet_len > 0U &&
           (rx[packet_len - 1U] == '\r' ||
            rx[packet_len - 1U] == '\n'))
    {
        rx[--packet_len] = '\0';
    }

    if (!parse_command(rx, &seq, &speed, &steering))
    {
        udp_send_text(pcb, addr, port, "ERR,FORMAT");
        goto cleanup;
    }

    if (speed < SPEED_MIN_MM_S || speed > SPEED_MAX_MM_S)
    {
        udp_send_text(pcb, addr, port, "ERR,SPEED_RANGE");
        goto cleanup;
    }

    if (steering < STEER_MIN_MDEG ||
        steering > STEER_MAX_MDEG)
    {
        udp_send_text(pcb, addr, port, "ERR,STEER_RANGE");
        goto cleanup;
    }

    /* Duplicate sequence: acknowledge only an identical command. */
    if (have_sequence && seq == last_command_seq)
    {
        if (speed == last_accepted_speed &&
            steering == last_accepted_steering)
        {
            snprintf(tx, sizeof(tx), "ACK,%lu,%ld,%ld",
                     (unsigned long)seq,
                     (long)speed,
                     (long)steering);

            udp_send_text(pcb, addr, port, tx);
        }
        else
        {
            udp_send_text(pcb, addr, port, "ERR,SEQ_CONFLICT");
        }

        goto cleanup;
    }

    /* Reject stale or out-of-order commands. */
    if (have_sequence && !sequence_is_newer(seq, last_command_seq))
    {
        udp_send_text(pcb, addr, port, "ERR,SEQ_OLD");
        goto cleanup;
    }

    /*
     * Accept command.
     * This updates setpoints only; it does not drive a motor.
     */
    speed_setpoint_mm_s = speed;
    steering_setpoint_mdeg = steering;

    last_command_seq = seq;
    last_accepted_speed = speed;
    last_accepted_steering = steering;
    last_command_tick = HAL_GetTick();

    have_sequence = 1U;
    command_valid = 1U;
    command_timeout = 0U;

    snprintf(tx, sizeof(tx), "ACK,%lu,%ld,%ld",
             (unsigned long)seq,
             (long)speed,
             (long)steering);

    udp_send_text(pcb, addr, port, tx);

cleanup:
    pbuf_free(p);
}


static void udp_server_init(void)
{
    udp_server_pcb = udp_new();

    if (udp_server_pcb == NULL)
        return;

    if (udp_bind(udp_server_pcb,
                 IP_ADDR_ANY,
                 UDP_SERVER_PORT) != ERR_OK)
    {
        udp_remove(udp_server_pcb);
        udp_server_pcb = NULL;
        return;
    }

    udp_recv(udp_server_pcb, udp_server_recv, NULL);
}


static void command_timeout_process(void)
{
    if (command_valid &&
        (uint32_t)(HAL_GetTick() - last_command_tick)
            >= CMD_TIMEOUT_MS)
    {
        speed_setpoint_mm_s = 0;
        command_valid = 0U;
        command_timeout = 1U;
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
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  udp_server_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  while (1)
	  {
	      MX_LWIP_Process();
	      command_timeout_process();
	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

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
#ifdef USE_FULL_ASSERT
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
