/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LM35 -> ADC + DMA (stockage) -> traitement CPU -> LCD
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VREF           3.0f      /* STM32F407 Discovery : VDDA = 3.0 V */
#define ADC_RESOLUTION     4095.0f
#define LM35_MV_PER_C      10.0f     /* LM35 : 10 mV / degre C */

#define ADC_BUFFER_SIZE    64        /* valeurs ADC stockees par le DMA (circulaire) */
#define MA_WINDOW          16        /* fenetre de la moyenne glissante */
#define PROCESS_PERIOD_MS  100       /* le CPU traite le buffer toutes les 100 ms */
#define LCD_PERIOD_MS      500       /* rafraichissement du LCD */
#define STAT_N             50        /* mesures utilisees pour juger la stabilite (5 s) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Stockage : rempli en continu par le DMA (ADC continu + DMA circulaire) */
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

/* Copie ordonnee dans le temps (ancien -> recent) et temperatures */
uint16_t adc_snap[ADC_BUFFER_SIZE];   /* valeurs ADC (copie du buffer DMA) bch cpu yetreteha   */
float    t_raw[ADC_BUFFER_SIZE];      /* temperatures brutes (C) calculee directement mel adc   */
float    t_filt[ADC_BUFFER_SIZE];     /* temperatures apres Filtrage (moyenne glissante)  */

/* Valeurs affichees */
float temp_raw   = 0.0f;   /* brut : dernier echantillon du buffer        */
float temp_filt  = 0.0f;   /* filtre : moyenne glissante                  */

/* Fluctuations (ecart max - min, en C) */
float fluct_raw  = 0.0f;   /* sur l'affichage brut   (derniers STAT_N)    */
float fluct_filt = 0.0f;   /* sur l'affichage filtre (derniers STAT_N)    */
float fluct_buf_raw  = 0.0f;   /* a l'interieur d'un buffer DMA, brut     */
float fluct_buf_filt = 0.0f;   /* a l'interieur d'un buffer DMA, filtre   */

char lcd_buffer[32];

static float   stat_raw[STAT_N];   /*ya7fed les dernières températures brutes*/
static float   stat_filt[STAT_N];
static uint8_t stat_idx   = 0;     /* indice de la prochaine case à remplir */
static uint8_t stat_count = 0;      /* hatkch 50 khater kol 100ms ysir changement donc d 5s anna 50*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- Mise en forme de texte (sans printf / sans float printf) ------- */

static char *put_str(char *p, const char *s)  /*T7awwel char  l texte*/
{
    while (*s) *p++ = *s++;
    return p;
}

static char *put_uint(char *p, uint32_t v, uint8_t min_digits) /*T7awwel entier positif l texte*/
{
    char tmp[10];
    uint8_t n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v != 0 || n < min_digits);

    while (n > 0) *p++ = tmp[--n];
    return p;
}

/* Reel positif avec 1 ou 2 decimales : 31.4  /  0.62 */
static char *put_fixed(char *p, float v, uint8_t decimals)  /*t7awwel nombre float l texte*/
{
    uint32_t scale = (decimals == 2) ? 100U : 10U;
    if (v < 0.0f) v = 0.0f;

    uint32_t x = (uint32_t)(v * (float)scale + 0.5f);
    p = put_uint(p, x / scale, 1);
    *p++ = '.';
    p = put_uint(p, x % scale, decimals);
    return p;
}

static void pad16(char *start, char *end)
{
    while (end < start + 16) *end++ = ' ';
    *end = '\0';
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------- Traitement des donnees stockees -------------------------------- */

/* Valeur ADC -> temperature en degres C */
static float adc_to_temp(float counts)
{
    float volts = (counts * ADC_VREF) / ADC_RESOLUTION;
    return (volts * 1000.0f) / LM35_MV_PER_C;
}

/* Copie le buffer DMA dans l'ordre chronologique (du plus ancien au plus recent).
   NDTR = transferts restants -> ADC_BUFFER_SIZE - NDTR = prochaine case ecrite
   = la plus ancienne valeur du buffer circulaire. */
static void dma_snapshot(uint16_t *dst)
{
    uint32_t ndtr  = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
    uint32_t start = (ADC_BUFFER_SIZE - ndtr) % ADC_BUFFER_SIZE;
/*Dans un buffer circulaire de 64 mesures, ken NDTR = 20, ya3ni:
               start}=(64-20) mod 64=44           */
    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        dst[i] = adc_buffer[(start + i) % ADC_BUFFER_SIZE];
    }
}   /* ta3mel retour à la case 0 ki l'indice yousel lel 64 */

/* Moyenne glissante : out[i] = moyenne des k valeurs se terminant en i */
static void moving_average(const float *in, float *out, uint32_t n, uint32_t k)
{
    float sum = 0.0f;

    for (uint32_t i = 0; i < n; i++)
    {
        sum += in[i];
        if (i >= k) sum -= in[i - k];
        out[i] = sum / (float)((i + 1U < k) ? (i + 1U) : k);
    }
}

/* Fluctuation = max - min sur n valeurs */
static float peak_to_peak(const float *a, uint32_t n)
{
    float mn = a[0], mx = a[0];
    for (uint32_t i = 1; i < n; i++)
    {
        if (a[i] < mn) mn = a[i];
        if (a[i] > mx) mx = a[i];
    }
    return mx - mn;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* Le DMA range les mesures de l'ADC dans adc_buffer, sans le CPU */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_Delay(100);

  LCD_Init();

  LCD_SetCursor(0, 0);
  LCD_SendString("LM35 Temperature");
  LCD_SetCursor(1, 0);
  LCD_SendString("ADC + DMA  ");

  HAL_Delay(1500);
  LCD_Clear();

  uint32_t next_tick = HAL_GetTick();
  uint32_t last_lcd  = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- Etape 4 : traitement par le CPU des valeurs stockees par le DMA --- */

    /* 1) Lire le buffer DMA (ordre chronologique) */
    dma_snapshot(adc_snap);    /*Le CPU ya3ayet lel fonction
    dma_snapshot() bech ya5ou copie des 64 valeurs stockées par DMA*/

    /* 2) Calculer la temperature de chaque valeur ADC */
    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        t_raw[i] = adc_to_temp((float)adc_snap[i]);

    /*        adc_snap[0] = 500  → t_raw[0] = 36.6 °C
		      adc_snap[1] = 502  → t_raw[1] = 36.8 °C  ..   */

    }

    /* 3) Filtre numerique : moyenne glissante sur MA_WINDOW valeurs */
    moving_average(t_raw, t_filt, ADC_BUFFER_SIZE, MA_WINDOW);

    /* 4) Valeurs affichees : brut = derniere valeur, filtre = derniere sortie du filtre */
    temp_raw  = t_raw[ADC_BUFFER_SIZE - 1];
    temp_filt = t_filt[ADC_BUFFER_SIZE - 1];

    /* 5a) Comparaison a l'interieur d'un buffer (zone ou le filtre est etabli) */
    fluct_buf_raw  = peak_to_peak(&t_raw[MA_WINDOW - 1],  ADC_BUFFER_SIZE - (MA_WINDOW - 1));
    fluct_buf_filt = peak_to_peak(&t_filt[MA_WINDOW - 1], ADC_BUFFER_SIZE - (MA_WINDOW - 1));

    /* 5b) Comparaison de ce qui est AFFICHE, sur les STAT_N dernieres mesures */
    stat_raw[stat_idx]  = temp_raw;
    stat_filt[stat_idx] = temp_filt;
    stat_idx = (uint8_t)((stat_idx + 1U) % STAT_N);
    if (stat_count < STAT_N) stat_count++;

    fluct_raw  = peak_to_peak(stat_raw,  stat_count);
    fluct_filt = peak_to_peak(stat_filt, stat_count);

    /* ---- Affichage LCD (toutes les LCD_PERIOD_MS) --------------------------
         "A:0490 T:0.396V"    A = valeur ADC, T = tension
         "R:39.6C F:39.2C"    R = temperature brute, F = temperature filtree  */
    if ((HAL_GetTick() - last_lcd) >= LCD_PERIOD_MS)
    { /* Le LCD ne doit pas être rafraîchi à chaque tour de la boucle,sinon on risque de solliciter inutilement l'écran */
        last_lcd = HAL_GetTick();
        char *p;

        uint16_t adc_last = adc_snap[ADC_BUFFER_SIZE - 1];
        uint32_t mv = (uint32_t)(((float)adc_last * ADC_VREF * 1000.0f) / ADC_RESOLUTION + 0.5f);

        /* Ligne 1 : "A:0490 T:0.396V" */
        p = lcd_buffer;
        p = put_str(p, "A:");
        p = put_uint(p, adc_last, 4);
        p = put_str(p, " T:");
        p = put_uint(p, mv / 1000, 1);
        p = put_str(p, ".");
        p = put_uint(p, mv % 1000, 3);
        p = put_str(p, "V");
        pad16(lcd_buffer, p);
        LCD_SetCursor(0, 0);
        LCD_SendString(lcd_buffer);

        /* Ligne 2 : "R:39.6C F:39.2C" */
        p = lcd_buffer;
        p = put_str(p, "R:");
        p = put_fixed(p, clampf(temp_raw,  0.0f, 99.9f), 1);
        p = put_str(p, "C F:");
        p = put_fixed(p, clampf(temp_filt, 0.0f, 99.9f), 1);
        p = put_str(p, "C");
        pad16(lcd_buffer, p);
        LCD_SetCursor(1, 0);
        LCD_SendString(lcd_buffer);
    }

    /* Periode fixe de PROCESS_PERIOD_MS */
    next_tick += PROCESS_PERIOD_MS;
    while ((int32_t)(HAL_GetTick() - next_tick) < 0)
    {
    }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
