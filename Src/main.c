/*                              ------ hacking hoverboard -------
MOTOR LEFT
            HALL_A = PB5
            HALL_B = PB6
            HALL_C = PB7
            A-MOSFET_+ = PC6    [Attivo alto]
            A-MOSFET_- = PA7    [Attivo basso]
            B-MOSFET_+ = PC7    [Attivo alto]
            B-MOSFET_- = PB0    [Attivo basso]
            C-MOSFET_+ = PC8    [Attivo alto]
            C-MOSFET_- = PB1    [Attivo basso]
            CURRENT_SENSE_shunt = PC0 [Analog, a vuoto 1.5747V, Rshunt=0.0036525/Guadagno_OPAMP = 10.4]
            A-VOLTAGE_PHASE_A_BACK_EMF = PA0    [Analog]  -- {NOT USE}
            B-VOLTAGE_PHASE_A_BACK_EMF = PC3    [Analog]  -- {NOT USE}

MOTOR RIGHT
            HALL_A = PC10
            HALL_B = PC11
            HALL_C = PC12
            A-MOSFET_+ = PA8    [Attivo alto]
            A-MOSFET_- = PB13   [Attivo basso]
            B-MOSFET_+ = PA9    [Attivo alto]
            B-MOSFET_- = PB14   [Attivo basso]
            C-MOSFET_+ = PA10   [Attivo alto]
            C-MOSFET_- = PB15   [Attivo basso]
            CURRENT_SENSE_shunt = PC1 [Analog, a vuoto 1.5747V, Rshunt=0.0036525/Guadagno_OPAMP = 10.4]
            A-VOLTAGE_PHASE_A_BACK_EMF = PC4    [Analog]  -- {NOT USE}
            B-VOLTAGE_PHASE_A_BACK_EMF = PC5    [Analog]  -- {NOT USE}

VARIE
            LED = PB2
            BUZZER = PA4
            FRONT_LEFT[connettore 4 poli] = +15V ; PA2 ; PA3 ; MASSA    -- {NOT USE}
            FRONT_RIGHT[connettore 4 poli] = +15V ; PB10 ; PB11 ; MASSA -- {USE x JOYSTICK}
            SWITCH = PA1                                                -- {NOT USE}
            VBATT_MEASURE = PC2         [Analog, rapporto 956.5 ohm/29910 ohm]
            IS_BATTERY_IN_CHARGE = PA12 [input, need pullup]
            AUTO-RITENUTA-SWITCH = PA5 [output, mantiene attivo il tip127 che funge da interruttore generale]   -- {NOT USE}
            NOT-IDENTIFITY = PB12
            NOT-IDENTIFITY = PA6

*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor_L.h"
#include "motor_R.h"
#include "varie.h"
#include "ADC_L.h"
#include "ADC_R.h"
#include "delay.h"
#include "pid.h"
#include "application.h"
#include "telemetry.h"
#include "hd44780.h"
#include <math.h>

// copied from STMBL
#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))

#define PI 3.14159265

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
//extern struct PID_dati pid_R_;
static void MX_IWDG_Init(void);
IWDG_HandleTypeDef hiwdg;

volatile __IO int16_t speed = 0;
extern struct TELEMETRY_dati telemetry;
//extern struct COMMAND_data commandsequence;

//TEMP
/*
extern volatile __IO struct MOTOR_Rdati motorR;
volatile __IO uint8_t temp8,temp8case,temp_MOTOR_R_START;
volatile __IO int32_t temp_SET_SPPED;
volatile __IO uint8_t bufferTX[100],ai2cBuffer[10];
int32_t speed;
*/
volatile __IO uint32_t counterTemp,counterTempTT;
LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  Button_init();

  Power_Set(1);


  Telemetry_init();
  MX_I2C2_Init();

  Buzzer_init();
  Led_init();
  IS_Charge_init();

  ADC_L_init();
  ADC_R_init();
  MotorL_init();
  MotorR_init();


  //PID_init(0,900); //pwm limit
  //PID_set_L_costant(0.05,0.01,0.0);
  //PID_set_R_costant(2.0,0.5,0.0);

//DebugPin_init();
  HAL_Delay(350);
  while(IS_Button()) {
    Led_Set(0);
  }

  applcation_init();
  Battery_TASK();

  MX_IWDG_Init();

  Led_Set(1);
  Buzzer_TwoBeep();
  HAL_Delay(350);



  lcd.pcf8574.PCF_I2C_ADDRESS = 7;
	lcd.pcf8574.PCF_I2C_TIMEOUT = 1000;
	lcd.pcf8574.i2c = hi2c2;
	lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
	lcd.type = TYPE0;

	if(LCD_Init(&lcd)!=LCD_OK){
		// error occured
		while(1);
	}

	LCD_ClearDisplay(&lcd);
	LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, "pi:");
	LCD_SetLocation(&lcd, 0, 1);
	LCD_WriteString(&lcd, "e:");

  MotorR_start();
  MotorL_start();
  //MotorR_pwm(80);
  //MotorL_pwm(-200);

  uint32_t sinValue = 45 * 50;
  uint8_t state = 0;
  int lastSpeedL = 0, lastSpeedR = 0;
  while(1){
    sinValue++;
    counterTemp = HAL_GetTick();
    if(IS_Button()) {
      while(IS_Button()) {
        HAL_IWDG_Refresh(&hiwdg);
      }
      Buzzer_OneLongBeep();
      HAL_Delay(350);
      Power_Set(0);
    }
    if ((sinValue) % (500) == 0) {
      uint16_t distance = CLAMP(ADC_PA3() - 175, 0, 4095);
      int16_t steering = ADC_PA2() - 2048;
      int speedL = -CLAMP((distance - 1000) +  CLAMP((steering / 10.0), -50, 50), -800, 800);
      int speedR = -CLAMP((distance - 1000) -  CLAMP((steering / 10.0), -50, 50), -800, 800);
      if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50)) {
        if (distance > 850) {
          MotorL_pwm(speedL);
          MotorR_pwm(speedR);
        } else {
          MotorL_pwm(0);
          MotorR_pwm(0);
        }
      }
      if (distance > 3000) { // Error, robot too far away!
        MotorL_pwm(0);
        MotorR_pwm(0);
        while(1) {
          Power_Set(0);
          HAL_IWDG_Refresh(&hiwdg);
        }
      }

      char str[100];
      memset(&str[0], 0, sizeof(str));
      sprintf(str, "%i;%i\n\r", distance, steering);
      Console_Log(str);


      lastSpeedL = speedL;
      lastSpeedR = speedR;
    }



    Battery_TASK();
    //Current_Motor_TASK();
    //sWiiNunchuck_TASK();
    //applcation_TASK();
    //Telemetry_TASK();

    //Batteria Scarica?
    if(GET_BatteryAverage() < 31.0 || ABS(getMotorCurrentR() * 0.02) > 20.0 || ABS(getMotorCurrentL() * 0.02) > 20.0){
      MotorL_pwm(0);
      MotorR_pwm(0);
      Buzzer_OneLongBeep();
      HAL_Delay(350);
      Power_Set(0);
    }
    //In Carica?
    /*if(IS_Charge()==0){
      WAIT_CHARGE_FINISH();
    }*/

    HAL_IWDG_Refresh(&hiwdg);   //819mS

    counterTempTT = HAL_GetTick() - counterTemp;



  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{
  __HAL_RCC_WWDG_CLK_ENABLE();
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_IWDG_Start(&hiwdg);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if(hadc->Instance == ADC1){
    ADC_R_callback();
  }
  if(hadc->Instance == ADC3){
    ADC_L_callback();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  MotorR_stop();
  MotorL_stop();
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

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
