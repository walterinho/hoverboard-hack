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
#include "eeprom.h"
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

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x1337};
uint16_t VarDataTab[NB_OF_VAR] = {0};
uint16_t VarValue = 0;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
//extern struct PID_dati pid_R_;
static void MX_IWDG_Init(void);
IWDG_HandleTypeDef hiwdg;

volatile __IO int16_t speed = 0;
extern struct TELEMETRY_dati telemetry;
extern struct MOTOR_Ldati motorL;
extern struct MOTOR_Rdati motorR;

extern uint8_t LCDerrorFlag;
//extern struct COMMAND_data commandsequence;

volatile __IO uint32_t counterTemp,counterTempTT;
LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
uint16_t saveValue = 0;

TIM_HandleTypeDef htim2;

uint16_t captured_value[8] = {0};
uint16_t rc_data[5] = {0};
uint8_t pointer = 0;
uint8_t data_ready = 0;
/* USER CODE END PV */
uint8_t rx_count = 0;
uint32_t timeout = 0;

void PPM_ISR_Callback() {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_delay = TIM2->CNT;
  _stop_timer();

  if (rc_delay > 3000) {
    rx_count = 0;
  }
  else if (rx_count < 6){
    timeout = 0;
    captured_value[rx_count] = CLAMP(rc_delay, 1000, 2000) - 1000;
    rx_count++;
  }
  _init_us();
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  Button_init();

  Power_Set(1);

  HAL_Delay(10);

  ADC_L_init();
  ADC_R_init();


  //Telemetry_init();
  MX_I2C2_Init();

  /* Unlock the Flash Program Erase controller */
  //HAL_FLASH_Unlock();

  /* EEPROM Init */
  //EE_Init();


  lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
	lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
	lcd.pcf8574.i2c = hi2c2;
	lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
	lcd.type = TYPE0;

	if(LCD_Init(&lcd)!=LCD_OK){
		// error occured
		//TODO while(1);
	}

	LCD_ClearDisplay(&lcd);
  LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, "SesselOtter V5.2");
  LCD_SetLocation(&lcd, 0, 1);
	LCD_WriteString(&lcd, "Initializing...");

  Buzzer_init();
  Led_init();
  IS_Charge_init();

//  Timer_init();
  //Timer_init();
  //MX_TIM2_Init();


  //PID_init(0,900); //pwm limit
  //PID_set_L_costant(0.05,0.01,0.0);
  //PID_set_R_costant(2.0,0.5,0.0);

//DebugPin_init();
  //HAL_Delay(350);
  while(IS_Button()) {
    Led_Set(0);
  }


  MotorL_init();
  MotorR_init();

  applcation_init();
  Battery_TASK();

  MX_IWDG_Init();

  Led_Set(1);
  Buzzer_TwoBeep();
  HAL_Delay(250);

  MotorR_start();
  MotorL_start();
  //MotorR_pwm(80);
  //MotorL_pwm(-200);

  Timer_init();

  //MotorR_pwm(200);
  //MotorL_pwm(-150);

  //MotorR_pwm(-50);
  //MotorL_pwm(50);
  int16_t speedR = 0;
  int16_t speedL = 0;

  uint8_t state = 0;

  if(!LCDerrorFlag) {

    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd, 0, 1);
  	LCD_WriteString(&lcd, "Bat:");
    LCD_SetLocation(&lcd, 8, 1);
    LCD_WriteString(&lcd, "V");

    LCD_SetLocation(&lcd, 15, 1);
    LCD_WriteString(&lcd, "A");

    LCD_SetLocation(&lcd, 0, 0);
  	LCD_WriteString(&lcd, "Speed:");
    LCD_SetLocation(&lcd, 12, 0);
    LCD_WriteString(&lcd, "km/h");
  }

  uint32_t sinValue = 1999;

  int lastSpeedL = 0, lastSpeedR = 0;
  int32_t lastMotorposR = 0, lastMotorposL = 0;
  float speedReading = 0.0;
  while(1){
    sinValue++;
    if ((sinValue) % (100) == 0) {
      //state = !state;
      //Led_Set(state);
      //Console_Log("otter!\n\r");
      //char str[200];
      //memset(&str[0], 0, sizeof(str));
      //sprintf(str, "%i;%i;%i;%i;%i;%i\n\r", captured_value[0], captured_value[1], captured_value[2], captured_value[3], captured_value[4], captured_value[5]);
      int readR = -(CLAMP((((captured_value[1]-500)-(captured_value[0]-500)/2.0)*(captured_value[2]/500.0)), -1000, 1000));
      int readL = -(CLAMP((((captured_value[1]-500)+(captured_value[0]-500)/2.0)*(captured_value[2]/500.0)), -1000, 1000));

      int16_t tempL = speedL;
      speedL -=  tempL / 1.0;
      speedL += readL / 1.0;

      int16_t tempR = speedR;
      speedR -=  tempR / 1.0;
      speedR += readR / 1.0;


      if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < 1000) {
        float scale = get_powerMax(GET_BatteryAverage());
        MotorR_pwm((int)(speedR * scale));
        MotorL_pwm((int)(speedL * scale));
      }
      lastSpeedL = speedL;
      lastSpeedR = speedR;
      //MotorR_pwm(-250);
      //MotorL_pwm(250);
      //Console_Log(str);
    }
    timeout++;

    if (timeout > 1000) {
      MotorR_pwm(0);
      MotorL_pwm(0);
    }


    if (counterTemp + 500 < HAL_GetTick()) {
      speedReading = MAX(ABS(((motorL.motorpos - lastMotorposL) / 90.0)*3.6), ABS(((motorR.motorpos - lastMotorposR) / 90.0)*3.6));
      counterTemp = HAL_GetTick();
      lastMotorposL = motorL.motorpos;
      lastMotorposR = motorR.motorpos;
    }

    if(IS_Button()) {
      MotorL_pwm(0);
      MotorR_pwm(0);
      while(IS_Button()) {
        HAL_IWDG_Refresh(&hiwdg);
      }
      Buzzer_OneLongBeep();
      HAL_Delay(350);
      Power_Set(0);
    }

    if ((sinValue) % (1500) == 0 && !LCDerrorFlag) {
      //LCD_SetLocation(&lcd, 4, 0);
      //LCD_WriteFloat(&lcd,distance/1345.0,2);
      if (speedReading < 10.0) {
        LCD_SetLocation(&lcd, 7, 0);
        LCD_WriteString(&lcd, " ");
        LCD_SetLocation(&lcd, 8, 0);
      } else {
        LCD_SetLocation(&lcd, 7, 0);
      }
      LCD_WriteFloat(&lcd,speedReading,2);
      LCD_SetLocation(&lcd, 4, 1);
      LCD_WriteFloat(&lcd,GET_BatteryAverage(),1);
      float current = ABS(getMotorCurrentR() * 0.02) + ABS(getMotorCurrentL() * 0.02);
      if (current < 10.0) {
        LCD_SetLocation(&lcd, 10, 1);
        LCD_WriteString(&lcd, " ");
        LCD_SetLocation(&lcd, 11, 1);
      } else {
        LCD_SetLocation(&lcd, 10, 1);
      }
      LCD_WriteFloat(&lcd,ABS(current),2);
      LCD_SetLocation(&lcd, 15, 1);
      LCD_WriteString(&lcd, "A");
    }





    Battery_TASK();
    //Current_Motor_TASK();
    //sWiiNunchuck_TASK();
    //applcation_TASK();
    //Telemetry_TASK();

    //Batteria Scarica?

    if(ABS(getMotorCurrentR() * 0.02) > 47.0 || ABS(getMotorCurrentL() * 0.02) > 47.0){
      MotorL_pwm(0);
      MotorR_pwm(0);
      Buzzer_OneLongBeep();
      LCD_ClearDisplay(&lcd);
      HAL_Delay(5);
      LCD_SetLocation(&lcd, 0, 0);
      LCD_WriteString(&lcd, "Emergency Off!");
      LCD_SetLocation(&lcd, 0, 1);
      LCD_WriteString(&lcd, "Overcurrent.");
      HAL_Delay(500);
      HAL_IWDG_Refresh(&hiwdg);
      HAL_Delay(500);
      Power_Set(0);
    }

    if(GET_BatteryAverage() < 31.0){
      MotorL_pwm(0);
      MotorR_pwm(0);
      Buzzer_OneLongBeep();
      LCD_ClearDisplay(&lcd);
      HAL_Delay(5);
      LCD_SetLocation(&lcd, 0, 0);
      LCD_WriteString(&lcd, "Emergency Off!");
      LCD_SetLocation(&lcd, 0, 1);
      LCD_WriteString(&lcd, "Battery low.");
      HAL_Delay(500);
      HAL_IWDG_Refresh(&hiwdg);
      HAL_Delay(500);
      Power_Set(0);
    }


    HAL_IWDG_Refresh(&hiwdg);   //819mS

    //counterTempTT = HAL_GetTick() - counterTemp;



  }

}

void saveConfig() {
  EE_WriteVariable(VirtAddVarTab[0], saveValue);
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
