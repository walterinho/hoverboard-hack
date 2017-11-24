#ifndef __ADC_R__H
#define __ADC_R__H

#ifdef __cplusplus
 extern "C" {
#endif


   #include "stm32f1xx_hal.h"

   #define ADC_BATTERY_VOLT     0.0276
   #define MOTOR_R_AMP_CONV_AMP 0.01935483870967741935483870967742
   #define ADC_MOTOR_R_CENTER   1900

   //1A = adc 3150

   struct ADC_Rdati{
    volatile __IO uint16_t data[5];
    volatile __IO uint8_t convflag;
   };
// ----------------------PUBLIC----------------------
   void ADC_R_init(void);
// ------------NORMALIZATE----------------
   float GET_BATTERY_VOLT(void);
   float GET_MOTOR_R_AMP(void);
// ------------RAW----------------
   uint16_t ADC_MOTOR_RIGHT(void);
   uint8_t ADC_MOTOR_RIGHT_IS_CONV(void);
   uint16_t ADC_BATTERY(void);
   uint16_t ADC_PA2(void);
   uint16_t ADC_PA3(void);

   uint16_t ADC_PA0(void);
   uint16_t ADC_PA1(void);
   void ADC_R_ResetFlag(void);

// ----------------------PRIVATE----------------------
   void ADC_R_callback(void);

       extern void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
