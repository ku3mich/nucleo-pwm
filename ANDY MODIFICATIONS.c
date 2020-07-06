  //Reference: 
  //Application note AN4013 - STM32 cross-series timer overview
  //Page 15


  // Here are the DIRECT REGISTERS to control TIM2-Channel2 and TIM3-Channel1 
  // Please add them in your code


  // Registers to create PWM on TIMER2, CHANNEL2 (PB3)

  // OC2PE = enable preload register for CCRx
  // OC2M  = PWM Mode 1, upcounting
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_3 |  TIM_CCMR1_OC2PE ;

  // ENABLE OUPUT COMPARE FOR CHANNEL2
  TIM2->CCER |= TIM_CCER_CC2E ;

  // ARR VALUE FOR FREQUENCY SETTING, 1MHz
  TIM2->ARR = 71 ; 

  // CCR2 VALUE FOR DUTY CYCLE, 50%
  TIM2->CCR2 = 35 ;	

  //TIMER2, SETTING PRESCALER
  TIM2->PSC = 1 ;

  // CHANNEL 2, ENABLE PRELOAD (BUFFER) FOR ARR VALUE
  TIM2->CR1 |= TIM_CR1_ARPE ;

  // CONTROL REGISTER 1, ENABLE TIMER2
  TIM2->CR1 |= TIM_CR1_CEN ;

  

  // Registers to create PWM on TIMER3, CHANNEL1 (PB4)

  TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_3 |  TIM_CCMR2_OC3PE ;

  TIM3->CCER |= TIM_CCER_CC1E ;

  TIM3->ARR = 71 ; 

  TIM3->CCR2 = 35 ; 

  TIM3->PSC = 1 ;

  TIM3->CR1 |= TIM_CR1_ARPE ;

  TIM3->CR1 |= TIM_CR1_CEN ;


  //For ARR and CCR value, what I want is something simple like Arduino Map
  //https://www.arduino.cc/reference/en/language/functions/math/map/
  value = map(value, fromLow, fromHigh, toLow, toHigh)

  //To control TIMER2 ARR and CCR2, we have ADC value at PA4 and PB0 (they run from 0 to 4069)

  //Now, to simplify this project, let's forget about "CCR2 must smaller than ARR"

  //I simply want ARR to run from 18 to 144, CCR2 run from 0 to ARR-1 (17 to 143)

  TIM2->ARR = map(ADC-PA4, 0, 4096, 18, 144) ;
  TIM2->CCR2 = map(ADC-PA4, 0, 4096, 17, 143) ;