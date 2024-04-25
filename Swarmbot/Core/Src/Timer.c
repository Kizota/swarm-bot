#include "Timer.h"
#include <stdlib.h>
#include <stdio.h>
#define CLOCK_FREQUENCE 16000000
#define MAX_COUNT_16Bts 65535

#define MAX_RATE 100
#define MIN_RATE 0

uint8_t create_timer(Timer *timer, uint32_t interval) {
	if (timer == NULL) {
		return 0;
	}

	timer->interval = interval;
	timer->tick = 0;
	timer->timeout = 0;

	return 1;
}

uint8_t timing(Timer *timer) {
	if (timer == NULL) {
		return 0;
	}

	if (timer->tick >= timer->interval) {

		timer->timeout = 1;
		timer->tick = 0;

	} else {
		timer->tick++;
	}

	return 1;
}

void SysTick_init() {
	//configure the systick
	SysTick->CTRL = 0; //disable the counter
	SysTick->LOAD = (1 / 1000.0) * CLOCK_FREQUENCE - 1; //1ms

	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	SysTick->VAL = 0; //reset the counter and the flag
	SysTick->CTRL = 0x07; //enable the counter
}

uint8_t set_pulse_width(volatile uint32_t *comp_val, float rate) {
	if (comp_val == NULL) {
		return 0;
	}

	if (rate > MAX_RATE || rate < MIN_RATE) {
		return 0;
	}

	*comp_val = ((float)MAX_COUNT_16Bts * rate) / 100;

	return 1;
}

//send PWM signal to the serval
void Timer_2_init() {
	//part 1: timer general configure
	TIM2->PSC = 4; // 50MHz

	TIM2->ARR = MAX_COUNT_16Bts;
	TIM2->CNT = 0;

	//PA5 -D13 -  channel 1 - Timer 2 - right servo
	//PA1 - A1 - channel 2 - timer 2  - left servo
	//part 2: channel configuration
	//channel 1
	TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_1
			| TIM_CCMR1_OC1M_2; //set PWM 1 mode
	TIM2->CCR1 =(float)MAX_COUNT_16Bts *7.5 /100;
	TIM2->CCER |= TIM_CCER_CC1E; //enable the channel

	//channel 2
	TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC2M) | TIM_CCMR1_OC2M_1
			| TIM_CCMR1_OC2M_2; //set PWM 1 mode
	TIM2->CCR2 = (float)MAX_COUNT_16Bts *7.5 /100;

	TIM2->CCER |= TIM_CCER_CC2E; //enable the channel

	//part 3: alternative function - PA5 - PB3 - AF1
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5) | GPIO_MODER_MODER5_1;
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER1) | GPIO_MODER_MODER1_1;

	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL5)| 0b0001 << GPIO_AFRL_AFRL5_Pos;
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL1)| 0b0001 << GPIO_AFRL_AFRL1_Pos;


	//enable the timer interrupts
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE |TIM_DIER_CC2IE; //updated interrupt & compare channel 2 interrupt
	TIM2->CR1 |= TIM_CR1_CEN; //enable the counter
}



//PB5 - D4 - trigger - ultrasonic
void Timer_3_init() {
	//part 1: timer general configure
	TIM3->PSC = 15; // count for every 1us

	TIM3->ARR = MAX_COUNT_16Bts;
	TIM3->CNT = 0;

	//part 2: output channel 2 configuration
	//auto output mode
	TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC2M) | TIM_CCMR1_OC2M_1
			| TIM_CCMR1_OC2M_2; //set PWM 1 mode
	TIM3->CCR2 = 10; //set pulse width of 10us

	TIM3->CCER |= TIM_CCER_CC2E; //enable the channel
	//part 3: alternative function - PB5
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER5) | GPIO_MODER_MODER5_1; //set alternative function for PB5
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL5)
			| 0b0010 << GPIO_AFRL_AFRL5_Pos; //set AF2 for PB5

	//enable the timer interrupts
	NVIC_EnableIRQ(TIM3_IRQn);

	TIM3->DIER |= TIM_DIER_UIE | TIM_DIER_CC2IE; //updated interrupt & compare channel 2 interrupt
	TIM3->CR1 |= TIM_CR1_CEN; //enable the counter
}

//PB8 - D15 -    echo - ultrasonic
void Timer_4_init() {
	//Part 1:Timer general configure
	TIM4->PSC = 15; //count for every 1 MHz

	TIM4->ARR = MAX_COUNT_16Bts; //set ARR to max for timer 4
	TIM4->CNT = 0;

	//Part 2: input Channel configuration
	//Timer 4 - channel 3
	TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_CC3S) | TIM_CCMR2_CC3S_0; //set Channel as input and map to TI3
	TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_IC3F) | TIM_CCMR2_IC3F_0; //sampling 2 times
	TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_IC3PSC); //set PreScaler at 0

	//enable detector for both edge
	TIM4->CCER |= TIM_CCER_CC3P;
	TIM4->CCER |= TIM_CCER_CC3NP;

	TIM4->CCER |= TIM_CCER_CC3E; //enable the channel

	//Part 3: alternative function configuration - PB8
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER8) | GPIO_MODER_MODER8_1; //set Alternative function - PB8

	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFRH0)
			| 0b0010 << GPIO_AFRH_AFRH0_Pos;  //set AF2 on PB8

	//enable the timer interrupts
	NVIC_EnableIRQ(TIM4_IRQn);

	TIM4->DIER |= TIM_DIER_UIE | TIM_DIER_CC3IE; //only clear interrupt flags which is enable
	TIM4->CR1 |= TIM_CR1_CEN; //enable the counter
}


