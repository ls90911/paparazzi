/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_dualpwm_arch.c
 *  STM32 dual PWM servos handling.
 */

//VALID TIMERS IS TIM5 ON THE LISA/M

#include "subsystems/actuators/actuators_shared_arch.h"
#include "subsystems/actuators/actuators_dshot_arch.h"
#include "subsystems/actuators/actuators_dshot.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio_arch.h"

#define DSHOT_HZ  100000

// 1MHz / 150khz = 6.666

#define MOTOR_BIT_0  3
#define MOTOR_BIT_1  7



int32_t actuators_dshot_values[ACTUATORS_DSHOT_NB];



typedef struct
{
  // Userspace variable
  uint8_t requestTelemetry;

  // Multithread variable
  volatile uint16_t dshotcode;

  // ISR variables
  uint8_t progress;
  uint16_t buff[16];
} actuator_dshot_motor;


static actuator_dshot_motor actuator_dshot_motors[4];


static void loadDmaBufferDshot(actuator_dshot_motor *const motor);
uint16_t prepareDshotPacket(actuator_dshot_motor *const motor, const uint16_t value);

void actuators_dshot_arch_init(void) {

  /*-----------------------------------
   * Configure timer peripheral clocks
   *-----------------------------------*/
#if PWM_USE_TIM1
  rcc_periph_clock_enable(RCC_TIM1);
#endif
#if PWM_USE_TIM2
  rcc_periph_clock_enable(RCC_TIM2);
#endif
#if PWM_USE_TIM3
  rcc_periph_clock_enable(RCC_TIM3);
#endif
#if PWM_USE_TIM4
  rcc_periph_clock_enable(RCC_TIM4);
#endif
#if PWM_USE_TIM5
  rcc_periph_clock_enable(RCC_TIM5);
#endif
#if PWM_USE_TIM8
  rcc_periph_clock_enable(RCC_TIM8);
#endif
#if PWM_USE_TIM9
  rcc_periph_clock_enable(RCC_TIM9);
#endif
#if PWM_USE_TIM12
  rcc_periph_clock_enable(RCC_TIM12);
#endif

  /*----------------
   * Configure GPIO
   *----------------*/
#ifdef PWM_SERVO_0
  gpio_setup_pin_af(PWM_SERVO_0_GPIO, PWM_SERVO_0_PIN, PWM_SERVO_0_AF, TRUE);
#endif
#ifdef PWM_SERVO_1
  gpio_setup_pin_af(PWM_SERVO_1_GPIO, PWM_SERVO_1_PIN, PWM_SERVO_1_AF, TRUE);
#endif
#ifdef PWM_SERVO_2
  gpio_setup_pin_af(PWM_SERVO_2_GPIO, PWM_SERVO_2_PIN, PWM_SERVO_2_AF, TRUE);
#endif
#ifdef PWM_SERVO_3
  gpio_setup_pin_af(PWM_SERVO_3_GPIO, PWM_SERVO_3_PIN, PWM_SERVO_3_AF, TRUE);
#endif
#ifdef PWM_SERVO_4
  gpio_setup_pin_af(PWM_SERVO_4_GPIO, PWM_SERVO_4_PIN, PWM_SERVO_4_AF, TRUE);
#endif
#ifdef PWM_SERVO_5
  gpio_setup_pin_af(PWM_SERVO_5_GPIO, PWM_SERVO_5_PIN, PWM_SERVO_5_AF, TRUE);
#endif
#ifdef PWM_SERVO_6
  gpio_setup_pin_af(PWM_SERVO_6_GPIO, PWM_SERVO_6_PIN, PWM_SERVO_6_AF, TRUE);
#endif
#ifdef PWM_SERVO_7
  gpio_setup_pin_af(PWM_SERVO_7_GPIO, PWM_SERVO_7_PIN, PWM_SERVO_7_AF, TRUE);
#endif
#ifdef PWM_SERVO_8
  gpio_setup_pin_af(PWM_SERVO_8_GPIO, PWM_SERVO_8_PIN, PWM_SERVO_8_AF, TRUE);
#endif

#if PWM_USE_TIM1
  set_servo_timer(TIM1, DSHOT_HZ, PWM_TIM1_CHAN_MASK);
#endif

#if PWM_USE_TIM2
  set_servo_timer(TIM2, DSHOT_HZ, PWM_TIM2_CHAN_MASK);
#endif

#if PWM_USE_TIM3
  set_servo_timer(TIM3, DSHOT_HZ, PWM_TIM3_CHAN_MASK);
#endif

#if PWM_USE_TIM4
  set_servo_timer(TIM4, DSHOT_HZ, PWM_TIM4_CHAN_MASK);
#endif

#if PWM_USE_TIM5
  set_servo_timer(TIM5, DSHOT_HZ, PWM_TIM5_CHAN_MASK);
#endif

#if PWM_USE_TIM8
  set_servo_timer(TIM8, DSHOT_HZ, PWM_TIM8_CHAN_MASK);
#endif

#if PWM_USE_TIM9
  set_servo_timer(TIM9, DSHOT_HZ, PWM_TIM9_CHAN_MASK);
#endif

#if PWM_USE_TIM12
  set_servo_timer(TIM12, DSHOT_HZ, PWM_TIM12_CHAN_MASK);
#endif


#if PWM_USE_TIM4
#warning YOU_SELECTED_A_LISA_S
  nvic_set_priority(NVIC_TIM4_IRQ, 2);
  nvic_enable_irq(NVIC_TIM4_IRQ);
  timer_enable_irq(TIM4, TIM_DIER_CC1IE);
#endif

#if PWM_USE_TIM3
#warning YOU_SELECTED_A_LISA_MX_S
  nvic_set_priority(NVIC_TIM3_IRQ, 2);
  nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_enable_irq(TIM3, TIM_DIER_CC1IE);
#endif


  for (int i=0; i<4; i++) {
    actuator_dshot_motors[i].requestTelemetry = 0;
    prepareDshotPacket(&actuator_dshot_motors[i],1000);
    loadDmaBufferDshot(&actuator_dshot_motors[i]);
  }
}


static void loadDmaBufferDshot(actuator_dshot_motor *const motor)
{
  uint16_t packet = motor->dshotcode;
  for (int i = 0; i < 16; i++) {
    if (packet & 0x8000) {
      motor->buff[i] = MOTOR_BIT_1;  // MSB first
    } else {
      motor->buff[i] = MOTOR_BIT_0;
    }
    packet <<= 1;
  }
}

uint16_t prepareDshotPacket(actuator_dshot_motor *const motor, const uint16_t value)
{
  uint16_t packet = (value << 1) | (motor->requestTelemetry ? 1 : 0);
  motor->requestTelemetry = 0;    // reset telemetry request to make sure it's triggered only once in a row

  // compute checksum
  int csum = 0;
  int csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^=  csum_data;   // xor data by nibbles
    csum_data >>= 4;
  }
  csum &= 0xf;
  // append checksum
  packet = (packet << 4) | csum;

  motor->dshotcode = packet;
  return packet;
}

//void dshot_isr(void);
static inline void dshot_isr(void)
{
//  actuator_dshot_motors[0].progress++;
  static uint8_t status = 0; //actuator_dshot_motors[0].progress;
  status++;

  for (int i=0; i<4; i++) {
    if (status == 32) {
      loadDmaBufferDshot(&actuator_dshot_motors[i]);
    }

    if (status < 16)
    {
#ifdef PWM_SERVO_0
  timer_set_oc_value(PWM_SERVO_0_TIMER, PWM_SERVO_0_OC, actuator_dshot_motors[0].buff[status]);
#endif
#ifdef PWM_SERVO_1
  timer_set_oc_value(PWM_SERVO_1_TIMER, PWM_SERVO_1_OC, actuator_dshot_motors[1].buff[status]);
#endif
#ifdef PWM_SERVO_2
  timer_set_oc_value(PWM_SERVO_2_TIMER, PWM_SERVO_2_OC, actuator_dshot_motors[2].buff[status]);
#endif
#ifdef PWM_SERVO_3
  timer_set_oc_value(PWM_SERVO_3_TIMER, PWM_SERVO_3_OC, actuator_dshot_motors[3].buff[status]);
#endif
    } else {
#ifdef PWM_SERVO_0
  timer_set_oc_value(PWM_SERVO_0_TIMER, PWM_SERVO_0_OC, 0);
#endif
#ifdef PWM_SERVO_1
  timer_set_oc_value(PWM_SERVO_1_TIMER, PWM_SERVO_1_OC, 0);
#endif
#ifdef PWM_SERVO_2
  timer_set_oc_value(PWM_SERVO_2_TIMER, PWM_SERVO_2_OC, 0);
#endif
#ifdef PWM_SERVO_3
  timer_set_oc_value(PWM_SERVO_3_TIMER, PWM_SERVO_3_OC, 0);
#endif
    }
  }
}




#if PWM_USE_TIM4
void tim4_isr(void)
{
  timer_clear_flag(TIM4, TIM_SR_CC1IF);

  dshot_isr();
}
#endif

#if PWM_USE_TIM3
void tim3_isr(void)
{
  timer_clear_flag(TIM3, TIM_SR_CC1IF);

  dshot_isr();
}
#endif

/** Set pulse widths from actuator values, assumed to be in us
 */

void actuators_dshot_commit(void) {
  for (int i = 0; i< 4; i++){
    actuator_dshot_motors[i].requestTelemetry = 0;
    //actuators_dshot_values[i] = 1500;
    prepareDshotPacket(&actuator_dshot_motors[i],  actuators_dshot_values[i]);
  }
}

