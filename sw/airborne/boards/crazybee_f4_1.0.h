
/* https://github.com/betaflight/betaflight/blob/628fdb8adc85314121bdc0336f15958eff2c80f4/src/main/target/MATEKF411RX/target.h
https://github.com/betaflight/betaflight/blob/master/docs/boards/Board%20-%20CrazyBeeF4FRPro.md

Resource mapping
Label   Pin     Timer   DMA     Default     Note
MPU6000_INT_EXTI    PA1
MPU6000_CS_PIN  PA4                 SPI1
MPU6000_SCK_PIN     PA5                 SPI1
MPU6000_MISO_PIN    PA6                 SPI1
MPU6000_MOSI_PIN    PA7                 SPI1
OSD_CS_PIN  PB12                SPI2
OSD_SCK_PIN     PB13                SPI2
OSD_MISO_PIN    PB14                SPI2
OSD_MOSI_PIN    PB15                SPI2
OK PWM1    PB8     TIM2, CH3
OK PWM2    PB9     TIM4, CH1
OK PWM3    PA3     TIM4, CH2
OK PWM4    PA2     TIM4, CH3
OK VBAT_ADC_PIN    PB0                 ADC1
OK CURRENT_ADC_PIN     PB1                 ADC1

OK BEEPER  PC15

OK UART1 TX    PA9
OK UART1 RX    PA10
OK UART2 TX    PA2
OK UART2 RX    PA3                 connect to built-in DSMX receiver


# resources
OK resource BEEPER 1 C15
resource MOTOR 1 B10
resource MOTOR 2 B06
resource MOTOR 3 B07
resource MOTOR 4 B08

resource PPM 1 A03
resource PWM 1 A02
resource PWM 2 A09
resource PWM 3 A10

resource LED_STRIP 1 A00

OK resource SERIAL_TX 1 A09
OK resource SERIAL_TX 2 A02

OK resource SERIAL_RX 1 A10
OK resource SERIAL_RX 2 A03

OK resource LED 1 C13

resource RX_BIND_PLUG 1 B02

resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SCK 3 B03
resource SPI_MISO 1 A06
resource SPI_MISO 2 B14
resource SPI_MISO 3 B04
resource SPI_MOSI 1 A07
resource SPI_MOSI 2 B15
resource SPI_MOSI 3 B05
resource ADC_BATT 1 B00
resource ADC_CURR 1 B01
resource OSD_CS 1 B12
resource SPI_PREINIT_IPU 1 A04
resource SPI_PREINIT_IPU 3 A15
resource SPI_PREINIT_IPU 4 B12
resource RX_SPI_CS 1 A15
resource GYRO_EXTI 1 A01
resource GYRO_CS 1 A04

*/

#ifndef CONFIG_CRAZYBEE_F4_1_0_H
#define CONFIG_CRAZYBEE_F4_1_0_H

#define BOARD_CRAZYBEE_F4_V1

/** Clock config - STM32F4 - STM32F411CEU6 **/
#define EXT_CLK 12000000                         // 8,12,16??Mhz  (TODO: Find out if 16Mhz works)
#define AHB_CLK 100000000                       // 100Mhz

/** LEDs **/
/* Blue LED on flight controller */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)
/* The ?? Power LED in not controllable, not on a MCU pin ???*/

/** UART **/
/* UART1, Rx (A10) Tx (A9), (AHB1 DMA2), inverter select (C0)?? */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9

/* UART2 Connector (A10) Tx (A9), (AHB1 DMA2), inverter select (C0)?? */
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

/* Turn SBUS invert on UART1 (C0) */
//#define RC_POLARITY_GPIO_PORT GPIOC
//#define RC_POLARITY_GPIO_PIN GPIO0

/*
//TODO: What is this for resetting??
#define RADIO_CONTROL_POWER_PORT GPIOA
#define RADIO_CONTROL_POWER_PIN GPIO10
#define RADIO_CONTROL_POWER_ON gpio_clear // yes, inverted
#define RADIO_CONTROL_POWER_OFF gpio_set
*/

/* Soft binding Spektrum */
/*
//TODO: Do we need this?
#define SPEKTRUM_UART2_RCC RCC_USART1
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO10
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART2_ISR usart1_isr
#define SPEKTRUM_UART2_DEV USART1
*/

/*
// TODO: what is this? 3.3v pin?
#define PERIPHERAL3V3_ENABLE_PORT GPIOC //VDD_3V3_PERIPHERAL_EN
#define PERIPHERAL3V3_ENABLE_PIN GPIO5
#define PERIPHERAL3V3_ENABLE_ON gpio_set
#define PERIPHERAL3V3_ENABLE_OFF gpio_clear
*/

/* PPM
 *
 * Default is PPM config 2, input on GPIOA1 (Servo pin 6)
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 2
#endif

#if PPM_CONFIG == 1
/* input on PA10 (UART1_RX) */
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC3IE
#define PPM_CC_IF           TIM_SR_CC3IF
#define PPM_GPMotorIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         GPIO_AF1

#elif PPM_CONFIG == 2
/* input on PA01 (Servo 6 pin) */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO3
#define PPM_GPIO_AF         GPIO_AF1

// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif
#define USE_AD_TIM1 1

#else
#error "Unknown PPM config"

#endif // PPM_CONFIG




/** SPI **/
/* SPI1 for MPU accel/gyro (MPU6000*/
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

/* SPI2 for OSD */
//#define SPI3_GPIO_AF GPIO_AF5
//#define SPI3_GPIO_PORT_MISO GPIOC
//#define SPI3_GPIO_MISO GPIO11
//#define SPI3_GPIO_PORT_MOSI GPIOC
//#define SPI3_GPIO_MOSI GPIO12
//#define SPI3_GPIO_PORT_SCK GPIOC
//#define SPI3_GPIO_SCK GPIO10
//#define SPI3_GPIO_PORT_NSS GPIOB
//#define SPI3_GPIO_NSS GPIO3

/* SPI slave pin declaration */ //??MPU6000_INT_EXTI    PA1
/*  ACC_GYRO_CS on SPI1 ICM 20609-G*/
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO1
/* OSD on SPI2 */
//#define SPI_SELECT_SLAVE1_PORT GPIOB
//#define SPI_SELECT_SLAVE1_PIN GPIO3

/** Onboard ADCs **/
#define USE_AD_TIM3 1//??
/* Current (C1) */
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 11
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO1
#endif
/* Voltage (C2) */
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL 12
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOB
#define ADC_2_GPIO_PIN GPIO0
#endif

#define ADC_CHANNEL_CURRENT ADC_1
#define ADC_CHANNEL_VSUPPLY ADC_2
#define DefaultVoltageOfAdc(adc) (0.01164*adc)                                    // TODO: Calibrate better (multimeter says 3.80, PPRZ says 3.8/3.9)
#define MilliAmpereOfAdc(adc)((float)adc) * (3.3f / 4096.0f) * (90.0f / 5.0f)     // TODO: Calibrate

/* I2C mapping  TODO: What is this?*/
#define I2C1_GPIO_AF GPIO_AF4
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* PWM */
#define PWM_USE_TIM2 1
#define PWM_USE_TIM4 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1

/* ESC or Servo 1 (B8, TIM2, CH3)*/
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO8
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<3)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

/* ESC or Servo 2 (B9, TIM4, CH1/)*/
#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO9
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC1
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

/* ESC or Servo 3 (A3, TIM4, CH2) */
#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC2
#define PWM_SERVO_3_OC_BIT (1<<1)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

/* ESC or Servo 4 (A2, TIM4, CH3) */
#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO2
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC3
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#define PWM_TIM2_CHAN_MASK (PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT)

/* Buzzer (B4, inverted) */
#if USE_BUZZER
#define PWM_BUZZER
#define PWM_BUZZER_GPIO GPIOC
#define PWM_BUZZER_PIN GPI15
#define PWM_BUZZER_AF GPIO_AF1
#define PWM_BUZZER_GPIO_ON  gpio_clear
#define PWM_BUZZER_GPIO_OFF gpio_set
#endif

#endif /* CONFIG_FLIP32_F4_1_0_H */
