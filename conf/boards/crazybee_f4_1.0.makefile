# Hey Emacs, this is a -*- makefile -*-
#
# crazybee_f4_1.0.makefile
#
# Take a look at https://www.openuas.org/  airframes for details

# Board is a crazybee F4 v1.0
BOARD=crazybee_f4
BOARD_VERSION=1.0
BOARD_CFG=\"boards/crazybee_f4_1.0.h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/crazybee_f4_1.0.ld 

HARD_FLOAT=yes

# default flash mode is the BetaFlight bootloader
# possibilities: DFU-UTIL, SWD, JTAG_BMP
#FIXME flash modes todo
FLASH_MODE=DFU-UTIL

#idVendor=0483, idProduct=5740
#USB device strings: Mfr=1, Product=2, SerialNumber=3
#Product: Product: CrazyBee F4 (x)
#Manufacturer: Betaflight
#SerialNumber: 0x8000000

#TIP: ttyACM0: USB ACM device

# default LED configuration but all on same LED gives multi colors...
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (RC receiver, telemetry modem, GPS)
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART1
RADIO_CONTROL_SBUS_PORT   ?= UART1
SBUS_PORT   ?= UART1

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

# GPS via I2C just as Baro and Magneto... no serial ports left on this board
# Exept if one starts using a build in RX on SP then TX1/RX1 can be used.
#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
