/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
 * Modified 29 January 2018 by Justin Mattair
 *   for MattairTech boards (www.mattairtech.com)
 *
 * See README.md for documentation and pin mapping information
 */

#ifndef _VARIANT_HDUINO_POSEIDON_
#define _VARIANT_HDUINO_POSEIDON_

/* The definitions here need the MattairTech SAMD core >=1.6.8.
 * The format is different than the stock Arduino SAMD core,
 * which uses ARDUINO_SAMD_VARIANT_COMPLIANCE instead.
 */
#define HDUINO_ARDUINO_SAMD_VARIANT_COMPLIANCE 10608

/*----------------------------------------------------------------------------
 *        Board/Clock Configuration
 *----------------------------------------------------------------------------*/

/* Master clock frequency (also Fcpu frequency). With the D51, this can be
 * either 120000000ul or 48000000ul (selected in the menu). See README.md.
 */
#define VARIANT_MCK                       (F_CPU)

/* If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
 * must also be defined with the external crystal frequency in Hertz.
 */
#define HS_CRYSTAL_FREQUENCY_HERTZ      24000000UL

/* If the PLL is used (CLOCKCONFIG_32768HZ_CRYSTAL, or CLOCKCONFIG_HS_CRYSTAL
 * defined), then PLL_FRACTIONAL_ENABLED can be defined, which will result in
 * a more accurate 48MHz output frequency at the expense of increased jitter.
 */
//#define PLL_FRACTIONAL_ENABLED

/* If both PLL_FAST_STARTUP and CLOCKCONFIG_HS_CRYSTAL are defined, the crystal
 * will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being
 * multiplied by the PLL. This will result in a faster lock time for the PLL,
 * however, it will also result in a less accurate PLL output frequency if the
 * crystal is not divisible (without remainder) by 1MHz. In this case, define
 * PLL_FRACTIONAL_ENABLED as well.
 */
//#define PLL_FAST_STARTUP

/* The fine calibration value for DFLL open-loop mode is defined here.
 * The coarse calibration value is loaded from NVM OTP (factory calibration values).
 */
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (512)

/* Define CORTEX_M_CACHE_ENABLED to enable the Cortex M cache (D51 only).
 */
#define CORTEX_M_CACHE_ENABLED

/* When the battery charger (CHG) is installed, the charge status is available on pin 18 (A18, STA),
 * which drives low (open-drain) when the battery is charging. Defining BATTERY_CHARGER_INSTALLED
 * will prevent pin A18 from being configured as an output, thus avoiding contention.
 */
#define BATTERY_CHARGER_INSTALLED

/* When the accelerometer/gyroscope/magnetometer (IMU) is installed, all three interrupt outputs
 * are tied together and connected to pin 32 (B22, INT), which by default drives low in a push-pull,
 * active-high configuration until it is configured otherwise (to open-drain, active-low for example).
 * Defining IMU_INSTALLED will prevent pin B22 from being configured as an output, thus avoiding
 * contention.
 */
#define IMU_INSTALLED

/* When the Vin regulator (5V) is installed, an analog comparator (Rev B only) is used to drive the Vbus
 * ideal diode enable pin (EN) low, which disconnects Vbus from VccH. The Vbus voltage divider, which is
 * available on pin 6 (A6, VU), is also connected to the same enable pin. Defining VIN_5V_REGULATOR_INSTALLED
 * will prevent pin A6 from being configured as an output, thus avoiding contention.
 */
#define VIN_5V_REGULATOR_INSTALLED


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"
#include "sam.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define NUM_PIN_DESCRIPTION_ENTRIES     (34u)

#define PINS_COUNT           NUM_PIN_DESCRIPTION_ENTRIES
#define NUM_DIGITAL_PINS     PINS_COUNT
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (1u)


#define analogInputToDigitalPin(p)  (p)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( (g_APinDescription[P].ulPinAttribute & PIN_ATTR_TIMER_PWM) == PIN_ATTR_TIMER_PWM )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

/* LEDs
 * None of these defines are currently used by the core.
 * The Xeno Mini onboard LED is on pin 34.
 * The RX and TX LEDs are not present.
 * You may optionally add them to any free pins.
 */
#define PIN_LED1           (25u)
#define PIN_LED2           (26u)
#define LED1               PIN_LED1
#define LED2               PIN_LED2
#define PWE0		       (27u)
#define PWE1		       (19u)
#define PWE2		       (20u)
#define SYN                (12u)
#define PGi                (13u)
#define BATs               (14u)
#define RSTdv              (31u)

#define S1_0		   (0u)
#define S1_1		   (1u)
#define S1_2		   (2u)
#define S1_3		   (3u)
#define S4_0		   (4u)
#define S4_1		   (5u)
#define S4_2		   (6u)
#define S4_3		   (7u)
#define S3_0		   (8u)
#define S3_1		   (9u)
#define S3_2		   (10u)
#define S3_3		   (11u)
#define S0_0		   (20u)
#define S0_1		   (19u)
#define S0_2		   (18u)
#define S0_3		   (17u)

#define SDMO		   (21u)
#define SDSC		   (22u)
#define SDCS		   (23u)
#define SDMI		   (24u)

#define S5_2		   (28u)
#define S5_3		   (29u)
#define CONst		   (30u)

/*
 * Analog pins
 */
#define PIN_A0               (13ul)
#define PIN_A1               (14ul)
#define PIN_A2               (15ul)
#define PIN_A3               (16ul)
#define PIN_DAC0             (13ul)

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

// #define REMAP_ANALOG_PIN_ID(pin)	if ( pin < A0 ) pin += A0

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/* Reference voltage pins (define even if not enabled with PIN_ATTR_REF in the PinDescription table) */
#define REFA_PIN    (14ul)



/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIALW_TX       (28ul)
#define PIN_SERIALW_RX       (29ul)
#define PAD_SERIALW_TX       (UART_TX_PAD_2)
#define PAD_SERIALW_RX       (SERCOM_RX_PAD_3)

#define SERCOM_INSTANCE_SERIALW       &sercom5


/*
 * SPI Interfaces
 */
#if defined(TWO_SPI)
#define SPI_INTERFACES_COUNT 2
#elif defined(THREE_SPI)
#define SPI_INTERFACES_COUNT 3
#else
#define SPI_INTERFACES_COUNT 1
#endif

#define PIN_SPI_MISO         (24u)
#define PIN_SPI_MOSI         (21u)
#define PIN_SPI_SCK          (22u)
#define PIN_SPI_SS           (23u)
#define PERIPH_SPI           sercom2
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3

static const uint8_t SS   = PIN_SPI_SS ;        // The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;



/*
 * Wire Interfaces
 */

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (15u)
#define PIN_WIRE_SCL         (16u)
#define PERIPH_WIRE          sercom0

 #define WIRE_IT_HANDLER      SERCOM0_Handler


static const uint8_t sSDA = PIN_WIRE_SDA;
static const uint8_t sSCL = PIN_WIRE_SCL;


/*
 * USB - Define PIN_USB_HOST_ENABLE to assert the defined pin to
 * PIN_USB_HOST_ENABLE_VALUE during startup. Leave undefined to disable this pin.
 */
#define PIN_USB_DM                      (32ul)
#define PIN_USB_DP                      (33ul)
//#define PIN_USB_HOST_ENABLE             (19ul)
#define PIN_USB_HOST_ENABLE_VALUE	0


#ifdef __cplusplus
}
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart SerialW;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB

// SERIAL_PORT_MONITOR seems to be used only by the USB Host library (as of 1.6.5).
// It normally allows debugging output on the USB programming port, while the USB host uses the USB native port.
// The programming port is connected to a hardware UART through a USB-Serial bridge (EDBG chip) on the Zero.
// Boards that do not have the EDBG chip will have to connect a USB-TTL serial adapter to 'Serial' to get
// the USB Host debugging output.

#define SERIAL_PORT_MONITOR         SerialW


// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        SerialW
#define SERIAL_PORT_HARDWARE_OPEN   SerialW


// The Xeno does not have the EDBG support chip, which provides a USB-UART bridge
// accessible using Serial (the Arduino serial monitor is normally connected to this).
// So, the USB virtual serial port (SerialUSB) must be used to communicate with the host.
// Because most sketches use Serial to print to the monitor, it is aliased to SerialUSB.
// Remember to use while(!Serial); to wait for a connection before Serial printing.

// When USB CDC is enabled, Serial refers to SerialUSB, otherwise it refers to Serial1.
#if defined(CDC_ONLY) || defined(CDC_HID) || defined(WITH_CDC)
#define Serial                      SerialUSB
#else
#define Serial                      SerialW
#endif


#endif /* _VARIANT_HDUINO_POSEIDON_ */