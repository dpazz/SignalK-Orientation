/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * Copyright 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file board.h
    \brief Board configuration file

    Board hardware specifics, such as pinouts and I2C addresses.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

#include <Arduino.h>
#include "build.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#ifdef ESP8266
// #include "Arduino.h" has definitions for following, but if full Arduino.h is included, then some other defines
// like PI in sensor_fusion.h will clash. Also get errors if Arduino.h is inside extern "C" {} brackets.
    #ifndef HIGH
        #define HIGH (0x01)
    #endif
    #ifndef LOW
        #define LOW (0x00)
    #endif
    #ifndef OUTPUT
        #define OUTPUT (0x01)
    #endif
#endif
#ifdef ESP32
    //#include <Arduino.h> //Can use this instead (which includes the *_hal_gpio), but then some other
    // constants get defined too (like PI) which clash with defines in sensor_fusion.h
  #include <esp32-hal-gpio.h>       //needed for pinMode() etc.
#endif

// Specify the specific sensor IC(s) used 
//#include "sensor_fusion/driver_fxos8700.h"
//#include "sensor_fusion/driver_fxas21002.h"

// Board name and type, passed in packets to Sensor Toolbox.  
// Suspect these fields are only informational. 
#define BOARD_NAME "ESP32 WROOM"
#define THIS_BOARD  9   //impersonates a FRDM_K22F. Sent in packets to PC-based App.
#define THIS_SHIELD 4   //impersonates shield AGMP03. Sent in packets to PC-based App.

// sensor hardware details
#ifndef F_USING_SMARTSENSOR
#define GYRO_FIFO_SIZE  32	///< FXAX21000, FXAS21002 have 32 element FIFO
#define ACCEL_FIFO_SIZE 32	///< FXOS8700 (accel), MMA8652, FXLS8952 all have 32 element FIFO
#endif
#ifdef F_USING_SMARTSENSOR
#define GYRO_FIFO_SIZE  1	///< FXAX21000, FXAS21002 have 32 element FIFO
#define ACCEL_FIFO_SIZE 1	///< FXOS8700 (accel), MMA8652, FXLS8952 all have 32 element FIFO
#endif
#define MAG_FIFO_SIZE 	1	///< FXOS8700 (mag) and MAG3110 have no FIFO so equivalent to 1 element FIFO. For 
//these ICs we save 6 bytes * 31 = 186 bytes of RAM by setting this FIFO size to 1

// Board LED mappings for ESP32 WROVER-KIT
#define LOGIC_LED_ON  1U
#define LOGIC_LED_OFF 0U

#ifndef BOARD_LED_RED_GPIO_PIN
#define BOARD_LED_RED_GPIO_PIN (4)
#endif
#ifndef BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN (2)
#endif
#ifndef BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN (0)
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN BOARD_LED_RED_GPIO_PIN
#endif

// LED-related macros to replace the functions used in status.c
#define LED_RED_INIT(output)   \
    pinMode(BOARD_LED_RED_GPIO_PIN, OUTPUT);  // Enable LED_RED
#define LED_RED_ON()  digitalWrite(BOARD_LED_RED_GPIO_PIN, HIGH); // Turn on LED_RED 
#define LED_RED_OFF() digitalWrite(BOARD_LED_RED_GPIO_PIN, LOW); // Turn off LED_RED 
#define LED_RED_TOGGLE() \
    digitalWrite(BOARD_LED_RED_GPIO_PIN, !digitalRead(BOARD_LED_RED_GPIO_PIN)); // Toggle LED_RED

#define LED_GREEN_INIT(output) \
    pinMode(BOARD_LED_GREEN_GPIO_PIN, OUTPUT);  // Enable LED_GREEN
#define LED_GREEN_ON()  digitalWrite(BOARD_LED_GREEN_GPIO_PIN, HIGH); // Turn on LED_GREEN 
#define LED_GREEN_OFF() digitalWrite(BOARD_LED_GREEN_GPIO_PIN, LOW); // Turn off LED_GREEN 
#define LED_GREEN_TOGGLE() \
    digitalWrite(BOARD_LED_GREEN_GPIO_PIN, !digitalRead(BOARD_LED_GREEN_GPIO_PIN)); // Toggle LED_GREEN

#define LED_BLUE_INIT(output)  \
    pinMode(BOARD_LED_BLUE_GPIO_PIN, OUTPUT);  // Enable LED_BLUE
#define LED_BLUE_ON()  digitalWrite(BOARD_LED_BLUE_GPIO_PIN, HIGH); // Turn on LED_BLUE 
#define LED_BLUE_OFF() digitalWrite(BOARD_LED_BLUE_GPIO_PIN, LOW); // Turn off LED_BLUE 
#define LED_BLUE_TOGGLE() \
    digitalWrite(BOARD_LED_BLUE_GPIO_PIN, !digitalRead(BOARD_LED_BLUE_GPIO_PIN)); // Toggle LED_BLUE

// Functions that have no equivalent and are unneeded
#define CLOCK_EnableClock(x)        //found in status.c
#define PORT_SetPinMux(x,y,z)       //found in status.c 


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
