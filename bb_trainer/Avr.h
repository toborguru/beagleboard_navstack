/*
 * =====================================================================================
 *
 *       Filename:  Avr.h
 *
 *    Description:  Header file containing processor hardware specific macros and 
 *                  functions.
 *
 *        Version:  1.0
 *        Created:  07/22/2010 03:12:18 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Sawyer Larkin (sjl), DATAProject@therobotguy.com
 *        Company:  TheRobotGuy.com
 *
 * =====================================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef _AVR_H_
#define _AVR_H_

// Simplifying PIN, PORT, and DDR Macros
// Intermediate step to avoid compiler error
#define __PORT(port) PORT ## port 
#define __DDR(port)  DDR  ## port 
#define __PIN(port)  PIN  ## port 

// Macro API
#define _PORT(port) __PORT(port) 
#define _DDR(port)  __DDR(port) 
#define _PIN(port)  __PIN(port)

#define INPUT_PIN(port, pin)    BIT_CLEAR(_DDR(port), pin)
#define OUTPUT_PIN(port, pin)   BIT_SET(_DDR(port), pin)

#define DISABLE_PULLUP(port, pin)   BIT_CLEAR(_PORT(port), pin)
#define ENABLE_PULLUP(port, pin)    BIT_SET(_PORT(port), pin)

#define READ_PIN(port, pin)         (( _PIN(port) >> pin ) & 0x01)

#define ENABLE_INTERRUPTS()     sei()
#define DISABLE_INTERRUPTS()    cli()

#endif
