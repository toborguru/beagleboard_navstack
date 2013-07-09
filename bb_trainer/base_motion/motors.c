/* leds.c provides an API for controlling the leds on DATA.
*/

#include <avr/io.h>

#include "../macros.h"
#include "../Avr.h"

#include "motors.h"

/*-----------------------------------------------------------
 * DATA Base movement motor control routines
 *-----------------------------------------------------------*/

// Base motor pin assignments 
// Toshiba TB6612FNG Motor Driver Chip
// NOTE: Channels A + B on the driver chip may have their definitions reversed
// here in order to match the Timer1 compare match definitions for clarity.
#define MOTORS_A_PWM_PIN    1       
#define MOTORS_A_PWM_PORT   B
#define MOTORS_A_POWER      OCR1A

#define MOTORS_B_PWM_PIN    2
#define MOTORS_B_PWM_PORT   B
#define MOTORS_B_POWER      OCR1B

#define MOTORS_A_IN1_PIN    0
#define MOTORS_A_IN1_PORT   B

#define MOTORS_A_IN2_PIN    3
#define MOTORS_A_IN2_PORT   B

#define MOTORS_B_IN1_PIN    4
#define MOTORS_B_IN1_PORT   B

#define MOTORS_B_IN2_PIN    5
#define MOTORS_B_IN2_PORT   B

#define MOTORS_STANDBY_PIN  0
#define MOTORS_STANDBY_PORT C

// MOT - raw direction control
#define MOTORS_A_ON()       BIT_SET(_PORT(MOTORS_A_PWM_PORT), MOTORS_A_PWM_PIN);
#define MOTORS_A_OFF()      BIT_CLEAR(_PORT(MOTORS_A_PWM_PORT), MOTORS_A_PWM_PIN);
#define MOTORS_A_CW()       BIT_CLEAR(_PORT(MOTORS_A_IN2_PORT), MOTORS_A_IN2_PIN);  \
                            BIT_SET(_PORT(MOTORS_A_IN1_PORT), MOTORS_A_IN1_PIN); // clear first for coast
#define MOTORS_A_CCW()      BIT_CLEAR(_PORT(MOTORS_A_IN1_PORT), MOTORS_A_IN1_PIN);  \
                            BIT_SET(_PORT(MOTORS_A_IN2_PORT), MOTORS_A_IN2_PIN); // clear first for coast
#define MOTORS_A_BRAKE()    BIT_SET(_PORT(MOTORS_A_IN1_PORT), MOTORS_A_IN1_PIN);    \
                            BIT_SET(_PORT(MOTORS_A_IN2_PORT), MOTORS_A_IN2_PIN);
#define MOTORS_A_COAST()    BIT_CLEAR(_PORT(MOTORS_A_IN1_PORT), MOTORS_A_IN1_PIN);  \
                            BIT_CLEAR(_PORT(MOTORS_A_IN2_PORT), MOTORS_A_IN2_PIN);

#define MOTORS_B_ON()       BIT_SET(_PORT(MOTORS_B_PWM_PORT), MOTORS_B_PWM_PIN);
#define MOTORS_B_OFF()      BIT_CLEAR(_PORT(MOTORS_B_PWM_PORT), MOTORS_B_PWM_PIN);
#define MOTORS_B_CCW()      BIT_CLEAR(_PORT(MOTORS_B_IN2_PORT), MOTORS_B_IN2_PIN);  \
                            BIT_SET(_PORT(MOTORS_B_IN1_PORT), MOTORS_B_IN1_PIN); // clear first for coast
#define MOTORS_B_CW()       BIT_CLEAR(_PORT(MOTORS_B_IN1_PORT), MOTORS_B_IN1_PIN);  \
                            BIT_SET(_PORT(MOTORS_B_IN2_PORT), MOTORS_B_IN2_PIN); // clear first for coast
#define MOTORS_B_BRAKE()    BIT_SET(_PORT(MOTORS_B_IN1_PORT), MOTORS_B_IN1_PIN);    \
                            BIT_SET(_PORT(MOTORS_B_IN2_PORT), MOTORS_B_IN2_PIN);
#define MOTORS_B_COAST()    BIT_CLEAR(_PORT(MOTORS_B_IN1_PORT), MOTORS_B_IN1_PIN);  \
                            BIT_CLEAR(_PORT(MOTORS_B_IN2_PORT), MOTORS_B_IN2_PIN);

// Base motor interface
#define MOTORS_ESTOP()       MOTORS_A_BRAKE(); MOTORS_B_BRAKE();

#define MOTORS_ENABLE()      BIT_SET(_PORT(MOTORS_STANDBY_PORT), MOTORS_STANDBY_PIN);
#define MOTORS_DISABLE()     BIT_CLEAR(_PORT(MOTORS_STANDBY_PORT), MOTORS_STANDBY_PIN);

// Helpful substitutions for Left and Right MOTORS_
#define MOTORS_L_FORWARD()  MOTORS_B_CCW()
#define MOTORS_L_REVERSE()  MOTORS_B_CW()
#define MOTORS_L_BRAKE()    MOTORS_B_BRAKE()
#define MOTORS_L_COAST()    MOTORS_B_COAST() 

#define MOTORS_L_ON()       MOTORS_B_ON()
#define MOTORS_L_OFF()      MOTORS_B_OFF()

#define MOTORS_R_FORWARD()  MOTORS_A_CW()
#define MOTORS_R_REVERSE()  MOTORS_A_CCW()
#define MOTORS_R_BRAKE()    MOTORS_A_BRAKE()
#define MOTORS_R_COAST()    MOTORS_A_COAST() 

#define MOTORS_R_ON()       MOTORS_A_ON()
#define MOTORS_R_OFF()      MOTORS_A_OFF()

#define MOTORS_L_POWER  MOTORS_B_POWER
#define MOTORS_R_POWER  MOTORS_A_POWER

void Motors_Init( void )
{
    OUTPUT_PIN(MOTORS_STANDBY_PORT, MOTORS_STANDBY_PIN);
    OUTPUT_PIN(MOTORS_A_IN1_PORT, MOTORS_A_IN1_PIN);
    OUTPUT_PIN(MOTORS_A_IN2_PORT, MOTORS_A_IN2_PIN);
    OUTPUT_PIN(MOTORS_B_IN1_PORT, MOTORS_B_IN1_PIN);
    OUTPUT_PIN(MOTORS_B_IN2_PORT, MOTORS_B_IN2_PIN);

    OUTPUT_PIN(MOTORS_A_PWM_PORT, MOTORS_A_PWM_PIN);
    OUTPUT_PIN(MOTORS_B_PWM_PORT, MOTORS_B_PWM_PIN);

    // Setup Timer: Timer 1, /1 Prescale, 8-bit phase correct PWM
    TCCR1A = BIT(COM1A1) |
             BIT(COM1B1) |
             BIT(WGM10);
              
    TCCR1B = BIT(CS10);

    OCR1A = 0;
    OCR1B = 0;

    BIT_CLEAR( PRR, PRTIM1 );

    Motors_Set_Direction(MOTORS_L_INDEX, MOTORS_BRAKE);
    Motors_Set_Direction(MOTORS_R_INDEX, MOTORS_BRAKE);

    return ;
}

void Motors_Enable()
{
    MOTORS_ENABLE();
}

void Motors_Disable()
{
    MOTORS_DISABLE();
}

/** Sets the motor direction for the appropriate motor.
 *
 *  If you try to change motor direction it will switch to motor brake until called again.
 *
 */
void Motors_Set_Direction( uint8_t motor_index, uint8_t motor_direction )
{
    static uint8_t last_direction[2];

    if ((MOTORS_L_INDEX == motor_index) && (last_direction[MOTORS_L_INDEX] != motor_direction))
    {
        if ((MOTORS_BRAKE == last_direction[MOTORS_L_INDEX]) ||
            (MOTORS_COAST == last_direction[MOTORS_L_INDEX]))
        {
            if (MOTORS_FORWARD == motor_direction)
            {
                MOTORS_L_FORWARD();
            }
            else if (MOTORS_REVERSE == motor_direction)
            {
                MOTORS_L_REVERSE();
            }
            else if (MOTORS_COAST == motor_direction)
            {
                MOTORS_L_COAST();
            }
            else if (MOTORS_BRAKE == motor_direction)
            {
                MOTORS_L_BRAKE();
            }

            last_direction[MOTORS_L_INDEX] = motor_direction;
        }
        else
        {
            if (MOTORS_BRAKE == motor_direction)
            {
                MOTORS_L_BRAKE();
                last_direction[MOTORS_L_INDEX] = MOTORS_BRAKE;
            }
            else
            {
                MOTORS_L_COAST();
                last_direction[MOTORS_L_INDEX] = MOTORS_COAST;
            }
        }
    }
    else if ((MOTORS_R_INDEX == motor_index) && (last_direction[MOTORS_R_INDEX] != motor_direction))
    {
        if ((MOTORS_BRAKE == last_direction[MOTORS_R_INDEX]) ||
            (MOTORS_COAST == last_direction[MOTORS_R_INDEX]))
        {
            if (MOTORS_FORWARD == motor_direction)
            {
                MOTORS_R_FORWARD();
            }
            else if (MOTORS_REVERSE == motor_direction)
            {
                MOTORS_R_REVERSE();
            }
            else if (MOTORS_COAST == motor_direction)
            {
                MOTORS_R_COAST();
            }
            else if (MOTORS_BRAKE == motor_direction)
            {
                MOTORS_R_BRAKE();
            }

            last_direction[MOTORS_R_INDEX] = motor_direction;
        }
        else
        {
            if (MOTORS_BRAKE == motor_direction)
            {
                MOTORS_R_BRAKE();
                last_direction[MOTORS_R_INDEX] = MOTORS_BRAKE;
            }
            else
            {
                MOTORS_R_COAST();
                last_direction[MOTORS_R_INDEX] = MOTORS_COAST;
            }
        }
    }
}

/** Sets the PWM power for the appropriate motor.
 */
void Motors_Set_Power( uint8_t motor_index, uint8_t motor_power)
{
    if (MOTORS_L_INDEX == motor_index)
    {
        MOTORS_L_POWER = motor_power;
    }
    else if (MOTORS_R_INDEX == motor_index)
    {
        MOTORS_R_POWER = motor_power;
    }
}
