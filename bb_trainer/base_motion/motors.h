#include <inttypes.h>

#define MOTORS_LEFT_INDEX   0
#define MOTORS_RIGHT_INDEX  1

#define MOTORS_L_INDEX      MOTORS_LEFT_INDEX
#define MOTORS_R_INDEX      MOTORS_RIGHT_INDEX

#define MOTORS_BRAKE    0
#define MOTORS_COAST    1
#define MOTORS_FORWARD  2
#define MOTORS_REVERSE  3

void Motors_Init( void );
void Motors_Enable( void );
void Motors_Disable( void );
void Motors_Set_Power( uint8_t motor_index, uint8_t motor_power );
void Motors_Set_Direction ( uint8_t motor_index, uint8_t motor_direction );
