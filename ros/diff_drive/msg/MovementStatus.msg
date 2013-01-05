# Motors State Enum Types
uint8 CORRECT = 0           # Robots stasis wheel is spinning at the same speed as drive wheels
uint8 STALLED = 1           # Robots wheels are not spinning
uint8 STASIS  = 2           # Robots wheels are spinning and the stasis wheel is not
uint8 FREE_WHEELING = 3     # Stasis wheel is spinning faster then robot wheels
uint8 SETUP_ERROR = 255     # Problem with node setup

uint8   motors_state            # As listed above
float32 linear_velocity         # m/s
float32 linear_velocity_average # m/s
float32 stasis_velocity         # m/s
float32 stasis_velocity_average # m/s
bool    stasis_wheel_enabled    # 
