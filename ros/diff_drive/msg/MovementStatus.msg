# Motors State Enum Types
uint8 CORRECT = 0   # Robot is tracking commands accurately
uint8 SLOW = 1      # Robot is maintaining course but unable to keep up 
uint8 STALLED = 2   # Robots wheels are not spinning
uint8 STASIS  = 3   # Robots wheels are spinning and the stasis wheel is not

uint8   motors_state          # As listed above
float32 linear_velocity       # m/s
float32 stasis_wheel_velocity # m/s
