"""
Motor controller serial packet definitions

Packet Structure:
  [ WRITEFLAG || PAYLOAD SIZE(4bit)] [INSTRUCTION BASE] [INSTRUCTION] [PAYLOAD (0-16 bytes)] [CHECKSUM Fletcher 16(2)]

Ex: Write Current Kp value
  0x84 0x05 0x00 0x00 0x00 0x00 0x00 CHECKSUM1 CHECKSUM0
"""

WRITE_MASK     = 0x80   # Write flag mask
PAYLOAD_OFFSET = 3      # Payload offset in packet

TIMEOUT_RESET = 200   # Timeout reset in systick ticks (ms)

ERR_BAD_CHECKSUM       = 0x01   # Bad checksum
ERR_NOT_WRITEABLE      = 0x02   # Field not writeable
ERR_UNDEF_CLASS        = 0x03   # Undefined class
ERR_UNDEF_INSTRUCTION  = 0x04   # Undefined field instruction
ERR_TIMEOUT            = 0x05   # Serial timeout
ERR_INVALID_VALUE      = 0x06   # Invalid value
ERR_PARAM_NOT_EDITABLE = 0x07   #

# Parameter Index code definitions
PARAM_BASE         = 0x01   # System parameter class base
PARAM_MOTOR_PP     = 0x00   # Motor pole pair count
PARAM_MOTOR_CPR    = 0x01   # Motor encoder counts per revolution
PARAM_CONTROL_MODE = 0x05   # Control mode

# System State code definitions
SYSTEM_STATE_BASE = 0x04   # System state class base
SYS_V_SUPPLY      = 0x00   # Main supply voltage
SYS_DRIVE_STATUS  = 0x05   # Drive status
SYS_OUTPUT_ENABLE = 0x06   # Output enable

# Current/Torque code definitions
CURRENT_BASE = 0x05   # Current control loop class base
I_KP         = 0x00   # Current loop proportional gain      [RW,  ]
I_KI         = 0x01   # Current loop integral gain          [RW,  ]
I_ISAT       = 0x02   # Current loop integral saturation    [RW,  ]
I_IDRAIN     = 0x03   # Current loop integral drain         [RW,  ]
I_LIM_CONT   = 0x04   # Current limit continuous            [RW, A]
I_LIM_PEAK   = 0x05   # Current limit peak                  [RW, A]
I_I2T_LIM    = 0x06   # Current I2T time limit              [RW, s]
I_REF        = 0x07   # Current command reference           [RW, A]
I_MEASURED   = 0x10   # Current measured total              [R , A]

# Velocity code definitions
V_BASE     = 0x06   # Velocity control loop class base
V_KP       = 0x00   # Velocity loop proportional gain      [RW,  ]
V_KI       = 0x01   # Velocity loop integral gain          [RW,  ]
V_KD       = 0x02   # Velocity loop derivative gain        [RW,  ]
V_ISAT     = 0x03   # Velocity loop integral saturation    [RW,  ]
V_IDRAIN   = 0x04   # Velocity loop integral drain         [RW,  ]
V_REF      = 0x05   # Velocity loop reference command      [RW, RPM]
V_LIM_MIN  = 0x06   # Velocity loop minimum limit          [RW, RPM]
V_LIM_MAX  = 0x07   # Velocity loop maximum limit          [RW, RPM]
V_MEASURED = 0x08   # Velocity measured                    [R , RPM]
V_ENCODER  = 0x0A   # Encoder Velocity                     [R , RPM]

# Position code defintions
P_BASE        = 0x07   # Position control loop class base
P_KP          = 0x00   # Position proportional gain         [RW,  ]
P_KI          = 0x01   # Position integral gain             [RW,  ]
P_KD          = 0x02   # Position derivative gain           [RW,  ]
P_ISAT        = 0x03   # Position integral saturation       [RW,  ]
P_IDRAIN      = 0x04   # Position integral drain            [RW,  ]
P_REF         = 0x05   # Position reference                 [RW, Revs]
P_LIM_MIN     = 0x06   # Position limit maximum             [RW, Revs]
P_LIM_MAX     = 0x07   # Position limit minimum             [RW, Revs]
P_MEASURED    = 0x08   # Measured Position                  [RW, Revs]
P_HALL_COUNTS = 0x09   # Hall counts                        [R , counts]
P_QUAD_COUNTS = 0x0A   # Quadrature counts                  [R , counts]
