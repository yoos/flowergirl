#ifndef _SERIAL_DEFINES_H_
#define _SERIAL_DEFINES_H

/* Packet Structure
 * [ WRITEFLAG || PAYLOAD SIZE(4bit)] [INSTRUCTION BASE] [INSTRUCTION] [PAYLOAD (0-16 bytes)] [CHECKSUM Fletcher 16(2)]
 * Ex: Write Current Kp value
 * 0x84 0x05 0x00 0x00 0x00 0x00 0x00 CHECKSUM1 CHECKSUM0
 */

#define WRITE_MASK		0x80	// Write flag mask
#define PAYLOAD_OFFSET	3		// Payload offset in packet

#define TIMEOUT_RESET	200		// Timeout reset in systick ticks (ms)

#define ERR_BAD_CHECKSUM 		0x01	// Bad checksum
#define ERR_NOT_WRITEABLE		0x02	// Field not writeable
#define ERR_UNDEF_CLASS			0x03	// Undefined class
#define ERR_UNDEF_INSTRUCTION	0x04	// Undefined field instruction
#define ERR_TIMEOUT				0x05	// Serial timeout
#define ERR_INVALID_VALUE		0x06	// Invalid value
#define ERR_PARAM_NOT_EDITABLE	0x07	//
/**
 * Parameter Index code definitions
 */
#define PARAM_BASE					0x01	// System parameter class base
#define PARAM_MOTOR_PP			0x00	// Motor pole pair count
#define PARAM_MOTOR_CPR			0x01	// Motor encoder counts per revolution
#define PARAM_CONTROL_MODE		0x05	// Control mode

/**
 * System State code definitions
 */
#define SYSTEM_STATE_BASE		0x04		// System state class base
#define SYS_V_SUPPLY		0x00		// Main supply voltage
#define SYS_DRIVE_STATUS	0x05		// Drive status
#define SYS_OUTPUT_ENABLE	0x06		// Output enable

/**
 * Current/Torque code definitions
 */
#define CURRENT_BASE		0x05			// Current control loop class base
#define	I_KP		0x00				// Current loop proportional gain	[RW,  ]
#define I_KI		0x01				// Current loop integral gain		[RW,  ]
#define I_ISAT		0x02				// Current loop integral saturation	[RW,  ]
#define I_IDRAIN	0x03				// Current loop integral drain		[RW,  ]
#define I_LIM_CONT	0x04				// Current limit continuous			[RW, A]
#define I_LIM_PEAK	0x05				// Current limit peak 	 			[RW, A]
#define I_I2T_LIM	0x06				// Current I2T time limit 			[RW, s]
#define I_REF		0x07				// Current command reference [RW, A]
#define I_MEASURED	0x10				// Current measured total			[R , A]

/**
 * Velocity code definitions
 */
#define V_BASE				0x06			// Velocity control loop class base
#define V_KP		0x00				// Velocity loop proportional gain	[RW,  ]
#define V_KI		0x01				// Velocity loop integral gain		[RW,  ]
#define V_KD		0x02				// Velocity loop derivative gain	[RW,  ]
#define V_ISAT		0x03				// Velocity loop integral saturation[RW,  ]
#define V_IDRAIN	0x04				// Velocity loop integral drain		[RW,  ]
#define V_REF		0x05				// Velocity loop reference command  [RW, RPM]
#define V_LIM_MIN	0x06				// Velocity loop minimum limit		[RW, RPM]
#define V_LIM_MAX	0x07				// Velocity loop maximum limit		[RW, RPM]
#define V_MEASURED	0x08				// Velocity measured				[R , RPM]
#define V_ENCODER	0x0A				// Encoder Velocity					[R , RPM]

/**
 * Position code defintions
 */
#define P_BASE				0x07			// Position control loop class base
#define P_KP		0x00				// Position proportional gain		[RW,  ]
#define P_KI		0x01				// Position integral gain			[RW,  ]
#define P_KD		0x02				// Position derivative gain			[RW,  ]
#define P_ISAT		0x03				// Position integral saturation		[RW,  ]
#define P_IDRAIN	0x04				// Position integral drain			[RW,  ]
#define P_REF		0x05				// Position reference				[RW, Revs]
#define P_LIM_MIN	0x06				// Position limit maximum			[RW, Revs]
#define P_LIM_MAX	0x07				// Position limit minimum			[RW, Revs]
#define P_MEASURED	0x08				// Measured Position				[RW, Revs]
#define P_HALL_COUNTS	0x09			// Hall counts						[R , counts]
#define P_QUAD_COUNTS	0x0A			// Quadrature counts				[R , counts]


#endif