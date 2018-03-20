#ifndef _CONFIG_H_
#define _CONFIG_H_

// ramps configuration
#define RAMP_UINCPOW 6
#define RAMP_MAXERROR 1

// motors inputs
#define MOTC_END KZ1
#define MOTC_ENDLEVEL 0
#define MOTC_A KZ0
#define MOTC_B KZ0

#define MOTD_END KZ1
#define MOTD_ENDLEVEL 0
#define MOTD_A KZ0
#define MOTD_B KZ0

// I2C 1: SCL=K2 SDA=K3
#define I2CMASTER_PORT 1

// DMX IN USART1: RX=K6 ; 256 channels
#define DMX_SLAVE_NBCHAN 256

// Dimmer I/Os:
#define DIMMER_INTPIN K9
#define DIMMER_K0 K8
// Adjust TMIN for inductive fan motor: (default 9000)
#define DIMMER_TMIN 15000UL

#endif // _CONFIG_H_

