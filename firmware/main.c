/*********************************************************************
 *
 *                Lyre for 8X2A + 2 VNH
 *
 *********************************************************************
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Antoine Rousseau  aug 28 2015     Original.
                     apr 11 2016     Update to latest Fraise & change mag hmc5883 for incremental sensor + zero
 ********************************************************************/

/*
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
*/
#define BOARD 8X2A

#include <fruit.h>
#include <ramp.h>
#include <dcmotor.h>
#include <i2c_master.h>
#include <PCA9685.h>
#include <dimmer.h>
#include <dmx_slave.h>

#define PWMADDR 0x40

DCMOTOR_DECLARE(C);
DCMOTOR_DECLARE(D);

t_ramp ledRamps[16];
unsigned char DMXchannels[19];

const unsigned int logTab[1024] = {
#include "../logTab2.txt"
};

const unsigned int fanTab[256] = {
#include "../fanTab2.txt"
};

t_delay mainDelay;

unsigned char PERIOD=25;
unsigned char t=25,t2;

// --------------------------------------------------------
// ---------- interrupts
// --------------------------------------------------------
void highInterrupts()
{
	dimmerHighInterrupt();
}

void lowInterrupts()
{
	DMXSlaveISR();
	dimmerLowInterrupt();
}

// --------------------------------------------------------
// ---------- setup
// --------------------------------------------------------
void setup(void)
{
	unsigned char i;
	fruitInit();
	
	dcmotorInit(C);
	dcmotorInit(D);
	
	dimmerInit();        // init dimmer module

// setup I2C master
	i2cm_init(I2C_MASTER, I2C_SLEW_ON, FOSC/400000/4-1);
	
	PCA9685_Init(PWMADDR);
	DMXSlaveInit();

	EEreadMain();
	
	for(i=0; i<16; i++) {
		rampInit(&ledRamps[i]);
		ledRamps[i].maxSpeed = 4000;
		ledRamps[i].maxAccel = 4000;
		ledRamps[i].maxDecel = 4000;
	}
	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms
}

// --------------------------------------------------------
// ---------- DMX receive
// --------------------------------------------------------
void DMXtoParams()
{
	static unsigned char phase = 0;

	if(phase < 16) {
		rampGoto(&ledRamps[phase],((unsigned int)DMXSlaveGet(DMXchannels[phase])) * 4U); // ramp input:{0 - 1023}
	} else if(phase == 16) {
		DCMOTOR(C).Vars.PWMConsign = ((unsigned int)DMXSlaveGet(DMXchannels[16])) * 4U;
	} else if (phase == 17) {
		DCMOTOR(D).Vars.PWMConsign = ((unsigned int)DMXSlaveGet(DMXchannels[17])) * 4U;
	} else if (phase == 18) {
		dimmerSet(0, ((unsigned int)fanTab[DMXSlaveGet(DMXchannels[18])]) * 256U);
	}
	
	phase++;
	if(phase == 19) phase = 0;
}

// --------------------------------------------------------
// ---------- Main loop
// --------------------------------------------------------

void loop() {
	unsigned char i,j;
	int val, val2;
	static unsigned char phase = 0;
	fraiseService();
	dimmerService();	// dimmer management routine
	fraiseService();

	if(delayFinished(mainDelay)) // when mainDelay triggers 
	{
		delayStart(mainDelay, 5000); 	// re-init mainDelay

		DCMOTOR_COMPUTE(C, ASYM);
		DCMOTOR_COMPUTE(D, ASYM);

		DMXtoParams();
		fraiseService();
		DMXtoParams();
		fraiseService();
		
		for(i=0; i<8; i++) {
			if(phase) j = i + 8;
			else j = i;
			rampCompute(&ledRamps[j]);
			val = rampGetPos(&ledRamps[j]);
			if(val < 0) val = 0;
			if(val > 1023) val = 1023;
			if(j == 0) {
				val2 = logTab[val];
			}
			PCA9685_SetPWM(PWMADDR, j, logTab[val]);
		}
		phase = !phase;
		
		if(!t--){
			t=PERIOD;
			t2++;
			//dimmerPrintDebug();
			printf("C L %d\n",val2);
		}
	}
}

// --------------------------------------------------------
// ---------- Fraise receive
// --------------------------------------------------------

void fraiseReceiveChar()
{
	unsigned char c;
	unsigned char l = fraiseGetLen();	
	c=fraiseGetChar();

	if(c=='E') {
		printf("C");
		for(c=1;c<l;c++) printf("%c",fraiseGetChar());
		putchar('\n');
	}
	else if(c=='S') { //EEPROM save
		if((fraiseGetChar() == 'A')
		&& (fraiseGetChar() == 'V')
		&& (fraiseGetChar() == 'E'))
			EEwriteMain();
	}
}


void fraiseReceive()
{
	unsigned char c,c2;
	unsigned int chan;
	static unsigned char buf[22] = { 'B' }; //'B' + 3x4 chars
		
	c=fraiseGetChar();

	switch(c) {
		PARAM_CHAR(1,t2); break;
		PARAM_CHAR(2,PERIOD); break;
		case 10: // get DMX channels
			putchar('B');
			buf[1] = 10;
			for(chan = 0; chan < 19; chan++) buf[chan + 2] = DMXchannels[chan];
			buf[21] = '\n';
			fraiseSend(buf,22);
			break;
		case 11: // set DMX channel
			c2 = fraiseGetChar();
			chan = fraiseGetChar();
			DMXchannels[c2] = chan;
			break;
		case 12: // get DMX values
			putchar('B');
			buf[1] = 12;
			for(chan = 0; chan < 19; chan++) buf[chan + 2] = DMXSlaveGet(DMXchannels[chan]);
			buf[21] = '\n';
			fraiseSend(buf,22);
			break;
		/*case 20: c2 = fraiseGetChar(); // if first byte is 20, then set one of the PWM channels.
			PCA9685_SetPWM(PWMADDR, c2, fraiseGetInt()); 
			break;*/
		case 21: PCA9685_Init(PWMADDR);
		//case 40: dimmerReceive(); break; // if first byte is 40, then call dimmer receive function.
		case 60: chan = fraiseGetInt(); DMXSlaveSet(chan, fraiseGetChar()); break; // set DMX values
		/*case 120 : DCMOTOR_INPUT(C); break;
		case 121 : DCMOTOR_INPUT(D); break;*/
	}
}

// --------------------------------------------------------
// ---------- EEPROM
// --------------------------------------------------------

void EEdeclareMain()
{
	EEdeclareChar(&DMXchannels[0]);
	EEdeclareChar(&DMXchannels[1]);
	EEdeclareChar(&DMXchannels[2]);
	EEdeclareChar(&DMXchannels[3]);
	EEdeclareChar(&DMXchannels[4]);
	EEdeclareChar(&DMXchannels[5]);
	EEdeclareChar(&DMXchannels[6]);
	EEdeclareChar(&DMXchannels[7]);
	EEdeclareChar(&DMXchannels[8]);
	EEdeclareChar(&DMXchannels[9]);
	EEdeclareChar(&DMXchannels[10]);
	EEdeclareChar(&DMXchannels[11]);
	EEdeclareChar(&DMXchannels[12]);
	EEdeclareChar(&DMXchannels[13]);
	EEdeclareChar(&DMXchannels[14]);
	EEdeclareChar(&DMXchannels[15]);
	EEdeclareChar(&DMXchannels[16]);
	EEdeclareChar(&DMXchannels[17]);
	EEdeclareChar(&DMXchannels[18]);
}
