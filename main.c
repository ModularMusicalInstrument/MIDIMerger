//////////////////////////////////////////////////////////////////////
//
// main.c
//
// MIDI Merger (V2.00) using 18F4320
//
// Initial design provides for 4 channels of MIDI IN
// routed to one MIDI OUT, plus indication on 16 LED's
// of currently active MIDi channels
//
// (C) 2010 J.W.Brown & connectable.org.uk
//
// Based on my earlier work of seperate Merger and indicators on http://connectable.org.uk
// Check out also: http://joebrown.org.uk for new hardware/software designs
// Check out : http://picprojects.info for in-depth discussion of PIC code.
//
// First release: JWB 25th June 2010
//
// Facilities: autonomous 4-socket MIDI input Merger
//				1 X Hardware UART RX
//				3 X Start-bit triggered Software UART
//
//			   autonomous MIDI Output
//				1 X Hard UART TX
//
//             MIDI Channel indicator on 16 LEDs.
//             User Interface Key matrix of 16 keys, plus 1 processor RESET key.
//             Supporting: (to date) MIDI channel re-assign
//						    MIDI minimum volme on selected channel(s)
//							Removal of selected MIDI commands (filter)
//							(for example pitch-wheel storms!) 
//	
//
// Release 2: JWB 9th August 2010
//
// Added Example EDIT Routines as follows:
//				
//				Program Change:   				EDIT 2 <channel> <program number> 
//              SYSEX XG Mode ON: 				EDIT 0 (no parameters)
//              Bank select ('normal') voices: 	EDIT 1 <channel> <bank>
//
//				(see code 'DoEditRoutines' for details of these routines)
//                
///////////////////////////////////////////////////////////////////////

//
// For the implementation as presented here, a PIC18F4320 is perfectly adequate
// However the unit has great potential for expansion, and if you intend this
// then use a PIC18F4620 (or similar) with much more memory.
// I used the 18F4320 because Crownhill were selling these off for a quid (£1.00)
// apiece - and no, they have none left as I bought their remaining stock!
//
// Changing PIC (to 18F4620) will mean you need to alter the names of some of the config items
// otherwise no other changes need be made (you may want to extend addressing of EEPROM routines)
//

#include <p18cxxx.h>

#pragma	config PWRT = OFF
#pragma	config OSC = INTIO1
#pragma	config WDT = OFF
#pragma	config LVP = OFF
#pragma	config BOR = ON
#pragma	config MCLRE = ON
#pragma	config PBAD = DIG
#pragma	config CCP2MX = ON
#pragma	config DEBUG = OFF
#pragma	config STVR = OFF
#pragma	config CP0 = OFF
#pragma	config CP1 = OFF
#pragma	config CP2 = OFF
#pragma	config CP3 = OFF
#pragma	config CPB = OFF
#pragma	config CPD = OFF
#pragma	config WRT0 = OFF
#pragma	config WRT1 = OFF
#pragma	config WRT2 = OFF
#pragma	config WRT3 = OFF
#pragma	config WRTB = OFF
#pragma	config WRTC = OFF
#pragma	config WRTD = OFF
#pragma	config EBTR0 = OFF
#pragma	config EBTR1 = OFF
#pragma	config EBTR2 = OFF
#pragma	config EBTR3 = OFF
#pragma	config EBTRB = OFF


//#include <timers.h>
//#include <plib.h>
#include <delays.h>
#include <usart.h>
#include <stdio.h>

// reminders only
#define FOSC 8000000
#define baudrate 31250

// MIDI buffer size for each channel
#define BUFSIZE 16
// number of channels handled
#define NUM_OF_HANDLERS 4

//////////////////////////////////////
// Timer re-load values
// Note these incorporate tweaks to
// accomodate latency in the interrupt
// handler.
//////////////////////////////////////

//#define TMR0_RELOAD 0x60
//#define TMR0_RELOAD 0x6c
//e0
#define TMR0_RELOAD 0xff

//#define TMR1_RELOAD 0xff70
#define TMR1_RELOAD 0xffe6

#define TMR3_RELOAD 0xff7a

#define TMR2INTCOUNTVAL 200
#define TMR2UIINTCOUNT 50

// comment the following out if no Indicator LED display is needed
// #define DISPLAY_IMPLEMENTED
// comment the following out if no start-up tune needed
// #define PLAY_SIGNATURE
// comment out the following if no key matrix implemented
//#define USER_INPUT_IMPLEMENTED


// ResetTimeout MACRO
// ResetTimeout() is simply an assignment
// of the midi_timeout counter in the main
// program loop with the constant
// value given below.
// !!! TODO check timeout value below
#define ResetTimeout() midi_timeout=0xe00000

// eeprom addresses
	extern unsigned char midi_min_volume_copy;
	extern unsigned char midi_assign_copy;
	extern unsigned char midi_filter_copy;

// default eeprom values are defined in eepromstuff.asm
// structure sto support read/write of int and shortlong
// in eeprom
union ShortLong
{
  short long sl;
  char bt[3];
};

union IntUnion
{
	int i;
	char bt[2];
};

// allocate some resources in ACCESS RAM for speedy access in interrupt routines
// most locations in this section are volatile
#pragma udata access X_access

// MIDI stream buffers
ram near unsigned char buf0[BUFSIZE];
ram near unsigned char buf1[BUFSIZE];
ram near unsigned char buf2[BUFSIZE];
ram near unsigned char buf3[BUFSIZE];

// midi byte bit counters
ram near unsigned char suart0_count;
ram near unsigned char suart1_count;
ram near unsigned char suart2_count;

// midi bytes
ram near unsigned char suart0_in;
ram near unsigned char suart1_in;
ram near unsigned char suart2_in;

// buffer input indices
ram near unsigned char in_index[NUM_OF_HANDLERS];

// current midi byte
ram near unsigned char midi_byte;

// holds a timeout count
ram near unsigned short long midi_timeout;

ram near unsigned int tmr2intcount;
ram near unsigned int tmr2intcountreload;
ram near unsigned char interpret_level;

// RAM variables declared below may take longer to access.
#pragma udata

// pointer to the current handlers MIDI buffer
ram near char* curbuf;

// table of the MIDI handler buffer locations
ram near char* bufptr[] = {buf0, buf1, buf2, buf3};

// array of the midi runstatus for each stream
char midi_runstatus[NUM_OF_HANDLERS];

// array of the indices to next available midi-byte in each buffer
unsigned char out_index[NUM_OF_HANDLERS];

// these arrays are initialised from eeprom now
unsigned char midi_min_volume[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; // minimum volume for each midi channel
unsigned char midi_assign[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

// MIDI filter switch		   0x0a 0x0b 0x0c 0x0d 0x0e 0x0f
unsigned char midi_filter[] = {1,   1,   1,   1,   1,   1}; // default no filters

unsigned char filter;
unsigned char assign_to;

// number of expected bytes in stream
unsigned char expected_midi_bytes;

#if defined (USER_INPUT_IMPLEMENTED)

#define DECIMAL_VAL 1
#define ASSIGN_VAL 2
#define FUNC_VAL 3
#define EDIT_VAL 4
#define CLEAR_VAL 5
#define CANCEL_VAL 6
#define PARAM_VAL 7
#define L_R 0
#define R_L 1

// user interface workspace
unsigned char hexstore;
unsigned char matrixkey;
unsigned char keytype;
unsigned char keyvalue;

#endif
#pragma code


// prototypes
void DisplayChannel (unsigned char result);
unsigned char CMax(unsigned char mb);
void delay_ms(int count);
void delay_us(int count);
void TxMIDI(unsigned char ch);
void retrigger (unsigned char channel);
void DoEditRoutines(void);

#if defined (DISPLAY_IMPLEMENTED)
void SwitchLED(unsigned char lednum, unsigned char onnoff);
#endif

#if defined (USER_INPUT_IMPLEMENTED)
void FinishUI(void);
void StartUI(void);
void LoopTime(void);
void InitKeyMatrix(void);
unsigned char RawKeyScan(void);
unsigned char KeyScan(void);
void InterpretKeyPress(void);
unsigned char TranslateKey(unsigned char key);
void IndicateOnOffRequired(void);
void SwitchLEDsOff(void);
#endif

//eeprom protos
void WriteEEPROM(int address, char value);
void WriteEEPROMShortLong(int address, short long value);
void WriteEEPROMInt(int address, int value);
char ReadEEPROM(int address);
short long ReadEEPROMShortLong(int address);
int ReadEEPROMInt(int address);

// example functions
void AssignMIDIChannel(void);
void ParamMinVolume(void);
void FuncFilter(void);


#if defined(PLAY_SIGNATURE)

void PlaySignature(void);

#endif

////////////////////////////////////////////////
//
// main
//
////////////////////////////////////////////////
void main (void)
{
    // Internal Oscillator set clock to 8 Mhz
    //OSCCONbits.IRCF     =0b111;
    //OSCCONbits.SCS      =0b00;
    //***
    int counter; 
	int handler;
    int counter2;
    int midi_chan;
	unsigned char addr_mv,addr_ma,addr_mf;
    
    
    // TODO: Figure out what these are setting
    // Osscillator settings
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    
    TRISAbits.TRISA0 = 0; // set pin as output

	// main control variables initialised to zero
	for (handler = 0; handler < NUM_OF_HANDLERS; handler++)
	{
		in_index[handler] = 0;
		out_index[handler] = 0;
		midi_runstatus[handler] = 0;
	}

	// set up default min_volume, assign and filter arrays
	// get default values from eeprom
	// 1st get addr (lo) for each array
	addr_mv = (unsigned char)&midi_min_volume_copy;
	addr_ma = (unsigned char)&midi_assign_copy;
	addr_mf = (unsigned char)&midi_filter_copy;

	for (counter = 0; counter < 16; counter++)
	{
		midi_min_volume[counter] = ReadEEPROM(addr_mv + counter);
		midi_assign[counter] = ReadEEPROM(addr_ma + counter);
		if (counter < 6)
		{
			midi_filter[counter] = ReadEEPROM(addr_mf + counter);
		}
	}

	interpret_level = 0; // init recursion blocker

	//
	// Configuration of Analog/Digital Port pins
	// Currently no analog i/p functions are needed
	// so all relevant pins are made digital
	//
	ADCON1=0x0f;
	//
	//////////////////////////////
	// Initialise timers 0,1,2 & 3
	//////////////////////////////
	//
	// TMR0, TMR1 and TMR3 are used as baud-rate clocks
	//
	// a) Timer 0
	// TMR0 clock source = FOSC/4 = 10.00 Mhz
	// If we pre-scale by 1:2, the effective
	// frequency becomes 5.00 Mhz
	// For a baud rate of 31,250hz, we need a further
	// division of 160 (5000000/160 = 31,250)
	// so the value we need to pre-load the TMR0L register
	// is: 256-160 = 96 = 0x60. This will need to be tweaked
	// because of latency issues in the interrupt handler.
	// This, then is the value #defined as TMR0_RELOAD
	//
	// The value needed to initialise TMR0 (T0CON register)
    // is derived as follows (see DS39599C-page 117):
	//
	// En   8/16 CLK0 N/A  PRE  <  prescale  >
	// bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
	//   1    1    0    0    0    0    0    0 
	// = 0xc0
    T0CON = 0xc0;   
	// TMR0 interrupts are used as a baud-clock
	// to ensure reading the incoming serial stream
	// takes place at the correct time, hopefully
	// sampling exactly in the middle of each bit
	// The priority of this interrupt is high
	INTCON2bits.TMR0IP = 1; 
	//
	// Enable and disabling of the TMR0 interrupts is controlled
	// in the high-priority edge-triggered interrupt, and TMR0
	// interrupt routines.
	//
	//////////////////////////////////////////////////////////////
	//
	// b) Timer 1
	// TMR1 clock source is similar to TMR0 above in that it is
	// FOSC/4 = 10.00Mhz. Pre-scaled by 1:2, this is an effective
	// frequency of 5.00Mhz, and we can follow the same strategy
	// as for TMR0 is generating the baud clock, except that TMR1
	// is 16-bit only, and therefore the re-load value needs adjusting
	// accordingly. i.e. 65536-160 = 65376 = 0xff60
	// This will need to be tweaked
	// because of latency issues in the interrupt handler.
	// This, then is the value #defined as TMR1_RELOAD
	//
	// The value needed to initialise TMR1 (T1CON register)
    // is derived as follows (see DS39599C-page 121):
	//
	// RD16 T1RUN <prescale> OSCEn N/A  CKsel T1EN
	// bit7 bit6 bit5 bit4    bit3 bit2 bit1  bit0
	//   1    0    0    1       0    0    0     1
	// = 0x91
	T1CON = 0x91;
	// Interrupt priority for TMR1 is also high (see notes for TMR0 above)
	IPR1bits.TMR1IP = 1;
	//
	///////////////////////////////////////////////////////////////
	//
	// c) Timer 3
	// Timer 3 is very largely the same as Timer 1, so the notes
	// given above for Timer 1 will apply here, and therefore have not been
	// repeated.
	// 
	// The value needed to initialise TMR3 (T3CON register)
    // is derived as follows (see DS39599C-page 129):
	//
	// rd16 CCPX <prescale> CCPX  SYNC Cksel T3EN
	// bit7 bit6 bit5 bit4  bit3  bit2 bit1  bit0
	//  1    0     0    1     0     0    0     1
	// = 0x91
	T3CON = 0x91;
	// Interrupt priority for TMR3 is also high (see notes for TMR0 and 1 above)
	IPR2bits.TMR3IP = 1;
	//
	///////////////////////////////////////////////////////////////
	//
	// d) Timer 2
	// TMR2 is used to blink 'Alive' indicator
	// and provide debounce counter for key matrix (if implemented)
	//
	T2CON = 0x7f; // see DS39599C-page 127 
	IPR1bits.TMR2IP = 0; // low priority interrupt

#if defined (DISPLAY_IMPLEMENTED) || defined(USER_INPUT_IMPLEMENTED)
	PIE1bits.TMR2IE = 1;
	tmr2intcountreload = TMR2INTCOUNTVAL; // initial reload for blink (and debounce)
	tmr2intcount = TMR2INTCOUNTVAL; // initial count for blink (and debounce)
#endif
	
	////////////////////////////////////////////////
	// Hardware USART is used for output of merged
    // MIDI and also for debug purposes
	////////////////////////////////////////////////

	// NOTE: Microchip's OpenUSART code does not
	// cater for the case when a high priority needs
	// to be given to the UART interrupt.
	// So before making the call, intialise high priority
	// by setting the RCIP bit in IPR1
	IPR1bits.RCIP = 1;

	OpenUSART ( USART_TX_INT_OFF &
				USART_RX_INT_ON &     // NOTE Receive interrupts enabled
				USART_ASYNCH_MODE &
				USART_EIGHT_BIT &
				USART_CONT_RX &
				USART_BRGH_HIGH,
				//79 );                 // baudrate set to 31,250
				15 );                   // baudrate set to 31,250, 15 is for 8 Mhz


	// init 'comfort' LED indicator and rest of PortB
	// This output is also used (could-be commented-out debug code)
	// to check the position of data bit sample points (when viewed on 'scope)
	PORTB = 0x00; 
	TRISB = 0x07; // RB7:RB3 outputs, RB2:RB0 inputs

	PORTD = 0;
	TRISD = 0; // PORTD all outputs

	PORTA = 0;
	TRISA = 0xf0; // RA3:RA0 outputs.
    
    /*
    while(1) // debug only
    {
        // RB7 has an IND LED connected, so flash it On
        PORTAbits.RA0 = 1;
        PORTAbits.RA0 = 1;
        delay_ms(1000);
        // debug only - output an 'A' on hardware UART
        //	TxMIDI ('A');
        PORTAbits.RA0 = 0; // IND LED off
        delay_ms(1000);

        // debug only - output a 'B' on hardware UART
        //    TxMIDI ('B');
    }  /**/

	// Rest of PORTA, PORTE, PORTC (other than UART)
	// may be used by key matrix or SPI etc.

#if defined (USER_INPUT_IMPLEMENTED)
	InitKeyMatrix(); // see this routine for Port pins used on key matrix
#endif

	// PORTB pins 0,1 and 2 are used as input lines
	// for the 3 'software' UARTS. The 'edge' interrupts on 
	// these inputs are used to detect the negative transition of
	// the start bit.
	//
	// Init RB0,1,2 interrupts
	INTCONbits.INT0IE = 1;   // INT0 has default high priority

	INTCON3bits.INT1IE = 1; // INT1
	INTCON3bits.INT1IP = 1; // INT1 Priority

	INTCON3bits.INT2IE = 1; //INT2
	INTCON3bits.INT2IP = 1; // INT2 Priority

	// define transition direction for an interrupt
	INTCON2bits.INTEDG0 = 0; // EDG0 - falling edge of startbit
	INTCON2bits.INTEDG1 = 0; // EDG1       "            "
	INTCON2bits.INTEDG2 = 0; // EDG2       "            "

	// last but not least, enable the interrupt system
	RCONbits.IPEN = 1;  // Enable High priority interrupts
	INTCONbits.GIEL = 1; // low priority/peripheral ints
	INTCONbits.GIEH = 1; // enable high priority ints

    /*
    while(1) // debug only
    {
        // RB7 has an IND LED connected, so flash it On
        PORTAbits.RA0 = 1;
        delay_ms(1000);
        // debug only - output an 'A' on hardware UART
        //	TxMIDI ('A');
        PORTAbits.RA0 = 0; // IND LED off
        delay_ms(1000);

        // debug only - output a 'B' on hardware UART
        //	TxMIDI ('B');
    } /**/


#if defined(DISPLAY_IMPLEMENTED)
		SwitchLED(0,0);
        delay_ms(1000);
        SwitchLED(0,1);
		delay_ms(500);
		SwitchLED(0,0);
        delay_ms(500);
        SwitchLED(0,1);
		delay_ms(300);
		SwitchLED(0,0);
        delay_ms(500);
        SwitchLED(0,1);
		delay_ms(100);
		SwitchLED(0,0);
        delay_ms(100);
        SwitchLED(0,1);

#define pausetime 350

	// Have a bit of fun, showing
	// that all display LEDs are working
	for (counter = 0; counter < 16; counter++)
	{
		SwitchLED(counter,1);
		delay_ms(pausetime);
		SwitchLED(counter,0);
	}
	for (counter = 15; counter > 0 ; counter--)
	{
		SwitchLED(counter-1,1);
		delay_ms(pausetime);
		SwitchLED(counter-1,0);
	}
	for (counter = 0, counter2 = 15; counter < 8 ; counter++, counter2--)
	{
		SwitchLED(counter,1);
		SwitchLED(counter2,1);
		delay_ms(pausetime);
		SwitchLED(counter,0);
		SwitchLED(counter2,0);
	}
	for (counter = 8, counter2 = 8; counter > 0 ; counter--, counter2++)
	{
		SwitchLED(counter-1,1);
		SwitchLED(counter2,1);
		delay_ms(pausetime);
		SwitchLED(counter-1,0);
		SwitchLED(counter2,0);
	}
	SwitchLEDsOff();
  
#endif

    
#if defined(PLAY_SIGNATURE)
	while(1) {
        PlaySignature(); // plays a short tune	
    }
#endif


	//////////////////////////////////////
	// program main loop
	// loop, servicing MIDI input streams,
	// and merge these into one output.
	// handle commands from key matrix - if
	// implemented
	//////////////
    ////////////////////////
 
	while(1)
	{
    	// foreground loop services NUM_OF_HANDLERS buffers.
		// In this section all candidates are equal!
		// no distinction need be made between the
		// hardware/software driven source/implementation of the midi streams
		for (handler = 0; handler < NUM_OF_HANDLERS; handler++)
		{
//            SwitchLED(0,0);
//            delay_ms(500);
//            SwitchLED(0,1);

			curbuf = bufptr[handler];
			expected_midi_bytes = 0;
			ResetTimeout();
			ClrWdt(); // clear watchdog timer

			while (midi_timeout != 0) // leave loop if timed-out waiting for an expected byte
			{
				midi_timeout++;
				// is there a midi byte in buffer?
				if (out_index[handler] != in_index[handler])
				{
					// there is a midi byte in buffer
					midi_byte = curbuf[out_index[handler]];
					// rotate index 
					out_index[handler]++;
					if (out_index[handler] == BUFSIZE)
					{
						out_index[handler] = 0; 
					}
					if (midi_byte & 0b10000000) // test for midi command bit
					{

						// it's a midi command
						if (midi_byte >= 0xf8)  // MIDI ?
						{
                           
							TxMIDI(midi_byte);
						}
						else
						{
							// pre-process on original channel before
							// any re-assignment
						

							// at this point re-assign MIDI channel if
							// set up to do so
							midi_chan = midi_assign[(midi_byte & 0x0f)];
							midi_byte = ((midi_byte & 0xf0) | midi_chan);
							midi_runstatus[handler] = midi_byte;
							if (midi_runstatus[handler] >= 0xa0)
							{
								// apply filter to 0x0a - 0xf0
								filter = (midi_byte >> 4) - 10; 
								if (midi_filter[filter] == 1)
								{ 	
									// yeah, let it through
									TxMIDI(midi_byte);
								}
							}
							else
							{
								// not to be filtered, let it through
								TxMIDI(midi_byte);
							}
							expected_midi_bytes = CMax(midi_byte); // calc tail bytes
						}
						ResetTimeout();
					}
					else // not a midi command byte
					{
						// !!! TODO needs looking at
						if (midi_runstatus[handler] == 0xf0) // last was a sysex?
						{
							// sysex
							TxMIDI(midi_byte);
						}
						else if (expected_midi_bytes == 0)
						{
							expected_midi_bytes = CMax(midi_runstatus[handler]);
						    // apply filter to tails of 0xa0 - 0xf0 midi commands
							if (midi_runstatus[handler] >= 0xa0)
							{
								filter = (midi_runstatus[handler] >> 4) - 10;
								if (midi_filter[filter] == 1)
								{ 	
									// yeah, let it through
									TxMIDI(midi_runstatus[handler]); // repeat last command
								}
							}
							else
							{
								TxMIDI(midi_runstatus[handler]); // repeat last command
							}
						}
						expected_midi_bytes--;
						// apply filter to tails of 0xa0 - 0xf0 midi commands
						if (midi_runstatus[handler] >= 0xa0)
						{
							filter = (midi_runstatus[handler] >> 4) - 10; // example: 0xa0 becomes 0, 0xe0 becomes 4
							if (midi_filter[filter] == 1)
							{ 	
								// yeah, let it through
								TxMIDI(midi_byte);
							}
						}
						else
						{
							assign_to = midi_runstatus[handler] & 0x0f;
							if ( ( (midi_runstatus[handler] & 0x90) == 0x90) && expected_midi_bytes == 0) // note ON AND volume byte
							{
								if (midi_min_volume[assign_to] > midi_byte)
								{
									midi_byte = midi_min_volume[assign_to];
								}
							}
							TxMIDI(midi_byte);
						}
						ResetTimeout();
					}
				}
				else if (expected_midi_bytes == 0)  // no byte in buffer and not expecting any?
				{
					midi_timeout = 0; // nothing to do, force exit from loop, and give other handlers a chance
				}
				else // no bytes in buffer, but expecting one to arrive
				{
					ClrWdt(); // clear watchdog and try again
				}
			} // end while

			// check finish status for this handler index
			if (expected_midi_bytes != 0) // unexpected end?
			{
				// loop_timed out, so send MIDI 0xff
				TxMIDI(0xff);
			}
		} // next handler

#if defined (USER_INPUT_IMPLEMENTED)

		// check user interface
		if (RawKeyScan() != 0xff) // has a key been pressed?
		{
			KeyScan(); // matrixkey also has key value
			// debug only 
            TxMIDI(TranslateKey(matrixkey));
			InterpretKeyPress();
		}

#endif

	} // outer while

}

#if defined (DISPLAY_IMPLEMENTED)
     

/////////////////////////////////////////////////
//
// Toggle relevant LED when either a MIDI ON
// or MIDI OFF message is detected
//
/////////////////////////////////////////////////
void DisplayChannel (unsigned char result)
{
	unsigned char midi_status;
	unsigned char channel = result & 0x0f;

#if defined(USER_INPUT_IMPLEMENTED)

	if (interpret_level != 0)
	{
		return; // don't use display, as user requires it
				// strictly not necessary at present
				// but will be needed if UI is allowed to
				// span across whole of main program loop
	}
#endif

	midi_status = result & 0x90;
	if (midi_status == 0x90)   // If midi message = NOTE ON turn on LED corresponding to channel
	{
		switch (channel)
		{
   			case 0: PORTAbits.RA0 = 1;
					break;
			case 1: PORTAbits.RA1 = 1;
					break; 
			case 2: PORTAbits.RA2 = 1;
					break;
			case 3: PORTAbits.RA3 = 1;
					break;
			case 4: PORTBbits.RB3 = 1;
					break;
			case 5: PORTBbits.RB4 = 1;
					break;
			case 6: PORTBbits.RB5 = 1;
					break;
			case 7: PORTBbits.RB6 = 1;
					break;
			case 8: PORTDbits.RD0 = 1;
					break;
			case 9: PORTDbits.RD1 = 1;
					break;
			case 10: PORTDbits.RD2 = 1;
					break;
			case 11: PORTDbits.RD3 = 1;
					break;
			case 12: PORTDbits.RD4 = 1;
					break;
			case 13: PORTDbits.RD5 = 1;
					break;
			case 14: PORTDbits.RD6 = 1;
					break;
			case 15: PORTDbits.RD7 = 1;
					break;
		}
	}
	else  if (midi_status == 0x80) // toggle off
	{
		switch (channel)
		{
   			case 0: PORTAbits.RA0 = 0;
					break;
			case 1: PORTAbits.RA1 = 0;
					break; 
			case 2: PORTAbits.RA2 = 0;
					break;
			case 3: PORTAbits.RA3 = 0;
					break;
			case 4: PORTBbits.RB3 = 0;
					break;
			case 5: PORTBbits.RB4 = 0;
					break;
			case 6: PORTBbits.RB5 = 0;
					break;
			case 7: PORTBbits.RB6 = 0;
					break;
			case 8: PORTDbits.RD0 = 0;
					break;
			case 9: PORTDbits.RD1 = 0;
					break;
			case 10: PORTDbits.RD2 = 0;
					break;
			case 11: PORTDbits.RD3 = 0;
					break;
			case 12: PORTDbits.RD4 = 0;
					break;
			case 13: PORTDbits.RD5 = 0;
					break;
			case 14: PORTDbits.RD6 = 0;
					break;
			case 15: PORTDbits.RD7 = 0;
					break;
		}
	}
}
#endif

//////////////////////////////////////////////////////
//
// calculate tail bytes expected for each command byte
//
//////////////////////////////////////////////////////
unsigned char CMax(unsigned char mb)
{
	switch (mb)
	{
		case 0xf0:
		case 0xf1:
			return 1;
		case 0xf2:
			return 2;
	}
	switch (mb & 0xf0)
	{
		case 0x80:
		case 0x90:
		case 0xa0:
		case 0xb0:
		case 0xe0:
			return 2;
		case 0xc0:
		case 0xd0:
			return 1;
	}
	return 0;
}

///////////////////////////////////////////////////////////
//
// Interrupt Service Routines
//
///////////////////////////////////////////////////////////
void InterruptHandlerHigh (void); // prototype for int handler

// High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{ 
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}

void InterruptHandlerLow (void); // prototype for int handler

#pragma code InterruptVector = 0x18
// low priority interrupt vector
void
InterruptVectorLow (void)
{ 
  _asm
    goto InterruptHandlerLow //jump to interrupt routine
  _endasm
}

/////////////////////////////////////////////
// High priority interrupt routine
// JWB 14th June 2010
// Dont mess with this without good reason !!
// (But, a good reason would be to learn more
// by 'tweaking' it! - Don't forget to inspect
// the disassembly code to view the results
// of your tweaks)
/////////////////////////////////////////////
#pragma code

#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh ()
{
    //===========================================
    // Handlers for Software Uart implementation
	//===========================================

	if ( INTCONbits.INT0IE == 1 && INTCONbits.INT0IF == 1)		// test for RB0 edge interrupt
	{
		// disable further edge interrupts on this pin
		INTCONbits.INT0IE = 0;   // INT0
		// then clear INT0 interrupt flag
		INTCONbits.INT0IF = 0;

		suart0_count = 8; // count of bits to receive
		suart0_in = 0;
        //0xd0
		TMR0L = 0xff; // reload TMR0, plus a half-bit time to move sample point to middle of 1st bit
		INTCONbits.T0IF = 0; // clear TMR0 interrupt flag.
		INTCONbits.T0IE = 1; // enable TMR0 interrupt
	}

	if ( INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1)		// test for RB1 edge interrupt
	{
		// disable further edge interrupts on this pin
		INTCON3bits.INT1IE = 0;   // INT0
		// then clear INT1 interrupt flag
		INTCON3bits.INT1IF = 0;

		suart1_count = 8; // count of bits to receive
		suart1_in = 0;
		TMR1H = TMR1_RELOAD  >> 8;
        TMR1L =  0xff; // reload TMR1, plus a half-bit time to move sample point to middle of 1st bit

		//TMR1L =  0x30; // reload TMR1, plus a half-bit time to move sample point to middle of 1st bit
		PIR1bits.TMR1IF = 0; // clear TMR1 interrupt flag.
		PIE1bits.TMR1IE = 1; // enable TMR1 interrupt

	}

	if (INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1)		// test for RB2 edge interrupt
	{
		// disable further edge interrupts on this pin
		INTCON3bits.INT2IE = 0;   // INT2
		// clear INT2 interrupt flag
		INTCON3bits.INT2IF = 0;

		suart2_count = 8; // count of bits to receive
		suart2_in = 0;    // init rec byte to zero
		TMR3H = TMR3_RELOAD >> 8;
		TMR3L = 0x3a; // reload TMR3, plus a half-bit time to move sample point to middle of 1st bit
		PIR2bits.TMR3IF = 0; // clear TMR3 interrupt flag.
		PIE2bits.TMR3IE = 1; // enable TMR3 interrupt

	}

    // TMR0 interrupt, triggered by ~32us timeout
    if (INTCONbits.TMR0IE == 1 && INTCONbits.TMR0IF == 1) // test T0 rollover 
	{
		INTCONbits.TMR0IF = 0; // clear TMR0 interrupt flag
		_asm
			// debug only - toggle RB7
			// if you un-comment this, then you should disable
			// the toggling of this bit in the low-priority interrupt handler.
			// to get a correct 'scope trace
			bsf PORTB,7,0 // take high

			movlw TMR0_RELOAD // TMR0 is only 8-bit
			movwf TMR0L,0

			// build the input byte from incoming bits
			rrncf suart0_in,1,0 // bit7 will be clear after this
			btfsc PORTB, 0,0    // test incoming data bit
			bsf suart0_in,7,0

			bcf PORTB,7,0 // debug timing bit low again

			decfsz suart0_count,1,0 // decrement bitcount and test
			goto fin0
		_endasm
			// All 8 bits recieved, put new byte into buffer
			buf0[in_index[0]] = suart0_in;
			in_index[0]++;
			if (in_index[0] == BUFSIZE)
				in_index[0] = 0;
			// finished receiving byte, so setup to detect edge again
			// enable further edge interrupts on this pin
			INTCONbits.INT0IF = 0; // clr any pending INT0 interrupt
			INTCONbits.INT0IE = 1;   // INT0 enabled
			INTCONbits.TMR0IE = 0; // TMR0 int disabled

fin0:;
	}


    // TMR1 interrupt, triggered by ~32us timeout
    if (PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1) // test T1 rollover 
	{
		
		PIR1bits.TMR1IF = 0; // clear flag
		_asm
			// debug only - toggle RB7
			// if you un-comment this, then you should disable
			// the toggling of this bit in the low-priority interrupt handler.
			// to get a correct 'scope trace
			bsf PORTB,7,0 // take high

			movlw TMR1_RELOAD >> 8
			movwf TMR1H,0  // preset TMR1 high byte
			movlw TMR1_RELOAD
			movwf TMR1L,0  // this loads both TMR1L and TMR1H

			// build the input byte from incoming bits
			rrncf suart1_in,1,0 // bit7 will be clear after this
			btfsc PORTB, 1,0    // test incoming data bit
			bsf suart1_in,7,0

			bcf PORTB,7,0 // debug timing bit low again

			decfsz suart1_count,1,0 // decrement bitcount and test
			goto fin1
		_endasm
			// All 8 bits recieved, put new byte into buffer
			buf1[in_index[1]] = suart1_in;
			in_index[1]++;
			if (in_index[1] == BUFSIZE)
				in_index[1] = 0;
			// finished receiving byte, so setup to detect edge again
			// enable further edge interrupts on this pin
			INTCON3bits.INT1IF = 0; // clr any pending INT1 flag
			INTCON3bits.INT1IE = 1; // INT1 enabled
			PIE1bits.TMR1IE = 0; // disable TMR1 interrupt
fin1:;
	}
    // TMR3 interrupt, triggered by ~32us timeout
    if ( PIE2bits.TMR3IE == 1 && PIR2bits.TMR3IF == 1) // test T3 rollover 
	{
		PIR2bits.TMR3IF = 0; // clear TMR3 interrupt flag.
		_asm
			// debug only - toggle RB7
			// if you un-comment this, then you should disable
			// the toggling of this bit in the low-priority interrupt handler.
			// to get a correct 'scope trace
			bsf PORTB,7,0 // take high

			movlw TMR3_RELOAD >> 8
			movwf TMR3H,0 // preset TMR3 high byte
			movlw TMR3_RELOAD 
			movwf TMR3L,0 // this loads both TMR3L and TMR3H

			// build the input byte from incoming bits
			rrncf suart2_in,1,0 // bit7 will be clear after this
			btfsc PORTB, 2,0    // test incoming data bit
			bsf suart2_in,7,0

			bcf PORTB,7,0 // debug timing bit low again

			decfsz suart2_count,1,0 // decrement bitcount and test
			goto fin2
		_endasm
			// All 8 bits recieved, put new byte into buffer
			buf2[in_index[2]] = suart2_in;
			in_index[2]++;
			if (in_index[2] == BUFSIZE)
				in_index[2] = 0;
			// finished receiving byte, so setup to detect edge again
			// enable further edge interrupts on this pin
			// clear any pending INT2 interrupt flag
			INTCON3bits.INT2IF = 0;
			INTCON3bits.INT2IE = 1;   // INT2 enabled
			PIE2bits.TMR3IE = 0; // disable TMR3 interrupt

fin2:;
	}
	///////////////////////////////////////////////////////////////////////////////////////////
	// Handler for Hardware UART
	// I've placed this handler last, as this interrupt will be handled quicker
	// by the PIC anyway. If you want it to be serviced as soon as possible
	// (VERY high priority), then place it at the start of the interrupt
	// service rountine. Be prepared to have to re-tweak the timer re-load values to compensate.
	///////////////////////////////////////////////////////////////////////////////////////////

	if (PIR1bits.RCIF == 1) // Hardware UART RX interrupt? (Note interrupt is permanently enabled so no need to test enable flag)
	{
		PIR1bits.RCIF = 0; // clear the interrupt flag
		buf3[in_index[3]] = RCREG; // get MIDI byte from hardware into our receive buffer
		in_index[3]++;  // bump input index
		if (in_index[3] == BUFSIZE)
			in_index[3] = 0;
		
	}

}

#pragma interrupt InterruptHandlerLow

void InterruptHandlerLow ()
{

//#if defined (DISPLAY_IMPLEMENTED)

	// currently TMR2 interrupt is used to flash IND LED to indicate alive
	// and is used for key matrix debouncing
	//
	// This routine should be kept as minimalist as possible
	// otherwise the software UART performance will be
	// affected
	//
	if (PIR1bits.TMR2IF)  //check for TMR2 interrupt
 	{   
	   	PIR1bits.TMR2IF = 0;            //clear interrupt flag
		if (--tmr2intcount == 0)
		{
			tmr2intcount = tmr2intcountreload; // reset counter
			LATBbits.LATB7 = !LATBbits.LATB7; // toggle comfort IND LED on RB7
		}
	}

//#endif
}
// end of interrupt routines
//-----------------------------------------------------------------------------

#if defined (DISPLAY_IMPLEMENTED)

///////////////////////////////////
//
// Toggle an LED either on of off
//
///////////////////////////////////
void SwitchLED(unsigned char lednum, unsigned char onnoff)
{
	switch (lednum)
	{
  		case 0: PORTAbits.RA0 = onnoff;
				break;
		case 1: PORTAbits.RA1 = onnoff;
				break; 
		case 2: PORTAbits.RA2 = onnoff;
				break;
		case 3: PORTAbits.RA3 = onnoff;
				break;
		case 4: PORTBbits.RB3 = onnoff;
				break;
		case 5: PORTBbits.RB4 = onnoff;
				break;
		case 6: PORTBbits.RB5 = onnoff;
				break;
		case 7: PORTBbits.RB6 = onnoff;
				break;
		case 8: PORTDbits.RD0 = onnoff;
				break;
		case 9: PORTDbits.RD1 = onnoff;
				break;
		case 10: PORTDbits.RD2 = onnoff;
				break;
		case 11: PORTDbits.RD3 = onnoff;
				break;
		case 12: PORTDbits.RD4 = onnoff;
				break;
		case 13: PORTDbits.RD5 = onnoff;
				break;
		case 14: PORTDbits.RD6 = onnoff;
				break;
		case 15: PORTDbits.RD7 = onnoff;
				break;
	}

}

//////////////////////////////////
//
// Turn All LEDs off
//
//////////////////////////////////
void SwitchLEDsOff(void)
{
	int counter;
	for (counter = 0; counter < 16; counter++)
	{
		SwitchLED( counter, 0);
	}
}

#endif

////////////////////////////////
// software delays
void delay_ms(int count)
{
	int i;	
	for (i = 0; i < count; i++)
	{
		Delay1KTCYx(2);
	}
}

void delay_us(int count)
{
	int i;	
	for (i = 0; i < count; i++)
	{
		Delay1TCY();
	}
}

///////////////////////////////////
//
// Output data to Hardware UART TX
//
///////////////////////////////////
void TxMIDI(unsigned char ch)
{
	while(BusyUSART()); // wait till UART TX ready
	WriteUSART(ch);

#if defined (DISPLAY_IMPLEMENTED)
	DisplayChannel(ch);
#endif

}			

#if defined (USER_INPUT_IMPLEMENTED)

/////////////////////////////////////
//
// Initialise 8 port lines used
// on key matrix
//
/////////////////////////////////////
void InitKeyMatrix(void)
{
	// outputs
	// Bit3 Bit2 Bit1 Bit0
	// RE2  RC2  RC1  RC0

	// inputs
	// Bit3 Bit2 Bit1 Bit0
	// RA5  RA4  RE1  RE0

	// make initial data out all 0
	PORTCbits.RC0 = 0;
	PORTCbits.RC1 = 0;
	PORTCbits.RC2 = 0;
	PORTEbits.RE2 = 0;
	
	// make required lines outputs
	TRISCbits.TRISC0 = 0;
	TRISCbits.TRISC1 = 0;
	TRISCbits.TRISC2 = 0;
	TRISEbits.TRISE2 = 0;

	// init data inputs
	TRISAbits.TRISA5 = 1;
	TRISAbits.TRISA4 = 1;
	TRISEbits.TRISE1 = 1;
	TRISEbits.TRISE0 = 1;
}

////////////////////////////////////////
//
// Translate from keynum delivered
// by hardware, to keyvalue required.
// (leeway when wiring up switches -
//  isn't software wonderful?)
//
////////////////////////////////////////
unsigned char TranslateKey(unsigned char key)
{
	switch (key)
	{
		case 0x0d: return 0;
		case 0x09: return 1;
		case 0x05: return 2;
		case 0x01: return 3;
		case 0x0e: return 4;
		case 0x0a: return 5;
		case 0x06: return 6;
		case 0x02: return 7;
		case 0x0f: return 8;
		case 0x0b: return 9;
		case 0x07: return 0x0a;
		case 0x03: return 0x0b;
		case 0x10: return 0x0c;
		case 0x0c: return 0x0d;
		case 0x08: return 0x0e;
		case 0x04: return 0x0f;
	}
}

/////////////////////////
// Raw Scan of keyboard
/////////////////////////
unsigned char RawKeyScan(void)
{ 
	// outputs
	// Bit3 Bit2 Bit1 Bit0
	// RE2  RC2  RC1  RC0

	// inputs
	// Bit3 Bit2 Bit1 Bit0
	// RA5  RA4  RE1  RE0
	
	unsigned char count; 
	hexstore = 0;

	for (count = 0; count < 4; count++)
	{
		// set an output line depending on count
		switch (count)
		{
			case 0: // make Bit0 low, others high
				PORTCbits.RC0 = 0;
				PORTCbits.RC1 = 1;
				PORTCbits.RC2 = 1;
				PORTEbits.RE2 = 1;
				break;
			case 1: // make Bit1 low, others high
				PORTCbits.RC0 = 1;
				PORTCbits.RC1 = 0;
				PORTCbits.RC2 = 1;
				PORTEbits.RE2 = 1;
				break;
			case 2: // make Bit2 low, others high
				PORTCbits.RC0 = 1;
				PORTCbits.RC1 = 1;
				PORTCbits.RC2 = 0;
				PORTEbits.RE2 = 1;
				break;
			case 3: // make Bit3 low, others high
				PORTCbits.RC0 = 1;
				PORTCbits.RC1 = 1;
				PORTCbits.RC2 = 1;
				PORTEbits.RE2 = 0;
				break;

		}
		// test input lines
		if (PORTEbits.RE0 == 0) // test Bit0
		{
			hexstore++;
			return hexstore;
		}
		if (PORTEbits.RE1 == 0) // test Bit1
		{
			hexstore +=2;
			return hexstore;
		}
		if (PORTAbits.RA4 == 0) // test Bit2
		{
			hexstore += 3;
			return hexstore;
		}
		if (PORTAbits.RA5 == 0) // test Bit3
		{
			hexstore += 4;
			return hexstore;
		}
		// still here? then bump count by 4
		hexstore += 4;
	}
	// after all that, you're still here?
	// no key pressed
	hexstore = 0xff;
	return hexstore;
}



///////////////////////////////////
// Delay for a couple of tmr2 ticks
///////////////////////////////////
void Delay20(void)
{
	while ((tmr2intcount % 3) != 0);
}

///////////////////////////////////////////////////////////
// debounced Key Scan
// on exit matrixkey has keyscanned 1 -> 0x10 or no_key (0xff)
///////////////////////////////////////////////////////////
unsigned char KeyScan()
{
	char val;
	val = RawKeyScan(); // poll keypad
	if (val == 0xff) // no switch depressed
	{
		Delay20();	 // delay and try again
		val = RawKeyScan();
		if (val == 0xff) // still no key? then exit
		{
matrix_switch_up:
			matrixkey = val;
			return matrixkey;
		}
	}

matrix_switch_down:
	// matrix_switch_down: a key is depressed
	Delay20(); // wait awhile
	val = RawKeyScan();
	if (val == 0xff)  // key has gone up?
	{
		matrixkey = val;
		return matrixkey;
	}
	matrixkey = val;
	// key still down, wait till user releases key
	// tell user key is down
//	Beep(4);
matrix_debounce_up:	
	val = RawKeyScan();
	while (val != 0xff)
	{
		val = RawKeyScan();
	}
	Delay20();
	val = RawKeyScan();
	if (val != 0xff)
	{
		goto matrix_debounce_up;
	}
	return matrixkey;
}

///////////////////////////////////////////////////////////
// Interpret and act (if required) on the last key pressed
// The key type is put in keytype
// The equivalent value is put in keyvalue
///////////////////////////////////////////////////////////
void InterpretKeyPress(void)
{
	keyvalue = TranslateKey(matrixkey);
	switch (keyvalue)
	{
		case 0: 
		case 1: 
		case 2: 
		case 3:
		case 4:
		case 5: 
		case 6: 
		case 7: 
		case 8: 
		case 9: keytype = DECIMAL_VAL; // digit
			break;
		case 10: keytype = FUNC_VAL;
				if (interpret_level == 0)
				{ 
					interpret_level++; // block further recursion
					FuncFilter(); // example Func
					interpret_level--;
				}
			break;
		case 11: keytype = EDIT_VAL;
				if (interpret_level == 0)
				{ 
					interpret_level++; // block further recursion
					DoEditRoutines(); // example edit synth commands
					interpret_level--;
				}
			break;
		case 12: keytype = ASSIGN_VAL;
				if (interpret_level == 0)
				{ 
					interpret_level++; // block further recursion
					AssignMIDIChannel();
					interpret_level--;
				}
			break;
		case 13: keytype = PARAM_VAL;
				if (interpret_level == 0)
				{ 
					interpret_level++; // block further recursion
					ParamMinVolume(); // example param global replace
					interpret_level--;
				}
			break;
		case 14: keytype = CLEAR_VAL;
			break;
		case 15: keytype = CANCEL_VAL;
			break;
	}

}


////////////////////////////////////
//
// Flash 16 LEDs in sequence in the 
// required direction, scanning
// key matrix 'til key is pressed.
//
////////////////////////////////////
void IndicateChannelRequired(unsigned char direction)
{
	int counter;
	// light LEDS in sequence
	while (RawKeyScan() == 0xff)
	{
		if (direction == L_R)
		{
		    for (counter = 0; counter < 16; counter++)
			{
				SwitchLED(counter,1);
				delay_ms(pausetime);
				SwitchLED(counter,0);
				if (RawKeyScan() != 0xff)
				{
					break;
				}
			}
		}
		else
		{
			for (counter = 15; counter > 0 ; counter--)
			{

				SwitchLED(counter-1,1);
				delay_ms(pausetime);
				SwitchLED(counter-1,0);
				if (RawKeyScan() != 0xff)
				{
					break;
				}
			}
		}
	}
}

////////////////////////////////////
//
// Flash upper 6 LEDs in sequence 
// scanning key matrix 'til key is pressed.
//
////////////////////////////////////
void IndicateAlphaRequired(void)
{
	// light LEDS in sequence
	// 'alpha' refers to keys 0x0a - 0x0f
	// of key matrix
	unsigned char counter;
	while (RawKeyScan() == 0xff)
	{
		for (counter = 10; counter < 16; counter++)
		{

			SwitchLED(counter,1);
			delay_ms(pausetime);
			SwitchLED(counter,0);
			if (RawKeyScan() != 0xff)
			{
				break;
			}
		}
	}
}

////////////////////////////////////
//
// Flash lowest 2 LEDs in sequence  
// scanning key matrix 'til key is pressed.
//
////////////////////////////////////
void IndicateOnOffRequired(void)
{
	// light 2 LEDS in sequence
	unsigned char counter;

	while (RawKeyScan() == 0xff)
	{
		for (counter = 0; counter < 2 ; counter++)
		{

			SwitchLED(counter,1);
			delay_ms(pausetime);
			SwitchLED(counter,0);
			if (RawKeyScan() != 0xff)
			{
				break;
			}
		}
	}

}
////////////////////////////////////
//
// Flash lower 10 LEDs in sequence  
// scanning key matrix 'til key is pressed.
//
////////////////////////////////////
void IndicateDecimalRequired(void)
{
	unsigned char counter;
	// light LEDS in sequence
	while (RawKeyScan() == 0xff)
	{
		for (counter = 0; counter < 10; counter++)
		{
#if defined (DISPLAY_IMPLEMENTED)

			SwitchLED(counter,1);
			delay_ms(pausetime);
			SwitchLED(counter,0);
#endif
			if (RawKeyScan() != 0xff)
			{
				break;
			}
		}
	}	
}

//#endif

////////////////////////////////////
//
// Get a decimal number 0-255 from key matrix  
// Put result in 'number'
// return 1 if success else 0
//
////////////////////////////////////
char GetNumber(char * number)
{
	char result;
	// get decimal value 0-255
	// light LEDS in sequence 0-9 to get 1st digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{
		//volume has accumulating volume 0-127
		result = keyvalue;
	}
	
	// light LEDS in sequence 0-9 to get 2nd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{	result *= 10;
		// bump volume
		result += keyvalue;
	}

	// light LEDS in sequence 0-9 to get 3rd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{	result *= 10;
		result += keyvalue;
	}
	*number = result; // pass out result
	return 1;  // indicate OK
}

/*
////////////////////////////////////
//
// Get a decimal number 0-999 from key matrix  
// Put result in 'number'
// return 1 if success else 0
//
////////////////////////////////////
char GetInt(int * number)
{
	int result;
	// get decimal value 0-255
	// light LEDS in sequence 0-9 to get 1st digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{
		//volume has accumulating volume 0-999
		result = keyvalue;
	}
	
	// light LEDS in sequence 0-9 to get 2nd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{	result *= 10;
		// bump volume
		result += keyvalue;
	}

	// light LEDS in sequence 0-9 to get 3rd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		return 0;
	}
	else
	{	result *= 10;
		result += keyvalue;
	}
	*number = result; // pass out result
	return 1;  // indicate OK
}

*/
/////////////////////////////////////////
//
// Example add-ons 1.
//
// Assign channel to another channel
// assigns a MIDI chan from <chan> to <chan>
// note an assignment can be to the same channel
//
/////////////////////////////////////////
void AssignMIDIChannel(void)
{
	unsigned char assign_to,assign_from;
	int counter;

	// change flash rate of IND lamp etc.
	StartUI(); 

	IndicateChannelRequired(L_R);
	KeyScan();
	InterpretKeyPress();
	//assign <from>
	assign_from = keyvalue;

	IndicateChannelRequired(R_L);
	KeyScan();
	InterpretKeyPress();
	//assign <to>
	assign_to = keyvalue;
	midi_assign[assign_from] = assign_to;
	FinishUI();
}

////////////////////////////////////////
//
// Example add-ons 2.
//
// Filter out (or not) chosen cmd byte
// 0xa0 - 0xf0
//
////////////////////////////////////////
void FuncFilter(void)
{
	unsigned char filter_num, channel;

	// change flash rate of IND lamp etc.
	StartUI(); // 

	// get a func number 10-15
	IndicateAlphaRequired();
	KeyScan();
	InterpretKeyPress();
	if (keyvalue < 0x0a)
	{
		// abort
		FinishUI();
		return;
	}
	else
	{
		//assign value is func to filter
		filter_num = keyvalue - 10;

/* !!! TODO for on-channel basis	
		IndicateChannelRequired(L_R);
		KeyScan();
		InterpretKeyPress();
		channel = keyvalue;
*/
		IndicateOnOffRequired();
		KeyScan();
		InterpretKeyPress();
		//assign value if 0: filter this func OFF, if 1 func back on, other key abort
		if (keyvalue == 0 || keyvalue == 1)
		{
			midi_filter[filter_num] = keyvalue;
		}
	}
	FinishUI();	
}

////////////////////////////////////////
//
// Example add-ons 4.
//
// Edit routines for DB50XG synth
//  SYSEX XG Mode ON
//  BANK SELECT
//  PROGRAM CHANGE
//
////////////////////////////////////////
void DoEditRoutines(void)
{
	char channel, editnum, banknum,program;
	int counter,patch;

	// change flash rate of IND lamp etc.
	StartUI(); 

	// get edit function number 0-15
	IndicateChannelRequired(R_L); // ignore name - flashes all 16 lEDS to indicate func choice 0-15
	KeyScan();
	InterpretKeyPress();
	editnum = keyvalue;
	switch (editnum)
	{
		case 0: // SYSEX XG Mode ON
			TxMIDI( 0xf0);
			TxMIDI( 0x43);
			TxMIDI( 0x10);
			TxMIDI( 0x4c);
			TxMIDI( 0x00);
			TxMIDI( 0x00);
			TxMIDI( 0x7e);
			TxMIDI( 0x00);
			TxMIDI( 0xf7);
			break;

		case 1: // Bank num for XG
			// get channel
			IndicateChannelRequired(L_R);
			KeyScan();
			InterpretKeyPress();
			channel = keyvalue;
		
			if (GetNumber(&banknum)) 	// get decimal number 0-127 
			{
				TxMIDI(0xb0 + channel); // send status byte
				TxMIDI(0); // CTRL number = 0
				TxMIDI(0); // Bank sel MSB = 0 for 'normal' voices

				TxMIDI(0xb0 + channel); // send status byte
				TxMIDI(0x20); // CTRL number = 32
				TxMIDI(banknum); // Bank sel MSB = 0 for 'normal' voices
				// NOTE always need a Program change msg for this to take effect
			}
			break;

		 case 2: // Program change message
			// get channel
			IndicateChannelRequired(L_R);
			KeyScan();
			InterpretKeyPress();
			channel = keyvalue;
			if (GetNumber(&program))
			{
				TxMIDI(0xc0 + channel); // send status byte
				TxMIDI(program);
			}
			break;
/*
		 case 3: // Direct XG patch select message
			// get channel
			IndicateChannelRequired(L_R);
			KeyScan();
			InterpretKeyPress();
			channel = keyvalue;
			if (GetInt(&patch))
			{
				TxMIDI(0xc0 + channel); // send status byte
				TxMIDI(program);
			}
			break;
*/

	}
	FinishUI();

}



//////////////////////////////////////////////
// Example add-ons 3.
//
// Set up minimum volume for a chosen channel
//
//////////////////////////////////////////////
void ParamMinVolume(void)
{
	// assign a MIDI chan minimum volume
	unsigned char channel;
	char volume;

	// change flash rate of IND lamp etc.
	StartUI(); // 

	// light all LEDS in sequence to get channel
	IndicateChannelRequired(L_R);
	KeyScan();
	InterpretKeyPress();
	channel = keyvalue;
	
	// light LEDS in sequence 0-9 to get 1st digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		FinishUI();
		return;
	}
	else
	{
		//volume has accumulating volume 0-127
		volume = keyvalue;
	}
	
	// light LEDS in sequence 0-9 to get 2nd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		FinishUI();
		return;
	}
	else
	{	volume *= 10;
		// bump volume
		volume += keyvalue;
	}

	// light LEDS in sequence 0-9 to get 3rd digit
	IndicateDecimalRequired();
	KeyScan();
	InterpretKeyPress();
	if (keytype != DECIMAL_VAL)
	{
		// abort
		FinishUI();
		return;
	}
	else
	{	volume *= 10;
		// bump volume
		volume += keyvalue;
		// now assign min-volume to the channel
		midi_min_volume[channel] = volume;
	}
	FinishUI();
}

/////////////////////////////////////
//
// Signify User Interface is complete
// so restore system to 'normal'
//
/////////////////////////////////////
void FinishUI(void)
{
	tmr2intcountreload = TMR2INTCOUNTVAL;
	tmr2intcount = TMR2INTCOUNTVAL;
}

/////////////////////////////////////
//
// Signify User Interface active.
// Change flash rate of IND LED etc.
//
/////////////////////////////////////
void StartUI(void)
{
	tmr2intcountreload = TMR2UIINTCOUNT;
	tmr2intcount = TMR2UIINTCOUNT;
	SwitchLEDsOff();
}

#endif

// EEPROM access routines
void WriteEEPROM(int address, char value)
{
	int saveit;
	unsigned char intconcopy;

//	EEADRH = address >> 8; // 4320 has only 256 bytes of eeprom
	EEADR = address & 0x00ff;
	EEDATA = value;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	//////////////////////////////
	// disable all interrupts here	
    intconcopy = INTCON;            // Save INTCON bits
    INTCONbits.GIEH = 0;            // Disable all interrupts from CPU
    INTCONbits.GIEL = 0;
	//////////////////////////////
	EECON2 = 0x55;
	EECON2 = 0xaa;
	EECON1bits.WR = 1;
	EECON1bits.WREN = 0;
	//////////////////////	
	// re-enable ints here
    saveit = intconcopy & 0b11000000; // Reenable interrupts 
    INTCON = INTCON | saveit;        
	//////////////////////
	while (EECON1bits.WR == 1); // wait till done
}

void WriteEEPROMShortLong(int address, short long value)
{
	union ShortLong val;
	val.sl = value;
	WriteEEPROM(address, val.bt[0]);
	WriteEEPROM(address+1, val.bt[1]);
	WriteEEPROM(address+2, val.bt[2]);
}

void WriteEEPROMInt(int address, int value)
{
	union IntUnion val;
	val.i = value;
	WriteEEPROM(address, val.bt[0]);
	WriteEEPROM(address+1, val.bt[1]);
}

char ReadEEPROM(int address)
{
	char val;
//	EEADRH = address / 256;
	EEADR = address;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;
	val = EEDATA;
	return val;
}

short long ReadEEPROMShortLong(int address)
{
	union ShortLong val;
	
	val.bt[0] = ReadEEPROM(address);
	val.bt[1] = ReadEEPROM(address+1);
	val.bt[2] = ReadEEPROM(address+2);
	return val.sl;
}

int ReadEEPROMInt(int address)
{
	union IntUnion val;
	
	val.bt[0] = ReadEEPROM(address);
	val.bt[1] = ReadEEPROM(address+1);
	return val.i;
}
//---------------------------------------------

#if defined(PLAY_SIGNATURE)


void PlaySignature(void) // A bit of fun. play a couple of bars of the classic Annie Lennox 'Sweet Dreams Are Made Of This'
{
int count;
//	for (count = 0; count < 2; count++)
//	{
        
while(1){
    /*
        TxMIDI(0x90);
        TxMIDI(0x40);
        TxMIDI(0x40);
        delay_ms(1000);
        TxMIDI(0x80);
        TxMIDI(0x40);
        TxMIDI(0x00);
        delay_ms(1000);

		}
    */
        TxMIDI(0x92);
        TxMIDI(0x30);       
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x30);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x30);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x30);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
//	I quickly got fed up with this so shortened it!
	//Eb
		TxMIDI(0x92);
		TxMIDI(0x33);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x33);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x33);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x33);
		TxMIDI(0x00);
	// C
		TxMIDI(0x92);
		TxMIDI(0x30);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x30);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x30);
		TxMIDI(0x00);
	// G
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);
	
	// Bb
	
		TxMIDI(0x92);
		TxMIDI(0x2e);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2e);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x2e);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2e);
		TxMIDI(0x00);
	//G
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);
	
		TxMIDI(0x92);
		TxMIDI(0x2b);
		TxMIDI(0x7f);
		delay_ms(1000);
		TxMIDI(0x82);
		TxMIDI(0x2b);
		TxMIDI(0x00);

	}
}
#endif
