#define DelayTXBitUARTcycles ((((2*FOSC)/(4*baudrate))+1)/2) -12
#define DelayRXHalfBitUARTcycles ((((2*FOSC)/(8*baudrate))+1)/2) - 9
#define DelayRXBitUARTcycles ((((2*FOSC)/(4*baudrate))+1)/2) - 14

void DelayRXHalfBitUART(void);

void DelayRXBitUART(void);

void DelayTXBitUART(void);
