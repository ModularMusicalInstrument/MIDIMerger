/* 
 * File:   main.c
 * Author: Hussein
 *
 * Created on July 12, 2016, 8:06 PM
 */

#include <stdio.h>
#include <stdlib.h>

/*
 * 
 */
void main(void) {
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
    }  
    return (EXIT_SUCCESS);
}

void delay_ms(int count)
{
    static long timel = 0;
    timel = count * 125l;
    for( ;timel; timel--);// no initial condition, while time is >0, decrement time each loop
    /*
	int i;	
	for (i = 0; i < count; i++)
	{
		Delay1KTCYx(2);
	}
    */
}

