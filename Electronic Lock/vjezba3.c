#define F_CPU 8000000ul

#include <avr/io.h>
#include <util/delay.h>
#include "USART/USART.h"

void inicijalizacija(){
	
	DDRD |= (1<<PD5);	// pin PD5 postavljen kao izlaz
	ICR1 = 2499;		// TOP count za timer1

	// Fast PWM, TOP in ICR1, Clear OC1A on compare match, CLK/64
	TCCR1A = (1<<WGM11)|(1<<COM1A1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
	
	usart_init(9600);		// baud rate postavljen na 9600
	sei();					// globalno omoguæenje prekida
	
	usart_write("page 2%c%c%c", 255, 255, 255);
	usart_write("t1.txt=\"90°\"%c%c%c", 255, 255, 255);
	usart_write("h0.val=90%c%c%c", 255, 255, 255);
	
	OCR1A = 228; // 228 = 1.29*90+112
	
}

int main(void)
{
	
	inicijalizacija();
	
	uint8_t i;
	uint16_t pozicija = 0;
	
	while(1)
	{
		// primljena je poruka sa zakljuènim znakom end_char definiranim u zaglavlju USART.h
		if(usart_read_all() == 1){
			pozicija = 0;
			for(i = 0; i < 4; i++){
				if(usart_buffer[i] == '\0') break;
				pozicija = pozicija * 10 + (usart_buffer[i] - 48);
			}
			OCR1A = 1.29*pozicija+112;
		}
		_delay_ms(100);
	}
}