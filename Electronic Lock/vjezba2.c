#include "AVR lib/AVR_lib.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART/USART.h"
#include "timer/timer.h"
#include <util/delay.h>
#include "ADC/adc.h"

uint16_t brojac_int0;
uint16_t ADC5_PWM;
uint32_t ADC5;

ISR(INT0_vect) // prekidna rutina za INT0
{
	brojac_int0++;
}

ISR(TIMER1_OVF_vect){
	
	usart_write("t1.txt=\"%d rpm\"%c%c%c", brojac_int0/3, 255, 255, 255);
	
	TCNT1 = 3036;
	brojac_int0 = 0;
}

void inicijalizacija(){
	
	usart_init(9600);		// baud rate postavljen na 9600
	
	// rastuæi brid generira prekid INT0
	set_bit_reg(MCUCR,ISC00); // ISC00 = 1
	set_bit_reg(MCUCR,ISC01); // ISC01 = 1

	set_bit_reg(GICR,INT0); // omoguæen vanjski prekid INT0
	
	input_port(DDRD,PD2); // pin PD2 postavljen kao ulazni 
	set_port(PORTD,PD2,1); // ukljuèen pritezni otpornik na PD2
	
	//normalan naèin rada - timer1
	reset_bit_reg(TCCR1A,WGM10); // WGM10 = 0
	reset_bit_reg(TCCR1A,WGM11); // WGM11 = 0
	reset_bit_reg(TCCR1B,WGM12); // WGM12 = 0
	reset_bit_reg(TCCR1B,WGM13); // WGM13 = 0
	
	// F_CPU / 64
	set_bit_reg(TCCR1B,CS10); // CS10 = 1
	set_bit_reg(TCCR1B,CS11); // CS11 = 1
	reset_bit_reg(TCCR1B,CS12); // CS12 = 0
	
	set_bit_reg(TIMSK,TOIE1); // TOIE1 = 1
	
	//Definirati Fast PWM nacin rada za TIMER2
	set_bit_reg(TCCR2,WGM20); //WGM00 = 1
	set_bit_reg(TCCR2,WGM21); //WGM01 = 1
		
	// F_CPU/128
	set_bit_reg(TCCR2,CS20); // CS20 = 1
	reset_bit_reg(TCCR2,CS21); // CS21 = 0
	set_bit_reg(TCCR2,CS22); // CS22 = 1
		
	//Postaviti neinvertirajuci PWM za TIMER0
	reset_bit_reg(TCCR2,COM20); // COM00 = 0
	set_bit_reg(TCCR2,COM21); // COM01 = 1
		
	//Postaviti pin od TIMERA2 kao izlazni pin
	output_port(DDRD,PD7); // pin PD3 postavljen kao izlazni pin
	output_port(DDRD,PD2); // pin PD2 postavljen kao izlazni pin
	
	//Inicijlizirati
	OCR2 = 0; // duty cycle
				
	
	sei(); // globalno omoguæenje prekida
	
	TCNT1 = 3036; //500ms
	adc_init();
	
	usart_write("page 1%c%c%c", 255, 255, 255);
	
}

int main(void){
	
	inicijalizacija();
		
		
		
		
	while(1)
	{	
		ADC5 = adc_read_10bit(5);
		ADC5_PWM= ADC5/10;
		usart_write("t2.txt=\"%d %%PWM \"%c%c%c", ADC5_PWM, 255, 255, 255);
		usart_write("j0.val=%d%c%c%c",ADC5_PWM , 255, 255,255);
		
		OCR2 = ADC5 * 255/ 1023;
	
		_delay_ms(100);
	}
	
	return 0;
}