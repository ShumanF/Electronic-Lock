#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "AVR lib/AVR_lib.h"
#include "USART/USART.h"
#include "LCD/lcd.h"
#include "ADC/adc.h"

volatile uint8_t timerOverflow = 0; // Counter za pracenje broja overflowa timera
uint16_t broj_impulsa = 0;

ISR(INT0_vect) // prekidna rutina za INT0
{
	// ako je brid rastuæi
	if(get_pin(PIND,PD2) == 1){
		TCNT1 = 0; // t1 = 0
	}
	else // ako je brid padajuæi
	{
		broj_impulsa = TCNT1; // t2 = TCNT1
	}
}
void timer_init()
{
	// F_CPU / 1024
	set_bit_reg(TCCR0,CS00); // CS00 = 1
	reset_bit_reg(TCCR0,CS01); // CS01 = 0
	set_bit_reg(TCCR0,CS02); // CS02 = 1

	// Omoguci prekid TOV0
	set_bit_reg(TIMSK,TOIE0); // TOIE0 = 1
}

void timer_start()
{
	TCNT0 = 0; // Inicijalizacija countera
	timerOverflow = 0; // Reset overflow counter
}

ISR(TIMER0_OVF_vect)
{
	timerOverflow++; // Povecaj overflow counter
}

int main(void)
{
	// Configure input
	input_port(DDRB,PB0);
	input_port(DDRB,PB1);
	input_port(DDRD,PD2); // PD2 postavljen kao ulazni

	// Configure output
	output_port(DDRB,PB4);
	output_port(DDRB,PB5);
	output_port(DDRB,PB7);
	output_port(DDRD,PD7);
	
	//pull-up
	PORTB |= (1 << PB0);
	PORTB |= (1 << PB1);
	

	timer_init(); // Inicijalizacija timera
	sei();        // Omoguci interrupt
	lcd_init();
	adc_init(); // inicijalizacija AD pretvorbe

	
	set_port(PORTB,PB4,0);
	set_port(PORTB,PB5,0);
	set_port(PORTB,PB7,0); // Postavljen u nisko stanje
	set_port(PORTD,PD7,0); // Motor
	
	// oba brida generiraju prekid INT0
	set_bit_reg(MCUCR,ISC00);
	reset_bit_reg(MCUCR,ISC01);
	
	set_bit_reg(GICR,INT0); // omoguæenje vanjskog prekida INT0
	
	//F_CPU/8 za timer1
	reset_bit_reg(TCCR1B,CS10);
	set_bit_reg(TCCR1B,CS11);
	
	output_port(DDRB,PB4); // pin PB4 postavljen kao izlazni
	input_port(DDRD,PD2); // PD2 postavljen kao ulazni
	
	uint16_t ADC5; // rezultat AD pretvorbe
	float VPA5; // napon na pinu PA5
	const float VREF = 5.0; // AVCC
	bool otklucano = true;
	
	while (1)
	{
		set_port(PORTB, PB4, 1);
		_delay_us(20); // trigger impuls
		set_port(PORTB, PB4, 0);
		_delay_ms(500);
		float d = broj_impulsa * 17.0 / 1000.0;
		
		lcd_clrscr();
		lcd_home();
		ADC5 = adc_read_10bit(5);
		VPA5 = ADC5 * VREF / 1024.0;
		
		bool otvoreno;
		if (ADC5 % 2 == 0)
		{
			lcd_print("Odobriti? PB0=OK\n");
			lcd_print("<da>   ne");
			_delay_ms(1000);
			otklucano = true;
			}else{
			lcd_print("Odobriti? PB0=OK\n");
			lcd_print("da   <ne>");
			_delay_ms(1000);
			otklucano = false;
		}
		
		// otvori
		if (!(PINB & (1 << PB0)) && otklucano==true)
		{
			otvoreno=true;
			set_port(PORTB,PB5,1); // Pali zelenu LED
			BUZZ(0.5,500);
			
			lcd_clrscr();
			lcd_home();
			lcd_print("Otkljucano");
			timer_start(); // Startaj timer
			
			while (timerOverflow < 15) // 15 je otprilike 0.5 sekundi (8MHz clock, 1024 prescaler)
			{
				set_port(PORTD,PD7,1); // Pali motor
			}
			set_port(PORTD,PD7,0); // Gasi motor
			set_port(PORTB,PB5,0); // Gasi zelenu LED
		}
		//odbiji otovaranje
		if (!(PINB & (1 << PB0)) && otklucano==false)
		{
			set_port(PORTB,PB7,1); // Pali crvenu LED
			BUZZ(0.5,250);
			
			lcd_clrscr();
			lcd_home();
			lcd_print("odbijeno");
			
			
			set_port(PORTB,PB7,0); // gasi crvenu LED
			
			
		}
			
		if (otklucano==true && d <= 5)
		{
			set_port(PORTB,PB7,1); // Pali crvenu LED
			BUZZ(0.5,250);
			
			lcd_clrscr();
			lcd_home();
			lcd_print("zatvoreno");
			timer_start(); // Start the timer
			while (timerOverflow < 15) // 15 je otprilike 0.5 sekundi (8MHz clock, 1024 prescaler)
			{
				// Šta treba radit
				set_port(PORTB,PB7,0); // Gasi crvenu LED
				set_port(PORTD,PD7,1); // Pali motor
			}
			set_port(PORTB,PB7,0); // gasi crvenu LED
			set_port(PORTD,PD7,0); // Gasi motor
			otvoreno=false;
		}
		
	}

	return 0;
}
