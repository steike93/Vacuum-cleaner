/*
 * main.c
 *
 * Created: 5/27/2022 2:02:23 PM
 *  Author: Erlend
 */ 


#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define UBRR_value (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

uint16_t distance1;
uint16_t distance2;
uint16_t distance3;


void USART_init(long UBRR);
void USART_TransmitPolling(char *distance);
void USART_putstring(char *StringPtr);
void wheels_right(void);
void wheels_left(void);
void SonarSensor_init0(void);
void SonarSensor_init1(void);
void SonarSensor_init2(void);


int main(void)
{
	//SonarSensor_init0();
	//SonarSensor_init1();
	SonarSensor_init2();
	
	USART_init(UBRR_value);
	sei();
	
	while(1)
	{
     	//wheels_right();
		//wheels_left();
		
		char newline[10] = " \n";
		char String[10];
		
		PORTD |= (1 << PIND4);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTD &= ~(1<<PIND4);															// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		

		
		itoa(distance3, String, 10);
		
		USART_putstring(String);
		USART_putstring(newline);
		
		/*
		
		PORTB |= (1 << PINB0);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTB &= ~(1<<PINB0);
		
		_delay_ms(10);
		
		PORTC |= (1 << PINC4);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTC &= ~(1<<PINC4);
		
		*/
		_delay_ms(1000);
		
	}
}

void wheels_right(void)
{
	//DDRD |= (0 << PIND3);  
	
	TCCR2A |= (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20);			// Compare output mode. Fast-PWM mode. Inverting mode.
	TCCR2B |= (1 << CS20) | (1 << CS22);											// No prescaler.
																		
	if(distance1 < 10)
	{
		OCR2B = 70;																	  
	}
	
	if(distance1 > 10)
	{
		OCR2B = 30;																  
	}
	
	
	/* 70 er nullpunktet. 120 går mot klokken 10 runder på 10 sekunder . 30 går med klokken 10 runder på 10 sekunder. */
	
	
}


void wheels_left(void)
{
	
	DDRB |= (1 << PINB3);
	
	TCCR2A |= (1 << COM2A1) | (1 << COM2A0) | (1 << WGM21) | (1 << WGM20);			// Compare output mode. Fast-PWM mode. Inverting mode.
	TCCR2B |= (1 << CS20) | (1 << CS22);											// No prescaler.
	
	if(distance2 < 10)
	{
		OCR2A = 70;
	}
	
	if(distance2 > 10)
	{
		OCR2A = 30;
	}
	
	
	/* 70 er nullpunktet. 120 går mot klokken 10 runder på 10 sekunder . 30 går med klokken 10 runder på 10 sekunder. */
	
	
}


void SonarSensor_init2(void)
{
	DDRD = 0xFF;							// Port D all output.
	DDRD &= ~(1<<DDD5);
	
	PORTD |= (1<<PORTD5);					// Enable pull up on D5 (echo)
	PORTD &= ~(1<<PIND4);					// Init D4 as low (trigger)
	
	PRR &= ~(1<<PRTIM2);					// To activate timer0 module
	TCNT2 = 0;								// Initial timer value
	TCCR2B |= (1<<CS22) | (1 << CS21);		// Timer with prescaler. Since default clock for atmega328p is 1Mhz period is 1uS
	PCICR  |= (1<<PCIE2);					// Enable PCINT[20:24] we use pin D4 which is PCINT20
	PCMSK2 |= (1<<PCINT21);					// Enable D5 interrupt
}


void SonarSensor_init1(void)
{	
	DDRC = 0xFF;							// Port C all output.
	DDRC &= ~(1<<DDC5);
	
	PORTC |= (1<<PORTC5);					// Enable pull up on C5 (echo)
	PORTC &= ~(1<<PINC4);					// Init C4 as low (trigger)
	
	PRR &= ~(1<<PRTIM1);					// To activate timer1 module
	TCNT1 = 0;								// Initial timer value
	TCCR1B |= (1<<CS12);					// Timer without prescaler. Since default clock for atmega328p is 1Mhz period is 1uS
	TCCR1B |= (1<<ICES1);					// First capture on rising edge
	PCICR  |= (1<<PCIE1);						// Enable PCINT[14:8] we use pin C5 which is PCINT13
	PCMSK1 |= (1<<PCINT13);					// Enable C5 interrupt
}


void SonarSensor_init0(void)
{
	DDRB = 0xFF;							// Port B all output.
	DDRB &= ~(1<<DDB1);
	
	PORTB |= (1<<PORTB1);					// Enable pull up on B1 (echo)
	PORTB &= ~(1<<PINB0);					// Init B0 as low (trigger)
	
	PRR &= ~(1<<PRTIM0);					// To activate timer0 module
	TCNT0 = 0;								// Initial timer value
	TCCR0B |= (1<<CS02) | (1 << CS00);		// Timer with prescaler. Since default clock for atmega328p is 1Mhz period is 1uS
	PCICR |= (1<<PCIE0);					// Enable PCINT[0:7] we use pin B1 which is PCINT1
	PCMSK0 |= (1<<PCINT1);					 // Enable B1 interrupt
}




void USART_init(long UBRR)
{
	UBRR0H = (UBRR >> 8);																// Shift the register 8 bits to the right. UBBR value is only 103.
	UBRR0L = (UBRR); 
	
	UCSR0C = ((1 << UMSEL01) | (1 << UPM00) | (1 << UPM01) | 
	(1 << USBS0) | (3<<UCSZ00));														// Syncrhonous USART | Odd parity | 1 stop bit | 8 bit. 
	
	UCSR0B = ((1 << RXEN0) | (1 << TXEN0));												// Enables receiver and transmitter
}


void USART_TransmitPolling(char *String)
{
	while (( UCSR0A & (1<<UDRE0)) == 0) {};												// Do nothing until UDR is ready
	UDR0 = String;
}


void USART_putstring(char *StringPtr){
	while(*StringPtr != 0x00){
		USART_TransmitPolling(*StringPtr);
		StringPtr++;}
}




ISR(PCINT2_vect) {
	
	if ( (PIND & (1 << PIND5)) == (1 << PIND5))								// Checks if echo is high
	{
		TCNT2 = 0;
		PORTB |= (1 << PINB5);												// Toggles debug led
	}
	
	else
	{
		distance3 = TCNT2/3;					    // Save Timer value
		PORTB &= ~(1 << PINB5);			    	// Toggles debug led
		//cli();
	}
}



ISR(PCINT1_vect) {
	
	if ( (PINC & (1 << PINC5)) == (1 << PINC5))								// Checks if echo is high
	{
		TCNT1 = 0;		
		PORTB |= (1 << PINB5);											   // Toggles Debug Led
	}
	
	else
	{
		distance2 = TCNT1/3;					// Save Timer value
		PORTB &= ~(1 << PINB5);					// Toggles Debug led
		//cli();
	}
}


ISR(PCINT0_vect) {
	
	if ( (PINB & (1 << PINB1)) == (1 << PINB1))								// Checks if echo is high
	{
		TCNT0 = 0;
		PORTB |= (1 << PINB5);												// Toggles debug led
	}
	
	else
	{
		distance1 = TCNT0;					    // Save Timer value
		PORTB &= ~(1 << PINB5);			    	// Toggles debug led
		//cli();
	}
}
