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


int countTimer = 0;

int distance = 0;


ISR(TIMER1_OVF_vect)
{
	countTimer++;
}


int SonarSensor_init(void);
void USART_init(long UBRR);
void USART_TransmitPolling(char *distance);
void USART_putstring(char *StringPtr);
void wheels(void);




int main(void)
{
	
	char String[10]; 
	USART_init(UBRR_value);
	
	char test2[] = " \n";
	
	
	while(1)
	{
		distance = SonarSensor_init();
		wheels();
		
		itoa(distance, String, 10);
		USART_putstring(String);
		USART_putstring(test2);

	}
}

void wheels(void)
{
	//DDRD |= (0 << PIND3);  
	
	TCCR2A |= (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20);				// Set OC2B on compare match. Set OC2B at BOTTOM. Fast-PWM mode. Inverting mode.
	TCCR2B |= (1 << CS20) | (1 << CS22);												// No prescaler.
	
																			
	
	if(distance < 10)
	{
		OCR2B = 70;																	  // 70 er nullpunktet. Da går ikke servoen!
	}
	
	if(distance > 10)
	{
		OCR2B = 30;																  // går mot klokken
	}
	
	
	// 70 er nullpunktet. 120 går mot klokken 10 runder på 10 sekunder . 30 går med klokken 10 runder på 10 sekunder.
	
	
}




int SonarSensor_init(void)
{
	int counter = 0;
	DDRB = 0x00;														// Echo port as input. Echo pin will be PB0 (ICP1)
	
	DDRD = 0x0FF;														// Trigger port as output
	PORTD = 0xFF;														// Set port D as output
	
	TCCR1A = (0 << WGM12) | (0 << WGM11) | (0 << WGM10);				// Normal mode
	TCCR1B = (1 << CS10);												// No prescaler
	TCCR1B = (1 << ICES1);												// Rising edge triggers the capture
	
	TIMSK1 = (1 << TOIE1);												// Enable Timer1 overflow interrupts
	
	TCNT1 = 0;															// Clear Timer counter
	
	sei();																// Enable global interrupt
	
	while(1)
	{
		PORTD |= (1 << PIND0);
		_delay_us(10);													// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTD = ~(1 << PIND0);
		
		TIFR1 = (1 << ICF1);											// ICF1 is set when the counter reaches TOP value
		TIFR1 = (1 << TOV1);											// TOV1 is set when the timer overflows.
		
		
		while((TIFR1 & (1 << ICF1)) == 0)								// Waiting for rising edge. Echo pin should be low when the sound has traveled back. The duration of the pulse determines the distance.
		{
		}
		
		TCNT1 = 0;	/* Clear Timer counter */
		TCCR1B = 0x01;	/* Capture on falling edge, No prescaler */
		TIFR1 = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		TIFR1 = 1<<TOV1;	/* Clear Timer Overflow flag */
		countTimer = 0;/* Clear Timer overflow count */
		
		while((TIFR1 & (1 << ICF1)) == 0)								// Waiting for falling edge
		{
		}
		counter = ICR1 + (65535 * countTimer);						// ICR1 measures the time from rising edge to falling edge for the echo pulse. In case of ICR1 overflow, it starts counting in countTimer.
		return counter/580;
	}
}



void USART_init(long UBRR)
{
	UBRR0H = (UBRR >> 8);			// Shift the register 8 bits to the right. UBBR value is only 103.
	UBRR0L = (UBRR); 
	
	UCSR0C = ((1 << UMSEL01) | (1 << UPM00) | (1 << UPM01) | (1 << USBS0) | (3<<UCSZ00));			// Syncrhonous USART | Odd parity | 1 stop bit | 8 bit. 
	
	UCSR0B = ((1 << RXEN0) | (1 << TXEN0));	 // Enables receiver and transmitter
	
}


void USART_TransmitPolling(char *String)
{
	while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
	UDR0 = String;
}


void USART_putstring(char *StringPtr){
	while(*StringPtr != 0x00){
		USART_TransmitPolling(*StringPtr);
		StringPtr++;}
	
}