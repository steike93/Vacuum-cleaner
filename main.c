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



uint16_t distance0_left;
uint16_t distance1_front;
uint16_t distance2_right;

uint16_t batteryVoltage;

char textOnLCD[24] = "Batteriprosenten er: ";


void USART_init(long UBRR);
void USART_TransmitPolling(char *distance);
void USART_putstring(char *StringPtr);
void wheels_init(void);
void wheels_adjusted(void);
void SonarSensor_init0_left(void);
void SonarSensor_init1_front(void);
void SonarSensor_init2_right(void);

void batteryMonitoring_init(void);
void batteryMonitoring(void);

void Write_SPI(char *screenData);
void SPI0_Master_init(void);
void SPI0_Transmitt(char *screenData);
void LCD_Send_Command();
void LCD_Clear_Screen();


int main(void)
{
	SonarSensor_init0_left();
	SonarSensor_init1_front();
	SonarSensor_init2_right();
	
	wheels_init();
	
	batteryMonitoring_init();	
	
	SPI0_Master_init();								// Sets SPI Master
	LCD_Send_Command();								// Display ON, clears screen
	
	//USART_init(UBRR_value);
	sei();
	
	while(1)
	{
		SPI0_Master_init();
		LCD_Send_Command();
		
		wheels_adjusted();
		
		batteryMonitoring();
		
		Write_SPI(textOnLCD);
		

		char newline[10] = " \n";
		char String[10];
		
		PORTD |= (1 << PIND4);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTD &= ~(1<<PIND4);															// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		
		//itoa(distance2_right, String, 10);
		_delay_ms(10);
		
		char right[10] = "right: :";
		//USART_putstring(right);
		//USART_putstring(String);
		//USART_putstring(newline);
		
		
		PORTB |= (1 << PINB0);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTB &= ~(1<<PINB0);
		
		//itoa(distance0_left, String, 10);
		
		//char left[10] = "left: :";
		//USART_putstring(left);
		//USART_putstring(String);
		//USART_putstring(newline);
		
		
		_delay_ms(10);
		
		PORTC |= (1 << PINC4);
		_delay_us(10);																	// 10 us trigger. Echo pin is pulled high by control circuit of sonar sensor.
		PORTC &= ~(1<<PINC4);
		
		//itoa(distance1_front, String, 10);
		
		//char front[10] = "front :";
		//USART_putstring(front);
		//USART_putstring(String);
		//USART_putstring(newline);
			
		_delay_ms(10);
		
		
		
		//ADCSRA |= (1 << ADIE);														 // Enables ADC interrupt. Skal vel ikke være her?
		
		// Må legge inn slik at vifte kan skrus av via knapp.
		
		LCD_Clear_Screen();
		
		
		
	}
}

void wheels_init(void)
{
	
	DDRD =  (1 << DDD1) | (1 << DDD2);
	PORTD =	(1 << PORTD1) | (1 << PORTD2);							// Must write a 1 to bit 2 of PORTD in order to get PWM output on PD2 from either Timer3 or Timer4
	
	TCCR3A = (1 << COM3B1) | (1 << WGM30);							// Fast PWM 8-bit
	TCCR3B = (1 << CS30) | (1 << CS32) | (1 << WGM32);				// 1024 pre-scaling. Clear on compare match.
	
	TCCR4A |= (1 << COM4A1) | (1 << WGM40);
	TCCR4B |= (1 << CS40)  | (1 << CS42)| (1 << WGM42);
	
}


void wheels_adjusted()
{
														
	if((distance1_front > 10) & (distance2_right > 10) & (distance0_left > 10))
	{
		OCR3B = 25;																// Høyre hjul
		OCR4A = 25;																// Venstre hjul  
	}
	
	
	else if((distance1_front < 10) & (distance2_right > distance0_left))
	{
		OCR3B = 20;		
		OCR4A = 23;															  
	}
	
	else if((distance1_front < 10) & (distance2_right < distance0_left))
	{
		OCR3B = 23;
		OCR4A = 23;
	}
	
	else if((distance1_front > 10) & (distance2_right < 10) & (distance0_left > 10))
	{
		OCR3B = 20;
		OCR4A = 23;
	}
	
	else if((distance1_front > 10) & (distance2_right > 10) & (distance0_left < 10))
	{
		OCR3B = 23;
		OCR4A = 20;
	}
	
	
	// 23 er nullpunktet. 20 går med klokken en runde per sekund. 25 går mot klokken en runde per sekund..
	
	
}




void SonarSensor_init2_right(void)
{
	//DDRD = 0xFF;							// Port D all output.
	DDRD = ~(1<<DDD5);
	
	PORTD |= (1<<PORTD5);					// Enable pull up on D5 (echo)
	PORTD &= ~(1<<PIND4);					// Init D4 as low (trigger)
	
	PRR0 &= ~(1<<PRTIM2);					// To activate timer0 module
	TCNT2 = 0;								// Initial timer value
	TCCR2B |= (1<<CS22) | (1 << CS21);		// Timer with prescaler. Since default clock for atmega328p is 1Mhz period is 1uS
	PCICR  |= (1<<PCIE2);					// Enable PCINT[20:24] we use pin D4 which is PCINT20
	PCMSK2 |= (1<<PCINT21);					// Enable D5 interrupt
}


void SonarSensor_init1_front(void)
{	
	
	DDRC = ~(1<<DDC5);						// Set as input
	
	PORTC |= (1<<PORTC5);					// Enable pull up on C5 (echo)
	PORTC &= ~(1<<PINC4);					// Init C4 as low (trigger)
	
	PRR0 &= ~(1<<PRTIM1);					// To activate timer1 module
	TCNT1 = 0;								// Initial timer value
	TCCR1B |= (1<<CS12);					// Timer without prescaler. Since default clock for atmega328p is 1Mhz period is 1uS
	TCCR1B |= (1<<ICES1);					// First capture on rising edge
	PCICR  |= (1<<PCIE1);					// Enable PCINT[14:8] we use pin C5 which is PCINT13
	PCMSK1 |= (1<<PCINT13);					// Enable C5 interrupt
}


void SonarSensor_init0_left(void)
{
	//DDRB = 0xFF;							// Port B all output.
	DDRB = ~(1<<DDB1);
	
	PORTB |= (1<<PORTB1);					// Enable pull up on B1 (echo)
	PORTB &= ~(1<<PINB0);					// Init B0 as low (trigger)
	
	PRR0 &= ~(1<<PRTIM0);					// To activate timer0 module
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


void USART_putstring(char *StringPtr)
{
	while(*StringPtr != 0x00){
		USART_TransmitPolling(*StringPtr);
		StringPtr++;}
}

void batteryMonitoring_init()
{
	DDRC &= ~(1 << DDC0);																// Set C0 as input
	DDRC |= (1 << DDC1);																// Set C1 as output
	PORTC |= (1 << PINC0);																// Set internal pull-up for PINC0
	
	ADMUX = 0x00;																		// Only ADC0 is used
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);					// Enables ADC. Prescaler sets ADC frequency to 250 kHz.

}

void batteryMonitoring()
{
	ADCSRA |= (1 << ADSC);																// Starts ADC conversion
	
	if(batteryVoltage > 876)
	{
		PORTC |= (1 << PORTC1);
	}
	else
	{
		PORTC &= ~(1 << DDC1);
	}
	
	/* Maximum theoretical battery voltage 8.4 V. When battery voltage is 7.2 the battery is starting to be drained out. 
	A voltage divider of 1 kohm and 680 ohm makes sure the PC0 never reads anything above 5.0 V (8.4 V). When the battery is starting to drain out PC0 reads 4.28 V.
	
	ADC formula = (Vin*1024)/(VREF). VREF (AVCC shall be connected externally to VCC through LP-filter.
	
	*/
	
}



void SPI0_Master_init()
{

	DDRB |= (1<< DDB3) | (1<< DDB5);				// SET MOSI/SDA (pin3) and SCK (pin5) as output
	DDRB &= (1 << DDB2);							// SET nCS/nSS (pin2) as output. Drives the SLAVE.	
	
	PORTB |= (1 << PORTB2);							// Set nCS HIGH (inactive)
	
	// Enable SPI, Master, set clock rate fck/16 
	SPCR0 |= (1 << SPE) | (1<<MSTR) | (1<<SPR0);
	
}


void Write_SPI(char *screenData)
{
	SPI0_Transmitt(screenData);
	
	char strBatteryVoltage[2];
	
	sprintf(strBatteryVoltage, "%d", batteryVoltage); 

	SPI0_Transmitt(strBatteryVoltage);
	
}


void LCD_Send_Command()
{
	PORTB &= ~(1 << PORTB2);						// Sets nCS LOW (active)
	SPDR0 = 0x41;									// Display on
	_delay_us(100);
	SPDR0 = 0x51;									// Clears screen
	_delay_ms(2);	 
	PORTB &= (1 << PORTB2);							// Sets nCS HIGH (inactive)
	
}

void LCD_Clear_Screen()
{
	PORTB &= ~(1 << PORTB2);						// Sets nCS LOW (active)
	SPDR0 = 0x51;									// Clears screen
	_delay_ms(2);
	PORTB &= (1 << PORTB2);							// Sets nCS HIGH (inactive)
}



void SPI0_Transmitt(char *screenData)
{
	PORTB &= ~(1 << PORTB2);						// Sets nCS LOW (active)
	
	while(*screenData)
	{
		SPDR0 = screenData;							// Må kanskje bruke SPRD0 = *screenData;
		screenData++;
	}
	
	while(!(SPSR0 & (1<<SPIF)))						 // Waits until transmission is complete
	{
	}
	
	PORTB &= (1 << PORTB2);							// Sets nCS HIGH (inactive)
	
	
}



ISR(PCINT2_vect) {
	
	if (PIND & (1 << PIND5))															// Checks if echo is high
	{
		TCNT2 = 0;
		PORTB |= (1 << PINB5);															// Toggles debug led
	}
	
	else
	{
		distance2_right = TCNT2/3;														// Save Timer value
		PORTB &= ~(1 << PINB5);			    											// Toggles debug led
		//cli();
	}
}



ISR(PCINT1_vect) {
	
	if (PINC & (1 << PINC5))															// Checks if echo is high
	{
		TCNT1 = 0;		
		PORTB |= (1 << PINB5);															// Toggles Debug Led
	}
	
	else
	{
		distance1_front = TCNT1/3;														// Save Timer value
		PORTB &= ~(1 << PINB5);															// Toggles Debug led
		//cli();
	}
}


ISR(PCINT0_vect) {
	
	if (PINB & (1 << PINB1))															// Checks if echo is high
	{
		TCNT0 = 0;
		PORTB |= (1 << PINB5);															// Toggles debug led
	}
	
	else
	{
		distance0_left = TCNT0;															// Save Timer value
		PORTB &= ~(1 << PINB5);			    											// Toggles debug led
		//cli();
	}
}


ISR(ADC_vect)
{
	while(1)
	{
	if (ADCSRA & (1 << ADIF)) 															// Checks if ADC conversion is completed		
	{
		batteryVoltage = ADC;
		//ADCSRA &= ~(1 << ADSC);														// Writing it to 0 has no effect, sets to 0 automatically when ADC conversion is complete.
	}
	}
}
	
	
	