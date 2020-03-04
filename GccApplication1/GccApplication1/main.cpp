#define F_CPU 8000000
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"
#define MYUBRR 51
float heading = 0;


#define COMPASS_ADDRESS_WRITE 0b00011010
#define COMPASS_ADDRESS_READ  0b00011011
int xhigh = 0, yhigh = 0;
int xlow = 0,  ylow = 0;



volatile unsigned long long counter_1 =0;
char buffer[64];
volatile int bufferIndex = 0;
char strBuffer[10];
int speed = 10;
int requested_speed = 120;

inline void USART_Init()
{
	UBRR0H = ((unsigned char)(MYUBRR>>8));
	UBRR0L = ((unsigned char)MYUBRR);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	UCSR0C = (3<<UCSZ00);
}

void serialSend(char* sendString){
	for (int i = 0; i < strlen(sendString); i++){
		while (( UCSR0A & (1<<UDRE0))  == 0){};
		UDR0 = sendString[i];
		_delay_ms(1);
	}
}
void Process_UART_RX(){
	if(bufferIndex > 0){
		serialSend("Speed: ");
		serialSend(buffer);
		int speed = 10;
		if(buffer[0]=='-')
		{
			PORTB &= 0b11001111;
			PORTB |= 0b00100000;
		}
		else if(buffer[0]!='0')
		{
			PORTB &= 0b11001111;
			PORTB |= 0b00010000;
		}else
		{
			PORTB &= 0b11001111;
		}
		buffer[0]=='0';
		requested_speed = abs(atoi(buffer));
		
		for(int i=0;i<64;i++)
		buffer[i] = 0;
		
		bufferIndex = 0;
		serialSend("\n");
	}
}


ISR(INT4_vect) {
	counter_1++;
}
ISR(USART0_RX_vect){
	char rcvChar = UDR0;
	if(rcvChar != '\n')
	{
		buffer[bufferIndex] = rcvChar;
		bufferIndex++;
	}
	
}

void initInterrupt4() {
	EIMSK = (1 << INT4); /* enable INT4 */
	EICRB = 1; /* trigger when button changes */
	PORTE = 0b00000000;
}

ISR(TIMER1_COMPA_vect) {
	
	itoa(counter_1 ,strBuffer,10);
	serialSend("S:");
	serialSend(strBuffer);
	itoa((int)heading,strBuffer,10);
	serialSend(" H:");
	serialSend(strBuffer);
	serialSend("\n");

	if(counter_1<requested_speed-5)
	speed += 2;
	else if(counter_1>requested_speed+5)
	speed -= 2;
	
	//speed = speed > 255 ? 255 : speed <= 0 ? 0 : speed;

	
	OCR2 = abs(255 - speed);


	counter_1 = 0;
}

static inline void initTimer1(void) {
	TCCR1B |= (1 << WGM12); /* CTC mode */
	TCCR1B |= (1 << CS11) | (1<<CS10);//(1 << CS12);
	// (1 << CS12) ---> 1 second
	//(1 << CS11) | (1<<CS10) 0.25 sec.
	TIMSK |= (1 << OCIE1A); /* enable output compare interrupt */
	OCR1A = 31250;
}
void initPWM(){
	TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<WGM20) |(1<<COM21) |(1<<COM20) |(1<<CS21) | (1<<CS20);
	OCR2 = 0xFF;
}

void Start_Compass_Device(){

	i2c_init();
	
	if(!i2c_start(COMPASS_ADDRESS_WRITE))
	{
		
		i2c_write(0x0B);
		i2c_write(0x01);
		i2c_stop();
		
	}


	_delay_ms(10);

	if(!i2c_start(COMPASS_ADDRESS_WRITE))
	{
		i2c_write(0x09);
		i2c_write(0x1D);
		i2c_stop();
	}
	
}

void Read_Heading(){

	uint8_t data[9];
	for(int i =0 ;i<9;i++)
	data[i] = 0;
	
	if(!i2c_readReg(COMPASS_ADDRESS_WRITE,COMPASS_ADDRESS_READ,6,data,1))
	{
		if((data[0] & 0x03) == 0x01){
			if(!i2c_readReg(COMPASS_ADDRESS_WRITE,COMPASS_ADDRESS_READ,0,data,9))
			{
				
				int x = data[1] << 8 | data[0];
				int y = data[3] << 8 | data[2];
				int z = data[5] << 8 | data[4];
				/*int temp = 0x7FFF & (data[8] << 8 | data[7]);*/
				
				if(x<xlow) xlow = x;
				if(x>xhigh) xhigh = x;
				if(y<ylow) ylow = y;
				if(y>yhigh) yhigh = y;

				/* Bail out if not enough data is available. */
				
				if( xlow==xhigh || ylow==yhigh ) return;

				/* Recenter the measurement by subtracting the average */

				x -= (xhigh+xlow)/2;
				y -= (yhigh+ylow)/2;

				/* Rescale the measurement to the range observed. */
				
				float fx = (float)x/(xhigh-xlow);
				float fy = (float)y/(yhigh-ylow);

				heading = 180.0*atan2(fy,fx)/M_PI;
				if(heading<=0) heading += 360;
				
			}
			else
			{
				serialSend("e1\n");
				_delay_ms(200);
				Start_Compass_Device();
			}
		}
		else
		{
			
			serialSend("e2\n");
			_delay_ms(200);
			Start_Compass_Device();
		}
	}
	else
	{
		
		_delay_ms(200);
		serialSend("e3\n");
		Start_Compass_Device();
	}
	
	
}




int main (void)
{
	for(int i=0;i<64;i++)
	buffer[i] = 0;
	
	DDRA = 0b00000011;
	DDRB = 0b10110000;
	
	PORTB = 0b10000000;
	
	USART_Init();
	initInterrupt4();
	initTimer1();
	initPWM();
	sei();
	
	Start_Compass_Device();
	
	for(;;){
		Process_UART_RX();
		Read_Heading();
		_delay_ms(50);	
	}
}
