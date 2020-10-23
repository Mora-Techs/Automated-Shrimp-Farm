
#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif


#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTC6
#define EN eS_PORTC7

#include <avr/io.h>
#include <util/delay.h>
//#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "USART_RS232_H_file.h"		/* include USART library */

//#include <util/delay.h>

#include "lcd.h"
#define B PORTB
#define C PORTC

#define digitalWrite(port, bit, state) \
if(state == 1){ \
	(port) |= (1 << (bit)); \
} \
else if(state == 0){ \
	(port) &= ~(1 << (bit)); \
}
void motor(void);
void servo(void);
void pumpsensor(void);
void M1ONN();
void M1OFF();
void M2ONN();
void M2OFF();
void M3ONN();
void M3OFF();
void M4ONN();
void M4OFF();
void paddlewheel();
void feeders();
int ph();
void uart_init (void);
void uart_transmit  (uint8_t data);
void uart_print(char *data);
unsigned char uart_recieve (void);
void InitADC();
uint16_t ReadADC(uint8_t ch);						/* initialize USART with 9600 baud rate */
void uart_init (void)
{
	UBRRH=(BAUDRATE>>8);
	UBRRL=BAUDRATE;
	UCSRB|=(1<<TXEN)|(1<<RXEN);
	UCSRC|=(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
}

void uart_transmit  (uint8_t data)
{
	while (!( UCSRA & (1<<UDRE)));
	UDR = data;
}

void uart_print(char *data){
	for(int i=0;i<strlen(data);i++){
		uart_transmit(data[i]);
	}
}

unsigned char uart_recieve (void)
{
	while(!(UCSRA) && (1<<RXC));
	return UDR;
}

void InitADC() //needed ---------------------------------
{
	ADMUX=(1<<REFS0);                         // For Aref=AVcc;
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescalar div factor =128
}

uint16_t ReadADC(uint8_t ch) //needed---------------------------
{
	//Select ADC Channel ch must be 0-7
	ch=ch&0b00000111;
	ADMUX|=ch;

	//Start Single conversion
	ADCSRA|=(1<<ADSC);

	//Wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));

	//Clear ADIF by writing one to it
	//Note you may be wondering why we have write one to clear it
	//This is standard way of clearing bits in io as said in datasheets.
	//The code writes '1' but it result in setting bit to '0' !!!

	ADCSRA|=(1<<ADIF);

	return(ADC);
}

void M1ONN()//MOTOR 1 ONN
{
	digitalWrite(B, 2, 1);
	digitalWrite(B, 3, 0);
}

void M1OFF()//MOTOR 1 OFF
{
	digitalWrite(B, 2, 0);
	digitalWrite(B, 3, 0);
}
void M2ONN()//MOTOR 2 ONN
{
	digitalWrite(B, 0, 1);
	digitalWrite(B, 1, 0);
}

void M2OFF()//MOTOR 2 OFF
{
	digitalWrite(B, 0, 0);
	digitalWrite(B, 1, 0);
}
void M3ONN()//MOTOR 3 ONN
{
	digitalWrite(B, 4, 1);
	digitalWrite(B, 5, 0);
}

void M3OFF()//MOTOR 3 OFF
{
	digitalWrite(B, 4, 0);
	digitalWrite(B, 5, 0);
}
void M4ONN()//MOTOR 4 ONN
{
	digitalWrite(C, 0, 1);
	digitalWrite(C, 1, 0);
}

void M4OFF()//MOTOR 4 OFF
{
	digitalWrite(C, 0, 0);
	digitalWrite(C, 1, 0);
}

void paddlewheel()
{
	 DDRB=0xFF;//MAKE PORT OUTPUT TO CONNECT MOTOR PINS
	 DDRC=0xFF;//MAKE PORT OUTPUT TO CONNECT MOTOR PINS
	 Lcd4_Write_String("PADDLE*WHEELS*ON");
	 USART_SendString("PADDLE*WHEELS*ON");
	 _delay_ms(1000);
	 M1ONN();
	 _delay_ms(7000);
	 M1OFF();
	 Lcd4_Clear();
	 Lcd4_Write_String("PADDLE*WHEELS*OFF");
	 USART_SendString("PADDLE*WHEELS*OFF");
	  _delay_ms(2000);
	 Lcd4_Clear();
	
}

void feeders()
{
	 DDRB=0xFF;//MAKE PORT OUTPUT TO CONNECT MOTOR PINS
	 DDRC=0xFF;//MAKE PORT OUTPUT TO CONNECT MOTOR PINS
	 Lcd4_Write_String("INITIALIZING");
	 Lcd4_Set_Cursor(2,1);
	 Lcd4_Write_String("FEEDERS");
	 _delay_ms(3000);//E
	 Lcd4_Clear();
	 Lcd4_Write_String("FEEDERS ON");
	 USART_SendString("                                                 FEEDERS ON");
	 M2ONN();
	 _delay_ms(7000);
	 M2OFF();
	 Lcd4_Clear();
	 Lcd4_Write_String("FEEDERS OFF");
	 USART_SendString("                                                 FEEDERS OFF");
	 _delay_ms(2000);
	 Lcd4_Clear();
	
}



int ph(void)
{
	InitADC();
	uint16_t temp;
	//float val;
	uart_init();
	DDRB = 0xff;
	float ph;
	//char data[10];
	/* Replace with your application code */
	while (1)
	{
		
		//reading
		Lcd4_Write_String("PH VALUE");
		_delay_ms(2000);
		Lcd4_Clear();
		temp = ReadADC(0);
		ph = (14-(temp*14.0/1024));
		ph = ph - pow((7-ph),2)*0.6;
		int k=(int)ph;
		//////////////////////////////
		if(k>7)
		{
			//ph less than 7
			Lcd4_Write_String("PH VALUE >7");
			_delay_ms(2000);
			Lcd4_Clear();
			Lcd4_Write_String("PH neutalized");
			USART_SendString("                                                            PH neutalized");
			M4ONN();
			_delay_ms(7000);
			M4OFF();
			Lcd4_Clear();
			Lcd4_Write_String("neutralization done");
			USART_SendString("                                                            neutalization done");
			_delay_ms(2000);
			break;
		}
		if(k<7)
		{
			//ph greater than 7
			Lcd4_Write_String("PH VALUE <7");
			_delay_ms(2000);
			Lcd4_Clear();
			Lcd4_Write_String("PH neutalized");
			USART_SendString("                                                             PH neutalized");
			M3ONN();
			_delay_ms(7000);
			M3OFF();
			Lcd4_Clear();
			Lcd4_Write_String("neutralization done");
			USART_SendString("                                                             neutalization done");
			_delay_ms(2000);
			break;
		}
		
		//PORTB = 0xff;
		//_delay_ms(1000);
		//PORTB = 0x00;
		//_delay_ms(1000);
		//sprintf(data,"ph = %.2f",ph);
		
		//uart_print(data);
		//uart_transmit('K');
		//_delay_ms(3000);
	}
	return 0;
}

int main(void)
{
   USART_Init(9600);
   DDRD = 0xFF;                          
   DDRC = 0xFF;
   PORTC|=(1<<PINC2);                  //initialy turned pump off
   int i=0;
   Lcd4_Init();
   while(1)
   {
       i=i+1;
	   Lcd4_Clear();
	   Lcd4_Set_Cursor(1,1);
	   Lcd4_Write_String("Automatic shrimp");
	   _delay_ms(1000);
	   Lcd4_Set_Cursor(2,1);
	   Lcd4_Write_String("******FARM******");
	   _delay_ms(1000);
	   Lcd4_Clear();
	   //ph();
	   pumpsensor();
	   if(i==3)
	   {
		 //motor();//motor function
		 //PADDLEFUNCTION()
		 //ph();
		 paddlewheel();
		 _delay_ms(3000);
		 feeders();
		 _delay_ms(3000);
		 ph();
		 _delay_ms(3000);
		 //servo();//servo function
		 //FEEDERSFUNCTION();
		 //CHECKPH()
	 	 i=0;  
		   
	   }
	   
	   Lcd4_Set_Cursor(2,1);
	   _delay_ms(100);
   }
}




void motor(void)
{
	DDRB |= 1<<PINB3; //PORTB as Output
		//Rotates Motor in Antilockwise
		Lcd4_Write_String("PADDLE*WHEELS*ON");
		USART_SendString("                                                 PADDLE*WHEELS*ON");
		PORTB |= 1<<PINB3; //00000001
		_delay_ms(1000);
        Lcd4_Clear();
		//Stops Motor
		Lcd4_Write_String("PADDLE*WHEELS*OFF");
		USART_SendString("                                                PADDLE*WHEELS*OFF");
		PORTB &= ~(1<<PINB3) ; //00000000
		_delay_ms(1000);
        Lcd4_Clear();
		
}


void servo(void)
{
	int a=0;
	DDRC |= 1<<PINC0; //Makes RC0 output pin
	PORTC &= ~(1<<PINC0);
	Lcd4_Write_String("INITIALIZING");
	Lcd4_Set_Cursor(2,1); 
	Lcd4_Write_String("FEEDERS");
	_delay_ms(100);//E
	while(1)
	{
		if(a==2)
		{
			break;
		}
		
		//Rotate Motor to 0 degree
		//Lcd4_Write_String("FEEDERS ON");
		_delay_us(100);
		PORTC |= 1<<PINC0;
		_delay_us(100);
		PORTC &= ~(1<<PINC0);

		_delay_ms(200);

		//Rotate Motor to 180 degree
		PORTC |= 1<<PINC0;
		_delay_us(150);
		PORTC &= ~(1<<PINC0);

		_delay_ms(200);
		
		//Rotate Motor to 0 degree
		PORTC |= 1<<PINC0;
		_delay_us(100);
		PORTC &= ~(1<<PINC0);

		_delay_ms(200);
		a=a+1;
		Lcd4_Clear();
		Lcd4_Write_String("FEEDING IN ");
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String("PROGRESS");
		_delay_us(100);
		Lcd4_Clear();
		//Lcd4_Write_String("FEEDING COMPLETED");
		_delay_us(100);
		Lcd4_Clear();
		
	}
	Lcd4_Write_String("FEEDING COMPLETED");
}


void pumpsensor(void)
{
	DDRC |=1<<PINC2;                    //Defining PC1 as output FOR WATER PUMP
	DDRA &=~(1<<PINA1);                 //DATA DIRRECTION REGISTER INPUT AS PINB1 FOR WATER SENSOR CHANGED FROM B1
	PORTC|=(1<<PINC2);                  //initialy turned pump off
	//_delay_ms(1000);
	while(1)
	{
		if(bit_is_clear(PINA,1))//this will return non zero if bit is clear,0 if bit is set
		{
			Lcd4_Write_String("tank not full");
			_delay_ms(1000);
			Lcd4_Clear();
			Lcd4_Write_String("                                           water pump on");
			USART_SendString("                                            water pump on");
			_delay_ms(1000);
			Lcd4_Clear();
			PORTC&=~(1<<PINC2);//turns  the water pump on
			
		}
		else 
		{
			Lcd4_Write_String("tank full");
			_delay_ms(1000);
			Lcd4_Clear();
			Lcd4_Write_String("                                            water pump off");
			USART_SendString("                                             water pump oFF");
			_delay_ms(1000);
			Lcd4_Clear();
			PORTC|=(1<<PINC2 );//turns water pump off
			break;
		}
		
		
		
	}
}