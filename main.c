/*
 * PrintMashine.c
 *
 * Created: 07.02.2019 10:29:32
 * Author : Игорь
 */ 

#define F_CPU 16000000 // Рабочая частота контроллера
#define BAUD 9600 // Скорость обмена данными
#define UBRRL_value (F_CPU/(BAUD*16))-1 //Согластно заданной скорости подсчитываем значение для регистра UBRR
#define ADDRESS 1 //адрес в сети RS485

enum command{
	COMMAND_REPORT=0,
	COMMAND_NO_COMMAND
};

//запросы
enum requestType{
	REQUEST_EMPTY=0,
	REQUEST_GET_DATA,
	REQUEST_CLEAR
};

//ответы
enum answerType{
	ANSWER_OK=0,
	ANSWER_ERROR,
	ANSWER_NO_DATA,
	ANSWER_CLEARED
};


//события
enum eventType{
	EVENT_CONTROLLER_FAULT=0,
	EVENT_NO_TYPE,
	EVENT_NOT_READY,
	EVENT_NO_RESPONCE,
	EVENT_OK
};


struct minutePoint{
	unsigned int value;
	unsigned char event;
};

//103
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void configurationGPIO();
void configurationTimers();
void configuration();
void configureUART();
void configurationINTS();
void decodePacket();
unsigned char CRC16(unsigned char *pcBlock, unsigned short len);
void transmitReport();
void perpareDataPacket();
void prepareErrorPacket();//ответ-ошибка в случае несовпадения СRC
void prepareNoDataPacket();
void prepareClearedPacket();//ответ на требование стереть память

unsigned char USARTInputArray[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char reportsArray[100]={10,20,30,40};
struct minutePoint minutesArray[60];
unsigned int reportsCounter=0;
unsigned int timer0Counter=0;
unsigned char USARTCounter=0;//
unsigned char secondCounter=0;
unsigned char minutesCounter=0;
char USARTReciveEnd=0;
unsigned int currentValue=0;
struct minutePoint currentMinute;
int currentCommand=COMMAND_NO_COMMAND;
unsigned char tmp=0;//для отладки


///////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	configuration();
    /* Replace with your application code */
    while (1){
		switch(currentCommand){
			case(COMMAND_REPORT):{
				transmitReport();				
				break;	
			}
		}
    }
}
///////////////////////////////////////////////////////////////////////////////////
void configuration(){
	configureUART();
	configurationGPIO();
	configurationTimers();
	configurationINTS();
	sei();
}
////////////////////////////////////////////////////////////////////////////////
void configureUART(){
	UBRR0L = 103;//Младшие 8 бит UBRRL_value
	UBRR0H = 103>>8;//Старшие 8 бит UBRRL_value
	UCSR0B|=(1<<RXEN0);//разрешение приема
	UCSR0B|=(1<<RXCIE0);//разрешения прерывания по завершению приемa
	UCSR0B|=(1<<TXCIE0);//разрешение прерывания по завершении передачи
	UCSR0C|=(3<<UCSZ00);//формат 8 бит данных
}
////////////////////////////////////////////////////////////////////////
void configurationGPIO(){
	DDRB|=(1<<PORTB1);
	PORTB|=(1<<PORTB1);
	DDRC|=(1<<PORTC4);//переключение вход/выход преобразователя
	PORTC|=(1<<PORTC4);
	int t=0;
	for(int n=0;n!=255;n++){
		t++;
	}
	PORTC&=(~(1<<PORTC4));//режим приема
}
////////////////////////////////////////////////////////////////////////////////
void configurationTimers(){
	TIMSK0|=(1<<TOIE0);
	TCNT0=230;
	TCCR0B|=(1<<CS00)|(1<<CS02);
}
/////////////////////////////////////////////////////////////////////////////////
void configurationINTS(){
	EIMSK|=(1<<INT0);
	EICRA|=(1<<ISC00);//прерывание INT0 по любому изменению уровня
}
/////////////////////////////////////////////////////////////////////////////////
ISR(USART_RX_vect){
	USARTInputArray[USARTCounter]=UDR0;	
	USARTCounter++;	
	if(USARTInputArray[0]==USARTCounter){
		USARTReciveEnd=1;
		USARTCounter=0;
		decodePacket();	
	}
	else if(USARTInputArray>10){
		USARTReciveEnd=1;
	}
}
////////////////////////////////////////////////////////////
ISR(USART_TX_vect){
	if(reportsCounter!=reportsArray[0]){//если переданный байт не был последним
		currentCommand=COMMAND_REPORT;//то продолжаем
	}
	else{//иначе переключаемся на прием
		UCSR0B&=(~(1<<TXEN0));//запрет передачи
		UCSR0B|=(1<<RXEN0);//разрешение приема
		TCNT0=0;
		PORTC&=(~(1<<PORTC4));//режим приема
		TCCR0B|=(1<<CS00)|(1<<CS02);//запускаем таймер
	}	
}
/////////////////////////////////////////////////////////////////
ISR(TIMER0_OVF_vect){
	
	TCNT0=230;
	timer0Counter++;
	if(timer0Counter==625){
		secondCounter++;
		if(secondCounter==60){
			secondCounter=0;
			if(minutesCounter==60){//если массив не запрошен в течении часа, то начинаем с начала
				minutesCounter=0;
			}
			currentMinute.value=136;//currentValue;
			currentMinute.event=EVENT_OK;
			currentValue=0;
			minutesArray[minutesCounter]=currentMinute;
			minutesCounter++;
		}
		timer0Counter=0;
	}
}
///////////////////////////////////////////////////////////////
ISR(INT0_vect){
	currentValue++;
}
/////////////////////////////////////////////////////////////////
void decodePacket(){
	if(USARTInputArray[1]==ADDRESS){
		unsigned char size=USARTInputArray[0];
		unsigned char crc=CRC16(USARTInputArray,size-1);
		if(crc==USARTInputArray[size-1]){
			switch(USARTInputArray[2]){//3-й байт - команда	
				case(REQUEST_GET_DATA):{
					if(minutesCounter!=0){
						prepareDataPacket();
					}
					else{
						prepareNoDataPacket();
					}
					break;	
				}
				case(REQUEST_CLEAR):{
					minutesCounter=0;
					prepareClearedPacket();
					break;
				}
				default:{
					return;
				}
			}
		}
		else{
			prepareErrorPacket();
		}
		currentCommand=COMMAND_REPORT;
	}
	else{
		PORTB|=(1<<PORTB1);
	}
}
///////////////////////////////////////////////////////////////////////////
unsigned char CRC16(unsigned char *pcBlock, unsigned short len){
	unsigned short crc=0xFFFF;
	unsigned char i;
	while(len--){
		crc^= *pcBlock++ << 8;
		for(i=0;i<8;i++){
			crc=crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
		}
	}
	return crc;
}
//////////////////////////////////////////////////////////////////////
void prepareDataPacket(){
	TCCR0B&=(~(1<<CS00))&(~(1<<CS02));//останавливаем таймер
	unsigned char size=minutesCounter*3+4;//количество минут+размер+CRC16+ответ
	reportsArray[0]=size;//размер пакета
	reportsArray[1]=(unsigned char)ANSWER_OK;
	reportsArray[2]=minutesCounter;//количество минут в пакете
	
	int n=0;
	for(n=0;n!=minutesCounter;n++){
		int offset=n*3+3;
		reportsArray[offset]=(unsigned char)minutesArray[n].value;
		reportsArray[offset+1]=(unsigned char)minutesArray[n].value>>8;
		reportsArray[offset+2]=currentMinute.event;
	}
	unsigned char crc=CRC16(reportsArray,size-1);
	reportsArray[3+n*3]=crc;
	//все обнуляем
	USARTCounter=0;
	reportsCounter=0;
	TCNT0=230;
}
/////////////////////////////////////////////////////////////////////////
void prepareErrorPacket(){
	unsigned char size=3;//размер+ответ+CRC
	reportsArray[0]=size;//размер пакета
	reportsArray[1]=(unsigned char)ANSWER_ERROR;
	unsigned char crc=CRC16(reportsArray,size-1);
	reportsArray[2]=crc;
	reportsCounter=0;
}
//////////////////////////////////////////////////////////////////////////
void prepareNoDataPacket(){
	unsigned char size=3;//размер+ответ+CRC
	reportsArray[0]=size;//размер пакета
	reportsArray[1]=(unsigned char)ANSWER_NO_DATA;
	unsigned char crc=CRC16(reportsArray,size-1);
	reportsArray[2]=crc;
	reportsCounter=0;
}
/////////////////////////////////////////////////////////////////////////
void prepareClearedPacket(){
	unsigned char size=3;//размер+ответ+CRC
	reportsArray[0]=size;//размер пакета
	reportsArray[1]=(unsigned char)ANSWER_CLEARED;
	unsigned char crc=CRC16(reportsArray,size-1);
	reportsArray[2]=crc;
	reportsCounter=0;
	PORTB&=(~(1<<PORTB1));
}
/////////////////////////////////////////////////////////////////////////
void transmitReport(){
	while ( !( UCSR0A & (1<<UDRE0)) );
	PORTC|=(1<<PORTC4);//режим передачи 
	UCSR0B&=(~(1<<RXEN0));//запрет на прием
	UCSR0B|=(1<<TXEN0);//разрешение передачи
	UDR0=reportsArray[reportsCounter];
	reportsCounter++;
	currentCommand=COMMAND_NO_COMMAND;
}