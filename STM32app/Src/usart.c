#include "usart.h"

void usartSendByte(USART_TypeDef * usart, uint8_t byte)
{
	usart->DR=	byte;
	while(!(usart->SR & 0x40)); //wait till it is sent
}


void usartWriteString(USART_TypeDef * usart, const char *string)
{
	int i=0;

	while(string[i]!='\0')
	{
		usartSendByte(usart, string[i]);// sending data on LCD byte by byte
		i++;
	}
}


void usartWriteNumber(USART_TypeDef * usart, uint32_t number)
{
	char arr[10];
	uint32_t tmp=number;
	uint8_t length=0;
	int8_t i=0;

	if(tmp==0) length=1;
	else
	{
		while(tmp)	// loop to find number of digits of which consist 'number'
		{
			tmp/=10;
			length++;
		}
	}

	arr[length]='\0';	//mark the end of recently created array containing digits of 'number'

	for(i=length-1 ; i>-1 ; i--)	//fill array with digits of 'number'
	{
		arr[i]='0'+number%10;
		number/=10;
	}

	usartWriteString(usart, arr);	//print array with digits as string
}
