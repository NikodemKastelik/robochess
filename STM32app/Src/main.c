/*
 * RoboChess ver 4.0
 * main.c
 *
 *  Created on: 7 lut 2017
 *      Author: Nikodem Kastelik
 *
 * Changelog:
 * ver 4.0:
 * + added joint control
 * + added 5th axis
 * + added queue
 * + added movePiece function
 * + added Usart4 functionality for Bluetooth dongle
 * - suspended list of commands to execute
 * - suspended LCD display control due to RasPi GUI
 *
 * @TODO:
 * > implement capture move command as series of move commands
 *
 */

#include "stm32f4xx.h"
#include <inttypes.h>
#include "fast_trig.h"
#include "my_string.h"
#include "messages.h"
#include "my_queue.h"
#include "usart.h"

//***** Glue macro ******//

#define GLUE(x,y) x ## y
#define GLUE2(x,y) GLUE(x,y)
//#define BUS_PORT GLUE(PORT, BUS)

//***** Definitions *****//

#define PI  		3.141592f
#define CPU_FREQ 	16000000
#define	LED_PIN 	5

#define DEBUG_UART	USART2
#define CMD_UART	UART4

#define PWM_1234_TIM	TIM3
#define PWM_56_TIM		TIM4
#define PWM_INCR_TIM	TIM7
#define LED_BLINK_TIM	TIM6

// lengths of robot elements

#define OA 83
#define	AB 220
#define	BC 220
#define	CT 98

// servo-related definitions

#define GRIPPER_CLOSED	0
#define GRIPPER_OPENED	1

#define MOVE_GLOBAL_DOF		4
#define MOVE_JOINT_DOF		5

#define MOVE_PIECE_COORDS	6
#define PIECE_FROM_POS_OFFSET	0
#define	PIECE_TO_POS_OFFSET		3

#define NO_OF_SERVOS 	5
//#define SERVO_INC		50

/*
#define servo_base_Max	2384
#define servo_base_Min	560

#define servo_A_Max		2304
#define servo_A_Min		816

#define servo_B_Max		2368
#define servo_B_Min		720

#define servo_C_Max		2176
#define servo_C_Min		560

#define servo_D_Max		0
#define servo_D_Min		0
*/

// servo calibration values

#define SERVO0_A	572.957795f
#define SERVO0_B	600.0f

#define SERVOA_A	643.718625f
#define SERVOA_B	277.593099f

#define SERVOB_A	-654.5700931f
#define SERVOB_B	2583.091787f

#define SERVOC_A	-576.571329f
#define SERVOC_B	2467.063624f

#define SERVOD_A	0
#define SERVOD_B	0


#define J1_MIN		-90
#define J1_MAX		-90

#define J2_MIN		0
#define J2_MAX		180

#define J3_MIN		0
#define J3_MAX		180

#define J4_MIN		270
#define J4_MAX		360

#define J5_MIN		-90
#define J5_MAX		90

//#define servo_gripper_close 0x500
//#define servo_gripper_open	0x600


#define X 0
#define Y 1
#define Z 2
#define OR 3

#define J1 0
#define J2 1
#define J3 2
#define J4 3
#define J5 4

// array lengths

#define RX_BUFFER_SIZE	40	// buffer for USART reception

#define CMD_LIST_SIZE	30  // array with commands for automatic command execution

#define INTERVAL		300 // time to wait between executions

const char command_err[]="Wrong format\n";

//--------------------------------------------------------------------------------------------------//
//******************************** Global variables ************************************************//
//--------------------------------------------------------------------------------------------------//

volatile myQueue queue;

volatile uint32_t* pos_curr[NO_OF_SERVOS]; 	// current position ; array of pointers pointing to TIM4 CCR registers
volatile uint32_t pos_dest[NO_OF_SERVOS]; 	// desired position (measured in PWM duty cycle)

volatile int cmd_list[CMD_LIST_SIZE]; 		// array consisting of points numbers for automatic command execution
volatile int exec_count=0;					// number of automatic command executions

volatile int pos_reached_flag = 1;			// flag indicating if desired position is reached

volatile uint16_t servo_gripper_close = 0x0500;
volatile uint16_t servo_gripper_open = 0x0600;

volatile uint16_t servo_inc_speed = 20;
volatile uint16_t servo_inc_time_ms = 50;

const float SERVOS_A[NO_OF_SERVOS] = {572.957795f, 643.718625f, -654.5700931f, -576.571329f, 0};
const float SERVOS_B[NO_OF_SERVOS] = {600.0f, 277.593099f, 2583.091787f, 2467.063624f, 0};
const uint16_t SERVOS_MIN[NO_OF_SERVOS] = {700, 500, 500, 700, 600};
const uint16_t SERVOS_MAX[NO_OF_SERVOS] = {2300, 1700, 2400, 2300, 2400};

//--------------------------------------------------------------------------------------------------//
//************************************* Functions **************************************************//
//--------------------------------------------------------------------------------------------------//

// >> LCD routines

/*
void lcd_write_number(uint32_t number)
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

	arr[length]='\0';	//mark the end of newly created array containing digits of 'number'

	for(i=length-1 ; i>-1 ; i--)	//fill array with digits of 'number'
	{
		arr[i]='0'+number%10;
		number/=10;
	}

	lcd_write_string(arr);	//print array with digits as string
}*/

//void lcd_update(uint16_t measurement)
//{
//	lcd_serial_command(CLEAR);
//	lcd_write_number(measurement);
//}

// >> USART routines

/*void usart2_send8bit(uint8_t byte)
{
	USART2->DR=	byte;
	while(!(USART2->SR & 0x40)); //wait till it is sent
}

void usart2_write_string(const char *string)
{
	int i=0;

	while(string[i]!='\0')
	{
		usart2_send8bit(string[i]);// sending data on LCD byte by byte
		i++;
	}
}

void usart2_write_number(uint32_t number)
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

	usart2_write_string(arr);	//print array with digits as string
}

uint8_t is_digit(char input)
{
	if(input<('9'+1) && input>('0'-1)) return 1;
	else return 0;
}


void usartWriteString(USART_TypeDef * usart_interface, const char * string2send)
{

}*/

// >> Robot movement and input recognition routines

float fast_sqrt(float x)
{
	union
	{
		int i;
		float x;
	} u;

	u.x = x;
	u.i = (1<<29) + (u.i >> 1) - (1<<22);

	// Two Babylonian Steps (simplified from:)
	// u.x = 0.5f * (u.x + x/u.x);
	// u.x = 0.5f * (u.x + x/u.x);
	u.x =       u.x + x/u.x;
	u.x = 0.25f*u.x + x/u.x;

	return u.x;
}

void gripper_open()
{
	TIM3->CCR2 = servo_gripper_open;
}

void gripper_close()
{
	TIM3->CCR2 = servo_gripper_close;
}

void moveToPoint(float *pos) //inverse kinematics - move to coordinates in pos=[x][y][z][o]
{

	float joint_angles[NO_OF_SERVOS];

	float gripper_angle = (pos[OR])*(PI/180);

	float v = fast_sqrt(pos[X]*pos[X]+pos[Y]*pos[Y]);

	joint_angles[J1] = fast_acos(pos[X]/v);

	float cv = v - CT*fast_cos(gripper_angle);
	float cz = pos[Z] - CT*fast_sin(gripper_angle);

	float AC_sq=((cv*cv)+(cz-OA)*(cz-OA));
	float AC=fast_sqrt(AC_sq);

	float AC_OX = fast_atan((cz-OA)/(cv));

	if(cv<0) AC_OX = PI+AC_OX;

	joint_angles[J2] = AC_OX + fast_acos(((AB*AB)+AC_sq-(BC*BC))/(2*AB*AC));

	joint_angles[J3] = fast_acos(((AB*AB)+(BC*BC)-AC_sq)/(2*AB*BC));

	joint_angles[J4] = 2*PI + gripper_angle - joint_angles[J2] - joint_angles[J3];

	if(joint_angles[J4]>2*PI)
	{
		joint_angles[J4] = joint_angles[J4] - ( (int)(joint_angles[J4]/(2*PI)) * 2*PI);
	}

	for(uint8_t i = 0 ; i < NO_OF_SERVOS ; i++)
	{
		uint16_t pwm_val = (uint16_t)(joint_angles[i] * SERVOS_A[i] + SERVOS_B[i]);
		if(pwm_val < SERVOS_MIN[i])
		{
			pwm_val = SERVOS_MIN[i];
		}
		if(pwm_val > SERVOS_MAX[i])
		{
			pwm_val = SERVOS_MAX[i];
		}
		pos_dest[i] = pwm_val;
	}
	//pos_dest[0] = ((base_angle/(PI))*(servo_base_Max-servo_base_Min)+servo_base_Min);
	//pos_dest[1] = (servo_A/(PI))*(servo_A_Max-servo_A_Min) + servo_A_Min;
	//pos_dest[2] = (servo_B/(PI))*(servo_B_Max-servo_B_Min) + servo_B_Min;
	//pos_dest[3] = servo_C_Max-((servo_C-(PI/2))/(PI))*(servo_C_Max-servo_C_Min);

	pos_reached_flag = 0;	//disable flag

}

void moveJoint(float *pos)  // pos = [j1][j2][j3][j4][j5]
{
	;
}

//transfer coordinates in format: [digit][digit][,][digit][,][digit] etc.
// to array 'pos' in format: [number][number][number];
void recog_coordinates(char* arr, float* pos, uint8_t num_of_coordinates)
{
	int j=0;
	int sign;

	for(int i=0 ; i<num_of_coordinates ; i++)
	{
		pos[i]=0;

		if(arr[j]=='-')
		{
			sign=-1;
			j++;
		}
		else sign=1;

		int length=0;

		while(arr[j] != ',' && arr[j] != '\0')
        {
            length++;
            j++;
        }

        j++;

		int mux = 1;
		for(int k = j-2 ; k>(j-2-length) ; k--)
		{
			pos[i] += ((arr[k]-'0')*mux);
			mux *= 10;
		}

		pos[i] *= sign;
	}
}

void recog_input(char *arr)	//recognise command by analyzing usart rx buffer
{
	if(str_str(arr, MSG(MSG_MOVE_GLOBAL)))
	{
		float pos[MOVE_GLOBAL_DOF];
		recog_coordinates(&arr[MSG_MOVE_GLOBAL_OFFSET], pos, MOVE_GLOBAL_DOF);
		moveToPoint(pos);
		TIM6->DIER|=	0x00000001U;
	}
	else if(str_str(arr, MSG(MSG_MOVE_JOINT)))
	{
		float pos[MOVE_JOINT_DOF];
		recog_coordinates(&arr[MSG_MOVE_JOINT_OFFSET], pos, MOVE_JOINT_DOF);
		moveJoint(pos);
		TIM6->DIER|=	0x00000001U;
	}
	else if(str_str(arr, MSG(MSG_MOVE_PIECE)))
	{
		float pos[MOVE_PIECE_COORDS]; // stores: p1x,p1y,p1z,p2x,p2y,p2z
		recog_coordinates(&arr[MSG_MOVE_PIECE_OFFSET], pos, MOVE_PIECE_COORDS );
		// perform piece move from p1 to p2:
		// 1. move to p1 with Z 50mm over piece

		// 2. open gripper

		// 3. move to p1

		// 4. close gripper

		// 5. lift piece 80mm over chessboard

		// 6. move to p2 with

		// 7.
	}

	else if(str_str(arr, MSG(MSG_CAPTURE_PIECE)))
	{
		float pos[MOVE_PIECE_COORDS]; // stores: p1x,p1y,p1z,p2x,p2y,p2z
		recog_coordinates(&arr[MSG_MOVE_PIECE_OFFSET], pos, MOVE_PIECE_COORDS );
		// perform capture move from p1 to p2:
		// 1. move to p2 with Z 50mm over piece
		// 2. open gripper
		// 3. move to p2
		// 4. catch piece
		// 5. lift piece
		// 6. rotate base as much to side as possible
		// 7. drop piece
		// 8. move to p1 with Z 50mm over piece
		// 9. open gripper
		// 10. move to p1
		// 11. grab piece
		// 12. lift piece
		// 13. move to p2 with Z 80mm over chessboard
		// 14. move to p2
		// 15. release piece
	}

	else if(str_str(arr, MSG(MSG_GRIPPER)))
	{
		static uint8_t gripper_state = GRIPPER_CLOSED;
		if(gripper_state == GRIPPER_OPENED)
		{
			gripper_close();
		}
		else if(gripper_state == GRIPPER_CLOSED)
		{
			gripper_open();
		}
	}

	else if(str_str(arr, "LED"))
	{
		GPIOA->ODR ^= 	0x00000020U;
		usartWriteString(USART2, "LED_OK\n");
	}
	else if(str_str(arr, "SPEED="))
	{
		servo_inc_speed = string_to_num(&arr[6]);
	}
	/*
	static float pos_list[10][4] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};	// array with memorized points

	if( arr[0] == 'm' &&
		arr[1] == 'o' &&
		arr[2] == 'v' &&
		arr[3] == ' ')
	{
		if(arr[4]=='p' && is_digit(arr[5])) // [m][o][v][_][p][1];
		{
			if(pos_list[(arr[5]-'0')][0] != -1)
			move(&pos_list[(arr[5]-'0')][0]);
		}
		else // [m][o][v][_][x][,][y][,][z][,][o][,];
		{
			float pos[4];
			recog_coordinates(&arr[4], &pos[0]);
			move(pos);
		}

		TIM6->DIER|=	0x00000001U; //enable tim6 interrupts in order to reach desired position
	}else

	if( arr[0] == 'p' &&	//[p][1][=][x][,][y][,][z][,][o][,]
		is_digit(arr[1]) &&
		arr[2] == '=' )
	{
		recog_coordinates(&arr[3], &pos_list[(arr[1]-'0')][0]);
	}else

	if( arr[0] == 'l' &&
		arr[1] == 'i' &&
		arr[2] == 's' &&
		arr[3] == 't' &&
		arr[4] == '=' )
	{
		int i=5;
		int j=0;
		while(arr[i] != '\0')
		{
			if( is_digit(arr[i]) || arr[i]=='o' || arr[i]=='c' || arr[i]=='d' )
			{
				cmd_list[j++] = arr[i++];
			} else break;
		}

		cmd_list[j]='\0';
	}else

	if( arr[0] == 'e' &&
		arr[1] == 'x' &&
		arr[2] == 'e' &&
		arr[3] == 'c' )
	{
		if(is_digit(arr[5])) exec_count = arr[5]-'0';
		else exec_count = -1;
	}else

	if( arr[0] == 'o') gripper_open();
	else if( arr[0] == 'c') gripper_close();

	else usart2_write_string(command_err);*/
}

int check_if_reached()
{
	int i;
	for(i=0 ; i<NO_OF_SERVOS ; i++)
	{
		if((*pos_curr[i])!=pos_dest[i]) return 0;
	}

	return 1;
}


void recog_debug_command(const char * arr)
{
	;
}

//-----------------------------//
//***** Interrupt Handlers *****//

void USART2_IRQHandler()//USART2_IRQHandler() //for debugging
{
	static uint8_t i=0;
	static char arr[RX_BUFFER_SIZE];

	if(USART2->SR & 0x20)
	{
		uint8_t byte = USART2->DR;

		if(byte != 0x0d && i!=(RX_BUFFER_SIZE-1))	// 0x0d is carriage return ascii code (PuTTY send this char when enter is clicked)
			arr[i++] = byte;
		else
		{
			arr[i] = '\0';			// mark end of array
			i=0;
			//lcd_serial_command(CLEAR);
			//lcd_write_string(arr);
			USART2->SR &= ~0x00000020U;		// clear RXNE flag
			byte = USART2->DR;				// artificially clear ORE flag in order to prevent
											// this IRQ to loop infinitely
			queue.put(&queue, arr);
			//recog_debug_command(arr);
			//move(pos);
		}
	}
}

void UART4_IRQHandler() //for bluetooth command reception
{
	static uint8_t i=0;
	static char arr[RX_BUFFER_SIZE];

	if(UART4->SR & 0x20)
	{
		uint8_t byte = UART4->DR;

		if(byte != 0x0d && i!=(RX_BUFFER_SIZE-1))	// 0x0d is carriage return ascii code (PuTTY send this char when enter is clicked)
			arr[i++] = byte;
		else
		{
			arr[i] = '\0';			// mark end of array
			i=0;
			//lcd_serial_command(CLEAR);
			//lcd_write_string(arr);
			UART4->SR &= ~0x00000020U;		// clear RXNE flag
			byte = UART4->DR;				// artificially clear ORE flag in order to prevent
											// this IRQ to loop infinitely
			queue.put(&queue, arr);
			//recog_input(arr);
			//move(pos);

		}
	}
}


void TIM7_IRQHandler()	//blink diode to indicate proper work of system
{
	TIM7->SR &=~	0x00000001U; 		//clear interrupt flag
	//ADC1->CR2|=		0x40000000U; 	// ADC conversion start
	GPIOA->ODR ^= 	0x00000020U; 		//pin5 portA (LED diode) toggle

	//usartWriteString(UART4, MSG(MSG_BT_ALIVE));
}


void TIM6_DAC_IRQHandler()	// increment PWM register every some time in order to provide seamless movement
{
	//*pos_curr[3]=pos_dest[3];	//move third arm instantly to desired position

	if(pos_reached_flag != 1)
	{
		for(int i=0 ; i< NO_OF_SERVOS ; i++)
		{
			if((*pos_curr[i])<pos_dest[i])
			{
				if((pos_dest[i]-(*pos_curr[i]))<servo_inc_speed)
					(*pos_curr[i])=pos_dest[i];

				else (*pos_curr[i])=(*pos_curr[i])+servo_inc_speed;
			}

			else if((*pos_curr[i])>pos_dest[i])
			{
				if(((*pos_curr[i])-pos_dest[i])<servo_inc_speed)
					(*pos_curr[i])=pos_dest[i];

				else (*pos_curr[i])=(*pos_curr[i])-servo_inc_speed;
			}

		}

		if(check_if_reached())
		{
			int i;
			for(i=0 ; i<NO_OF_SERVOS ; i++) (*pos_curr[i])=pos_dest[i];
			TIM6->DIER&= ~0x00000001U; //position reached - disable tim6 interrupts
			pos_reached_flag = 1;	 // indicate that position has been reached
		}

		TIM6->SR &=~	0x00000001U; //clear interrupt flag
	}
}

/*
void ADC_IRQHandler()
{
	//interrupt flag EOC cleared automatically when reading ADC_DR register
	uint16_t measurement=ADC1->DR;

	//lcd_update(measurement);
	//lcd_serial_command(CLEAR);
	//usart2_write_number(measurement);
	//usart2_send8bit('\n');

	if(measurement<1000) measurement=1000;
	else if(measurement>2000) measurement=2000;

	TIM4->CCR1=measurement;
	TIM4->CCR2=measurement+200;
	TIM4->CCR3=measurement+400;
	TIM4->CCR4=measurement+600;
}
*/

//-----------------------------//
//***** Peripheral initialization *****//

void GPIO_INIT()
{
	RCC->AHB1ENR|=	0x00000001U; //clock enable for port A
	GPIOA->MODER|=	0x00000400U; //pin5 as output (for internal LED)
	GPIOA->MODER|=	0x00003000U; //pin6 as analog input for ADC1
	GPIOA->MODER|=	0x000000A0U; //pin2 and 3 as alternate function
	GPIOA->AFR[0]|=	0x00007700U; //pin2 and 3 as TX/RX
	GPIOA->MODER|=  0x0000000AU; //pin 0 and 1 as alternate function
	GPIOA->AFR[0]|= 0x00000088U; //pin0 and 1 as AF8 -> USART4 TX/RX

	RCC->AHB1ENR|=	0x00000002U; //clock enable for port B
	GPIOB->MODER|=	0x000AA000U; //pin 6-9 as alternate function
	GPIOB->AFR[0]|=	0x22000000U;
	GPIOB->AFR[1]|=	0x00000022U; //pin 6-9 as AF2 - TIM4 PWM output
	GPIOB->MODER|=	0x00000A00U; //pin 4 and 5 as alternate function
	GPIOB->AFR[0]|=	0x00220000U; //pin 4 and 5 as AF2 - TIM3 PWM output CH1 and CH2
}


void TIM7_INIT()
{
	RCC->APB1ENR|=	0x00000020U;	//clock enable for tim7
	TIM7->ARR=		0x03E7;
	TIM7->PSC=		0x3E7F; 		// timer ovf interrupt firing every 1 second (16Mhz/((15999+1)*(999+1))
	TIM7->CR1|= 	0x00000001U;	//enable tim7
	TIM7->DIER|= 	0x00000001U;	//enable interrupts from tim7 OVF
	NVIC_EnableIRQ(TIM7_IRQn);		//enable interrupts in NVIC

}


void TIM4_INIT()
{
	RCC->APB1ENR|=	0x00000004U; 	//clock enable for tim4 - servo PWM control
	TIM4->ARR=		19999;
	TIM4->PSC=		15; 			//PWM frequency (16MHz/((15+1)*(19999+1))=20 ms
	TIM4->CR1|= 	0x00000001U; 	//enable tim4

	TIM4->CCR1=		1500; //default starting position in middle
	TIM4->CCR2=		1500;
	TIM4->CCR3=		2000;
	TIM4->CCR4=		1500;

	TIM4->CCMR1|=	0x00006060U;	//channels 1 and 2 as PWM mode 1 output
	TIM4->CCMR2|=	0x00006060U;	//channels 3 and 4 as PWM mode 1 output

	TIM4->CCMR1|=	0x00000808U;	// Enable preload register on every channel
	TIM4->CCMR2|=	0x00000808U;

	TIM4->CCER|=	0x00001111U;	// output compare enable on every channel

	//TIM4->DIER|= 	0x00000001U; 	//enable interrupts from tim4 OVF
	//NVIC_EnableIRQ(TIM4_IRQn);	//enable interrupts in NVIC
}


void TIM3_INIT()
{
	RCC->APB1ENR|=	0x00000002U; 	// clock enable for tim3 - gripper PWM open/close and rotation control
	TIM3->ARR=		19999;
	TIM3->PSC=		15;				// PWM frequency (16MHz/((15+1)*(19999+1))=20 ms
	TIM3->CR1|=		0x00000001U;
	TIM3->CCMR1|=	0x00006060U;	// channel 1 and 2 as PWM mode 1 output
	TIM3->CCMR1|=	0x00000808U;	// Enable preload register on channel 1 and 2
	TIM3->CCER|=	0x00000011U;	// output compare enable on channel 1 and 2
}

/*void TIM2_INIT()	//used for code benchmarking
{
	RCC->APB1ENR|=	0x00000001U; //clock enable for tim2
	TIM2->ARR=		0xffffffffU; // 32bit timer size
	TIM2->PSC=		1;
	TIM2->CR1|= 	0x00000001U; //enable tim2
}*/

void TIM6_INIT()
{
	RCC->APB1ENR|=	0x00000010U; 	//clock enable for tim6
	TIM6->ARR=		servo_inc_time_ms; // ovf every 60 ms
	TIM6->PSC=		15999;
	TIM6->CR1|= 	0x00000001U; 	//enable tim6
	//TIM6->DIER|= 	0x00000001U; 	//enable interrupts from tim6 OVF
	NVIC_EnableIRQ(TIM6_DAC_IRQn);	//enable interrupts in NVIC
}
/*
void ADC_INIT()
{
	RCC->APB2ENR|= 	0x00000100U; //clock enable for ADC1
	ADC1->CR2|=		0x00000001U; //switch ADC1 on
	//ADC1->CR1|=	0x00000020U; //enable interrupts from 'end of conversion' event
	ADC1->SMPR2|=	0x00080000U; //set sampling time for Channel_6 as 28 clock cycles
	ADC1->SQR3|=	0x00000006U; //set Channel_6 as one and only to convert in sequence
								 //in SQR1 number of conversions in sequence is set as 1 (L:0000b) (just Channel_6)
	//NVIC_EnableIRQ(ADC_IRQn); //enable interrupts in NVIC
}*/

void USART2_INIT()
{
	RCC->APB1ENR|=	0x00020000U; //clock enable for usart2
	USART2->CR1|=	0x00002000U; //enable usart, data length 8 bit, oversampling at 16
	USART2->BRR|=	0x00000683U; //set baud rate at 9600 (mantissa=0x68=104 , fractional=3/16=0.1875 thus 104.1875)
	USART2->CR1|=	0x0000000CU; //enable transmitter and receiver
	USART2->CR1|=	0x00000020U; //enable interrupt from USART byte receive
	NVIC_EnableIRQ(USART2_IRQn); //enable interrupts in NVIC
}


void UART4_INIT()
{
	RCC->APB1ENR|=	0x00080000U; //clock enable for uart4

	UART4->CR1|=	0x00002000U; //enable usart, data length 8 bit,
	                             //oversampling at 16

	UART4->BRR|=	0x00000683U; //set baud rate at 9600 (mantissa=0x68=104 ,
	                             //fractional=3/16=0.1875 thus 104.1875)

	UART4->CR1|=	0x0000000CU; //enable transmitter and receiver

	UART4->CR1|=	0x00000020U; //enable interrupt from USART byte receive
	NVIC_EnableIRQ(UART4_IRQn);
}


void SERVO_POS_INIT()
{
	pos_curr[0]=&(TIM4->CCR1);
	pos_curr[1]=&(TIM4->CCR2);
	pos_curr[2]=&(TIM4->CCR3);
	pos_curr[3]=&(TIM4->CCR4);
	pos_curr[4]=&(TIM3->CCR1);
}


int main(void)
{

	GPIO_INIT();

	//HC595_init();
	//lcd_hd44870_init();

	TIM7_INIT();
	//TIM2_INIT();
	TIM3_INIT();
	TIM4_INIT();
	//ADC_INIT();
	TIM6_INIT();
	USART2_INIT();
	UART4_INIT();
	SERVO_POS_INIT();

	myQueueInit(&queue);

	__enable_irq();

	gripper_close();

	//char tmp[]="mov p ";

	char buffer[ELEMENT_SIZE];
	while(1)
	{
		if(queue.semaphore !=0 && queue.trylock(&queue))
		{
			queue.get(&queue, buffer);
			queue.release(&queue);
			recog_input(buffer);
		}

		/*
		while(exec_count != 0)
		{
			int i=0;
			while(cmd_list[i] != '\0')
			{
				if(is_digit(cmd_list[i]))
				{
					tmp[5] = cmd_list[i];
					recog_input(tmp);
				}
				else if(cmd_list[i] == 'o') recog_input("o");
				else if(cmd_list[i] == 'c') recog_input("c");
				else if(cmd_list[i] == 'd') delay_ms(INTERVAL);

				i++;

				while(!pos_reached_flag) asm volatile ("nop");

				//delay_ms(INTERVAL);
			}


			exec_count--;
		}*/
	}

	return 0;
}


