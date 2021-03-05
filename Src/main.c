/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// variables and structs used in program

uint8_t start_pm[] = {0x7E, 0x00, 0x00, 0x02, 0x01, 0x03, 0xF9, 0x7E};
uint8_t stop_pm[] = {0x7E, 0x00, 0x01, 0x00, 0xFE, 0x7E};
uint8_t receive_pm[] = {0x7E, 0x00, 0x03, 0x00, 0xFC, 0x7E};

uint8_t rec1_pm[100];
uint8_t rec2_pm[100];
uint8_t received_data_pm[100];

char so2_command[]={'c'};
uint8_t so2_received[60];
uint8_t so2_rest[5];

int so2_flag=0;
int to_process_so2=0;
int rec_so2=0;
char so2_toppb[10];
int is_minus=0;
int so2_ppb=0;
int tocalc_so2ppb=0;
int so2_results[40];
int counter_results=0;
int flag_measurement_so2=1;
int counter_measurement_so2=0;
int table_measurement_so2[10];
int counter_ready_measurement=0;
float ready_measurement_so2[180];

int flag_pm=0;
int flag2_pm=0;
int receive_pm=0;
uint8_t to_pm_1[4];
float pm_1=0;
uint8_t to_pm_25[4];
float pm_25=0;
uint8_t to_pm_4[4];
float pm_4=0;
uint8_t to_pm_10[4];
float pm_10=0;

float tab_test_pm1[180];
float tab_test_pm25[180];
float tab_test_pm4[180];
float tab_test_pm10[180];
float tab_test_hcho[180];
int counter_test_pm=0;
int counter_hcho=0;
int aaa=0;
int clock_flag=1;

int size_pm1=0;
int size_pm25=0;
int size_pm4=0;
int size_pm10=0;
char pm1_tosend[15];
char pm25_tosend[15];
char pm4_tosend[15];
char pm10_tosend[15];
int size_so2=0;
char so2_tosend[15];
int size_hcho=0;
char hcho_tosend[15];
char packet_tosend[250];
int cplt_size=0;
int to_prep=0;
float tmp=0;
uint16_t ADC_measurement;
float R0=11.98;
float Rs;
float hcho;

int offset_pm1=0;
int offset_pm25=0;
int offset_pm4=0;
int offset_pm10=0;

int alternative_send=0;

char rest_pm[5];
int to_rest_pm=0;
int to_proc_pm=0;
int stop1=0;
int if_received=0;
int if_send_pm=0;
int start1=0;
int stop2=0;
int flag_help_pm=0;
bool GPS_data_Rx_flag = false;



#define message_length 420
struct AtComScript{
	char message[message_length];
	uint8_t WaitTime;
};

struct AtComScript GetConnection[12] = {
		{"AT+CGREG=1\r\n",1},
		{"AT+CGREG\?\r\n",1},
		{"AT+QIFGCNT=0\r\n",1},
		{"AT+QICSGP=1,\"plus\"\r\n",1},
		{"AT+QIMUX=0\r\n",1},
		{"AT+QIMODE=0\r\n",1},
		{"AT+QIDNSIP=1\r\n",1},
		{"AT+QIREGAPP\r\n",1},
		{"AT+QIACT\r\n",5},
		{"AT+qishowra=1\r\n",5},
		{"AT+QILOCIP\r\n",5},
		{"AT+QIOPEN=\"TCP\",\"www.atthost24.pl\",80\r\n",5}
};

struct AtComScript SendPost[2] = {
		{"AT+QISEND\r\n\r\n",10},
		{"post request content\r\n",10}
};

char test_connection[100];
char size_msg[3];
char GPS_post_ready[82];
#define GPS_data_size 1000
char GPS_data[GPS_data_size];
char GPS_Rx[GPS_data_size];
char GPS_prefix[6];//={'$','G','N','M','R','C'};
#define GPS_post_ready_size 82
#define Post_Length 333
char Post_Request[Post_Length];
int which_line=0;
int flag_gps=0;
int to_receive_gps=0;

int counter_answers_so2=0;
int is_receiving_so2=0;

//extern uint32_t rtc_time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// trigger flags to receive and process data
	if(clock_flag==1)
	{
		clock_flag=0;
		to_process_so2=1;
		flag_pm=1;
		if(counter_answers_so2>=1 && is_receiving_so2==0)
		{
			HAL_UART_Transmit(&huart4, so2_command, 1, 1000);
		}
		counter_measurement_so2++;
	}
	else if(flag2_pm==1)
	{
		flag_pm=0;
		receive_pm=1;
		clock_flag=1;
		if(counter_answers_so2>=1 && is_receiving_so2==0)
		{
			HAL_UART_Transmit(&huart4, so2_command, 1, 1000);
		}
		counter_measurement_so2++;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// receiving data from pm sensor
	if(huart->Instance==USART1)
	{
		if(stop1==1)
		{
			HAL_UART_Receive_IT(&huart1, rec2_pm, 7);
		}
		if(stop2==1)
		{
			HAL_UART_Receive_IT(&huart1, rec2_pm, 7);
		}
		if(start1==1)
		{
			HAL_UART_Receive_IT(&huart1, rec1_pm, 7);
		}
		if(flag_help_pm==1)
		{
			flag_help_pm=0;
			HAL_UART_Receive_IT(&huart1, received_data_pm, 47);
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// handling incoming data and setting flags for pm sensor
	if(huart->Instance==USART1)
	{
		if(stop1==1)
		{
			stop1=0;
			if_send_pm=1;

		}
		else if(stop2==1)
		{
			stop2=0;
		}
		else if(start1==1)
		{
			start1=0;
		}
		else if(to_rest_pm==0 && received_data_pm[46]!=0x7E)
		{
			to_rest_pm=1;
			HAL_UART_Receive_IT(&huart1, rest_pm, 1);
		}
		else if(to_rest_pm==1 && rest_pm[0]!=0x7E)
		{
			HAL_UART_Receive_IT(&huart1, rest_pm, 1);
		}
		else if(to_rest_pm==1 && rest_pm[0]==0x7E)
		{
			to_rest_pm=0;
			stop2=1;
			HAL_UART_Transmit_IT(&huart1, stop_pm, 6);
			to_proc_pm=1;
		}
		else
		{
			to_rest_pm=0;
			stop2=1;
			HAL_UART_Transmit_IT(&huart1, stop_pm, 6);
			to_proc_pm=1;
		}
	}

	// handling incoming data and setting flags for sulfur dioxide sensor
	if(huart->Instance==UART4)
	{
		so2_flag=1;
		is_receiving_so2=1;
		if(rec_so2==1)
		{
			if(so2_rest[0]=='\n')
			{
				rec_so2=0;
				so2_rest[0]='\0';
				tocalc_so2ppb=1;
				HAL_UART_Receive_IT(&huart4, so2_received, 60);
			}
			else
			{
				HAL_UART_Receive_IT(&huart4, so2_rest, 1);
			}
		}
		else if(so2_received[59]!='\n' && rec_so2==0)
		{
			rec_so2=1;
			HAL_UART_Receive_IT(&huart4, so2_rest, 1);
		}
	}
	// handling data from gps
  if(huart == &huart6)
  {
	  GPS_data_Rx_flag = true;
	  to_receive_gps=1;
	  strcpy(GPS_data, &GPS_Rx);
  }

}

// if internet connection isn't available
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin);
    alternative_send=1;
}

// checking if pm sensor sent one of special signs
char check_special_sign(char sign)
{
	if(sign==0x5E)
	{
		return 0x7E;
	}
	else if(sign==0x5D)
	{
		return 0x7D;
	}
	else if(sign==0x31)
	{
		return 0x11;
	}
	else
	{
		return 0x13;
	}
}

// calculate ppb for so2
int get_ppb(char tab[])
{
	so2_ppb=0;
	if(tab[14]=='-')
	{
		is_minus=1;
	}
	int cnt=0;
	for(int i=14+is_minus;i<20;i++)
	{
		if(tab[i]!=',')
		{
			so2_toppb[cnt]=tab[i];
			cnt++;
		}
		else
		{
			break;
		}
	}
	for(int i=0;i<cnt;i++)
	{
		int a = so2_toppb[i]-48;
		for(int j=cnt-i-1;j>0;j--)
		{
			a*=10;
		}
		so2_ppb+=a;
	}
	if(is_minus==1)
	{
		is_minus=0;
		so2_ppb*=-1;
	}
	if(flag_measurement_so2==1 && counter_measurement_so2<10)
	{
		table_measurement_so2[counter_measurement_so2]+=so2_ppb;
		counter_measurement_so2++;
	}
	else if(flag_measurement_so2==1 && counter_measurement_so2==10)
	{
		counter_measurement_so2=0;
		for(int i=0;i<10;i++)
		{
			ready_measurement_so2[counter_ready_measurement]+=table_measurement_so2[i];
			table_measurement_so2[i]=0;
		}
		ready_measurement_so2[counter_ready_measurement]/=1000.0;
		counter_ready_measurement++;
		to_process_so2=0;
		if(counter_ready_measurement==180)
		{
			counter_ready_measurement=0;
		}
	}
	counter_results++;
	return so2_ppb;
}

// convert IEE754 to floats
void get_pm_1(uint8_t tab[])
{
	pm_1=0;
	int32_t aaa= (tab[0]<<24 | tab[1]<<16 | tab[2]<<8 | tab[3]);
	int sign = aaa >> 31;
	int mantissa = (aaa & 0x7FFFFF) | 0x800000;
	int exp = ((aaa >> 23) & 0xFF) - 127 - 23;
	pm_1 = mantissa * pow(2.0, exp);
	tab_test_pm1[counter_test_pm]=pm_1;
}
void get_pm_25(uint8_t tab[])
{
	pm_25=0;
	float tmp=0.0;
	int32_t aaa= (tab[0]<<24 | tab[1]<<16 | tab[2]<<8 | tab[3]);
	int sign = aaa >> 31;
	int mantissa = (aaa & 0x7FFFFF) | 0x800000;
	int exp = ((aaa >> 23) & 0xFF) - 127 - 23;
	tmp = mantissa * pow(2.0, exp);
	pm_25=tmp-pm_1;
	tab_test_pm25[counter_test_pm]=pm_25;
}
void get_pm_4(uint8_t tab[])
{
	pm_4=0;
	float tmp=0.0;
	int32_t aaa= (tab[0]<<24 | tab[1]<<16 | tab[2]<<8 | tab[3]);
	int sign = aaa >> 31;
	int mantissa = (aaa & 0x7FFFFF) | 0x800000;
	int exp = ((aaa >> 23) & 0xFF) - 127 - 23;
	tmp = mantissa * pow(2.0, exp);
	pm_4=tmp-pm_25-pm_1;
	tab_test_pm4[counter_test_pm]=pm_4;
}
void get_pm_10(uint8_t tab[])
{
	pm_10=0;
	float tmp=0.0;
	int32_t aaa= (tab[0]<<24 | tab[1]<<16 | tab[2]<<8 | tab[3]);
	int sign = aaa >> 31;
	int mantissa = (aaa & 0x7FFFFF) | 0x800000;
	int exp = ((aaa >> 23) & 0xFF) - 127 - 23;
	tmp = mantissa * pow(2.0, exp);
	pm_10=tmp-pm_4-pm_25-pm_1;
	tab_test_pm10[counter_test_pm]=pm_10;
	counter_hcho=counter_test_pm;
	counter_test_pm++;
	if(counter_test_pm==180)
	{
		counter_test_pm=0;
	}
}

// functions for sending commands
void send_stop_pm1()
{
	flag2_pm=1;
	stop1=1;
	HAL_UART_Transmit_IT(&huart1, stop_pm, 6);
}

void send_stop_pm2()
{
	stop2=1;
	HAL_UART_Transmit_IT(&huart1, stop_pm, 6);
}

void send_pm()
{
	  flag2_pm=1;
	  start1=1;
	  HAL_UART_Transmit_IT(&huart1, start_pm, 8);

}

void get_pm()
{
	  receive_pm=0;
	  flag_help_pm=1;
	  HAL_UART_Transmit_IT(&huart1, receive_pm, 6);

}

// processing data received from pm and hcho sensors
void process_pm()
{

	  offset_pm1=0;
	  offset_pm25=0;
	  offset_pm4=0;
	  offset_pm10=0;

	  for(int i=0;i<4;i++)
	  {
		  if(received_data_pm[5+i+offset_pm1]!=0x7D)
		  {
			  to_pm_1[i]=received_data_pm[5+i+offset_pm1];
		  }
		  else if(received_data_pm[5+i+offset_pm1]==0x7D)
		  {
			  offset_pm1++;
			  to_pm_1[i]=check_special_sign(received_data_pm[5+i+offset_pm1]);
		  }
	  }

	  for(int i=0;i<4;i++)
		  {
			  if(received_data_pm[9+i+offset_pm1+offset_pm25]!=0x7D)
			  {
				  to_pm_25[i]=received_data_pm[9+i+offset_pm1+offset_pm25];
			  }
			  else if(received_data_pm[9+i+offset_pm1+offset_pm25]==0x7D)
			  {
				  offset_pm25++;
				  to_pm_25[i]=check_special_sign(received_data_pm[9+i+offset_pm1+offset_pm25]);
			  }
		  }

	  for(int i=0;i<4;i++)
		  {
			  if(received_data_pm[13+i+offset_pm1+offset_pm25+offset_pm4]!=0x7D)
			  {
				  to_pm_4[i]=received_data_pm[13+i+offset_pm1+offset_pm25+offset_pm4];
			  }
			  else if(received_data_pm[13+i+offset_pm1+offset_pm25+offset_pm4]==0x7D)
			  {
				  offset_pm4++;
				  to_pm_4[i]=check_special_sign(received_data_pm[13+i+offset_pm1+offset_pm25+offset_pm4]);
			  }
		  }

	  for(int i=0;i<4;i++)
		  {
			  if(received_data_pm[17+i+offset_pm1+offset_pm25+offset_pm4+offset_pm10]!=0x7D)
			  {
				  to_pm_10[i]=received_data_pm[17+i+offset_pm1+offset_pm25+offset_pm4+offset_pm10];
			  }
			  else if(received_data_pm[17+i+offset_pm1+offset_pm25+offset_pm4+offset_pm10]==0x7D)
			  {
				  offset_pm10++;
				  to_pm_10[i]=check_special_sign(received_data_pm[17+i+offset_pm1+offset_pm25+offset_pm4+offset_pm10]);
			  }
		  }

	  get_pm_1(to_pm_1);
	  get_pm_25(to_pm_25);
	  get_pm_4(to_pm_4);
	  get_pm_10(to_pm_10);
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	   ADC_measurement = HAL_ADC_GetValue(&hadc1);
	   Rs=(3722.727/ADC_measurement)-1;
	   hcho=pow(10.0,((log10(Rs/R0)-0.0827)/(-0.4807)));
	   tab_test_hcho[counter_hcho]=hcho;
	   }
	  to_prep=1;

}

// prepare packet that consists of data required by server which is ready to be sent
void prepare_packet()
{
	tmp=0;
	for(int i=0;i<250;i++)
	{
		packet_tosend[i]='\0';
		if(i<12)
		{
			pm1_tosend[i]=0;
			pm25_tosend[i]=0;
			pm4_tosend[i]=0;
			pm10_tosend[i]=0;
			so2_tosend[i]=0;
			hcho_tosend[i]=0;
		}
	}
	size_pm1= sprintf(pm1_tosend,"%.2f", pm_1);
	size_pm25= sprintf(pm25_tosend,"%.2f", pm_25);
	size_pm4= sprintf(pm4_tosend,"%.2f", pm_4);
	size_pm10= sprintf(pm10_tosend,"%.2f", pm_10);
	tmp=ready_measurement_so2[counter_ready_measurement-1];
	size_so2= sprintf(so2_tosend,"%.2f", tmp);
	size_hcho= sprintf(hcho_tosend,"%.2f", hcho);

	cplt_size=149+size_pm1+size_pm25+size_pm4+size_pm10+size_so2+size_hcho;
	sprintf(size_msg, "%d",cplt_size);

	strcat(packet_tosend, "\r\n\r\n{\"gps\":\"");
	//strcat(packet_tosend, "$GNRMC,030232.000,A,5223.6457,N,1656.9179,E,2.895,96.40,060120,,,A*7A,aaaaa");
	flag_gps=1;
	if(flag_gps==0)
	{
		strcat(packet_tosend, GPS_post_ready);
	}
	else if(flag_gps==1)
	{
		flag_gps=0;
		strcat(packet_tosend, "$GNRMC,030232.000,A,5223.6457,N,1656.9179,E,2.895,96.40,060120,,,A*7A,aaaaaaaaaaaa");
	}
	strcat(packet_tosend, "\",\"pm1\":\"");
	strcat(packet_tosend, pm1_tosend);
	strcat(packet_tosend,"\",\"pm25\":\"");
	strcat(packet_tosend, pm25_tosend);
	strcat(packet_tosend,"\",\"pm4\":\"");
	strcat(packet_tosend, pm4_tosend);
	strcat(packet_tosend,"\",\"pm10\":\"");
	strcat(packet_tosend, pm10_tosend);
	strcat(packet_tosend,"\",\"so2\":\"");
	strcat(packet_tosend, so2_tosend);
	strcat(packet_tosend,"\",\"hcho\":\"");
	strcat(packet_tosend, hcho_tosend);
	strcat(packet_tosend,"\"}");
}

void send_alternative_packet()
{
	tmp=0;

	prepare_packet();

}

// immitating ,,soft reset" of A9G
void Rst_button_press (){
	  HAL_GPIO_WritePin(RST_GPIO_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_RESET);
	  HAL_Delay(3000);
	  HAL_GPIO_WritePin(RST_GPIO_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_SET);
	  HAL_Delay(30000);
}

// providing connection to internet
void get_connection()
{
	int cnt_gc=0;
	while(cnt_gc<=10){
		for(int i=0;i<100;i++)
		{
			test_connection[i]=0;
		}
		HAL_UART_Transmit(&huart3, GetConnection[cnt_gc].message,  strlen(GetConnection[cnt_gc].message), GetConnection[cnt_gc].WaitTime*1000);
		HAL_UART_Receive(&huart3, test_connection, 100, GetConnection[cnt_gc].WaitTime*1000);
		cnt_gc++;
	}

	HAL_UART_Transmit(&huart3, GetConnection[11].message,  strlen(GetConnection[11].message), GetConnection[11].WaitTime*1000);
	HAL_UART_Receive(&huart3, test_connection, 100, 5000);
}

// function that finds proper strings indicating NMEA format in GPS data
void Find_nmea (){
	  char INPUT_prefix[6];
	  strcpy(INPUT_prefix, "BEDOES");

	  uint16_t cnt = 0;
	  bool GPS_post_end_flag = true;

	  while(strcmp(GPS_prefix,INPUT_prefix) != 0){
		  for(uint8_t a=0; a<6; a++)
		  {
			  INPUT_prefix[a]=GPS_data[cnt+a];
		  }
		  cnt++;
	  }

	  for(uint8_t a=0;a< GPS_post_ready_size;a++){
		  if(GPS_post_end_flag == true)
		  {
		  	  GPS_post_ready[a]=GPS_data[cnt + a - 1];
		  }
		  else if(GPS_post_end_flag == false)
		  {
			  GPS_post_ready[a]= 97;
		  }
		  if(GPS_post_ready[a] == '\r')
		  {
			  GPS_post_ready[a] = 44;
			  GPS_post_end_flag = false;
		  }
	  };
	  GPS_post_end_flag = true;
}

// merge data with server info to prepare whole post request
void Post_Rq_Merge(){


	memset(Post_Request, 0, Post_Length);
	strcat(Post_Request, "POST / HTTP/1.1\r\nHost:www.tomaszjankowski.atthost24.pl\r\nContent-Type:application/json\r\nContent-Length:");
	strcat(Post_Request, size_msg);
	strcat(Post_Request, "\r\nAuthorization: Basic ZHJvbmUwMTo4Yjc4MWE1ZA==");
	//strcat(Post_Request, "$GNRMC,030232.000,A,5223.6457,N,1656.9179,E,2.895,96.40,060120,,,A*7A,aaaaa");
	strcat(Post_Request, packet_tosend);
	strcat(Post_Request,"\r\n\r\n\r\n\032\r\n");
	//strcat(Post_Request, "\",\"pm1\":\"1\",\"pm25\":\"2\",\"pm4\":\"3\",\"pm10\":\"4\",\"so2\":\"5\",\"hcho\":\"6\"}\r\n\r\n\r\n\032\r\n");
}

// 
void Transmit_AtComScript (struct AtComScript Script[], uint8_t Script_Length)
{
	if(which_line==1)
	{
		if(GPS_data_Rx_flag == true)
		{
			GPS_data_Rx_flag=false;
			flag_gps=0;
			Find_nmea();
		}
		else
		{
			//strcat(GPS_post_ready, "$GNRMC,030232.000,A,5223.6457,N,1656.9179,E,2.895,96.40,060120,,,A*7A,aaaaa");
			flag_gps=1;
		}
		if(to_receive_gps==1)
		{
			HAL_UART_Receive_IT(&huart6, GPS_Rx, GPS_data_size);
			to_receive_gps=0;
		}

		  prepare_packet();
		  Post_Rq_Merge();
		  memset(SendPost[1].message, 0, message_length);
		  strcpy(SendPost[1].message, Post_Request); // putting new post request in proper location of AT SendPost
		  //HAL_UART_Receive(&huart3, test_connection, 20, 100);

	}

	int Msg_Length = strlen(Script[which_line].message);
	char Msg[Msg_Length];

	strcpy(Msg, Script[which_line].message);
	which_line++;

	if(which_line >= Script_Length){ // checking which line is being processed, returns flag if finished
		which_line=0;
		memset(test_connection,0,100);
	}

	HAL_UART_Transmit(&huart3, Msg, Msg_Length,600);
	if(which_line==0)
	{
		HAL_UART_Receive(&huart3, test_connection, 100, 5000);
		which_line=0;
	}



}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  strcpy(GPS_prefix, "$GNRMC");
  
  HAL_TIM_Base_Start_IT(&htim10);

  //rtc_time = RTC_GetCounter();
  HAL_UART_Receive_IT(&huart4, so2_received, 60);

  HAL_UART_Transmit(&huart4, so2_command, 1, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(tocalc_so2ppb==1 && to_process_so2==1)
	  {
		  tocalc_so2ppb=0;
		  get_ppb(so2_received);
		  for(int i=0;i<10;i++)
		  {
			  so2_toppb[i]='\0';
		  }
	  }
	  if(flag_pm==1)
	  {
		  //tocalc_so2ppb=1;
		  flag_pm=0;
		  send_stop_pm1();
		  //send_pm();
	  }
	  if(if_send_pm==1)
	  {
		  if_send_pm=0;
		  send_pm();
	  }
	  if(receive_pm==1)
	  {
		  get_pm();
	  }
	  else if(to_proc_pm==1)
	  {
		  to_proc_pm=0;
		  process_pm();
	  }
	  if(to_prep==1/* && to_process_so2==0*/)
	  {
		  to_prep=0;
		  //Transmit_AtComScript(SendPost, 2);
		  //Transmit_AtComScript(SendPost, 2);
		  send_alternative_packet();
		  HAL_UART_Transmit(&huart2, packet_tosend, 200, 100);
	  }
	  if(alternative_send==1)
	  {
		  alternative_send=0;
		  send_alternative_packet();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the reference Clock input 
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 20999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 49999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 19999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_GPIO_Pin */
  GPIO_InitStruct.Pin = RST_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
