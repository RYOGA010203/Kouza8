
// SBDBTからUART通信で受け取ったコントローラのデータを格納

#include "main.h"
#include "stm32f1xx_hal_uart.h"

UART_HandleTypeDef huart2;

// UART受信時のデータの保存先
uint8_t RxBuffer[8];

// ボタン等
uint8_t B     = 0;
uint8_t A     = 0;
uint8_t X     = 0;
uint8_t Y     = 0;

uint8_t RIGHT = 0;
uint8_t DOWN  = 0;
uint8_t LEFT  = 0;
uint8_t UP    = 0;

uint8_t R1    = 0;
uint8_t R2    = 0;
uint8_t L1    = 0;
uint8_t L2    = 0;

uint8_t START = 0;
uint8_t BACK  = 0;

uint8_t RightAxisX = 64;
uint8_t RightAxisY = 64;
uint8_t LeftAxisX  = 64;
uint8_t LeftAxisY  = 64;

// UART通信の受信完了時に、ボタンのON/OFFやアナログスティックの値を代入
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	B     = (RxBuffer[2] & 0b01000000) >> 6;
	A     = (RxBuffer[2] & 0b00100000) >> 5;
	X     = (RxBuffer[1] & 0b00000001);
	Y     = (RxBuffer[2] & 0b00010000) >> 4;

	RIGHT = (RxBuffer[2] & 0b00000100) >> 2;
	DOWN  = (RxBuffer[2] & 0b00000010) >> 1;
	LEFT  = (RxBuffer[2] & 0b00001000) >> 3;
	UP    = (RxBuffer[2] & 0b00000001);

	R1    = (RxBuffer[1] & 0b00001000) >> 3;
	R2    = (RxBuffer[1] & 0b00010000) >> 4;
	L1    = (RxBuffer[1] & 0b00000010) >> 1;
	L2    = (RxBuffer[1] & 0b00000100) >> 2;

	START = (RxBuffer[1] & 0b00100000) >> 5;
	BACK  = (RxBuffer[1] & 0b01000000) >> 6;

	RightAxisX = RxBuffer[5];
	RightAxisY = RxBuffer[6];
	LeftAxisX  = RxBuffer[3];
	LeftAxisY  = RxBuffer[4];

	HAL_UART_Receive_IT(&huart2, RxBuffer, 8);
}
