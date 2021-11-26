
#include <main.h>
#include "StepperClass.h"			// ステッピングモータを使用するためのライブラリ
#include "stm32f1xx_hal_uart.h"		// SBDBTとUART通信をするためのライブラリ

// UART用構造体変数
extern UART_HandleTypeDef huart2;
extern uint8_t RxBuffer[8];

extern uint8_t B;
extern uint8_t A;
extern uint8_t X;
extern uint8_t Y;
extern uint8_t RIGHT;
extern uint8_t DOWN;
extern uint8_t LEFT;
extern uint8_t UP;
extern uint8_t R1;
extern uint8_t R2;
extern uint8_t L1;
extern uint8_t L2;
extern uint8_t START;
extern uint8_t BACK;
extern uint8_t RightAxisX;
extern uint8_t RightAxisY;
extern uint8_t LeftAxisX;
extern uint8_t LeftAxisY;

// タイマ用構造体変数
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/*******************************************************************/

// 実体化、使用するステッピングモータの数だけ（最大4つ）
StepperClass stepper1;
StepperClass stepper2;
StepperClass stepper3;
StepperClass stepper4;

// ロボットの半径[m]
double R = 0.1;
// 車輪の半径[m]
double r = 0.025;

// ステッピングモータの一回転に要するパルス数
double PPR = 6400;
// モータの軸と車輪の軸の減速比
double d = 6;

// ロボットの速度[m/s]
double Vx = 0;
double Vy = 0;
// ロボットの角速度[rad/s]
double Omega = 0;

// 各タイヤの速度[m/s]
double V1 = 0;
double V2 = 0;
double V3 = 0;

// 各ステッピングモータの速度[pulse/s]
double v1 = 0;
double v2 = 0;
double v3 = 0;

/*******************************************************************/

// 三輪オムニ用速度制御関数
void update_3wheel_velocity()
{
	// LeftAxisX、LeftAxisY、R1、L1からVx、Vy、Omegaを計算
	// Vx：0.315 ～ -0.320 [m/s]
	// Vy：0.320 ～ -0.315 [m/s]
	// Omega：1.0 ～ -1.0 [rad/s]
	Vx = (LeftAxisX - 64) * 0.005;
	Vy = (64 - LeftAxisY) * 0.005;
	Omega = (L1 - R1);

	// Vx、Vy、OmegaからV1、V2、V3を計算
	V1 = -1*Vx + R*Omega;
	V2 = (0.5)*Vx + (0.866)*Vy + R*Omega;
	V3 = (0.5)*Vx - (0.866)*Vy + R*Omega;

	// V1[m/s] → v1[pulse/s]
	// V1、V2、V3からv1、v2、v3を計算
	v1 = (V1/r)*d*PPR/6.283;
	v2 = (V2/r)*d*PPR/6.283;
	v3 = (V3/r)*d*PPR/6.283;

	// v1、v2、v3を整数型(int32_t)にキャストして目標値に代入
	stepper1.target_vel = (int32_t)v1;
	stepper2.target_vel = (int32_t)v2;
	stepper3.target_vel = (int32_t)v3;
}

// メイン関数
void main_cpp(void)
{
	// UART開始
	HAL_UART_Receive_IT(&huart2, RxBuffer, 8);

	// LED緑を点灯
	HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

	// 各ステッピングモータへのタイマの割り当て(timer1～timer4)
	stepper1.initialize(&htim1);
	stepper2.initialize(&htim2);
	stepper3.initialize(&htim3);
	stepper4.initialize(&htim4);

	// ステッピングモータの最大加速度を設定
	// 大きすぎると脱調、小さすぎると応答が遅くなる
	// ここでは、PPRを20で割った値を設定している。
	stepper1.max_acceleration = PPR/20;
	stepper2.max_acceleration = PPR/20;
	stepper3.max_acceleration = PPR/20;
	stepper4.max_acceleration = PPR/20;

	// ステッピングモータの有効化
	stepper1.enable();
	stepper2.enable();
	stepper3.enable();
	stepper4.enable();
	// 有効化後、黄色LEDを点灯、赤色LEDを消灯
	HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);

	// メインループ
	while(1)
	{
		// ステッピングモータの状態更新
		stepper1.update();
		stepper2.update();
		stepper3.update();
		stepper4.update();
		// 足回り制御関数(3輪オムニ用)
		update_3wheel_velocity();
		// 制御周期(1ms)
		HAL_Delay(1);

		// もし、Aボタンが押されている(A == 1)なら、
		if(A == 1)
		{
			// ステッピングモータの無効化
			stepper1.disable();
			stepper2.disable();
			stepper3.disable();
			stepper4.disable();
			// 無効化後、黄色LEDを消灯、赤色LEDを点灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);
		}

		// もし、Bボタンが押されている(B == 1)なら、
		if(B == 1)
		{
			// ステッピングモータの有効化
			stepper1.enable();
			stepper2.enable();
			stepper3.enable();
			stepper4.enable();
			// 有効化後、黄色LEDを点灯、赤色LEDを消灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);
		}

	}
}
