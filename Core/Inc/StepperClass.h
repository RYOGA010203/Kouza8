
#include <main.h>

class StepperClass
{
public:
	// コンストラクタ
	StepperClass();
	// ユーザー関数、主にメイン関数で呼び出す
	void initialize(TIM_HandleTypeDef *htim);
	void enable();
	void disable();
	void set_PPR(uint32_t PPR);
	// 以下の変数の単位は[pulse/s]
	int32_t target_vel;
	int32_t current_vel;
	// 加速度(updateごとのvel変動値の最大)
	uint32_t max_acceleration = 10;
	// 回転方向　0:順　1:逆
	bool rotation_direction;
	// 周期関数
	void update();
private:
	// 以下の変数の単位はpulse(整数)
	uint32_t target_speed;
	uint32_t current_speed;
	// その他
	bool is_enabled;
	uint16_t tim_prescaler_value;
	// 	タイマの種類と方向ピン、有効化ピン
	TIM_HandleTypeDef *HTIM;
	GPIO_TypeDef* direction_GPIOx;
	uint16_t 	  direction_GPIO_Pin;
	GPIO_TypeDef* enable_GPIOx;
	uint16_t 	  enable_GPIO_Pin;
	// 定数
	const uint32_t min_prescaler_value = 3;
	const uint32_t max_prescaler_value = 65535;
	const uint32_t tim_frequency = 1000000;
	// 周期処理関数
	void update_PWM_period();
	void update_tim_prescaler_value();
	void update_current_speed();
	void update_current_vel();
};
