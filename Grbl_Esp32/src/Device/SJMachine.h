#pragma once
// clang-format off

/*
戎图测试机
*/

#define FIRMWARE_VERSION        "09"           //固件版本
#define MACHINE_NAME            "LOKLiK_iCraft"

//#define PIN_LED				GPIO_NUM_2
#define PIN_POWER			GPIO_NUM_15
#define PEN1_END_STOP 		GPIO_NUM_39
#define PEN2_END_STOP 		GPIO_NUM_34

#define PIN_MENU_KEY		GPIO_NUM_36
#define PIN_PAPER			GPIO_NUM_26
#define PIN_ONE_KEY			GPIO_NUM_13
#define PIN_SEEK_BOX		GPIO_NUM_35
#define PAPER_TOUCH			0
#define LIMIT_TOUCH			0
#define PEN_TOUCH			1

#define PIN_LED_PAPER		GPIO_NUM_12
#define PIN_LED_START		GPIO_NUM_14
#define PIN_LED_PAUSE		GPIO_NUM_27
#define PIN_LED_POWER		GPIO_NUM_5

#define PIN_LED_BLE		    GPIO_NUM_21
#define PIN_LED_USB		    GPIO_NUM_5


#define X_STEP_PIN              GPIO_NUM_19
#define X_DIRECTION_PIN         GPIO_NUM_18
#define Y_STEP_PIN              GPIO_NUM_2
#define Y_DIRECTION_PIN         GPIO_NUM_4

#define Z_STEP_PIN              GPIO_NUM_33
#define Z_DIRECTION_PIN         GPIO_NUM_32

#define X_LIMIT_PIN             GPIO_NUM_25
#define Z_LIMIT_PIN             GPIO_NUM_39

#define TMC_UART                UART_NUM_1
#define TMC_UART_RX             GPIO_NUM_16
#define TMC_UART_TX             GPIO_NUM_17


#define X_TRINAMIC_DRIVER       2209
#define X_RSENSE                0.1f
#define X_DRIVER_ADDRESS        0
#define DEFAULT_X_MICROSTEPS    32//
#define DEFAULT_X_CURRENT 0.8

#define Y_TRINAMIC_DRIVER       2209
#define Y_RSENSE                0.1f
#define Y_DRIVER_ADDRESS        1
#define DEFAULT_Y_MICROSTEPS    32
#define DEFAULT_Y_CURRENT 0.9

#define Z_TRINAMIC_DRIVER       2209
#define Z_RSENSE                0.1f
#define Z_DRIVER_ADDRESS        2
#define DEFAULT_Z_MICROSTEPS    32
#define DEFAULT_Z_CURRENT 0.9

// OK to comment out to use pin for other features
#define STEPPERS_DISABLE_PIN    GPIO_NUM_23
#define SEEK_BOX_DISABLE_PIN    GPIO_NUM_22

//#define SPINDLE_TYPE    SpindleType::NONE

#define SPINDLE_TYPE            SpindleType::LASER
//#define SPINDLE_OUTPUT_PIN      GPIO_NUM_19   // labeled SpinPWM
//#define SPINDLE_ENABLE_PIN      GPIO_NUM_18  // labeled SpinEnbl


// #define LASER_OUTPUT_PIN		GPIO_NUM_19
#define DEFAULT_SPINDLE_RPM_MAX 1000

//#define COOLANT_MIST_PIN        GPIO_NUM_21  // labeled Mist
//#define COOLANT_FLOOD_PIN       GPIO_NUM_25  // labeled Flood
//#define PROBE_PIN               GPIO_NUM_32  // labeled Probe   -------- right pen


/*
#define CONTROL_SAFETY_DOOR_PIN GPIO_NUM_35  // labeled Door,  needs external pullup
#define CONTROL_RESET_PIN       GPIO_NUM_34  // labeled Reset, needs external pullup
#define CONTROL_FEED_HOLD_PIN   GPIO_NUM_36  // labeled Hold,  needs external pullup
#define CONTROL_CYCLE_START_PIN GPIO_NUM_39  // labeled Start, needs external pullup
*/

#define DEFAULT_DIRECTION_INVERT_MASK 5

// 20230328 卡纸调压力
// #define DEFAULT_X_STEPS_PER_MM 160
// #define DEFAULT_Y_STEPS_PER_MM 190//188.4
// #define DEFAULT_Z_STEPS_PER_MM 90

// #define DEFAULT_X_ACCELERATION 2000
// #define DEFAULT_Y_ACCELERATION 500
// #define DEFAULT_Z_ACCELERATION 1000.0

// #define DEFAULT_X_MAX_RATE 5000.0  // mm/min 5000
// #define DEFAULT_Y_MAX_RATE 3000.0  // mm/min 3000
// #define DEFAULT_Z_MAX_RATE 3000.0  // mm/min 5000


#define DEFAULT_X_STEPS_PER_MM  318.572//160.16//160
#define DEFAULT_Y_STEPS_PER_MM  320.913//188.81//189//188.4/190


#define DEFAULT_Z_STEPS_PER_MM 160.745//49.5//90

#define DEFAULT_X_ACCELERATION 1200// mm/sec^2
#define DEFAULT_Y_ACCELERATION 1200// mm/sec^2
#define DEFAULT_Z_ACCELERATION 5000// mm/sec^2

#define DEFAULT_X_MAX_RATE 6100.0  // mm/min 5000
#define DEFAULT_Y_MAX_RATE 6100.0  // mm/min 3000
#define DEFAULT_Z_MAX_RATE 12000.0  // mm/min 5000

#define DEFAULT_HOMING_ENABLE	1
#define DEFAULT_HOMING_CYCLE_0	0
#define DEFAULT_HOMING_CYCLE_1 bit(X_AXIS)
#define DEFAULT_HOMING_DIR_MASK 5
#define DEFAULT_HOMING_FEED_RATE 500.0
#define DEFAULT_HOMING_SEEK_RATE 3000.0
#define DEFAULT_HOMING_PULLOFF 3.0//撞击行程开关后拉出距离
// #define DEFAULT_HOMING_PULLOFF 3.0//撞击行程开关后拉出距离
#define DEFAULT_HOMING_OFFSET 2.0//默认机器坐标与工作坐标之间的偏移
#define DEFAULT_WORKCOORD_X_OFFSET 1.5//默认机器坐标与工作坐标之间的X偏移
#define DEFAULT_WORKCOORD_Y_OFFSET 4.0//默认机器坐标与工作坐标之间的Y偏移
#define MACHINE_GENERATION 1//刻字机代号：0-一代刻字机；1-二代刻字机
// #define DEFAULT_HOMING_OFFSET 0.8//撞击行程开关后拉出距离与工作

// gcode  config
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 250 //保持电机
#define DEFAULT_X_MAX_TRAVEL 330.0 //最大X行程
#define DEFAULT_Y_MAX_TRAVEL 100000 //最大Y行程
#define DEFAULT_Y_HOMING_MPOS -50 //最大Y行程
#define DEFAULT_Z_MAX_TRAVEL 50.0 //最大Z行程
#define DEFAULT_Z_HOMING_MPOS -50 //最大Y行程
#define DEFAULT_SOFT_LIMIT_ENABLE   0	//防止X轴走的距离超315
#define DEFAULT_HARD_LIMIT_ENABLE 	0	//防止X轴碰到左边
#define DEFAULT_MATE_DIST 	-45	//默认物料进料距离

//#define SPINDLE_TYPE			SpindleType::PWM
#define DEFAULT_LASER_MODE		1
#define DEFAULT_SPINDLE_FREQ   20000//重要，激光的频率在10KHz功率最足。



#define ENABLE_BLEGATT

// #define ENABLE_BLUETOOTH  // enable bluetooth

//#define ENABLE_SD_CARD  // enable use of SD Card to run jobs

// #define ENABLE_WIFI  //enable wifi
// #define ENABLE_HTTP  

//报告切割机状态
#define REPORT_SJCUTTER_STATUS 
#define USE_LINE_NUMBERS


