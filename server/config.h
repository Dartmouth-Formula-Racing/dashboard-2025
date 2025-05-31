// GPIO pin definitions (replace with your actual pin numbers from config)
// These should match your config.h values
#ifndef CONFIG_H
#define CONFIG_H

#define IN_CAR 1
#define IMD_LED_GPIO 23
#define BMS_LED_GPIO 24
#define CAN_NRST_GPIO 5
#define CAN_STBY_GPIO 6
#define DRIVE_BUTTON_GPIO 22
#define DRIVE_LED_GPIO 27
#define NEUTRAL_BUTTON_GPIO 20
#define NEUTRAL_LED_GPIO 16
#define REVERSE_BUTTON_GPIO 13
#define REVERSE_LED_GPIO 19

#define WHEEL_DIAMETER 20.5 //inches
#define TRANSMISSION_RATIO 4.7

#define DASH_UPDATE_FREQUENCY 60 // Hz
#define BUTTON_TRANSMIT_INTERVAL 50 // ms
#define CAN_RECV_TIMEOUT  0.005 // s
#define CAN_TX_QUEUE_SIZE 10

#define CAN_RESERVED_GPIO  {7, 8, 9, 10, 11, 23, 25}
#define CAN_BASE_ID 0x750
#define CAN_EXTENDED_ID 0

#define CAN_INVERTER1_BASE 0x6C0
#define CAN_INVERTER2_BASE 0x5C0
#define CAN_INVERTER_EXTENDED_ID 1

#define CAN_BMS_BASE 0x320
#define CAN_BMS_EXTENDED_ID 0

#define BUTTON1_GPIO 0
#define BUTTON2_GPIO 0
#define SWITCH1_POS1_GPIO 0
#define SWITCH1_POS2_GPIO 0
#define SWITCH2_POS1_GPIO 0
#define SWITCH2_POS2_GPIO 0

#endif // CONFIG_H

