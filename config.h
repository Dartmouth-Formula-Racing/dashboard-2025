#ifndef CONFIG_H
#define CONFIG_H

#define SEND_INTERVAL_MS 50
#define CAN_POLL_INTERVAL_MS 10
#define CAN_RX_QUEUE_SIZE 50

// GPIO Assignments (using wiringPi pin numbering)
#define BUTTON_DRIVE_PIN 3
#define BUTTON_NEUTRAL_PIN 28
#define BUTTON_REVERSE_PIN 23
#define LED_DRIVE_PIN 2
#define LED_NEUTRAL_PIN 27
#define LED_REVERSE_PIN 24
#define LED_IMD_PIN 4
#define LED_BMS_PIN 5

#define CAN_NRST_GPIO 21
#define CAN_STBY_GPIO 22

// CAN IDs
// Command messages (sent by the backend)
#define CAN_BASE_ID 0x750
#define CAN_EXTENDED_ID 0

#define CAN_INVERTER1_BASE 0x6C0
#define CAN_INVERTER2_BASE 0x5C0
#define CAN_INVERTER_EXTENDED 1

#define CAN_BMS_BASE 0x320
#define CAN_BMS_EXTENDED 0

// CAN frame constants
#define CAN_MAX_DLC 8

// Math constants
#define PI 3.1415926535
#define TRANSMISSION_RATIO 4.7
#define WHEEL_DIAMETER 20.5  // in inches

#endif  // CONFIG_H