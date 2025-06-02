#ifndef CANBUS_H
#define CANBUS_H

#include <linux/can.h>
#include <pthread.h>

#define MAX_CAN_MESSAGES 1000

typedef struct {
    int bms;
    int imd;
    int bot;
    int brb;
    int cvc_overflow;
    int cvc_time;
    char drive_state[20];
    char vehicle_state[30];
    float leftinvtemp;
    float rightinvtemp;
    float acctemp;
    float throttle_position;
    int rpm;
    float speed;
    float mileage;
    float accumulator_voltage;
    float accumulator_current;
    float battery_percentage;
} vehicle_state_t;

typedef struct {
    struct can_frame frame;
    int valid;
} can_message_t;

typedef struct shared_state_t {
    can_message_t rx_messages[MAX_CAN_MESSAGES];
    can_message_t tx_messages[MAX_CAN_MESSAGES];
    int rx_head, rx_tail;
    int tx_head, tx_tail;
    int can_connected;
    volatile int running;
    pthread_mutex_t rx_mutex;
    pthread_mutex_t tx_mutex;
} shared_state_t;

extern vehicle_state_t vehicle_state;
extern shared_state_t* shared_state;

#endif
