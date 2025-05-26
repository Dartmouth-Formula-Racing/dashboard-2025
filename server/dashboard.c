#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>

#include <linux/can/raw.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <net/if.h>

// Assuming the pin numbers are defined in our config header
#include "config.h"

#define BUTTON_DEBOUNCE_TIME 50000  // microseconds (50ms)
#define BUTTON_SEND_INTERVAL 50000  // microseconds (50ms)

static volatile int running = 1;

// Button state tracking for debouncing
typedef struct {
    int last_state;
    uint32_t last_change_time;
    int debounced_state;
    int pin;
} button_state_t;

static button_state_t drive_button = {1, 0, 1, 0};      // Will be set to actual pin
static button_state_t neutral_button = {1, 0, 1, 0};
static button_state_t reverse_button = {1, 0, 1, 0};

// Signal handler for clean shutdown
void signal_handler(int sig) {
    running = 0;
}

// Callback function for button interrupts
void button_callback(int gpio, int level, uint32_t tick) {
    button_state_t* btn = NULL;
    
    // Determine which button was pressed
    if (gpio == DRIVE_BUTTON_GPIO) {
        btn = &drive_button;
    } else if (gpio == NEUTRAL_BUTTON_GPIO) {
        btn = &neutral_button;
    } else if (gpio == REVERSE_BUTTON_GPIO) {
        btn = &reverse_button;
    }
    
    if (!btn) return;
    
    // Simple debouncing - ignore changes too close together
    if ((tick - btn->last_change_time) > BUTTON_DEBOUNCE_TIME) {
        if (level != btn->debounced_state) {
            btn->debounced_state = level;
            btn->last_change_time = tick;
            
            // Only trigger on button press (falling edge, level = 0)
            if (level == 0) {
                if (gpio == DRIVE_BUTTON_GPIO) {
                    printf("Drive button pressed\n");
                    // Add your CAN message sending logic here
                } else if (gpio == NEUTRAL_BUTTON_GPIO) {
                    printf("Neutral button pressed\n");
                    // Add your CAN message sending logic here
                } else if (gpio == REVERSE_BUTTON_GPIO) {
                    printf("Reverse button pressed\n");
                    // Add your CAN message sending logic here
                }
            }
        }
    }
}

// Initialize GPIO pins
int init_gpio() {
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio library\n");
        return -1;
    }

    // Set up output pins
    gpioSetMode(BMS_LED_GPIO, PI_OUTPUT);
    gpioSetMode(IMD_LED_GPIO, PI_OUTPUT);
    gpioSetMode(CAN_NRST_GPIO, PI_OUTPUT);
    gpioSetMode(CAN_STBY_GPIO, PI_OUTPUT);
    gpioSetMode(DRIVE_LED_GPIO, PI_OUTPUT);
    gpioSetMode(NEUTRAL_LED_GPIO, PI_OUTPUT);
    gpioSetMode(REVERSE_LED_GPIO, PI_OUTPUT);

    // Set up input pins with pull-up resistors 
    // Not needed in the new dashboard

    // gpioSetMode(DRIVE_BUTTON_GPIO, PI_INPUT);
    // gpioSetPullUpDown(DRIVE_BUTTON_GPIO, PI_PUD_UP);
    
    // gpioSetMode(NEUTRAL_BUTTON_GPIO, PI_INPUT);
    // gpioSetPullUpDown(NEUTRAL_BUTTON_GPIO, PI_PUD_UP);
    
    // gpioSetMode(REVERSE_BUTTON_GPIO, PI_INPUT);
    // gpioSetPullUpDown(REVERSE_BUTTON_GPIO, PI_PUD_UP);

    // Set up interrupt callbacks for buttons (trigger on both edges)
    // don't need it in the new dashboard
    // drive_button.pin = DRIVE_BUTTON_GPIO;
    // neutral_button.pin = NEUTRAL_BUTTON_GPIO;
    // reverse_button.pin = REVERSE_BUTTON_GPIO;
    
    // gpioSetISRFunc(DRIVE_BUTTON_GPIO, EITHER_EDGE, 0, button_callback);
    // gpioSetISRFunc(NEUTRAL_BUTTON_GPIO, EITHER_EDGE, 0, button_callback);
    // gpioSetISRFunc(REVERSE_BUTTON_GPIO, EITHER_EDGE, 0, button_callback);

    // Set initial LED states (1 = ON), we can change it to 0
    gpioWrite(DRIVE_LED_GPIO, 1);
    gpioWrite(NEUTRAL_LED_GPIO, 1);
    gpioWrite(REVERSE_LED_GPIO, 1);
    gpioWrite(BMS_LED_GPIO, 1);
    gpioWrite(IMD_LED_GPIO, 1);

    printf("GPIO initialized successfully\n");
    return 0;
}

// Initialize the canbus
void canbus_init() {

}
// Thread function for periodic CAN message sending
void* periodic_can_thread(void* arg) {
    uint32_t last_send_time = gpioTick();
    
    while (running) {
        uint32_t current_time = gpioTick();
        
        // Send periodic CAN messages (handle tick wraparound)
        if ((current_time - last_send_time) >= BUTTON_SEND_INTERVAL) {
            // Add your periodic CAN message sending logic here
            // canbus_send_status();
            last_send_time = current_time;
        }
        
        // Small delay to prevent CPU spinning
        gpioDelay(1000); // 1ms delay
    }
    
    return NULL;
}

// Function to update LED based on system status
void update_status_leds(int bms_status, int imd_status) {
    if (IN_CAR) {
        gpioWrite(BMS_LED_GPIO, bms_status ? 1 : 0);
        gpioWrite(IMD_LED_GPIO, imd_status ? 1 : 0);
    }
}

// Function to update gear LEDs
void update_gear_leds(int drive_active, int neutral_active, int reverse_active) {
    if (IN_CAR) {
        gpioWrite(DRIVE_LED_GPIO, drive_active ? 1 : 0);
        gpioWrite(NEUTRAL_LED_GPIO, neutral_active ? 1 : 0);
        gpioWrite(REVERSE_LED_GPIO, reverse_active ? 1 : 0);
    }
}

// Function to control CAN transceiver
void set_can_transceiver(int reset_state, int standby_state) {
    if (IN_CAR) {
        gpioWrite(CAN_NRST_GPIO, reset_state ? 1 : 0);
        gpioWrite(CAN_STBY_GPIO, standby_state ? 1 : 0);
    }
}

int main() {
    printf("Racing Dashboard Starting...\n");
    
    // Set up signal handlers for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    if (IN_CAR) {
        if (init_gpio() != 0) {
            fprintf(stderr, "Failed to initialize GPIO\n");
            return -1;
        }
        
        // Enable CAN transceiver (example - adjust based on your hardware)
        set_can_transceiver(1, 0); // Reset = HIGH, Standby = LOW (active)
    }
    
    // Initialize your CAN bus and web modules here
    // canbus_init();
    // web_init();
    
    pthread_t periodic_thread;
    
    if (IN_CAR) {
        // Create periodic CAN message thread
        if (pthread_create(&periodic_thread, NULL, periodic_can_thread, NULL) != 0) {
            fprintf(stderr, "Failed to create periodic CAN thread\n");
            gpioTerminate();
            return -1;
        }
    }
    
    // Main loop - handle CAN messages and other tasks
    printf("Main loop started. Press Ctrl+C to exit.\n");
    
    while (running) {
        // Your main CAN message processing loop here
        // This is where you'd handle incoming CAN messages and update dashboard
        
        // Example: Update LEDs based on vehicle state
        if (IN_CAR) {
            // Example status updates (replace with actual logic)
            static int counter = 0;
            counter++;
            
            // Example: Toggle status LEDs every few seconds for demo
            if (counter % 1000 == 0) {
                // update_status_leds(bms_status, imd_status);
                // update_gear_leds(drive_active, neutral_active, reverse_active);
            }
        }
        
        // High-frequency CAN processing
        // canbus_process_messages();
        
        gpioDelay(1000); // 1ms delay - adjust based on your CAN message frequency
    }
    
    printf("Shutting down...\n");
    
    if (IN_CAR) {
        // Wait for periodic thread to finish
        pthread_join(periodic_thread, NULL);
        
        // Turn off all LEDs before shutdown
        gpioWrite(DRIVE_LED_GPIO, 0);
        gpioWrite(NEUTRAL_LED_GPIO, 0);
        gpioWrite(REVERSE_LED_GPIO, 0);
        gpioWrite(BMS_LED_GPIO, 0);
        gpioWrite(IMD_LED_GPIO, 0);
        
        // Put CAN transceiver in standby
        set_can_transceiver(0, 1); // Reset = LOW, Standby = HIGH (standby)
        
        // Clean up pigpio library
        gpioTerminate();
    }
    
    printf("Dashboard shutdown complete.\n");
    return 0;
}