#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include "config.h"
#include "canbus.h"
#include <math.h>
#include "websocket_server.h"


#define CAN_RECV_TIMEOUT_US 1000    // 1ms timeout in microseconds
#define CAN_BITRATE 500000
#define CAN_RESTART_MS 20
#define CAN_TXQUEUE_LEN 65536
#define MAX_CAN_MESSAGES 1000
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

// Add these structures and variables after your existing includes and before main()

// Vehicle state structure to match Python state dictionary
// typedef struct {
//     // BMS and safety states
//     int bms;
//     int imd;
//     int bot;
//     int brb;
//     int cvc_overflow;
//     int cvc_time;
    
//     // Drive states
//     char drive_state[20];
//     char vehicle_state[30];
    
//     // Temperature data
//     float leftinvtemp;
//     float rightinvtemp;
//     float acctemp;
    
//     // Driving data
//     float throttle_position;
//     int rpm;
//     float speed;
//     float mileage;
    
//     // Battery data
//     float accumulator_voltage;
//     float accumulator_current;
//     float battery_percentage;
// } vehicle_state_t;

vehicle_state_t vehicle_state = {0};
void process_can_message(uint32_t msg_id, uint8_t* data, uint8_t len, int is_extended);
void signal_handler(int sig);
void button_callback(int gpio, int level, uint32_t tick);
int create_can_socket(); 
int rx_queue_put(struct can_frame* frame);
int rx_queue_get(struct can_frame* frame);
int tx_queue_put(struct can_frame* frame);
int tx_queue_get(struct can_frame* frame);
void* can_process(void* arg);
int send_can_message(uint32_t id, uint8_t* data, uint8_t len);
int receive_can_message(uint32_t* id, uint8_t* data, uint8_t* len);
int init_shared_state();
void cleanup() ;
int send_button_message(int button_type);
int init_gpio();




// Function to process received CAN messages (based on your Python logic)
void process_can_message(uint32_t msg_id, uint8_t* data, uint8_t len, int is_extended) {
    if (is_extended) {
        // Extended ID messages (inverter temperatures)
        if (msg_id == CAN_INVERTER1_BASE + 0) { // Inverter 1 temperatures 1
            int16_t module_A_temp = (data[1] << 8) | data[0];
            module_A_temp = (module_A_temp > 32767) ? module_A_temp - 65536 : module_A_temp;
            float temp_A = module_A_temp / 10.0f;
            
            int16_t module_B_temp = (data[3] << 8) | data[2];
            module_B_temp = (module_B_temp > 32767) ? module_B_temp - 65536 : module_B_temp;
            float temp_B = module_B_temp / 10.0f;
            
            int16_t module_C_temp = (data[5] << 8) | data[4];
            module_C_temp = (module_C_temp > 32767) ? module_C_temp - 65536 : module_C_temp;
            float temp_C = module_C_temp / 10.0f;
            
            vehicle_state.leftinvtemp = fmaxf(fmaxf(temp_A, temp_B), temp_C);
            // printf("Left Inverter Temp: %.1f°C\n", vehicle_state.leftinvtemp);
        }
        else if (msg_id == CAN_INVERTER2_BASE + 0) { // Inverter 2 temperatures 1
            int16_t module_A_temp = (data[1] << 8) | data[0];
            module_A_temp = (module_A_temp > 32767) ? module_A_temp - 65536 : module_A_temp;
            float temp_A = module_A_temp / 10.0f;
            
            int16_t module_B_temp = (data[3] << 8) | data[2];
            module_B_temp = (module_B_temp > 32767) ? module_B_temp - 65536 : module_B_temp;
            float temp_B = module_B_temp / 10.0f;
            
            int16_t module_C_temp = (data[5] << 8) | data[4];
            module_C_temp = (module_C_temp > 32767) ? module_C_temp - 65536 : module_C_temp;
            float temp_C = module_C_temp / 10.0f;
            
            vehicle_state.rightinvtemp = fmaxf(fmaxf(temp_A, temp_B), temp_C);
            // printf("Right Inverter Temp: %.1f°C\n", vehicle_state.rightinvtemp);
        }
    }
    else {
        // Standard ID messages
        if (msg_id == CAN_BASE_ID + 1) { // Vehicle state
            vehicle_state.bms = data[0];
            vehicle_state.imd = data[1];
            int drive_state = data[2];
            int vehicle_state_val = data[3];
            vehicle_state.bot = data[4];
            vehicle_state.brb = data[5];
            vehicle_state.cvc_overflow = data[6];
            vehicle_state.cvc_time = data[7];

            // Update drive state string
            switch(drive_state) {
                case 0:
                    strcpy(vehicle_state.drive_state, "NEUTRAL");
                    gpioWrite(NEUTRAL_LED_GPIO, 1);
                    gpioWrite(DRIVE_LED_GPIO, 0);
                    gpioWrite(REVERSE_LED_GPIO, 0);
                    break;
                case 1:
                    strcpy(vehicle_state.drive_state, "DRIVE");
                    gpioWrite(DRIVE_LED_GPIO, 1);
                    gpioWrite(NEUTRAL_LED_GPIO, 0);
                    gpioWrite(REVERSE_LED_GPIO, 0);
                    break;
                case 2:
                    strcpy(vehicle_state.drive_state, "REVERSE");
                    gpioWrite(REVERSE_LED_GPIO, 1);
                    gpioWrite(DRIVE_LED_GPIO, 0);
                    gpioWrite(NEUTRAL_LED_GPIO, 0);
                    break;
            }

            // Update vehicle state string
            switch(vehicle_state_val) {
                case 0: strcpy(vehicle_state.vehicle_state, "Initial"); break;
                case 1: strcpy(vehicle_state.vehicle_state, "Voltage Check"); break;
                case 2: strcpy(vehicle_state.vehicle_state, "Wait for Precharge"); break;
                case 3: strcpy(vehicle_state.vehicle_state, "Precharge Stage 1"); break;
                case 4: strcpy(vehicle_state.vehicle_state, "Precharge Stage 2"); break;
                case 5: strcpy(vehicle_state.vehicle_state, "Precharge Stage 3"); break;
                case 6: strcpy(vehicle_state.vehicle_state, "Not Ready to Drive"); break;
                case 7: strcpy(vehicle_state.vehicle_state, "Buzzer"); break;
                case 8: strcpy(vehicle_state.vehicle_state, "Ready to Drive"); break;
                case 9: strcpy(vehicle_state.vehicle_state, "Charging"); break;
                default: strcpy(vehicle_state.vehicle_state, "Unknown"); break;
            }
            
            // Update BMS and IMD LEDs
            gpioWrite(BMS_LED_GPIO, vehicle_state.bms ? 0 : 1);
            gpioWrite(IMD_LED_GPIO, vehicle_state.imd ? 0 : 1);
            
            // printf("Vehicle State: %s, Drive: %s, BMS: %d, IMD: %d\n", 
                //    vehicle_state.vehicle_state, vehicle_state.drive_state, 
                //    vehicle_state.bms, vehicle_state.imd);
        }
        else if (msg_id == CAN_BASE_ID + 2) { // Driving data
            vehicle_state.throttle_position = ((data[0] << 8) | data[1]) / 10.0f;
            vehicle_state.rpm = (data[2] << 8) | data[3];
            
            // Calculate speed (assuming WHEEL_DIAMETER and TRANSMISSION_RATIO are defined)
            vehicle_state.speed = (vehicle_state.rpm * 60 * WHEEL_DIAMETER * 3.1415926535f) / 
                                 (12 * 5280 * TRANSMISSION_RATIO);
            vehicle_state.mileage = ((data[6] << 8) | data[7]) / 1000.0f;
            
            printf("Throttle: %.1f%%, RPM: %d, Speed: %.1f mph, Mileage: %.3f mi\n",
                   vehicle_state.throttle_position, vehicle_state.rpm, 
                   vehicle_state.speed, vehicle_state.mileage);
        }
        else if (msg_id == CAN_BMS_BASE + 1) { // BMS pack voltage
            vehicle_state.accumulator_voltage = ((data[5] << 24) | (data[6] << 16) | 
                                               (data[3] << 8) | data[4]) / 100.0f;
            // printf("Accumulator Voltage: %.2f V\n", vehicle_state.accumulator_voltage);
        }
        else if (msg_id == CAN_BMS_BASE + 5) { // BMS current
            uint16_t current_bytes = (data[0] << 8) | data[1];
            int16_t current_value = (current_bytes > 32767) ? current_bytes - 65536 : current_bytes;
            vehicle_state.accumulator_current = current_value / 10.0f;
            // printf("Accumulator Current: %.1f A\n", vehicle_state.accumulator_current);
        }
        else if (msg_id == CAN_BMS_BASE + 16) { // BMS state of charge
            uint16_t soc_bytes = (data[2] << 8) | data[3];
            vehicle_state.battery_percentage = soc_bytes / 100.0f;
            // printf("Battery SOC: %.1f%%\n", vehicle_state.battery_percentage);
        }
        else if (msg_id == CAN_BMS_BASE + 8) { // BMS cell temperatures
            vehicle_state.acctemp = data[1] - 100; // Temperature in Celsius
            // printf("Accumulator Temp: %.0f°C\n", vehicle_state.acctemp);
        }
    }
}

// Shared memory structures for IPC
// typedef struct {
//     struct can_frame frame;
//     int valid;
// } can_message_t;

// typedef struct {
//     can_message_t rx_messages[MAX_CAN_MESSAGES];
//     can_message_t tx_messages[MAX_CAN_MESSAGES];
//     int rx_head, rx_tail;
//     int tx_head, tx_tail;
//     int can_connected;
//     volatile int running;
//     pthread_mutex_t rx_mutex;
//     pthread_mutex_t tx_mutex;
// } shared_state_t;

shared_state_t* shared_state = NULL;
static int can_socket = -1;

// Signal handler
void signal_handler(int sig) {
    if (shared_state) {
        shared_state->running = 0;
    }
}

// Updated button callback function
void button_callback(int gpio, int level, uint32_t tick) {
    button_state_t* btn = NULL;
    
    // Determine which button was pressed
    if (gpio == DRIVE_BUTTON_GPIO) {
        printf("Drive button pressed\n\r");
        btn = &drive_button;
    } else if (gpio == NEUTRAL_BUTTON_GPIO) {
        printf("Neutral button pressed\n\r");
        btn = &neutral_button;
    } else if (gpio == REVERSE_BUTTON_GPIO) {
        printf("Reverse button pressed\n\r");
        btn = &reverse_button;
    }
    
    if (!btn) return;
    
    // Simple debouncing
    if ((tick - btn->last_change_time) > BUTTON_DEBOUNCE_TIME) {
        // printf("Button debounced\n\r");
        if (level != btn->debounced_state) {
            btn->debounced_state = level;
            btn->last_change_time = tick;
            
            // Only trigger on button press (falling edge, level = 0)
            if (level == 0) {
                if (gpio == DRIVE_BUTTON_GPIO) {
                    printf("Drive Message Sending\n\r");
                    send_button_message(0); // Drive
                } else if (gpio == NEUTRAL_BUTTON_GPIO) {
                    printf("Neutral Message Sending\n\r");
                    send_button_message(1); // Neutral
                } else if (gpio == REVERSE_BUTTON_GPIO) {
                    printf("reverse Message Sending\n\r");
                    send_button_message(2); // Reverse
                }
            }
        }
    }
}
// Initialize CAN interface
// int init_can_interface() {
//     char cmd[256];
//     int ret;
    
//     printf("Initializing CAN interface...\n");
    
//     // Put CAN transceiver in reset and standby mode
//     if (IN_CAR) {
//         gpioWrite(CAN_NRST_GPIO, 0);  // Reset = LOW
//         gpioWrite(CAN_STBY_GPIO, 1);  // Standby = HIGH
//         gpioDelay(100000);  // 100ms delay
//         printf("CAN transceiver in reset/standby\n");
        
//         // Pull chip out of reset
//         gpioWrite(CAN_NRST_GPIO, 1);  // Reset = HIGH
//         gpioDelay(100000);  // 100ms delay
//         printf("CAN transceiver out of reset\n");
//     }
    
//     // Bring down interface first
//     system("sudo ip link set can0 down 2>/dev/null");
//     gpioDelay(100000);
    
//     // Configure and bring up CAN interface
//     snprintf(cmd, sizeof(cmd), 
//         "sudo ip link set can0 up type can bitrate %d restart-ms %d", 
//         CAN_BITRATE, CAN_RESTART_MS);
//     ret = system(cmd);
//     if (ret != 0) {
//         fprintf(stderr, "Failed to bring up CAN interface\n");
//         return -1;
//     }
    
//     gpioDelay(100000);  // 100ms delay
    
//     // Set TX queue length
//     snprintf(cmd, sizeof(cmd), "sudo ifconfig can0 txqueuelen %d", CAN_TXQUEUE_LEN);
//     ret = system(cmd);
//     if (ret != 0) {
//         fprintf(stderr, "Warning: Failed to set TX queue length\n");
//     }
    
//     if (IN_CAR) {
//         // Pull chip out of standby mode
//         gpioWrite(CAN_STBY_GPIO, 0);  // Standby = LOW (active)
//         gpioDelay(100000);  // 100ms delay
//         printf("CAN transceiver active\n");
//     }
    
//     printf("CAN interface initialized\n");
//     return 0;
// }

// Create SocketCAN socket
int create_can_socket() {
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sock;
    
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket");
        return -1;
    }
    
    strcpy(ifr.ifr_name, "can0");
    ioctl(sock, SIOCGIFINDEX, &ifr);
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    return sock;
}

// Queue management functions
int rx_queue_put(struct can_frame* frame) {

    pthread_mutex_lock(&shared_state->rx_mutex);
    
    int next_head = (shared_state->rx_head + 1) % MAX_CAN_MESSAGES;
    if (next_head == shared_state->rx_tail) {
        // Queue full
        pthread_mutex_unlock(&shared_state->rx_mutex);
        return -1;
    }
    
    shared_state->rx_messages[shared_state->rx_head].frame = *frame;
    shared_state->rx_messages[shared_state->rx_head].valid = 1;
    shared_state->rx_head = next_head;
    
    pthread_mutex_unlock(&shared_state->rx_mutex);
    return 0;
}

int rx_queue_get(struct can_frame* frame) {
    pthread_mutex_lock(&shared_state->rx_mutex);
    
    if (shared_state->rx_head == shared_state->rx_tail) {
        // Queue empty
        // printf("Can Message queue is empty\r\n");
        pthread_mutex_unlock(&shared_state->rx_mutex);
        return -1;
    }
    
    *frame = shared_state->rx_messages[shared_state->rx_tail].frame;
    shared_state->rx_messages[shared_state->rx_tail].valid = 0;
    shared_state->rx_tail = (shared_state->rx_tail + 1) % MAX_CAN_MESSAGES;
    
    pthread_mutex_unlock(&shared_state->rx_mutex);
    return 0;
}

int tx_queue_put(struct can_frame* frame) {
    pthread_mutex_lock(&shared_state->tx_mutex);
    
    int next_head = (shared_state->tx_head + 1) % MAX_CAN_MESSAGES;
    if (next_head == shared_state->tx_tail) {
        // Queue full
        pthread_mutex_unlock(&shared_state->tx_mutex);
        return -1;
    }
    
    shared_state->tx_messages[shared_state->tx_head].frame = *frame;
    shared_state->tx_messages[shared_state->tx_head].valid = 1;
    shared_state->tx_head = next_head;
    
    pthread_mutex_unlock(&shared_state->tx_mutex);
    return 0;
}

int tx_queue_get(struct can_frame* frame) {
    pthread_mutex_lock(&shared_state->tx_mutex);
    
    if (shared_state->tx_head == shared_state->tx_tail) {
        // Queue empty
        pthread_mutex_unlock(&shared_state->tx_mutex);
        return -1;
    }
    
    *frame = shared_state->tx_messages[shared_state->tx_tail].frame;
    shared_state->tx_messages[shared_state->tx_tail].valid = 0;
    shared_state->tx_tail = (shared_state->tx_tail + 1) % MAX_CAN_MESSAGES;
    
    pthread_mutex_unlock(&shared_state->tx_mutex);
    return 0;
}

// CAN process function (equivalent to your Python run function)
void* can_process(void* arg) {
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    int ret;
    
    printf("CAN process started\n");
    
    while (shared_state->running) {
        // Initialize/reinitialize CAN socket
        if (can_socket < 0) {
            can_socket = create_can_socket();
            if (can_socket < 0) {
                printf("CAN initialization failed, retrying...\n");
                shared_state->can_connected = 0;
                gpioDelay(500000);  // 500ms delay
                continue;
            }
            shared_state->can_connected = 1;
            printf("CAN socket connected\n");
        }
        
        // Check for received data with timeout
        FD_ZERO(&readfds);
        FD_SET(can_socket, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = CAN_RECV_TIMEOUT_US;
        
        ret = select(can_socket + 1, &readfds, NULL, NULL, &timeout);
        
        if (ret > 0 && FD_ISSET(can_socket, &readfds)) {
            // Data available to read
            ssize_t nbytes = read(can_socket, &frame, sizeof(struct can_frame));
            if (nbytes == sizeof(struct can_frame)) {
                if (rx_queue_put(&frame) != 0) {
                    printf("RX queue full, dropping message\n");
                }
            } else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("CAN read error");
                close(can_socket);
                can_socket = -1;
                shared_state->can_connected = 0;
                continue;
            }
        } else if (ret < 0) {
            perror("select error");
            close(can_socket);
            can_socket = -1;
            shared_state->can_connected = 0;
            continue;
        }
        
        // Check for data to send
        while (tx_queue_get(&frame) == 0) {
            ssize_t nbytes = write(can_socket, &frame, sizeof(struct can_frame));
            if (nbytes != sizeof(struct can_frame)) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("CAN write error");
                    close(can_socket);
                    can_socket = -1;
                    shared_state->can_connected = 0;
                    break;
                }
                // Put message back in queue if temporary error
                tx_queue_put(&frame);
                break;
            }
        }
        
        // Small delay to prevent CPU spinning
        gpioDelay(100);  // 100µs
    }
    
    if (can_socket >= 0) {
        close(can_socket);
    }
    
    printf("CAN process stopped\n");
    return NULL;
}

// Helper function to send CAN message
int send_can_message(uint32_t id, uint8_t* data, uint8_t len) {
    struct can_frame frame;
    
    if (len > 8) {
        return -1;  // Invalid length
    }
    
    frame.can_id = id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    
    return tx_queue_put(&frame);
}

// Helper function to receive CAN message
int receive_can_message(uint32_t* id, uint8_t* data, uint8_t* len) {
    struct can_frame frame;
    // printf("Entered receive_can_message\r\n");
    if (rx_queue_get(&frame) == 0) {
        // printf("Entered rx_queue_get\r\n");
        *id = frame.can_id;
        *len = frame.can_dlc;
        memcpy(data, frame.data, frame.can_dlc);
        return 0;
    }
    
    return -1;  // No message available
}

// Initialize shared state and mutexes
int init_shared_state() {
    shared_state = malloc(sizeof(shared_state_t));
    if (!shared_state) {
        return -1;
    }
    
    memset(shared_state, 0, sizeof(shared_state_t));
    shared_state->running = 1;
    
    pthread_mutex_init(&shared_state->rx_mutex, NULL);
    pthread_mutex_init(&shared_state->tx_mutex, NULL);
    
    return 0;
}

// Cleanup function
void cleanup() {
    if (shared_state) {
        shared_state->running = 0;
        pthread_mutex_destroy(&shared_state->rx_mutex);
        pthread_mutex_destroy(&shared_state->tx_mutex);
        free(shared_state);
        shared_state = NULL;
    }
    
    if (can_socket >= 0) {
        close(can_socket);
        can_socket = -1;
    }
}

// Function to send button press messages
int send_button_message(int button_type) {
    uint8_t button_data[8] = {0};
    uint32_t button_msg_id = CAN_BASE_ID; // Assuming button messages use this ID
    
    switch(button_type) {
        case 0: // Drive button
            button_data[0] = 1; // Drive command
            printf("Sending DRIVE button command\n");
            break;
        case 1: // Neutral button
            button_data[0] = 0; // Neutral command
            printf("Sending NEUTRAL button command\n");
            break;
        case 2: // Reverse button
            button_data[0] = 2; // Reverse command
            printf("Sending REVERSE button command\n");
            break;
        default:
            return -1;
    }
    
    // Add timestamp or sequence number if needed
    button_data[1] = (uint8_t)(gpioTick() & 0xFF);
    
    return send_can_message(button_msg_id, button_data, 8);
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

    gpioSetMode(DRIVE_BUTTON_GPIO, PI_INPUT);
    gpioSetPullUpDown(DRIVE_BUTTON_GPIO, PI_PUD_UP);
    
    gpioSetMode(NEUTRAL_BUTTON_GPIO, PI_INPUT);
    gpioSetPullUpDown(NEUTRAL_BUTTON_GPIO, PI_PUD_UP);
    
    gpioSetMode(REVERSE_BUTTON_GPIO, PI_INPUT);
    gpioSetPullUpDown(REVERSE_BUTTON_GPIO, PI_PUD_UP);

    // Set up interrupt callbacks for buttons (trigger on both edges)
    // don't need it in the new dashboard
    drive_button.pin = DRIVE_BUTTON_GPIO;
    neutral_button.pin = NEUTRAL_BUTTON_GPIO;
    reverse_button.pin = REVERSE_BUTTON_GPIO;
    
    gpioSetISRFunc(DRIVE_BUTTON_GPIO, FALLING_EDGE, 0, button_callback);
    gpioSetISRFunc(NEUTRAL_BUTTON_GPIO, FALLING_EDGE, 0, button_callback);
    gpioSetISRFunc(REVERSE_BUTTON_GPIO, FALLING_EDGE, 0, button_callback);

    // Set initial LED states (1 = ON), we can change it to 0
    gpioWrite(DRIVE_LED_GPIO, 1);
    gpioWrite(NEUTRAL_LED_GPIO, 1);
    gpioWrite(REVERSE_LED_GPIO, 1);
    gpioWrite(BMS_LED_GPIO, 1);
    gpioWrite(IMD_LED_GPIO, 1);

    printf("GPIO initialized successfully\n");
    return 0;
}


// Example usage in main function
int main() {
    pthread_t can_thread;
    pthread_t ws_thread;
    struct can_frame rx_frame;
    uint32_t msg_id;
    uint8_t msg_data[8];
    uint8_t msg_len;
    
    printf("Racing CAN System Starting...\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize pigpio
    if (init_gpio() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return -1;
    }
    
    // Initialize shared state
    if (init_shared_state() != 0) {
        fprintf(stderr, "Failed to initialize shared state\n");
        gpioTerminate();
        return -1;
    }
    
    // Start CAN process thread
    if (pthread_create(&can_thread, NULL, can_process, NULL) != 0) {
        fprintf(stderr, "Failed to create CAN thread\n");
        cleanup();
        gpioTerminate();
        return -1;
    }

    // Start websocket thread
    if (pthread_create(&ws_thread, NULL, websocket_server, NULL) != 0) {
    fprintf(stderr, "Failed to create WebSocket thread\n");
    cleanup();
    gpioTerminate();
    return -1;
}
    
    printf("CAN system initialized. Main loop starting...\n");
    
    // Main loop - your dashboard logic here
    while (shared_state->running) {
        // Check for received CAN messages and process them
        if (receive_can_message(&msg_id, msg_data, &msg_len) == 0) {
           // printf("Received CAN message: ID=0x%03X, len=%d, data=", msg_id, msg_len);
            // for (int i = 0; i < msg_len; i++) {
            //     printf("%02X ", msg_data[i]);
            // }
           // printf("\n");
            int is_extended = (msg_id & CAN_EFF_FLAG) ? 1 : 0;
            msg_id &= ~(CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG);
            
            // Process the message using our new function
            process_can_message(msg_id, msg_data, msg_len, is_extended);
        }

        if (gpioRead(NEUTRAL_BUTTON_GPIO)) {
            printf("N\n");
        }
        if (gpioRead(DRIVE_BUTTON_GPIO)) {
            printf("D\n");
        }
        if (gpioRead(REVERSE_BUTTON_GPIO)) {
            printf("R\n");
        }
        
        // // Example: Send periodic CAN message
        // static int counter = 0;
        // if (++counter >= 1000) {  // Every ~100ms at 100µs loop
        //     uint8_t status_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        //     if (send_can_message(0x100, status_data, 8) != 0) {
        //         printf("Failed to queue CAN message\n");
        //     }
        //     counter = 0;
        // }
        
        // // Print connection status periodically
        // static int status_counter = 0;
        // if (++status_counter >= 10000) {  // Every ~1s
        //     printf("CAN connected: %s\n", shared_state->can_connected ? "YES" : "NO");
        //     status_counter = 0;
        // }
        
        gpioDelay(100);  // 100µs loop time
    }
    
    printf("Shutting down...\n");
    
    // Wait for CAN thread to finish
    pthread_join(can_thread, NULL);
    // Wait for Websocket thread to finish
    pthread_join(ws_thread, NULL);

    
    cleanup();
    gpioTerminate();
    
    printf("Shutdown complete\n");
    return 0;
}