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

#define CAN_RECV_TIMEOUT_US 1000    // 1ms timeout in microseconds
#define CAN_BITRATE 500000
#define CAN_RESTART_MS 20
#define CAN_TXQUEUE_LEN 65536
#define MAX_CAN_MESSAGES 1000

// Shared memory structures for IPC
typedef struct {
    struct can_frame frame;
    int valid;
} can_message_t;

typedef struct {
    can_message_t rx_messages[MAX_CAN_MESSAGES];
    can_message_t tx_messages[MAX_CAN_MESSAGES];
    int rx_head, rx_tail;
    int tx_head, tx_tail;
    int can_connected;
    volatile int running;
    pthread_mutex_t rx_mutex;
    pthread_mutex_t tx_mutex;
} shared_state_t;

static shared_state_t* shared_state = NULL;
static int can_socket = -1;

// Signal handler
void signal_handler(int sig) {
    if (shared_state) {
        shared_state->running = 0;
    }
}

// Initialize CAN interface
int init_can_interface() {
    char cmd[256];
    int ret;
    
    printf("Initializing CAN interface...\n");
    
    // Put CAN transceiver in reset and standby mode
    if (IN_CAR) {
        gpioWrite(CAN_NRST_GPIO, 0);  // Reset = LOW
        gpioWrite(CAN_STBY_GPIO, 1);  // Standby = HIGH
        gpioDelay(100000);  // 100ms delay
        printf("CAN transceiver in reset/standby\n");
        
        // Pull chip out of reset
        gpioWrite(CAN_NRST_GPIO, 1);  // Reset = HIGH
        gpioDelay(100000);  // 100ms delay
        printf("CAN transceiver out of reset\n");
    }
    
    // Bring down interface first
    system("sudo ip link set can0 down 2>/dev/null");
    gpioDelay(100000);
    
    // Configure and bring up CAN interface
    snprintf(cmd, sizeof(cmd), 
        "sudo ip link set can0 up type can bitrate %d restart-ms %d", 
        CAN_BITRATE, CAN_RESTART_MS);
    ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "Failed to bring up CAN interface\n");
        return -1;
    }
    
    gpioDelay(100000);  // 100ms delay
    
    // Set TX queue length
    snprintf(cmd, sizeof(cmd), "sudo ifconfig can0 txqueuelen %d", CAN_TXQUEUE_LEN);
    ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "Warning: Failed to set TX queue length\n");
    }
    
    if (IN_CAR) {
        // Pull chip out of standby mode
        gpioWrite(CAN_STBY_GPIO, 0);  // Standby = LOW (active)
        gpioDelay(100000);  // 100ms delay
        printf("CAN transceiver active\n");
    }
    
    printf("CAN interface initialized\n");
    return 0;
}

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
        // while (tx_queue_get(&frame) == 0) {
        //     ssize_t nbytes = write(can_socket, &frame, sizeof(struct can_frame));
        //     if (nbytes != sizeof(struct can_frame)) {
        //         if (errno != EAGAIN && errno != EWOULDBLOCK) {
        //             perror("CAN write error");
        //             close(can_socket);
        //             can_socket = -1;
        //             shared_state->can_connected = 0;
        //             break;
        //         }
        //         // Put message back in queue if temporary error
        //         tx_queue_put(&frame);
        //         break;
        //     }
        // }
        
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
    
    if (rx_queue_get(&frame) == 0) {
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

// Example usage in main function
int main() {
    pthread_t can_thread;
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
    
    // Initialize CAN interface
    // if (init_can_interface() != 0) {
    //     fprintf(stderr, "Failed to initialize CAN interface\n");
    //     cleanup();
    //     gpioTerminate();
    //     return -1;
    // }
    
    // Start CAN process thread
    if (pthread_create(&can_thread, NULL, can_process, NULL) != 0) {
        fprintf(stderr, "Failed to create CAN thread\n");
        cleanup();
        gpioTerminate();
        return -1;
    }
    
    printf("CAN system initialized. Main loop starting...\n");
    
    // Main loop - your dashboard logic here
    while (shared_state->running) {
        // Example: Check for received CAN messages
        if (receive_can_message(&msg_id, msg_data, &msg_len) == 0) {
            printf("Received CAN message: ID=0x%03X, len=%d, data=", msg_id, msg_len);
            for (int i = 0; i < msg_len; i++) {
                printf("%02X ", msg_data[i]);
            }
            printf("\n");
            
            // Process the message based on ID
            // switch(msg_id) {
            //     case 0x123:
            //         // Handle specific message
            //         break;
            // }
        }
        
        // Example: Send periodic CAN message
        // static int counter = 0;
        // if (++counter >= 1000) {  // Every ~100ms at 100µs loop
        //     uint8_t status_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        //     if (send_can_message(0x100, status_data, 8) != 0) {
        //         printf("Failed to queue CAN message\n");
        //     }
        //     counter = 0;
        // }
        
        // Print connection status periodically
        static int status_counter = 0;
        if (++status_counter >= 10000) {  // Every ~1s
            printf("CAN connected: %s\n", shared_state->can_connected ? "YES" : "NO");
            status_counter = 0;
        }
        
        gpioDelay(100);  // 100µs loop time
    }
    
    printf("Shutting down...\n");
    
    // Wait for CAN thread to finish
    pthread_join(can_thread, NULL);
    
    cleanup();
    gpioTerminate();
    
    printf("Shutdown complete\n");
    return 0;
}