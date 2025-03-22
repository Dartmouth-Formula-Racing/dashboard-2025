#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <wiringPi.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

#include "config.h"
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;

void parseFrame(const struct can_frame &frame);
void on_open(websocketpp::connection_hdl h);
void broadcastMessage(const std::string &msg);
void sendDriveModeCommand(int can_socket, int mode);
int setup_can_socket(const char *ifname);
void setupGPIO();
void initializeCAN();
void processButtons(int can_socket);

server ws_server;
websocketpp::connection_hdl currentConn;
std::map<int, std::chrono::steady_clock::time_point> lastSentTime;
std::map<int, bool> lastButtonState;

int main() {
    ws_server.init_asio();
    ws_server.set_open_handler(&on_open);
    ws_server.listen(8080);
    ws_server.start_accept();

    std::thread wsThread([&]() {
        ws_server.run();
    });

    setupGPIO();

    initializeCAN();

    // Setup SocketCAN on interface "can0".
    int can_socket = setup_can_socket("can0");

    // Setup epoll for CAN message reading.
    int epoll_fd = epoll_create1(0);
    if (epoll_fd < 0) {
        perror("epoll_create1");
        exit(1);
    }
    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = can_socket;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, can_socket, &ev) < 0) {
        perror("epoll_ctl");
        exit(1);
    }

    struct epoll_event events[CAN_RX_QUEUE_SIZE];

    // Main loop: process incoming CAN frames and button inputs.
    while (true) {
        // Wait for CAN events
        int nfds = epoll_wait(epoll_fd, events, CAN_RX_QUEUE_SIZE, CAN_POLL_INTERVAL_MS);
        if (nfds < 0) {
            perror("epoll_wait");
            break;
        }
        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd == can_socket && (events[i].events & EPOLLIN)) {
                struct can_frame frame;
                ssize_t nbytes = read(can_socket, &frame, sizeof(struct can_frame));
                if (nbytes > 0) {
                    // Decode the frame using the proper parser.
                    parseFrame(frame);
                }
            }
        }
        processButtons(can_socket);
    }
    close(can_socket);
    close(epoll_fd);
    ws_server.stop_listening();
    wsThread.join();
    return 0;
}

// WebSocket on_open handler.
void on_open(websocketpp::connection_hdl h) {
    currentConn = h;
    std::cout << "WebSocket connection opened." << std::endl;
}

// Broadcast a message over the WebSocket.
void broadcastMessage(const std::string &msg) {
    try {
        ws_server.send(currentConn, msg, websocketpp::frame::opcode::text);
    } catch (const websocketpp::exception &e) {
        std::cerr << "WebSocket send error: " << e.what() << std::endl;
    }
}

// Setup the SocketCAN socket.
int setup_can_socket(const char *ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket");
        exit(1);
    }
    // Set non-blocking mode.
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        exit(1);
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        exit(1);
    }
    return s;
}

// Setup GPIO using wiringPi.
void setupGPIO() {
    if (wiringPiSetup() == -1) {
        std::cerr << "wiringPi setup failed." << std::endl;
        exit(1);
    }
    // Configure button pins as input.
    pinMode(BUTTON_DRIVE_PIN, INPUT);
    pinMode(BUTTON_NEUTRAL_PIN, INPUT);
    pinMode(BUTTON_REVERSE_PIN, INPUT);

    // Set up internal pull-up resistors for buttons.
    pullUpDnControl(BUTTON_DRIVE_PIN, PUD_UP);
    pullUpDnControl(BUTTON_NEUTRAL_PIN, PUD_UP);
    pullUpDnControl(BUTTON_REVERSE_PIN, PUD_UP);

    // Configure LED pins as output.
    pinMode(LED_DRIVE_PIN, OUTPUT);
    pinMode(LED_NEUTRAL_PIN, OUTPUT);
    pinMode(LED_REVERSE_PIN, OUTPUT);
    pinMode(LED_IMD_PIN, OUTPUT);
    pinMode(LED_BMS_PIN, OUTPUT);

    // Initialize LEDs to on.
    digitalWrite(LED_DRIVE_PIN, HIGH);
    digitalWrite(LED_NEUTRAL_PIN, HIGH);
    digitalWrite(LED_REVERSE_PIN, HIGH);
    digitalWrite(LED_IMD_PIN, HIGH);
    digitalWrite(LED_BMS_PIN, HIGH);

    // Initialize button states.
    auto now = std::chrono::steady_clock::now();
    lastSentTime[BUTTON_DRIVE_PIN] = now;
    lastSentTime[BUTTON_NEUTRAL_PIN] = now;
    lastSentTime[BUTTON_REVERSE_PIN] = now;
    lastButtonState[BUTTON_DRIVE_PIN] = false;
    lastButtonState[BUTTON_NEUTRAL_PIN] = false;
    lastButtonState[BUTTON_REVERSE_PIN] = false;

    pinMode(CAN_NRST_GPIO, OUTPUT);
    pinMode(CAN_STBY_GPIO, OUTPUT);
}

// Process button inputs. When a button is pressed, send a drive mode command.
void processButtons(int can_socket) {
    auto now = std::chrono::steady_clock::now();

    // Read all three buttons.
    bool drive_pressed = (digitalRead(BUTTON_DRIVE_PIN) == LOW);
    bool neutral_pressed = (digitalRead(BUTTON_NEUTRAL_PIN) == LOW);
    bool reverse_pressed = (digitalRead(BUTTON_REVERSE_PIN) == LOW);

    // Update LEDs accordingly.
    digitalWrite(LED_DRIVE_PIN, drive_pressed ? HIGH : LOW);
    digitalWrite(LED_NEUTRAL_PIN, neutral_pressed ? HIGH : LOW);
    digitalWrite(LED_REVERSE_PIN, reverse_pressed ? HIGH : LOW);

    // Determine drive mode with priority: neutral (0) > drive (1) > reverse (2).
    int mode = -1;
    if (neutral_pressed) {
        mode = 0;
    } else if (drive_pressed) {
        mode = 1;
    } else if (reverse_pressed) {
        mode = 2;
    }

    // Static variables to track last sent mode and time.
    static auto last_sent_time = std::chrono::steady_clock::now();

    // Send command if a button has been pressed and it has been long enough since last send
    if (neutral_pressed || drive_pressed || reverse_pressed) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sent_time).count();
        if (duration >= SEND_INTERVAL_MS) {
            sendDriveModeCommand(can_socket, mode);
            last_sent_time = now;
        }
    }
}

// Helper function to send a drive mode command CAN message.
void sendDriveModeCommand(int can_socket, int mode) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_BASE_ID;
    // Set the extended identifier flag if required.
    if (CAN_EXTENDED_ID) {
        frame.can_id |= CAN_EFF_FLAG;
    }
    frame.can_dlc = 1;
    frame.data[0] = static_cast<uint8_t>(mode);

    int nbytes = write(can_socket, &frame, sizeof(frame));
    if (nbytes < 0) {
        perror("write");
    } else {
        std::cout << "Sent drive mode command with state: " << mode << std::endl;
    }
}

// Helper: Convert two bytes to an int16_t from little-endian data.
static inline int16_t toInt16(uint8_t low, uint8_t high) {
    // Combine bytes in little-endian order.
    uint16_t val = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
    return *reinterpret_cast<int16_t *>(&val);
}

// Helper: Convert two bytes to a uint16_t from little-endian data.
static inline uint16_t toUInt16(uint8_t low, uint8_t high) {
    return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

// --- Parser Functions ---

// Extended CAN messages for inverter temperatures.
// Inverter 1: CAN_INVERTER1_BASE + 0 -> left inverter, Inverter 2: CAN_INVERTER2_BASE + 0 -> right inverter.
void parseInverterTemps(const struct can_frame &frame, bool isLeft) {
    if (frame.can_dlc < 6)
        return;
    // For each inverter message, the temperature values are 16-bit (little-endian) and in tenths of a degree.
    float module_A = static_cast<float>(toInt16(frame.data[0], frame.data[1])) / 10.0f;
    float module_B = static_cast<float>(toInt16(frame.data[2], frame.data[3])) / 10.0f;
    float module_C = static_cast<float>(toInt16(frame.data[4], frame.data[5])) / 10.0f;
    float maxTemp = std::max({module_A, module_B, module_C});

    std::stringstream ss;
    if (isLeft)
        ss << "{\"leftinvtemp\": " << maxTemp << "}";
    else
        ss << "{\"rightinvtemp\": " << maxTemp << "}";
}

// Non-extended CAN message: Vehicle state (CAN_BASE_ID + 1)
void parseVehicleState(const struct can_frame &frame) {
    if (frame.can_dlc < 8)
        return;
    uint8_t bms = frame.data[0];
    uint8_t imd = frame.data[1];
    uint8_t drive_state = frame.data[2];
    uint8_t vehicle_state = frame.data[3];
    uint8_t bot = frame.data[4];
    uint8_t brb = frame.data[5];
    uint8_t cvc_overflow = frame.data[6];
    uint8_t cvc_time = frame.data[7];

    std::string drive;
    if (drive_state == 0)
        drive = "NEUTRAL";
    else if (drive_state == 1)
        drive = "DRIVE";
    else if (drive_state == 2)
        drive = "REVERSE";
    else
        drive = "UNKNOWN";

    std::string state;
    switch (vehicle_state) {
        case 0:
            state = "Initial";
            break;
        case 1:
            state = "Voltage Check";
            break;
        case 2:
            state = "Wait for Precharge";
            break;
        case 3:
            state = "Precharge Stage 1";
            break;
        case 4:
            state = "Precharge Stage 2";
            break;
        case 5:
            state = "Precharge Stage 3";
            break;
        case 6:
            state = "Not Ready to Drive";
            break;
        case 7:
            state = "Buzzer";
            break;
        case 8:
            state = "Ready to Drive";
            break;
        case 9:
            state = "Charging";
            break;
        default:
            state = "Unknown";
            break;
    }

    {
        std::stringstream ss;
        ss << "{\"bms\": " << static_cast<int>(bms) << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"imd\": " << static_cast<int>(imd) << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"drive_state\": \"" << drive << "\"}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"vehicle_state\": \"" << state << "\"}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"bot\": " << static_cast<int>(bot) << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"brb\": " << static_cast<int>(brb) << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"cvc_overflow\": " << static_cast<int>(cvc_overflow) << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"cvc_time\": " << static_cast<int>(cvc_time) << "}";
        broadcastMessage(ss.str());
    }
}

// Non-extended CAN message: Driving data (CAN_BASE_ID + 2)
void parseDrivingData(const struct can_frame &frame) {
    if (frame.can_dlc < 4)
        return;
    uint16_t throttle_raw = toUInt16(frame.data[0], frame.data[1]);
    float throttle = throttle_raw / 10.0f;

    uint16_t rpm = toUInt16(frame.data[2], frame.data[3]);
    double speed = rpm * 60.0 * WHEEL_DIAMETER * PI / (TRANSMISSION_RATIO * 12.0 * 5280.0);  // in mph

    {
        std::stringstream ss;
        ss << "{\"throttle\": " << throttle << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"rpm\": " << rpm << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"speed\": " << speed << "}";
        broadcastMessage(ss.str());
    }
}

// Non-extended CAN message: BMS pack voltage (CAN_BMS_BASE + 1)
void parseBmsPackVoltage(const struct can_frame &frame) {
    if (frame.can_dlc < 7)
        return;
    // The ordering: data[5] << 24 | data[6] << 16 | data[3] << 8 | data[4]
    uint32_t raw = (static_cast<uint32_t>(frame.data[5]) << 24) |
                   (static_cast<uint32_t>(frame.data[6]) << 16) |
                   (static_cast<uint32_t>(frame.data[3]) << 8) |
                   (static_cast<uint32_t>(frame.data[4]));
    double voltage = raw / 100.0;

    {
        std::stringstream ss;
        ss << "{\"accumulator_voltage\": " << voltage << "}";
        broadcastMessage(ss.str());
    }
}

// Non-extended CAN message: BMS state of charge (CAN_BMS_BASE + 5)
void parseBmsStateOfCharge(const struct can_frame &frame) {
    if (frame.can_dlc < 7)
        return;
    uint16_t soc_raw = toUInt16(frame.data[5], frame.data[6]);
    double soc = soc_raw / 100.0;

    // Current: first two bytes, interpreted as signed.
    int16_t current = toInt16(frame.data[0], frame.data[1]);
    float current_val = current / 10.0f;

    {
        std::stringstream ss;
        ss << "{\"battery_percentage\": " << soc << "}";
        broadcastMessage(ss.str());
    }
    {
        std::stringstream ss;
        ss << "{\"accumulator_current\": " << current_val << "}";
        broadcastMessage(ss.str());
    }
}

// Non-extended CAN message: BMS cell temperatures (CAN_BMS_BASE + 8)
void parseBmsCellTemperature(const struct can_frame &frame) {
    if (frame.can_dlc < 2)
        return;
    // Temperature: data[1] minus 100, result in Celsius.
    int temp = static_cast<int>(frame.data[1]) - 100;
    {
        std::stringstream ss;
        ss << "{\"acctemp\": " << temp << "}";
        broadcastMessage(ss.str());
    }
}

// Top-level function to parse and handle CAN frame.
void parseFrame(const struct can_frame &frame) {
    // Check if the frame is extended versus standard.
    bool isExtended = frame.can_id & CAN_EFF_FLAG;
    if (isExtended) {
        // Strip off the extended flag.
        unsigned int id = frame.can_id & CAN_EFF_MASK;
        if (id == (CAN_INVERTER1_BASE + 0)) {
            parseInverterTemps(frame, true);
        } else if (id == (CAN_INVERTER2_BASE + 0)) {
            parseInverterTemps(frame, false);
        }
    } else {
        // Standard identifier messages.
        if (frame.can_id == (CAN_BASE_ID + 1)) {
            parseVehicleState(frame);
        } else if (frame.can_id == (CAN_BASE_ID + 2)) {
            parseDrivingData(frame);
        } else if (frame.can_id == (CAN_BMS_BASE + 1)) {
            parseBmsPackVoltage(frame);
        } else if (frame.can_id == (CAN_BMS_BASE + 5)) {
            parseBmsStateOfCharge(frame);
        } else if (frame.can_id == (CAN_BMS_BASE + 8)) {
            parseBmsCellTemperature(frame);
        }
    }
}

void initializeCAN() {
    // Put chip in reset and standby mode.
    digitalWrite(CAN_NRST_GPIO, LOW);
    digitalWrite(CAN_STBY_GPIO, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait for chip to reset

    // Pull chip out of reset.
    digitalWrite(CAN_NRST_GPIO, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Bring up the CAN interface via system calls.
    // Note: SocketCAN typically needs the interface to be up. Depending on your system setup,
    // you might manage these commands externally; here we call them from the program.
    system("sudo ip link set can0 up type can bitrate 500000 restart-ms 20");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    system("sudo ifconfig can0 txqueuelen 65536");

    // Pull chip out of standby mode.
    digitalWrite(CAN_STBY_GPIO, LOW);
}
