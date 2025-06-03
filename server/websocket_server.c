// #include <libwebsockets.h>
// #include <string.h>
// #include <stdlib.h>
// #include <time.h>
// #include "canbus.h"
// extern vehicle_state_t vehicle_state;


// static int callback_can(struct lws *wsi, enum lws_callback_reasons reason,
//                         void *user, void *in, size_t len) {
//     switch (reason) {
//         case LWS_CALLBACK_ESTABLISHED:
//             lwsl_user("Client connected\n");
//             break;
//         case LWS_CALLBACK_SERVER_WRITEABLE: {
//             char buf[LWS_PRE + 128];
//             char *msg = &buf[LWS_PRE];

//             // Simulate CAN data
//             int speed = rand() % 200;         // km/h
//             int battery = rand() % 101;       // %
//             int throttle = rand() % 100;      // %

//         snprintf(msg, 128,
//          "{\"speed\": %.1f, \"battery\": %.1f, \"throttle\": %.1f, \"rpm\": %d, \"voltage\": %.2f, \"current\": %.1f}",
//          vehicle_state.speed,
//          vehicle_state.battery_percentage,
//          vehicle_state.throttle_position,
//          vehicle_state.rpm,
//          vehicle_state.accumulator_voltage,
//          vehicle_state.accumulator_current);

//             size_t n = strlen(msg);
//             lws_write(wsi, (unsigned char *)msg, n, LWS_WRITE_TEXT);

//             // Schedule another write
//             lws_callback_on_writable(wsi);
//             break;
//         }
//         default:
//             break;
//     }
//     return 0;
// }

// static struct lws_protocols protocols[] = {
//     {
//         .name = "can-protocol",
//         .callback = callback_can,
//         .per_session_data_size = 0,
//         .rx_buffer_size = 0,
//     },
//     { NULL, NULL, 0, 0 }
// };

// int main(void) {
//     srand(time(NULL));

//     struct lws_context_creation_info info;
//     memset(&info, 0, sizeof info);

//     info.port = 9000;
//     info.protocols = protocols;
//     info.gid = -1;
//     info.uid = -1;

//     struct lws_context *context = lws_create_context(&info);
//     if (!context) {
//         lwsl_err("lws init failed\n");
//         return -1;
//     }

//     lwsl_user("Starting WebSocket server on port 9000...\n");

//     // Periodically force all clients to be writable
//     while (1) {
//         lws_callback_on_writable_all_protocol(context, &protocols[0]);
//         lws_service(context, 0);
//         usleep(2000000);  // 100ms delay
//     }

//     lws_context_destroy(context);
//     return 0;
// }


#include <libwebsockets.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

// Expose your global vehicle_state
#include "canbus.h"
extern vehicle_state_t vehicle_state;
extern struct shared_state_t* shared_state;


static int callback_can(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED:
            lwsl_user("Client connected\n");
            break;

        case LWS_CALLBACK_SERVER_WRITEABLE: {
            char buf[LWS_PRE + 512];
            char *msg = &buf[LWS_PRE];

            // Serialize vehicle_state into JSON
            snprintf(msg, 512,
                "{"
                "\"speed\": %.2f, "
                "\"battery\": %.2f, "
                "\"throttle\": %.1f, "
                "\"rpm\": %d, "
                "\"voltage\": %.2f, "
                "\"current\": %.1f, "
                "\"mileage\": %.3f, "
                "\"drive_state\": \"%s\", "
                "\"vehicle_state\": \"%s\", "
                "\"leftinvtemp\": %.1f, "
                "\"rightinvtemp\": %.1f, "
                "\"acctemp\": %.1f"
                "}",
                vehicle_state.speed,
                vehicle_state.battery_percentage,
                vehicle_state.throttle_position,
                vehicle_state.rpm,
                vehicle_state.accumulator_voltage,
                vehicle_state.accumulator_current,
                vehicle_state.mileage,
                vehicle_state.drive_state,
                vehicle_state.vehicle_state,
                vehicle_state.leftinvtemp,
                vehicle_state.rightinvtemp,
                vehicle_state.acctemp
            );

            size_t n = strlen(msg);
            lws_write(wsi, (unsigned char *)msg, n, LWS_WRITE_TEXT);

            // Trigger another send
            lws_callback_on_writable(wsi);
            break;
        }

        default:
            break;
    }
    return 0;
}

static struct lws_protocols protocols[] = {
    {
        .name = "can-protocol",
        .callback = callback_can,
        .per_session_data_size = 0,
        .rx_buffer_size = 0,
    },
    { NULL, NULL, 0, 0 }
};

// Thread that runs the WebSocket server
void* websocket_server(void* arg) {
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof info);

    info.port = 9000;  // WebSocket port
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;

    struct lws_context *context = lws_create_context(&info);
    if (!context) {
        lwsl_err("lws init failed\n");
        return NULL;
    }

    lwsl_user("WebSocket server running on port 9000...\n");

    while (shared_state->running) {
        lws_callback_on_writable_all_protocol(context, &protocols[0]);
        lws_service(context, 100);  // 100 ms
        usleep(50000);             // 100 ms sleep
    }

    lws_context_destroy(context);
    return NULL;
}
