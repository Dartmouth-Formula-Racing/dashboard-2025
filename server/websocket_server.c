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
                "\"accumulator_voltage\": %.2f, "
                "\"accumulator_current\": %.1f, "
                "\"mileage\": %.3f, "
                "\"drive_state\": \"%s\", "
                "\"vehicle_state\": \"%s\", "
                "\"leftinvtemp\": %.1f, "
                "\"rightinvtemp\": %.1f, "
                "\"canconnected\": %d, "
                "\"imd\": %d, "
                "\"cvc_overflow\": %d, "
                "\"cvc_time\": %d, "
                "\"lap\": %d, "
                "\"lap_time\": %.2f, "
                "\"estimated_range\": %.2f, "
                "\"traction_control\": %d, "
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
                vehicle_state.can_connected,
                vehicle_state.imd,
                vehicle_state.cvc_overflow,
                vehicle_state.cvc_time,
                vehicle_state.lap,
                vehicle_state.laptime,
                vehicle_state.estimated_range,
                vehicle_state.traction_control,
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
        usleep(20000);             // 100 ms sleep
    }

    lws_context_destroy(context);
    return NULL;
}
