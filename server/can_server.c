#include <libwebsockets.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

static int callback_can(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED:
            lwsl_user("Client connected\n");
            break;
        case LWS_CALLBACK_SERVER_WRITEABLE: {
            char buf[LWS_PRE + 128];
            char *msg = &buf[LWS_PRE];

            // Simulate CAN data
            int speed = rand() % 200;         // km/h
            int battery = rand() % 101;       // %
            int throttle = rand() % 100;      // %

            snprintf(msg, 128, "{\"speed\": %d, \"battery\": %d, \"throttle\": %d}", speed, battery, throttle);

            size_t n = strlen(msg);
            lws_write(wsi, (unsigned char *)msg, n, LWS_WRITE_TEXT);

            // Schedule another write
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

int main(void) {
    srand(time(NULL));

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof info);

    info.port = 9000;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;

    struct lws_context *context = lws_create_context(&info);
    if (!context) {
        lwsl_err("lws init failed\n");
        return -1;
    }

    lwsl_user("Starting WebSocket server on port 9000...\n");

    // Periodically force all clients to be writable
    while (1) {
        lws_callback_on_writable_all_protocol(context, &protocols[0]);
        lws_service(context, 0);
        usleep(2000000);  // 100ms delay
    }

    lws_context_destroy(context);
    return 0;
}
