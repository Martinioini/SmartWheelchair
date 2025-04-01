#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#define BUTTON_ID 8
#define START_CMD "roslaunch wheelchair_control real_wheelchair.launch &"
#define KILL_CMD "pkill -f 'roslaunch wheelchair_control'"
#define CAN_UP_CMD "sudo ip link set can0 type can bitrate 125000 && sudo ip link set up can0"
#define CAN_DOWN_CMD "sudo ip link set down can0"

int node_running = 0;  // Flag to track node state

void toggle_ros_node() {
    if (node_running == 0) {
        printf("Bringing down CAN interface...\n");
        system(CAN_DOWN_CMD);

        printf("Setting up CAN interface...\n");
        system(CAN_UP_CMD);
        
        printf("Starting wheelchair control...\n");
        system(START_CMD);
        node_running = 1;
    } else {
        printf("Stopping wheelchair control...\n");
        system(KILL_CMD);
        node_running = 0;
    }
}

int main() {
    int fd = open("/dev/input/js0", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open joystick");
        return 1;
    }

    printf("Controller started. Press button %d to toggle wheelchair.\n", BUTTON_ID);
    
    struct js_event event;
    while (1) {
        if (read(fd, &event, sizeof(event)) > 0) {
            if (event.type == JS_EVENT_BUTTON && event.number == BUTTON_ID && event.value == 1) {
                toggle_ros_node();
            }
        }
        usleep(50000);  // Small delay
    }

    close(fd);
    return 0;
}
