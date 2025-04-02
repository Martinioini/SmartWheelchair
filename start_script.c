#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#define BUTTON_ID 10
#define START_CMD "roslaunch wheelchair_control real_wheelchair.launch &"
#define KILL_CMD "pkill -f 'roslaunch wheelchair_control'"
#define CAN_UP_CMD "sudo ip link set can0 type can bitrate 125000 && sudo ip link set up can0"
#define CAN_DOWN_CMD "sudo ip link set down can0"

int node_running = 0;  // Flag to track node state

void toggle_ros_node() {
    if (node_running == 0) {
        // First bring down the CAN interface to ensure clean state
        printf("Bringing down CAN interface...\n");
        system(CAN_DOWN_CMD);
        sleep(1);  // Wait for interface to go down
        
        printf("Setting up CAN interface...\n");
        system(CAN_UP_CMD);
        sleep(2);  // Give CAN interface time to initialize
        
        printf("Starting wheelchair control...\n");
        system(START_CMD);
        sleep(3);  // Give ROS nodes time to start
        node_running = 1;
    } else {
        printf("Stopping wheelchair control...\n");
        system(KILL_CMD);
        sleep(2);  // Give ROS nodes time to shut down
        
        printf("Bringing down CAN interface...\n");
        system(CAN_DOWN_CMD);
        sleep(1);  // Wait for interface to go down
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
    
    // Set non-blocking mode for joystick
    fcntl(fd, F_SETFL, O_NONBLOCK);
    
    struct js_event event;
    while (1) {
        // Read joystick events
        while (read(fd, &event, sizeof(event)) > 0) {
            if (event.type == JS_EVENT_BUTTON && event.number == BUTTON_ID && event.value == 1) {
                printf("Button %d pressed - toggling wheelchair control\n", BUTTON_ID);
                toggle_ros_node();
            }
        }
        usleep(50000);  // Small delay
    }

    close(fd);
    return 0;
}
