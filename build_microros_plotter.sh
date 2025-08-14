#!/bin/bash

# Microros Plotter Build Script
# Usage: ./build_microros_plotter.sh [flash|monitor|flash-monitor]

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
AGENT_IP="192.168.1.100"  # Change to your PC's IP
AGENT_PORT="8888"
WIFI_SSID="YOUR_WIFI_SSID"  # Change to your WiFi SSID
WIFI_PASS="YOUR_WIFI_PASS"  # Change to your WiFi password
SERIAL_PORT="/dev/ttyUSB0"  # Change if needed

echo -e "${GREEN}=== Microros Plotter Build Script ===${NC}"

# Check if we're in ros2_ws directory
if [[ ! "$PWD" == *"ros2_ws"* ]]; then
    echo -e "${RED}Error: Please run this script from ros2_ws directory${NC}"
    exit 1
fi

# Source ROS2 environment
echo -e "${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Function to configure firmware
configure_firmware() {
    echo -e "${YELLOW}Configuring firmware for WiFi/UDP transport...${NC}"
    
    # Create menuconfig parameters file
    cat > /tmp/microros_plotter_config.txt << EOF
CONFIG_ESP_WIFI_SSID="${WIFI_SSID}"
CONFIG_ESP_WIFI_PASSWORD="${WIFI_PASS}"
CONFIG_MICRO_ROS_AGENT_IP="${AGENT_IP}"
CONFIG_MICRO_ROS_AGENT_PORT="${AGENT_PORT}"
CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE=y
CONFIG_MICRO_ROS_ESP_NETIF_WLAN=y
EOF
    
    ros2 run micro_ros_setup configure_firmware.sh microros_plotter \
        --transport udp \
        --ip ${AGENT_IP} \
        --port ${AGENT_PORT}
    
    echo -e "${GREEN}Configuration complete!${NC}"
    echo -e "${YELLOW}Note: Configure WiFi credentials via menuconfig${NC}"
}

# Function to build firmware
build_firmware() {
    echo -e "${YELLOW}Building firmware...${NC}"
    ros2 run micro_ros_setup build_firmware.sh
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Build successful!${NC}"
    else
        echo -e "${RED}Build failed!${NC}"
        exit 1
    fi
}

# Function to flash firmware
flash_firmware() {
    echo -e "${YELLOW}Flashing firmware to ${SERIAL_PORT}...${NC}"
    ros2 run micro_ros_setup flash_firmware.sh -p ${SERIAL_PORT}
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Flash successful!${NC}"
    else
        echo -e "${RED}Flash failed! Check serial port.${NC}"
        exit 1
    fi
}

# Function to monitor serial output
monitor_firmware() {
    echo -e "${YELLOW}Starting serial monitor on ${SERIAL_PORT}...${NC}"
    echo -e "${GREEN}Press Ctrl+] to exit monitor${NC}"
    
    # Use screen or minicom for monitoring
    if command -v screen &> /dev/null; then
        screen ${SERIAL_PORT} 115200
    elif command -v minicom &> /dev/null; then
        minicom -D ${SERIAL_PORT} -b 115200
    else
        echo -e "${YELLOW}Using Python serial monitor...${NC}"
        python3 -m serial.tools.miniterm ${SERIAL_PORT} 115200
    fi
}

# Main script logic
case "$1" in
    configure)
        configure_firmware
        ;;
    build)
        build_firmware
        ;;
    flash)
        flash_firmware
        ;;
    monitor)
        monitor_firmware
        ;;
    flash-monitor)
        flash_firmware
        sleep 2
        monitor_firmware
        ;;
    all)
        configure_firmware
        build_firmware
        flash_firmware
        sleep 2
        monitor_firmware
        ;;
    clean)
        echo -e "${YELLOW}Cleaning build...${NC}"
        rm -rf firmware/build
        rm -rf firmware/install
        rm -rf firmware/log
        echo -e "${GREEN}Clean complete!${NC}"
        ;;
    *)
        echo "Usage: $0 {configure|build|flash|monitor|flash-monitor|all|clean}"
        echo ""
        echo "  configure     - Configure firmware for WiFi/UDP"
        echo "  build        - Build the firmware"
        echo "  flash        - Flash firmware to ESP32"
        echo "  monitor      - Monitor serial output"
        echo "  flash-monitor - Flash and then monitor"
        echo "  all          - Configure, build, flash and monitor"
        echo "  clean        - Clean build artifacts"
        echo ""
        echo "Current settings:"
        echo "  Agent IP: ${AGENT_IP}:${AGENT_PORT}"
        echo "  WiFi SSID: ${WIFI_SSID}"
        echo "  Serial Port: ${SERIAL_PORT}"
        exit 1
        ;;
esac

echo -e "${GREEN}Done!${NC}"