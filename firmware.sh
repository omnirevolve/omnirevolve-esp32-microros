#!/usr/bin/env bash
# micro-ROS ESP32 firmware helper
# Commands: prepare | configure | build | flash | monitor | menuconfig | clean

set -eo pipefail   # not using -u: some setup.bash read unset vars

# -------------------- defaults --------------------
WS="${WS:-$HOME/ros2_ws}"
APP="omnirevolve_esp32_microros"                               # app dir under firmware/freertos_apps/apps/
FW_DIR="$WS/firmware"
APP_DIR="$FW_DIR/freertos_apps/apps/$APP"
FW_EXT="$FW_DIR/freertos_apps/microros_esp32_extensions"

# host ROS 2 workspace messages pkg (source) and where to symlink it inside mcu_ws
LINK_SRC="$WS/src/omnirevolve_ros2_messages"
LINK_DST="$FW_DIR/mcu_ws/ros2/omnirevolve_ros2_messages"       # micro-ROS mcu_ws location

# micro-ROS transport / serial defaults
AGENT_IP="${AGENT_IP:-192.168.1.23}"
AGENT_PORT="${AGENT_PORT:-8888}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"

# extra component dirs for ESP-IDF build (resolved inside firmware ws)
EXTRA_COMPONENT_DIRS_VALUE="${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/omnirevolve_core;${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/omnirevolve_protocol;${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/u8g2"

# colors
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

# -------------------- usage --------------------
usage() {
  cat <<EOF
Usage:
  $(basename "$0") [global options] <command>

Commands:
  prepare       Create micro-ROS firmware workspace
  configure     Configure the app transport (uses agent ip/port)
  build         Build firmware
  flash         Flash firmware
  monitor       Open serial monitor
  menuconfig    Open ESP-IDF menuconfig
  clean         Remove firmware build artifacts

Global options:
  --ws PATH             ROS 2 workspace path (default: ${WS})
  --agent-ip IP         micro-ROS agent IP (default: ${AGENT_IP})
  --agent-port PORT     micro-ROS agent port (default: ${AGENT_PORT})
  --serial PATH         Serial port (default: ${SERIAL_PORT})
  -h, --help            Show this help

Examples:
  $(basename "$0") --agent-ip 10.0.0.5 configure
  $(basename "$0") --ws ~/ros2_ws --serial /dev/ttyUSB1 build
  $(basename "$0") monitor
EOF
}

# tolerant source for setup files
safe_source() {
  [ -f "$1" ] || { echo -e "${RED}ERROR:${NC} file not found: $1"; exit 1; }
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  # shellcheck disable=SC1090
  source "$1"
}

need_app_dir() {
  if [[ ! -d "$APP_DIR" ]]; then
    echo -e "${RED}ERROR:${NC} app directory not found: $APP_DIR"
    echo "Expected: $FW_DIR/freertos_apps/apps/$APP"
    exit 2
  fi
}

ensure_messages_link() {
  if [[ ! -d "$LINK_SRC" ]]; then
    echo -e "${RED}ERROR:${NC} messages repo not found: $LINK_SRC"
    echo "Expected omnirevolve_ros2_messages in $WS/src/"
    exit 3
  fi
  mkdir -p "$(dirname "$LINK_DST")"
  if [[ -e "$LINK_DST" && ! -L "$LINK_DST" ]]; then
    echo -e "${YELLOW}WARN:${NC} $LINK_DST exists and is not a symlink — removing it"
    rm -rf "$LINK_DST"
  fi
  if [[ ! -L "$LINK_DST" ]]; then
    ln -s "$LINK_SRC" "$LINK_DST"
    echo -e "${GREEN}[ok]${NC} linked $LINK_SRC -> $LINK_DST"
  fi

  local META="$APP_DIR/app-colcon.meta"
  if [[ -f "$META" ]] && ! grep -q '"omnirevolve_ros2_messages"' "$META"; then
    echo -e "${YELLOW}NOTE:${NC} consider adding \"omnirevolve_ros2_messages\" to $META (names section)."
  fi
}

ensure_micro_ros_setup() {
  safe_source "/opt/ros/humble/setup.bash"
  if ! ros2 pkg list | grep -qx "micro_ros_setup"; then
    if [[ -d "$WS/src/micro_ros_setup" ]]; then
      echo -e "${YELLOW}micro_ros_setup not found in environment; building it...${NC}"
      pushd "$WS" >/dev/null
        colcon build --packages-select micro_ros_setup --symlink-install \
          --event-handlers console_direct+ status-
      popd >/dev/null
      safe_source "$WS/install/setup.bash"
    else
      echo -e "${RED}ERROR:${NC} micro_ros_setup source not found at $WS/src/micro_ros_setup"
      echo "Clone it first: git -C \"$WS/src\" clone --recursive https://github.com/micro-ROS/micro_ros_setup.git"
      exit 5
    fi
  fi
}

monitor_firmware() {
  echo -e "${GREEN}Starting serial monitor on ${SERIAL_PORT}...${NC}"
  echo -e "${YELLOW}Press Ctrl+] to exit${NC}"

  if [[ -f "$FW_DIR/esp-idf/export.sh" ]]; then
    # shellcheck disable=SC1090
    source "$FW_DIR/esp-idf/export.sh"
  elif [[ -f "$FW_DIR/third_party/esp-idf/export.sh" ]]; then
    # shellcheck disable=SC1090
    source "$FW_DIR/third_party/esp-idf/export.sh"
  fi

  if command -v idf.py >/dev/null 2>&1; then
    EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
    idf.py -p "${SERIAL_PORT}" -b 115200 monitor && return
  fi

  if command -v esptool.py >/dev/null 2>&1; then
    esptool.py --chip auto --port "${SERIAL_PORT}" --baud 115200 \
      --before default_reset --after hard_reset chip_id >/dev/null 2>&1 || true
  fi

  if command -v screen &> /dev/null; then
    screen "${SERIAL_PORT}" 115200
  elif command -v minicom &> /dev/null; then
    minicom -D "${SERIAL_PORT}" -b 115200
  else
    python3 -m serial.tools.miniterm "${SERIAL_PORT}" 115200
  fi
}

# -------------------- parse global options + command --------------------
cmd=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)             WS="$2"; shift 2;;
    --agent-ip)       AGENT_IP="$2"; shift 2;;
    --agent-port)     AGENT_PORT="$2"; shift 2;;
    --serial|--serial-port) SERIAL_PORT="$2"; shift 2;;
    -h|--help)        usage; exit 0;;
    prepare|configure|build|flash|monitor|menuconfig|clean)
      cmd="$1"; shift; break;;
    *)
      echo "Unknown option or command: $1"; usage; exit 1;;
  esac
done

[[ -z "$cmd" ]] && { usage; exit 1; }

# recompute paths that depend on WS after flags
FW_DIR="$WS/firmware"
APP_DIR="$FW_DIR/freertos_apps/apps/$APP"
FW_EXT="$FW_DIR/freertos_apps/microros_esp32_extensions"
LINK_SRC="$WS/src/omnirevolve_ros2_messages"
LINK_DST="$FW_DIR/mcu_ws/ros2/omnirevolve_ros2_messages"

# -------------------- commands --------------------
case "$cmd" in
  prepare)
    mkdir -p "$FW_DIR"
    ensure_micro_ros_setup
    pushd "$WS" >/dev/null
      ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
    popd >/dev/null
    echo -e "${GREEN}[ok]${NC} firmware workspace prepared at $FW_DIR"
    ;;

  configure)
    need_app_dir
    ensure_messages_link
    ensure_micro_ros_setup
    pushd "$WS" >/dev/null
      ros2 run micro_ros_setup configure_firmware.sh "$APP" -t udp -i "$AGENT_IP" -p "$AGENT_PORT"
    popd >/dev/null
    echo -e "${GREEN}[ok]${NC} configured (APP=$APP, AGENT=${AGENT_IP}:${AGENT_PORT})"
    ;;

  build)
    need_app_dir
    ensure_messages_link
    ensure_micro_ros_setup

    EXTRA_INCLUDES="-I${APP_DIR}/components/omnirevolve_esp32_core/components/omnirevolve_protocol/include"
    EXTRA_INCLUDES="${EXTRA_INCLUDES} -I${APP_DIR}/components/omnirevolve_esp32_core/components/omnirevolve_core/include"
    EXTRA_INCLUDES="${EXTRA_INCLUDES} -I${APP_DIR}/components/omnirevolve_esp32_core/components/u8g2/include"

    pushd "$WS" >/dev/null
      echo -e "${YELLOW}EXTRA_COMPONENT_DIRS=${EXTRA_COMPONENT_DIRS_VALUE}${NC}"
      echo -e "${YELLOW}EXTRA_INCLUDES=${EXTRA_INCLUDES}${NC}"
      EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
      CFLAGS="${EXTRA_INCLUDES}" \
      CXXFLAGS="${EXTRA_INCLUDES}" \
      ros2 run micro_ros_setup build_firmware.sh
    popd >/dev/null
    echo -e "${GREEN}[ok]${NC} build finished"
    ;;

  flash)
    need_app_dir
    ensure_micro_ros_setup
    pushd "$WS" >/dev/null
      echo -e "${YELLOW}EXTRA_COMPONENT_DIRS=${EXTRA_COMPONENT_DIRS_VALUE}${NC}"
      EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
      ros2 run micro_ros_setup flash_firmware.sh
    popd >/dev/null
    ;;

  monitor)
    need_app_dir
    monitor_firmware
    ;;

  menuconfig)
    need_app_dir
    ensure_micro_ros_setup
    pushd "$WS" >/dev/null
      echo -e "${YELLOW}Opening menuconfig...${NC}"
      echo -e "${YELLOW}EXTRA_COMPONENT_DIRS=${EXTRA_COMPONENT_DIRS_VALUE}${NC}"
      EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
      ros2 run micro_ros_setup build_firmware.sh menuconfig
    popd >/dev/null
    ;;

  clean)
    echo -e "${YELLOW}Cleaning firmware build artifacts...${NC}"
    rm -rf "$FW_DIR/build" "$FW_DIR/install" "$FW_DIR/log"
    echo -e "${GREEN}Clean complete${NC}"
    ;;

  *)
    usage
    ;;
esac
