#!/usr/bin/env bash
set -eo pipefail   # без -u: setup.bash может ссылаться на unset-переменные

# --- настройки ---
WS="${WS:-$HOME/ros2_ws}"
APP="omnirevolve_esp32_microros"               # имя каталога в firmware/freertos_apps/apps/
FW_DIR="$WS/firmware"
APP_DIR="$FW_DIR/freertos_apps/apps/$APP"

# транспорт micro-ROS
AGENT_IP="192.168.1.23"
AGENT_PORT="8888"
SERIAL_PORT="/dev/ttyUSB0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
  echo "Usage: $0 {prepare|configure|build|flash|monitor|clean}"
  exit 1
}

# безопасный source (временно отключаем -u, если он включён где-то выше)
safe_source() { set +u; source "$1"; set -u 2>/dev/null || true; }

need_app_dir() {
  if [[ ! -d "$APP_DIR" ]]; then
    echo -e "${RED}ERROR:${NC} app directory not found: $APP_DIR"
    echo "Ожидается структура: $FW_DIR/freertos_apps/apps/$APP"
    exit 2
  fi
}

# Пути IDF-компонентов, которые должны попасть в сборку
EXTRA_COMPONENT_DIRS_VALUE="${APP_DIR}/omnirevolve_esp32_microros/components/micro_ros_espidf_component;${APP_DIR}/omnirevolve_esp32_microros/components/u8g2"

# 1) окружение ROS 2
if [[ -f "$WS/install/setup.bash" ]]; then
  safe_source "$WS/install/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  safe_source "/opt/ros/humble/setup.bash"
fi

cmd="${1:-}"; [[ -z "$cmd" ]] && usage

monitor_firmware() {
  echo -e "${GREEN}Starting serial monitor on ${SERIAL_PORT}...${NC}"
  echo -e "${YELLOW}Press Ctrl+] to exit monitor${NC}"

  # 1) Пытаемся использовать IDF monitor (правильно управляет DTR/RTS)
  if [[ -f "$FW_DIR/esp-idf/export.sh" ]]; then
    # экспортируем среду ESP-IDF (как делает micro_ros_setup)
    # shellcheck disable=SC1090
    source "$FW_DIR/esp-idf/export.sh"
  elif [[ -f "$FW_DIR/third_party/esp-idf/export.sh" ]]; then
    # на некоторых конфигурациях IDF лежит так
    # shellcheck disable=SC1090
    source "$FW_DIR/third_party/esp-idf/export.sh"
  fi

  if command -v idf.py >/dev/null 2>&1; then
    EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
    idf.py -p "${SERIAL_PORT}" -b 115200 monitor && return
  fi

  # 2) Fallback: дёрнуть DTR/RTS через esptool и открыть miniterm/screen
  if command -v esptool.py >/dev/null 2>&1; then
    # Быстрый «double reset»: esptool сам выставит линии правильно
    esptool.py --chip auto --port "${SERIAL_PORT}" --baud 115200 \
      --before default_reset --after hard_reset chip_id >/dev/null 2>&1 || true
  fi

  if command -v screen &> /dev/null; then
    screen "${SERIAL_PORT}" 115200
  elif command -v minicom &> /dev/null; then
    minicom -D "${SERIAL_PORT}" -b 115200
  else
    # у miniterm есть ключи --dtr/--rts, но не во всех версиях
    python3 -m serial.tools.miniterm "${SERIAL_PORT}" 115200
  fi
}

case "$cmd" in
  prepare)
    mkdir -p "$FW_DIR"
    pushd "$WS" >/dev/null
      ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
    popd >/dev/null
    echo "[ok] firmware ws prepared at $FW_DIR"
    ;;

  configure)
    need_app_dir
    pushd "$WS" >/dev/null
      # твоя версия micro_ros_setup принимает имя app первым аргументом
      ros2 run micro_ros_setup configure_firmware.sh "$APP" -t udp -i "$AGENT_IP" -p "$AGENT_PORT"
    popd >/dev/null
    echo "[ok] configured (APP=$APP, AGENT=$AGENT_IP:$AGENT_PORT)"
    ;;

  build)
    need_app_dir
    pushd "$WS" >/dev/null
      echo -e "${YELLOW}EXTRA_COMPONENT_DIRS=${EXTRA_COMPONENT_DIRS_VALUE}${NC}"
      EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
      ros2 run micro_ros_setup build_firmware.sh
    popd >/dev/null
    echo -e "${GREEN}[ok] build done${NC}"
    ;;

  flash)
    pushd "$WS" >/dev/null
      echo -e "${YELLOW}EXTRA_COMPONENT_DIRS=${EXTRA_COMPONENT_DIRS_VALUE}${NC}"
      EXTRA_COMPONENT_DIRS="$EXTRA_COMPONENT_DIRS_VALUE" \
      ros2 run micro_ros_setup flash_firmware.sh
    popd >/dev/null
    ;;

  monitor)
    need_app_dir
    pushd "$WS" >/dev/null
      monitor_firmware
    popd >/dev/null
    ;;
    
  clean)
    echo -e "${YELLOW}Cleaning build...${NC}"
    rm -rf "$FW_DIR/build" "$FW_DIR/install" "$FW_DIR/log"
    echo -e "${GREEN}Clean complete!${NC}"
    ;;

  *)
    usage
    ;;
esac
