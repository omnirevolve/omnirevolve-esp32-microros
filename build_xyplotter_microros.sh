#!/usr/bin/env bash
set -eo pipefail   # без -u: setup.bash может ссылаться на unset-переменные

# --- настройки ---
WS="${WS:-$HOME/ros2_ws}"
APP="omnirevolve_esp32_microros"               # имя каталога в firmware/freertos_apps/apps/
FW_DIR="$WS/firmware"
APP_DIR="$FW_DIR/freertos_apps/apps/$APP"
FW_EXT="$FW_DIR/freertos_apps/microros_esp32_extensions"

# где лежит пакет сообщений в host ROS ws и куда его линкуем в mcu_ws
LINK_SRC="$WS/src/omnirevolve_ros2_messages"
# ВАЖНО: правильное место — mcu_ws/ros2
LINK_DST="$FW_DIR/mcu_ws/ros2/omnirevolve_ros2_messages"

# на всякий случай подчистим старый неверный путь (создавался раньше)
OLD_WRONG_LINK="$FW_EXT/mcu_ws/src/omnirevolve_ros2_messages"

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

# Гарантируем, что пакет сообщений виден в mcu_ws/ros2 через симлинк
ensure_messages_link() {
  if [[ ! -d "$LINK_SRC" ]]; then
    echo -e "${RED}ERROR:${NC} messages repo not found at: $LINK_SRC"
    echo "Ожидается omnirevolve_ros2_messages в $WS/src/"
    exit 3
  fi

  mkdir -p "$(dirname "$LINK_DST")"
  if [[ -e "$LINK_DST" && ! -L "$LINK_DST" ]]; then
    echo -e "${YELLOW}WARN:${NC} $LINK_DST существует и это не симлинк — удаляю"
    rm -rf "$LINK_DST"
  fi
  if [[ ! -L "$LINK_DST" ]]; then
    ln -s "$LINK_SRC" "$LINK_DST"
    echo -e "${GREEN}[ok]${NC} linked $LINK_SRC -> $LINK_DST"
  fi

  # Подсказка про app-colcon.meta (не редактируем автоматически)
  local META="$APP_DIR/app-colcon.meta"
  if [[ -f "$META" ]] && ! grep -q '"omnirevolve_ros2_messages"' "$META"; then
    echo -e "${YELLOW}NOTE:${NC} при необходимости добавь \"omnirevolve_ros2_messages\" в $META (секция names)."
  fi
}

EXTRA_COMPONENT_DIRS_VALUE="${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/omnirevolve_core;${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/omnirevolve_protocol;${APP_DIR}/omnirevolve_esp32_microros/components/omnirevolve_esp32_core/components/u8g2"

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

  # 2) Fallback: дёрнуть DTR/RTS через esptool и открыть miniterm/screen
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
    ensure_messages_link
    pushd "$WS" >/dev/null
      ros2 run micro_ros_setup configure_firmware.sh "$APP" -t udp -i "$AGENT_IP" -p "$AGENT_PORT"
    popd >/dev/null
    echo "[ok] configured (APP=$APP, AGENT=$AGENT_IP:$AGENT_PORT)"
    ;;

  build)
    need_app_dir
    ensure_messages_link

    # Формируем дополнительные пути включения
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
