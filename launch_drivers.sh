#!/bin/bash
# -----------------------------------------
# Driver & Recorder Manager for ROS2 + GUI (Python virtual env)
# -----------------------------------------
set -o pipefail
cd "$(dirname "$0")"
CONFIG_FILE="config.yaml"
LAUNCH_DELAY=3
BAG_RECORDING_DELAY=3

# -----------------------------------------
# Colors
# -----------------------------------------
RESET="\e[0m"
BOLD="\e[1m"
WHITE="\e[97m"
CYAN="\e[96m"
YELLOW="\e[93m"
GREEN="\e[92m"
RED="\e[91m"
GRAY="\e[90m"

# -----------------------------------------
# Setup experiment + logging
# -----------------------------------------
read -p "Enter experiment name: " EXP_NAME
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/${EXP_NAME}_$(date +%Y%m%d_%H%M%S).log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

print_header() {
    clear
    echo -e "${CYAN}${BOLD}====================================================${RESET}"
    echo -e "${WHITE}${BOLD} Driver & Sensor Launch + Recording Manager ${RESET}"
    echo -e "${CYAN}${BOLD}====================================================${RESET}"
    echo
}

# -----------------------------------------
# Driver launching (parse ROS2 PID)
# -----------------------------------------
declare -a driver_tabs_cmds
declare -a driver_tab_titles
declare -A driver_pid_map
declare -A driver_terminal_pid_map
declare -A driver_pgid_map

launch_driver() {
    local cmd="$1"
    local name="$2"
    cmd="${cmd//<bag_experiment>/bags_${EXP_NAME}}"
    log "Preparing driver: $name"
    echo -e "${YELLOW}→ Preparing ${BOLD}$name${RESET}${YELLOW} to launch in a tab...${RESET}"

    tmp_log="/tmp/${name}_driver.log"

    gnome-terminal --tab --title="$name" -- bash -c "$cmd 2>&1 | tee $tmp_log" &
    term_pid=$!
    driver_terminal_pid_map["$name"]=$term_pid

    sleep "$LAUNCH_DELAY"
    # Wait for ROS2 PID to appear in log
    pid=""
    for i in {1..50}; do
        pid_candidate=$(grep -oP '\[\d{5}\]' "$tmp_log" | head -n1 | tr -d '[]')
        if [[ -n "$pid_candidate" ]]; then
            pid="$pid_candidate"
            break
        fi
        sleep 0.2
    done

    if [[ -n "$pid" ]]; then
        driver_pid_map["$name"]=$pid
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        driver_pgid_map["$name"]=$pgid
        log "Driver $name PID tracked from ROS2 logs: $pid (PGID $pgid)"
    else
        log "Could not find PID for $name from ROS2 logs"
    fi

    rm -f "$tmp_log"
}

# -----------------------------------------
# Recording management
# -----------------------------------------
recorder_pids=()
recorder_pgids=()
recorder_terminal_pid_map=()
record_dirs=()

launch_recording() {
    local partition_name="$1"
    shift
    local topics=("$@")

    mkdir -p "bags"
    ts=$(date +%Y%m%d_%H%M%S)
    local bag_name="${EXP_NAME}/${partition_name}_${ts}"
    mkdir -p "$(dirname "bags/$bag_name")"

    echo -e "${YELLOW}→ Starting ros2 bag recording: bags/$bag_name${RESET}"
    log "Starting ros2 bag recording: bags/$bag_name, topics: ${topics[*]}"

    regex_topics=()
    normal_topics=()
    for t in "${topics[@]}"; do
        if [[ "$t" == *".*"* ]]; then
            regex_topics+=("$t")
        else
            normal_topics+=("$t")
        fi
    done

    if [[ ${#regex_topics[@]} -gt 0 ]]; then
        rec_cmd="ros2 bag record --storage mcap -o \"bags/$bag_name\" --regex ${regex_topics[@]}"
    else
        rec_cmd="ros2 bag record --storage mcap -o \"bags/$bag_name\" ${normal_topics[*]}"
    fi

    gnome-terminal --tab --title="Recording $partition_name" \
        -- bash -c "exec setsid $rec_cmd" &

    term_pid=$!
    recorder_terminal_pid_map+=($term_pid)
    record_dirs+=("bags/$bag_name")
    sleep 1

    pid=$(pgrep -f "ros2 bag record.*$bag_name" | head -n1 || true)
    if [[ -n "$pid" ]]; then
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        recorder_pids+=($pid)
        recorder_pgids+=($pgid)
        echo -e "${GREEN}✔ Recording started: bags/$bag_name (PID: $pid, PGID: $pgid)${RESET}"
        log "Recording started: bags/$bag_name (PID: $pid, PGID: $pgid)"
    else
        recorder_pids+=("")
        recorder_pgids+=("")
        log "⚠ Could not track recorder PID for $bag_name"
    fi

    sleep "$BAG_RECORDING_DELAY"
}

# -----------------------------------------
# GUI launcher
# -----------------------------------------
gui_pid=""
launch_gui() {
    echo -e "${YELLOW}→ Launching Sensor Dashboard GUI...${RESET}"
    log "Launching Sensor Dashboard GUI"
    gnome-terminal --tab --title "Sensor Dashboard" -- bash -c "python3 main.py" &
    sleep 0.5
    gui_pid=$(pgrep -f "python3 main.py" | head -n1 || true)
    log "GUI PID (approx): $gui_pid"
}

# -----------------------------------------
# Graceful kill helper
# -----------------------------------------
graceful_kill() {
    local name="$1"
    local pid="$2"
    local pgid="$3"
    local friendly="${4:-$name}"

    [[ -z "$pid" ]] && return
    if ! kill -0 "$pid" 2>/dev/null; then
        return
    fi

    if [[ -n "$pgid" ]]; then
        target="-$pgid"
        log "Sending SIGINT to PGID $pgid ($friendly)"
    else
        target="$pid"
        log "Sending SIGINT to PID $pid ($friendly)"
    fi

    kill -SIGINT "$target" 2>/dev/null || true
    for i in {1..25}; do
        ! kill -0 "$pid" 2>/dev/null && return
        sleep 0.2
    done

    log "$friendly still alive; sending SIGTERM"
    kill -SIGTERM "$target" 2>/dev/null || true
    for i in {1..15}; do
        ! kill -0 "$pid" 2>/dev/null && return
        sleep 0.2
    done

    log "$friendly stubborn; sending SIGKILL"
    kill -SIGKILL "$target" 2>/dev/null || true
}

# -----------------------------------------
# Cleanup logic
# -----------------------------------------
cleanup() {
    echo -e "${RED}\n[!] Stopping all recordings and drivers...${RESET}"
    log "Stopping all recordings and drivers"

    # Stop recordings
    for idx in "${!recorder_pids[@]}"; do
        pid=${recorder_pids[$idx]}
        pgid=${recorder_pgids[$idx]}
        term_pid=${recorder_terminal_pid_map[$idx]}
        [[ -n "$pid" ]] && graceful_kill "recorder-$idx" "$pid" "$pgid"
        [[ -n "$term_pid" ]] && kill "$term_pid" 2>/dev/null || true
    done

    # Stop drivers
    for name in "${!driver_pid_map[@]}"; do
        pid="${driver_pid_map[$name]}"
        pgid="${driver_pgid_map[$name]}"
        term_pid="${driver_terminal_pid_map[$name]}"
        [[ -n "$pid" ]] && graceful_kill "$name" "$pid" "$pgid"
        [[ -n "$term_pid" ]] && kill "$term_pid" 2>/dev/null || true
    done

    # Stop GUI
    [[ -n "$gui_pid" ]] && graceful_kill "Sensor GUI" "$gui_pid" ""

    # Optional: copy bags
    read -p "Do you want to copy the bags to another destination? (y/n): " copy_choice
    if [[ "$copy_choice" =~ ^[Yy]$ ]]; then
        read -p "Enter destination path: " dest_path
        mkdir -p "$dest_path"
        cp -r "bags/$EXP_NAME" "$dest_path"
        echo -e "${GREEN}✔ Bags copied to $dest_path${RESET}"
    fi
}
trap cleanup SIGINT SIGTERM EXIT

# -----------------------------------------
# Python venv activation
# -----------------------------------------
VENV_PATH="/home/navinst/Desktop/Launching-Script--main/LaunchScript"
if [[ -f "$VENV_PATH/bin/activate" ]]; then
    log "Activating Python virtual environment at $VENV_PATH"
    source "$VENV_PATH/bin/activate"
else
    echo -e "${RED}❌ Virtual environment not found at $VENV_PATH${RESET}"
    log "Virtual environment not found at $VENV_PATH"
fi

# -----------------------------------------
# Launch all drivers
# -----------------------------------------
print_header
read -p "Do you want to launch ALL sensors automatically? (y/n): " launch_all_choice
launch_all=false
[[ "$launch_all_choice" =~ ^[Yy]$ ]] && launch_all=true
declare -A skipped_sensors
declare -A launched

groups=$(yq e '.sensors | keys | .[]' "$CONFIG_FILE")
for group in $groups; do
    sensor_count=$(yq e ".sensors.$group | length" "$CONFIG_FILE")
    for ((i=0; i<sensor_count; i++)); do
        launch_ref=$(yq e ".sensors.$group[$i].launch" "$CONFIG_FILE")
        [[ "${skipped_sensors[$launch_ref]}" == "1" ]] && continue
        [[ "${launched[$launch_ref]}" == "1" ]] && continue
        cmd=$(yq e ".launch_files.$launch_ref.command" "$CONFIG_FILE")
        [[ "$cmd" == "null" ]] && continue

        print_header
        if [[ "$launch_all" == false ]]; then
            read -p "Launch $launch_ref? (y/n): " choice
            [[ ! "$choice" =~ ^[Yy]$ ]] && skipped_sensors[$launch_ref]=1 && continue
        fi
        launch_driver "$cmd" "$launch_ref"
        launched["$launch_ref"]=1
        sleep 0.25
    done
done

extra_count=$(yq e ".launch_files.extra_launch_files | length" "$CONFIG_FILE")
for ((i=0; i<extra_count; i++)); do
    cmd=$(yq e ".launch_files.extra_launch_files[$i].command" "$CONFIG_FILE")
    if [[ "$launch_all" == false ]]; then
        read -p "Launch extra driver? (y/n): " choice
        [[ ! "$choice" =~ ^[Yy]$ ]] && continue
    fi
    launch_driver "$cmd" "extra_$i"
done

# Launch drivers, GUI, and recordings
echo -e "${CYAN}→ Launching all drivers...${RESET}"
launch_gui

# Start bag recordings
partitions=$(yq e '.record_partitions | keys | .[]' "$CONFIG_FILE")
for part in $partitions; do
    topics=$(yq e ".record_partitions.$part.topics[]" "$CONFIG_FILE")
    topic_array=()
    while IFS= read -r t; do
        [[ -n "$t" ]] && topic_array+=("$t")
    done <<< "$topics"
    launch_recording "$part" "${topic_array[@]}"
done

echo -e "${GREEN}System running. Press 'S' then Enter to stop...${RESET}"
while true; do
    read -r -n1 key
    [[ "$key" =~ [Ss] ]] && break
done

cleanup
echo -e "${GREEN}All processes terminated.${RESET}"

