#!/bin/bash
# -----------------------------------------
# Driver & Recorder Manager for ROS2 + GUI (Python virtual env)
# -----------------------------------------
set -o pipefail
cd "$(dirname "$0")"
CONFIG_FILE="config.yaml"

LAUNCH_DELAY=3   # seconds to wait after launching a driver
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
    echo -e "${WHITE}${BOLD}      Driver & Sensor Launch + Recording Manager     ${RESET}"
    echo -e "${CYAN}${BOLD}====================================================${RESET}"
    echo
}

# -----------------------------------------
# Driver launching (one tab per driver, sequential)
# -----------------------------------------
declare -a driver_tabs_cmds
declare -a driver_tab_titles
declare -A driver_pid_map
declare -A driver_terminal_pid_map

launch_driver() {
    local cmd="$1"
    local name="$2"
    cmd="${cmd//<bag_experiment>/bags_${EXP_NAME}}"

    log "Preparing driver: $name"
    echo -e "${YELLOW}→ Preparing ${BOLD}$name${RESET}${YELLOW} to launch in a tab...${RESET}"

    pid_file="/tmp/${name}_pid.txt"
    full_cmd="$cmd & echo \$! > $pid_file; wait"

    driver_tabs_cmds+=("$full_cmd")
    driver_tab_titles+=("$name")
}

launch_all_driver_tabs() {
    if [[ ${#driver_tabs_cmds[@]} -eq 0 ]]; then
        echo "No drivers to launch."
        return
    fi

    echo -e "${CYAN}→ Launching drivers sequentially in GNOME Terminal tabs...${RESET}"
    for i in "${!driver_tabs_cmds[@]}"; do
        name="${driver_tab_titles[$i]}"
        cmd="${driver_tabs_cmds[$i]}"
        tab_exec="$cmd; echo; echo 'Driver $name finished (tab will remain open)'; exec bash"
        gnome-terminal --tab --title="$name" -- bash -c "$tab_exec" &
        term_pid=$!
        driver_terminal_pid_map["$name"]=$term_pid

        pid_file="/tmp/${name}_pid.txt"
        local wait_count=0
        while [[ ! -s "$pid_file" ]]; do
            sleep 0.2
            wait_count=$((wait_count+1))
            if (( wait_count > 150 )); then
                log "Timed out waiting for PID file for $name; fallback"
                break
            fi
        done

        if [[ -s "$pid_file" ]]; then
            pid=$(cat "$pid_file")
            rm -f "$pid_file"
        else
            pid=$(pgrep -f "$name" | head -n1 || true)
        fi

        if [[ -n "$pid" ]]; then
            driver_pid_map["$name"]=$pid
            echo -e "    • $name → PID $pid"
            log "Driver $name PID tracked: $pid"
        else
            echo -e "${RED}✘ Could not determine PID for $name${RESET}"
            log "Failed to determine PID for $name"
        fi

        echo -e "${GRAY}Waiting ${LAUNCH_DELAY}s after launching $name...${RESET}"
        sleep "$LAUNCH_DELAY"
    done

    echo -e "${GREEN}✔ All drivers launched.${RESET}"
}

# -----------------------------------------
# Recording management
# -----------------------------------------
recorder_pids=()
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

    # Construct command
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

    # Launch recording in its own terminal tab
    gnome-terminal --tab --title="Recording $partition_name" -- bash -c "$rec_cmd; exec bash" &
    term_pid=$!
    recorder_terminal_pid_map+=($term_pid)
    record_dirs+=("bags/$bag_name")

    # Try to get PID of ros2 bag
    sleep 1
    pid=$(pgrep -f "ros2 bag record.*$bag_name" | head -n1 || true)
    recorder_pids+=($pid)

    echo -e "${GREEN}✔ Recording started: bags/$bag_name (PID: $pid, TERM PID: $term_pid)${RESET}"
    log "Recording started: bags/$bag_name (PID: $pid, TERM PID: $term_pid)"
    sleep "$BAG_RECORDING_DELAY"
}

# -----------------------------------------
# GUI launcher
# -----------------------------------------
gui_pid=""
launch_gui() {
    echo -e "${YELLOW}→ Launching Sensor Dashboard GUI...${RESET}"
    log "Launching Sensor Dashboard GUI"
    gnome-terminal --tab --title "Sensor Dashboard" -- bash -c "python3 main.py; exec bash" &
    sleep 0.5
    gui_pid=$(pgrep -f "python3 main.py" | head -n1 || true)
    log "GUI PID (approx): $gui_pid"
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
        term_pid=${recorder_terminal_pid_map[$idx]}
        out_dir=${record_dirs[$idx]}
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            log "Stopping recorder PID $pid"
            kill -SIGINT "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
        if [[ -n "$term_pid" ]] && kill -0 "$term_pid" 2>/dev/null; then
            log "Closing terminal tab for recording (PID $term_pid)"
            kill "$term_pid" 2>/dev/null || true
            wait "$term_pid" 2>/dev/null || true
        fi
    done

    # Stop drivers
    for name in "${!driver_pid_map[@]}"; do
        pid="${driver_pid_map[$name]}"
        term_pid="${driver_terminal_pid_map[$name]}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            log "Terminating driver $name (PID $pid)"
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
        if [[ -n "$term_pid" ]] && kill -0 "$term_pid" 2>/dev/null; then
            log "Closing terminal tab for driver $name (PID $term_pid)"
            kill "$term_pid" 2>/dev/null || true
            wait "$term_pid" 2>/dev/null || true
        fi
    done

    # Stop GUI
    if [[ -n "$gui_pid" ]] && kill -0 "$gui_pid" 2>/dev/null; then
        log "Closing Sensor Dashboard GUI"
        kill "$gui_pid" 2>/dev/null || true
    fi

    # Optional: Copy bags to user-specified location
    read -p "Do you want to copy the bags to another destination? (y/n): " copy_choice
    if [[ "$copy_choice" =~ ^[Yy]$ ]]; then
        read -p "Enter destination path: " dest_path
        mkdir -p "$dest_path"
        cp -r "bags/$EXP_NAME" "$dest_path"
        echo -e "${GREEN}✔ Bags copied to $dest_path${RESET}"
        log "Bags copied to $dest_path"

        # Verify copy
        diff -r "bags/$EXP_NAME" "$dest_path/$EXP_NAME" >/dev/null 2>&1
        if [[ $? -eq 0 ]]; then
            echo -e "${GREEN}✔ Copy verification successful${RESET}"
        else
            echo -e "${RED}✘ Copy verification failed${RESET}"
        fi

        read -p "Do you want to delete the original bags? (y/n): " del_choice
        if [[ "$del_choice" =~ ^[Yy]$ ]]; then
            rm -rf "bags/$EXP_NAME"
            echo -e "${YELLOW}✔ Original bags deleted${RESET}"
        fi
    fi
}
trap cleanup EXIT

# -----------------------------------------
# Python venv activation
# -----------------------------------------
VENV_PATH="/home/ahmed/Desktop/Launching-Script-/LaunchScript"
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

# Extra launch files (if any)
extra_count=$(yq e ".launch_files.extra_launch_files | length" "$CONFIG_FILE")
for ((i=0; i<extra_count; i++)); do
    cmd=$(yq e ".launch_files.extra_launch_files[$i].command" "$CONFIG_FILE")
    if [[ "$launch_all" == false ]]; then
        read -p "Launch extra driver? (y/n): " choice
        [[ ! "$choice" =~ ^[Yy]$ ]] && continue
    fi
    launch_driver "$cmd" "extra_$i"
done

launch_all_driver_tabs

# Launch GUI
launch_gui

# Start bag recordings for each partition
partitions=$(yq e '.record_partitions | keys | .[]' "$CONFIG_FILE")
for part in $partitions; do
    topics=$(yq e ".record_partitions.$part.topics[]" "$CONFIG_FILE")
    topic_array=()
    while IFS= read -r t; do
        [[ -n "$t" ]] && topic_array+=("$t")
    done <<< "$topics"

    launch_recording "$part" "${topic_array[@]}"
done

# Wait for 'S' key to stop everything
echo -e "${GREEN}System running. Press 'S' then Enter to stop all recordings and exit...${RESET}"
while true; do
    read -r -n1 key
    [[ "$key" =~ [Ss] ]] && break
done

cleanup
echo -e "${GREEN}All processes terminated.${RESET}"

