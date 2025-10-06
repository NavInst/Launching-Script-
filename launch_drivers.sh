#!/bin/bash
# -----------------------------------------
# Driver & Recorder Manager for ROS2 + GUI (Python virtual env)
# -----------------------------------------
set -o pipefail
cd "$(dirname "$0")"

CONFIG_FILE="config.yaml"
LAUNCH_DELAY=3
BAG_RECORDING_DELAY=3
STORAGE_PATH="/media/navinst/Lex_Data2"

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
EXPERIMENT_TIMESTAMP=$(date +%Y_%m_%d-%H_%M_%S)

# Define experiment folder and make the folder
EXP_DIR="${EXP_NAME}_${EXPERIMENT_TIMESTAMP}"
mkdir -p "$STORAGE_PATH/bags/$EXP_DIR"

LOG_DIR="$STORAGE_PATH/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/${EXP_DIR}.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" >> "$LOG_FILE"
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

    # Replace placeholders
    cmd="${cmd//<bag_experiment>/$STORAGE_PATH/bags/$EXP_DIR}"
    cmd="${cmd//<timestamp>/$EXPERIMENT_TIMESTAMP}"

    log "Preparing driver: $name"
    echo -e "${YELLOW}→ Preparing ${BOLD}$name${RESET}${YELLOW} to launch in a tab...${RESET}"

    local sanitized_name
    sanitized_name=$(echo "$name" | sed 's/[^a-zA-Z0-9_-]/_/g')
    local tmp_log="/tmp/${sanitized_name}_driver.log"

    gnome-terminal --tab --title="$name" -- bash -c "$cmd 2>&1 | tee $tmp_log" &
    local term_pid=$!
    driver_terminal_pid_map["$name"]=$term_pid
    sleep "$LAUNCH_DELAY"

    local pid=""
    local i
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
        local pgid
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
    local bag_name="${EXP_DIR}/${partition_name}"
    local bag_dir="$STORAGE_PATH/bags/$bag_name"
    mkdir -p "$(dirname "$bag_dir")"

    echo -e "${YELLOW}→ Starting ros2 bag recording: $bag_dir${RESET}"
    log "Starting ros2 bag recording: $bag_dir, topics: ${topics[*]}"

    regex_topics=()
    normal_topics=()
    local t
    for t in "${topics[@]}"; do
        if [[ "$t" == *".*"* ]]; then
            regex_topics+=("$t")
        else
            normal_topics+=("$t")
        fi
    done

    if [[ ${#regex_topics[@]} -gt 0 ]]; then
        rec_cmd="ros2 bag record --storage sqlite3 -o \"$bag_dir\" --regex ${regex_topics[@]}"
    else
        rec_cmd="ros2 bag record --storage sqlite3 -o \"$bag_dir\" ${normal_topics[*]}"
    fi

    gnome-terminal --tab --title="Recording $partition_name" \
        -- bash -c "exec setsid $rec_cmd" &
    local term_pid=$!
    recorder_terminal_pid_map+=($term_pid)
    record_dirs+=("$bag_dir")

    sleep 1

    local pid
    pid=$(pgrep -f "ros2 bag record.*$bag_name" | head -n1 || true)
    if [[ -n "$pid" ]]; then
        local pgid
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        recorder_pids+=($pid)
        recorder_pgids+=($pgid)
        echo -e "${GREEN}✓ Recording started: $bag_dir (PID: $pid, PGID: $pgid)${RESET}"
        log "Recording started: $bag_dir (PID: $pid, PGID: $pgid)"
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
    sleep "$LAUNCH_DELAY"
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
    if ! kill -0 "$pid" 2>/dev/null; then return; fi

    local target
    if [[ -n "$pgid" ]]; then
        target="-$pgid"
        log "Sending SIGINT to PGID $pgid ($friendly)"
    else
        target="$pid"
        log "Sending SIGINT to PID $pid ($friendly)"
    fi

    kill -SIGINT "$target" 2>/dev/null || true

    local i
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
# Cleanup logic with confirmation
# -----------------------------------------
cleanup() {
    echo -e "${RED}\n[!] Attempting to stop all recordings and drivers...${RESET}"
    read -p "Are you sure you want to kill all processes? (y/n): " kill_choice
    if [[ ! "$kill_choice" =~ ^[Yy]$ ]]; then
        echo -e "${CYAN}Cleanup cancelled. Processes are still running.${RESET}"
        return
    fi

    log "User confirmed: Stopping all recordings and drivers"

    local idx

    for idx in "${!recorder_pids[@]}"; do
        pid=${recorder_pids[$idx]}
        pgid=${recorder_pgids[$idx]}
        term_pid=${recorder_terminal_pid_map[$idx]}
        [[ -n "$pid" ]] && graceful_kill "recorder-$idx" "$pid" "$pgid"
        [[ -n "$term_pid" ]] && kill "$term_pid" 2>/dev/null || true
    done

    local name

    for name in "${!driver_pid_map[@]}"; do
        pid="${driver_pid_map[$name]}"
        pgid="${driver_pgid_map[$name]}"
        term_pid="${driver_terminal_pid_map[$name]}"
        [[ -n "$pid" ]] && graceful_kill "$name" "$pid" "$pgid"
        [[ -n "$term_pid" ]] && kill "$term_pid" 2>/dev/null || true
    done

    [[ -n "$gui_pid" ]] && graceful_kill "Sensor GUI" "$gui_pid" ""
}

trap cleanup SIGINT SIGTERM EXIT

# -----------------------------------------
# Launch all drivers - FIXED LOGIC
# -----------------------------------------
print_header
# read -p "Do you want to launch ALL sensors automatically? (y/n): " launch_all_choice
launch_all=false
# [[ "$launch_all_choice" =~ ^[Yy]$ ]] && launch_all=true

declare -A skipped_sensors
declare -A launched_sensors
declare -A launched_launch_files

groups=$(yq e '.sensors | keys | .[]' "$CONFIG_FILE")

for group in $groups; do
    sensor_count=$(yq e ".sensors.$group | length" "$CONFIG_FILE")
    echo "DEBUG: Processing group '$group' with $sensor_count sensors" | tee -a "$LOG_FILE"

    local j
    for ((j=0; j<sensor_count; j++)); do
        echo "DEBUG: ===== Processing sensor index $j in group $group =====" | tee -a "$LOG_FILE"

        sensor_name=$(yq e ".sensors.$group[$j].name" "$CONFIG_FILE" 2>/dev/null)
        launch_ref=$(yq e ".sensors.$group[$j].launch" "$CONFIG_FILE" 2>/dev/null)

        [[ -z "$sensor_name" || "$sensor_name" == "null" ]] && continue
        [[ -z "$launch_ref" || "$launch_ref" == "null" ]] && continue

        sensor_id="${group}_${j}_${sensor_name}"

        [[ "${skipped_sensors[$sensor_id]}" == "1" ]] && continue
        [[ "${launched_sensors[$sensor_id]}" == "1" ]] && continue

        cmd=$(yq e ".launch_files.$launch_ref.command" "$CONFIG_FILE" 2>/dev/null)
        [[ "$cmd" == "null" || -z "$cmd" ]] && continue

        print_header
        if [[ "$launch_all" == false ]]; then
            [[ "${launched_launch_files[$launch_ref]}" == "1" ]] && {
                echo -e "${CYAN}→ $sensor_name ($launch_ref) already running, skipping...${RESET}"
                launched_sensors["$sensor_id"]=1
                continue
            }

            read -p "Launch $sensor_name ($launch_ref)? (y/n): " choice
            if [[ ! "$choice" =~ ^[Yy]$ ]]; then
                skipped_sensors[$sensor_id]=1
                continue
            fi

            # ✅ Show pre-launch comment (if any)
            pre_comment=$(yq e ".sensors.$group[$j].pre_comment" "$CONFIG_FILE" 2>/dev/null)
            if [[ "$pre_comment" != "null" && -n "$pre_comment" ]]; then
                echo
                echo -e "${CYAN}${BOLD}────────────────────────────────────────────────────${RESET}"
                echo -e "${CYAN}${BOLD}→ Pre-launch note for $sensor_name:${RESET}"
                echo -e "   ${WHITE}$pre_comment${RESET}"
                echo -e "${CYAN}${BOLD}────────────────────────────────────────────────────${RESET}"
                echo
                read -p "Press Enter to continue with the launch..." _
                echo
            fi
        fi

        [[ "${launched_launch_files[$launch_ref]}" == "1" ]] && {
            launched_sensors["$sensor_id"]=1
            continue
        }

        # ✅ Launch the driver
        launch_driver "$cmd" "${sensor_name}_${launch_ref}"
        launched_sensors["$sensor_id"]=1
        launched_launch_files["$launch_ref"]=1
        sleep 0.5

        # ✅ Show post-launch comment (if any)
        post_comment=$(yq e ".sensors.$group[$j].post_comment" "$CONFIG_FILE" 2>/dev/null)
        if [[ "$post_comment" != "null" && -n "$post_comment" ]]; then
            echo
            echo -e "${GREEN}${BOLD}────────────────────────────────────────────────────${RESET}"
            echo -e "${GREEN}${BOLD}→ Post-launch note for $sensor_name:${RESET}"
            echo -e "   ${WHITE}$post_comment${RESET}"
            echo -e "${GREEN}${BOLD}────────────────────────────────────────────────────${RESET}"
            echo
            sleep 3
        fi

        # ✅ Pause before next launch
        echo
        read -p "Press Enter to proceed to the next launch..." _
        echo
    done
done


print_header
read -p "Do you want to launch GUI? (y/n): " gui_launch_choice

if [[ "$gui_launch_choice" =~ ^[Yy]$ ]]; then
    echo -e "${CYAN}→ Launching GUI...${RESET}"
    launch_gui
fi

print_header
read -p "Do you want to start bag recording? (y/n): " record_choice

if [[ "$record_choice" =~ ^[Yy]$ ]]; then
    partitions=$(yq e '.record_partitions | keys | .[]' "$CONFIG_FILE")
    for part in $partitions; do
        topics=$(yq e ".record_partitions.$part.topics[]" "$CONFIG_FILE")
        topic_array=()
        while IFS= read -r t; do
            [[ -n "$t" ]] && topic_array+=("$t")
        done <<< "$topics"
        launch_recording "$part" "${topic_array[@]}"
    done
    echo -e "${GREEN}System running with recording. Press 'S' then Enter to stop...${RESET}"
else
    echo -e "${GREEN}System running without recording. Press 'S' then Enter to stop...${RESET}"
fi

while true; do
    read -r -n1 key
    [[ "$key" =~ [Ss] ]] && break
done

echo -e "${GREEN}All processes terminated.${RESET}"
