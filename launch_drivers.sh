#!/bin/bash
# -----------------------------------------
# Driver & Recorder Manager for ROS2 + GUI (Python virtual env)
# -----------------------------------------

cd "$(dirname "$0")"
CONFIG_FILE="config.yaml"

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
# Setup experiment
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

launch_driver() {
    local cmd="$1"
    local name="$2"
    log "Launching driver: $name"
    echo -e "${YELLOW}→ Launching ${BOLD}$name${RESET}${YELLOW} in a new terminal tab...${RESET}"
    gnome-terminal --tab --title "$name" -- bash -c "$cmd; exec bash"
}

# -----------------------------------------
# Recording management
# -----------------------------------------
recorder_pids=()
record_dirs=()

launch_recording() {
    local base_name="$1"
    shift
    local topics=("$@")

    mkdir -p "bags"
    cd "bags" || exit 1

    local ts=$(date +%Y%m%d_%H%M%S)
    local bag_name="${base_name}_${ts}"

    echo -e "\033[93m→ Starting ros2 bag recording: ${bag_name}\033[0m"
    log "Starting ros2 bag recording: $bag_name, topics: ${topics[*]}"

    # Detect regex topics
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
        ros2 bag record --storage mcap -o "$bag_name" --regex "${regex_topics[@]}" &
    else
        ros2 bag record --storage mcap -o "$bag_name" "${normal_topics[@]}" &
    fi

    local pid=$!

    cd - >/dev/null || exit 1

    recorder_pids+=($pid)
    record_dirs+=("bags/$bag_name")

    echo -e "\033[92m✔ Recording started: $bag_name (PID: $pid)\033[0m"
    log "Recording started: $bag_name (PID: $pid)"
}


# -----------------------------------------
# GUI launcher
# -----------------------------------------
gui_pid=""
launch_gui() {
    echo -e "${YELLOW}→ Launching Sensor Dashboard GUI...${RESET}"
    log "Launching Sensor Dashboard GUI"
    gnome-terminal --tab --title "Sensor Dashboard" -- bash -c "python3 main.py; exec bash" &
    gui_pid=$!
}

# -----------------------------------------
# Cleanup logic
# -----------------------------------------
cleanup() {
    echo -e "${RED}\n[!] Stopping all recordings...${RESET}"
    log "Stopping all recordings"

    # Stop and save all recordings
    for i in "${!recorder_pids[@]}"; do
        pid=${recorder_pids[$i]}
        out_dir=${record_dirs[$i]}

        if kill -0 "$pid" 2>/dev/null; then
            echo -e "${GRAY}⏹ Sending SIGINT to recorder PID $pid${RESET}"
            log "Sending SIGINT to recorder PID $pid (dir=$out_dir)"
            kill -SIGINT "$pid"
            wait "$pid" 2>/dev/null
            echo -e "${GREEN}✔ Recorder PID $pid terminated and saved.${RESET}"
            log "Recorder PID $pid terminated safely"
        fi

        # Verify MCAP bag
        if [[ -d "$out_dir" ]]; then
            mcap_file=$(find "$out_dir" -maxdepth 1 -name "*.mcap" | head -n1)
            if [[ -f "$mcap_file" ]]; then
                if ros2 bag info "$mcap_file" >> "$out_dir/verify.log" 2>&1; then
                    echo -e "${GREEN}✔ Bag verified: $mcap_file${RESET}"
                    log "Bag verified successfully: $mcap_file"
                else
                    echo -e "${RED}✘ Bag verification failed: $mcap_file${RESET}"
                    log "Bag verification FAILED: $mcap_file"
                fi
            else
                echo -e "${RED}✘ No MCAP file found in $out_dir${RESET}"
                log "No MCAP file found in $out_dir"
            fi
        fi
    done

    # Close GUI if running
    if [[ -n "$gui_pid" ]]; then
        echo -e "${RED}→ Closing Sensor Dashboard GUI...${RESET}"
        kill "$gui_pid" 2>/dev/null
        log "Sensor Dashboard GUI closed"
    fi

    # Copy bags to destination
    read -p "Enter destination directory to copy all bags: " DEST
    log "User chose destination: $DEST"

    if [[ -n "$DEST" ]]; then
        mkdir -p "$DEST"
        echo -e "${YELLOW}→ Copying all bag directories to $DEST...${RESET}"
        log "Copying all bag directories: ${record_dirs[*]} → $DEST"
        rsync -avh --progress "${record_dirs[@]}" "$DEST"

        if [[ $? -eq 0 ]]; then
            echo -e "${GREEN}✔ All bags copied successfully to $DEST${RESET}"
            log "All bags copied successfully to $DEST"

            # Ask first confirmation for deletion
            read -p "Delete original bag directories? (y/n): " del1
            log "User first delete choice: $del1"

            if [[ "$del1" =~ ^[Yy]$ ]]; then
                # Ask second confirmation
                read -p "Are you absolutely sure? Type 'YES' to confirm: " del2
                log "User second delete confirmation: $del2"
                if [[ "$del2" == "YES" ]]; then
                    rm -rf "${record_dirs[@]}"
                    echo -e "${RED}✘ Deleted original bag directories${RESET}"
                    log "Deleted original bag directories"
                else
                    echo -e "${GRAY}⏩ Skipped deletion of original bag directories${RESET}"
                    log "Skipped deletion of original bag directories"
                fi
            else
                echo -e "${GRAY}⏩ Skipped deletion of original bag directories${RESET}"
                log "Skipped deletion of original bag directories"
            fi
        else
            echo -e "${RED}✘ Copy failed${RESET}"
            log "Copy failed"
        fi
    fi
}
trap cleanup EXIT

# -----------------------------------------
# Activate Python virtual environment for GUI
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
# Ask if user wants to launch all sensors at once
# -----------------------------------------
print_header
read -p "Do you want to launch ALL sensors automatically? (y/n): " launch_all_choice
log "User chose to launch all sensors automatically: $launch_all_choice"
launch_all=false
if [[ "$launch_all_choice" =~ ^[Yy]$ ]]; then
    launch_all=true
fi

# -----------------------------------------
# Track skipped sensors
# -----------------------------------------
declare -A skipped_sensors

# -----------------------------------------
# Launch drivers
# -----------------------------------------
declare -A launched
groups=$(yq e '.sensors | keys | .[]' "$CONFIG_FILE")

for group in $groups; do
    sensor_count=$(yq e ".sensors.$group | length" "$CONFIG_FILE")
    for ((i=0; i<sensor_count; i++)); do
        launch_ref=$(yq e ".sensors.$group[$i].launch" "$CONFIG_FILE")

        # Skip if previously skipped
        if [[ "${skipped_sensors[$launch_ref]}" == "1" ]]; then
            log "Skipping previously skipped sensor: $launch_ref"
            continue
        fi

        # Skip if already launched
        if [[ "${launched[$launch_ref]}" == "1" ]]; then
            continue
        fi

        cmd=$(yq e ".launch_files.$launch_ref.command" "$CONFIG_FILE")
        if [[ "$cmd" == "null" ]]; then
            echo -e "${RED}[!] Command not found for $launch_ref${RESET}"
            log "Command not found for $launch_ref"
            continue
        fi

        print_header
        log "Preparing to launch: $launch_ref"

        if [[ "$launch_all" == false ]]; then
            read -p "Launch $launch_ref? (y/n): " choice
            log "User chose: $choice"
            if [[ ! "$choice" =~ ^[Yy]$ ]]; then
                skipped_sensors[$launch_ref]=1
                echo -e "${GRAY}⏩ Skipped '$launch_ref'${RESET}"
                log "Skipped driver $launch_ref"
                continue
            fi
        fi

        launch_driver "$cmd" "$launch_ref"
        launched["$launch_ref"]=1
        echo -e "${GREEN}✔ Driver launched successfully.${RESET}"
        log "Driver $launch_ref launched successfully"
        sleep 1
    done
done

# -----------------------------------------
# Extra launch files
# -----------------------------------------
extra_count=$(yq e ".launch_files.extra_launch_files | length" "$CONFIG_FILE")
for ((i=0; i<extra_count; i++)); do
    cmd=$(yq e ".launch_files.extra_launch_files[$i].command" "$CONFIG_FILE")

    print_header
    log "Preparing to launch extra: $cmd"

    if [[ "$launch_all" == false ]]; then
        read -p "Launch extra driver? (y/n): " choice
        log "User chose: $choice"
        if [[ ! "$choice" =~ ^[Yy]$ ]]; then
            echo -e "${GRAY}⏩ Skipped extra '$cmd'${RESET}"
            log "Skipped extra driver: $cmd"
            continue
        fi
    fi

    launch_driver "$cmd" "extra_$i"
    echo -e "${GREEN}✔ Extra driver launched successfully.${RESET}"
    log "Extra driver launched successfully: $cmd"
    sleep 1
done

# -----------------------------------------
# Start recordings
# -----------------------------------------
partition_count=$(yq e ".record_partitions | length" "$CONFIG_FILE")
for ((i=0; i<partition_count; i++)); do
    group_name=$(yq e ".record_partitions | keys | .[$i]" "$CONFIG_FILE")
    mapfile -t topics < <(yq e ".record_partitions.$group_name.topics[]" "$CONFIG_FILE")

    print_header
    log "Preparing to record group: $group_name with topics: ${topics[*]}"
    echo -e "📦 Preparing to record: ${BOLD}${WHITE}$group_name${RESET}"
    echo -e "   Topics: ${topics[*]}"
    echo

    if [[ "$launch_all" == false ]]; then
        read -p "Proceed with recording? (y/n): " choice
        log "User chose: $choice"
        if [[ ! "$choice" =~ ^[Yy]$ ]]; then
            echo -e "${GRAY}⏩ Skipped recording '$group_name'${RESET}"
            log "Skipped recording $group_name"
            continue
        fi
    fi

    launch_recording "$group_name" "${topics[@]}"
    sleep 1
done

# -----------------------------------------
# Launch GUI after recordings
# -----------------------------------------
if [[ "$launch_all" == false ]]; then
    read -p "Do you want to launch the Sensor Dashboard GUI? (y/n): " gui_choice
    log "User chose to launch GUI: $gui_choice"
    if [[ "$gui_choice" =~ ^[Yy]$ ]]; then
        launch_gui
    fi
else
    launch_gui
fi

# -----------------------------------------
# Wait for user before cleanup
# -----------------------------------------
while true; do
    echo -e "${YELLOW}System running. Press 'S' to stop all recordings and exit...${RESET}"
    read -n1 -s key
    echo
    if [[ "$key" =~ [Ss] ]]; then
        echo -e "${RED}Are you sure you want to stop all recordings? (Y/N, Enter/Esc to cancel)${RESET}"
        read -n1 -s confirm
        echo
        log "User pressed stop key, confirm=$confirm"

        if [[ "$confirm" =~ [Yy]$ ]]; then
            echo -e "${GREEN}✔ Stopping all recordings...${RESET}"
            log "Stopping all recordings confirmed"
            break
        else
            echo -e "${GRAY}⏩ Stop request cancelled.${RESET}"
            log "Stop cancelled"
        fi
    fi
done

