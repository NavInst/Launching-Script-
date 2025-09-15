# -----------------------------------------
# Recording management (robust)
# -----------------------------------------
recorder_pids=()
recorder_pgid_map=()
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

    rec_cmd="ros2 bag record --storage mcap -o \"bags/$bag_name\" ${topics[*]}"

    # Start ros2 bag in background and track its PGID
    bash -c "$rec_cmd" &
    pid=$!
    pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')
    recorder_pids+=($pid)
    recorder_pgid_map[$pid]=$pgid
    record_dirs+=("bags/$bag_name")

    echo -e "${GREEN}✔ Recording started: bags/$bag_name (PID: $pid, PGID: $pgid)${RESET}"
    log "Recording started: bags/$bag_name (PID: $pid, PGID: $pgid)"
    sleep "$BAG_RECORDING_DELAY"
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

    if [[ -n "$pgid" ]]; then
        target="-$pgid"
        log "Sending SIGINT to PGID $pgid ($friendly)"
    else
        target="$pid"
        log "Sending SIGINT to PID $pid ($friendly)"
    fi

    # First attempt: SIGINT to allow clean shutdown
    kill -SIGINT "$target" 2>/dev/null || true
    for i in {1..50}; do
        ! kill -0 "$pid" 2>/dev/null && return
        sleep 0.2
    done

    # If still alive, escalate
    log "$friendly still alive; sending SIGTERM"
    kill -SIGTERM "$target" 2>/dev/null || true
    for i in {1..25}; do
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

    # Stop recordings first (use PGID for ros2 bag)
    for idx in "${!recorder_pids[@]}"; do
        pid=${recorder_pids[$idx]}
        pgid=${recorder_pgid_map[$pid]}
        [[ -n "$pid" ]] && graceful_kill "recorder-$idx" "$pid" "$pgid"

        # Verify bag file exists
        dir=${record_dirs[$idx]}
        if [[ -d "$dir" && $(ls "$dir"/*.mcap 2>/dev/null | wc -l) -gt 0 ]]; then
            log "Recording saved successfully: $dir"
            echo -e "${GREEN}✔ Recording saved: $dir${RESET}"
        else
            log "Warning: Recording may not have saved properly: $dir"
            echo -e "${YELLOW}⚠ Recording may not have saved: $dir${RESET}"
        fi
    done

    # ... rest of cleanup (drivers, GUI, bag copy) remains unchanged
}

