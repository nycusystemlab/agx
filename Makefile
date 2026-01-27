# ==============================================================================
#  AGX ROS Control Interface (Optimized)
# ==============================================================================

# --- [Configuration] ---
PROJECT_NAME   := agx_ros
TASK_CONTAINER := control
SERVICE_NAME   ?= control

# Docker Buildkit
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# --- [Auto-Detection & Multi-File Config] ---
CURRENT_CONTEXT := $(shell docker context show)
MODE := pc
ENV_FILE := .env
# 預設只載入基礎檔案 (PC 通用)
COMPOSE_FILES := -f docker-compose.yaml

# 偵測是否為 AGX 環境 (透過 Context 名稱或 CPU 架構)
ifneq (,$(findstring agx,$(CURRENT_CONTEXT)))
    MODE := agx
    ENV_FILE := .env.agx
    # AGX 模式下，追加載入 AGX 專用覆蓋檔 (Overlay)
    COMPOSE_FILES += -f docker-compose.agx.yml
else ifeq ($(shell uname -m), aarch64)
    MODE := agx
    ENV_FILE := .env.agx
    COMPOSE_FILES += -f docker-compose.agx.yml
endif

# 組合最終指令
COMPOSE_CMD := docker compose $(COMPOSE_FILES) --env-file $(ENV_FILE) -p $(PROJECT_NAME)
TASK_EXEC   := docker exec -it $(TASK_CONTAINER) bash -ic

# --- [Make Settings] ---
.DEFAULT_GOAL := help
# 全域靜音：隱藏指令回顯，保持輸出乾淨
.SILENT:
.PHONY: help build up rebuild down join logs ps clean run stop view guard-% fix-time

# ==============================================================================
#  Logic Definitions (Shell Scripts inside Makefile)
# ==============================================================================

# 1. RUN SCRIPT (任務啟動選單)
define SCRIPT_RUN
    echo "=========================================="
    echo " AGX Task Launcher"
    echo " Target: $(TASK_CONTAINER)"
    echo "=========================================="
    echo "1) Keyboard Control (agx_keyboard)"
    echo "2) Lidar Mapping    (agx_lidar)"
    echo "3) HDL Localization (agx_loc)"
    echo "4) Realsense Camera (agx_camera)"
    echo "q) Quit"
    echo "------------------------------------------"
    read -p "Select task: " choice
    if [ "$$choice" = "q" ]; then exit 0; fi

    S_NAME=""
    case $$choice in
        1) S_NAME="agx_keyboard"; CMD_MAIN="rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0"; CMD_SUB="rosrun six_wheels_teleop imu_teletop_0908"; TYPE="split";;
        2) S_NAME="agx_lidar";    CMD_MAIN="roslaunch velodyne_pointcloud VLP16_points.launch"; TYPE="single";;
        3) S_NAME="agx_loc";      CMD_MAIN="roslaunch hdl_localization hdl_localization.launch"; TYPE="single";;
        4) S_NAME="agx_camera";   CMD_MAIN="roslaunch realsense2_camera rs_camera.launch"; CMD_SUB="rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _remove_gravity_vector:=true _output_rate:=100.0 /imu/data_raw:=/camera/imu"; TYPE="split";;
        *) echo "[Error] Invalid option"; exit 1;;
    esac

    if [ -z "$$S_NAME" ]; then echo "[Error] Failed to set session name."; exit 1; fi

    # Check existence
    if TMUX= tmux has-session -t $$S_NAME 2>/dev/null; then
        echo "[Info] Task '$$S_NAME' is already running."
        echo "       Use 'make view' to monitor."
        exit 0
    fi

    echo "[Info] Launching task: $$S_NAME..."
    TMUX= tmux new-session -d -s $$S_NAME
    sleep 1
    tmux set -g mouse on
    tmux send-keys -t $$S_NAME:0 "$(TASK_EXEC) '$$CMD_MAIN'" C-m
    
    if [ "$$TYPE" = "split" ]; then
        tmux split-window -h -t $$S_NAME:0
        sleep 0.5
        tmux send-keys -t $$S_NAME:0.1 "sleep 2" C-m
        tmux send-keys -t $$S_NAME:0.1 "$(TASK_EXEC) '$$CMD_SUB'" C-m
    fi

    echo "[Info] Task started in background."
    echo "       Use 'make view' to monitor logs."
    echo "       Use 'make stop' to terminate."
endef
export SCRIPT_RUN

# 2. JOIN SCRIPT (互動式進入容器)
define SCRIPT_JOIN
    # Filter containers by project label
    LIST=$$(docker ps --filter "label=com.docker.compose.project=$(PROJECT_NAME)" --format "{{.Names}}" || true)
    if [ -z "$$LIST" ]; then echo "[Info] No project containers are running."; exit 0; fi
    echo "=========================================="
    echo " Select container to enter"
    echo "=========================================="
    echo "$$LIST" | awk '{print NR ") " $$0}'
    echo "q) Quit"
    echo "------------------------------------------"
    read -p "Enter number: " j_choice
    if [ "$$j_choice" = "q" ]; then exit 0; fi
    TARGET=$$(echo "$$LIST" | sed -n "$${j_choice}p")
    if [ -n "$$TARGET" ]; then
        echo "[Info] Entering $$TARGET..."
        docker exec -it $$TARGET bash
    else
        echo "[Error] Invalid number."
    fi
endef
export SCRIPT_JOIN

# 3. STOP SCRIPT (優雅停止：發送 SIGINT)
define SCRIPT_STOP
    LIST=$$(tmux ls -F "#{session_name}" 2>/dev/null | grep "^agx_" || true)
    if [ -z "$$LIST" ]; then echo "[Info] No active tasks running."; exit 0; fi
    echo "=========================================="
    echo " Select task to terminate"
    echo "=========================================="
    echo "$$LIST" | awk '{print NR ") " $$0}'
    echo "a) Terminate All"
    echo "q) Quit"
    echo "------------------------------------------"
    read -p "Enter number: " k_choice
    if [ "$$k_choice" = "q" ]; then exit 0; fi

    # Stop function
    do_stop() {
        local s_name=$$1
        echo "[Info] Stopping $$s_name..."
        # Send Ctrl+C to all panes to allow graceful shutdown of ROS nodes
        tmux list-panes -t "$$s_name" -F "#{pane_id}" | xargs -I {} tmux send-keys -t {} C-c
        sleep 1
        # Kill session
        tmux kill-session -t "$$s_name" 2>/dev/null || true
        echo "[Info] $$s_name terminated."
    }
    export -f do_stop

    if [ "$$k_choice" = "a" ]; then
        echo "$$LIST" | xargs -I {} bash -c "do_stop {}"
        echo "[Info] All tasks terminated."
        exit 0
    fi

    TARGET=$$(echo "$$LIST" | sed -n "$${k_choice}p")
    if [ -n "$$TARGET" ]; then
        do_stop "$$TARGET"
    else
        echo "[Error] Invalid number."
    fi
endef
export SCRIPT_STOP

# 4. VIEW SCRIPT (查看背景任務)
define SCRIPT_VIEW
    LIST=$$(tmux ls -F "#{session_name}" 2>/dev/null | grep "^agx_" || true)
    if [ -z "$$LIST" ]; then echo "[Info] No active tasks."; exit 0; fi
    echo "=========================================="
    echo " Select task to view"
    echo "=========================================="
    echo "$$LIST" | awk '{print NR ") " $$0}'
    echo "q) Quit"
    echo "------------------------------------------"
    read -p "Enter number: " v_choice
    if [ "$$v_choice" = "q" ]; then exit 0; fi
    TARGET=$$(echo "$$LIST" | sed -n "$${v_choice}p")
    if [ -n "$$TARGET" ]; then
        if [ -n "$$TMUX" ]; then
            echo "[Warn] Already in Tmux. Use 'Ctrl+B, s' to switch."
        else
            echo "[Info] Attaching to $$TARGET... (Press Ctrl+B, d to detach)"
            tmux attach-session -t "$$TARGET"
        fi
    else
        echo "[Error] Invalid number."
    fi
endef
export SCRIPT_VIEW

# ==============================================================================
#  Targets
# ==============================================================================

help: ## Show available commands
	echo "AGX ROS Control Interface"
	echo "   Task Target: $(TASK_CONTAINER)"
	echo "   Context:     $(CURRENT_CONTEXT)"
	echo "   Mode:        $(MODE)"
	echo "   Configs:     $(COMPOSE_FILES)"
	echo "------------------------------------------------"
	awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "%-10s %s\n", $$1, $$2}' $(MAKEFILE_LIST)

check-env:
	if [ ! -f $(ENV_FILE) ]; then echo "[Error] Config file '$(ENV_FILE)' not found."; exit 1; fi

# 通用容器檢查 Pattern Rule
guard-%:
	if [ -z "$$(docker ps -q -f name=$*)" ]; then \
		echo "[Error] Container '$*' is not running. Run 'make up' first."; \
		exit 1; \
	fi

fix-time: guard-$(TASK_CONTAINER) ## 🕒 Fix clock skew by touching all files in container
	echo "[Info] Resetting file timestamps in $(TASK_CONTAINER)..."
	docker exec -it $(TASK_CONTAINER) bash -c "find . -type f -exec touch {} +"
	echo "[Info] Done. Clock skew should be resolved."

build: check-env ## Build docker images
	echo "[Info] Building in $(MODE) mode..."
	$(COMPOSE_CMD) build $(s)

up: check-env ## Start services in background
	echo "[Info] Starting services..."
	$(COMPOSE_CMD) up -d $(s)
	echo "[Info] System started."

rebuild: check-env ## Force rebuild and recreate containers
	echo "[Info] Rebuilding and Restarting services..."
	$(COMPOSE_CMD) up -d --build --force-recreate $(s)
	echo "[Info] Rebuild complete."

down: ## Stop and remove containers
	echo "[Info] Stopping services..."
	$(COMPOSE_CMD) down --remove-orphans $(s)
	echo "[Info] Services stopped."

logs: ## Follow service logs
	$(COMPOSE_CMD) logs -f

ps: ## View container status
	$(COMPOSE_CMD) ps

clean: ## Remove containers and images
	$(COMPOSE_CMD) down --rmi local -v --remove-orphans

# ==============================================================================
#  Task Actions
# ==============================================================================

join: ## Enter container shell (Interactive)
	bash -c "$$SCRIPT_JOIN"

run: guard-$(TASK_CONTAINER) ## Launch ROS task (Background)
	bash -c "$$SCRIPT_RUN"

stop: ## Terminate running tasks
	bash -c "$$SCRIPT_STOP"

view: ## Attach to task session
	bash -c "$$SCRIPT_VIEW"