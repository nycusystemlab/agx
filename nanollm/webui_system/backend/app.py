import asyncio
import json
import re
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from vla_demo.topics import (
        CURRENT_STEP_TOPIC,
        LANDMARK_DETECTION_TOPIC,
        MISSION_STATE_TOPIC,
        ROUTE_BUFFER_STATUS_TOPIC,
        ROUTE_PLAN_TOPIC,
        ROUTE_REQUEST_TOPIC,
    )
except ImportError:
    ROUTE_REQUEST_TOPIC = "/vla/route_request"
    ROUTE_PLAN_TOPIC = "/vla/route_plan"
    CURRENT_STEP_TOPIC = "/vla/current_step"
    LANDMARK_DETECTION_TOPIC = "/vla/landmark_detection"
    MISSION_STATE_TOPIC = "/vla/mission_state"
    ROUTE_BUFFER_STATUS_TOPIC = "/vla/route_buffer_status"

DEFAULT_ENVIRONMENT_ID = "hallway_9f"
DEFAULT_CAMERA_SOURCE = "/camera/camera/color/image_raw"
DEFAULT_CLIP_DURATION_SEC = 3.0
DEFAULT_INFERENCE_INTERVAL_SEC = 1.5
DEFAULT_PLANNING_MODE = "default"


def guess_media_type(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix == ".mp4":
        return "video/mp4"
    if suffix in {".jpg", ".jpeg"}:
        return "image/jpeg"
    if suffix == ".png":
        return "image/png"
    if suffix == ".webp":
        return "image/webp"
    return "application/octet-stream"


def extract_location(text: str) -> str:
    known_places = [
        "後花園",
        "前庭",
        "大門口",
        "停車場",
        "中庭",
        "行政大樓",
        "實驗室",
        "倉庫",
        "操場",
        "屋頂",
    ]
    for place in known_places:
        if place in text:
            return place

    match = re.search(r"到(.+?)(去|做|進行|巡檢|查看|確認|$)", text)
    if match:
        return match.group(1).strip()

    return "未指定區域"
def pick_asset(text: str) -> str:
    if any(keyword in text for keyword in ["前往", "走過去", "到現場", "派遣地面車"]):
        return "AMR"
    return "AMR"


def build_fixed_reply(user_text: str) -> dict[str, str]:
    location = extract_location(user_text)
    asset = pick_asset(user_text)
    reply = f"任務啟動，出動設備：{asset}，地點：{location}"
    return {
        "raw_text": user_text,
        "asset": asset,
        "target_location": location,
        "reply": reply,
    }


def slugify_mission_id(text: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", text).strip("_").lower()
    return slug or "route"


def infer_source_mode(video_uri: str, preferred_mode: str = "") -> str:
    if preferred_mode == "live_camera":
        return preferred_mode
    return "video_file" if video_uri.strip() else "live_camera"


def build_route_request_payload(
    *,
    goal_text: str,
    mission_id: str = "",
    environment_id: str = DEFAULT_ENVIRONMENT_ID,
    source_mode: str = "",
    planning_mode: str = DEFAULT_PLANNING_MODE,
    video_uri: str = "",
    camera_source: str = DEFAULT_CAMERA_SOURCE,
    clip_duration_sec: float = DEFAULT_CLIP_DURATION_SEC,
    inference_interval_sec: float = DEFAULT_INFERENCE_INTERVAL_SEC,
) -> dict[str, Any]:
    normalized_goal = goal_text.strip()
    if not normalized_goal:
        raise ValueError("goal_text cannot be empty")

    normalized_video_uri = video_uri.strip()
    normalized_source_mode = infer_source_mode(normalized_video_uri, source_mode.strip())
    normalized_mission_id = mission_id.strip() or (
        f"webui_{slugify_mission_id(normalized_goal)}_{int(time.time())}"
    )

    payload = {
        "mission_id": normalized_mission_id,
        "goal_text": normalized_goal,
        "environment_id": environment_id.strip() or DEFAULT_ENVIRONMENT_ID,
        "source_mode": normalized_source_mode,
        "planning_mode": planning_mode.strip() or DEFAULT_PLANNING_MODE,
        "camera_source": camera_source.strip() or DEFAULT_CAMERA_SOURCE,
        "clip_duration_sec": float(clip_duration_sec),
        "inference_interval_sec": float(inference_interval_sec),
    }
    if normalized_video_uri:
        payload["video_uri"] = normalized_video_uri
    return payload


def resolve_frontend_dir() -> Path:
    current_dir = Path(__file__).resolve().parent
    candidates = [
        current_dir / "frontend",
        current_dir.parent / "frontend",
        current_dir,
    ]
    for path in candidates:
        if (path / "index.html").exists():
            return path
    return current_dir


class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast_json(self, payload: dict):
        dead = []
        for ws in self.active_connections:
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


class WebUIROSBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop, manager: ConnectionManager):
        super().__init__("webui_backend_bridge")

        self.loop = loop
        self.manager = manager

        self.current_system_prompt = ""
        self.current_video_uri = ""
        self.route_defaults = {
            "mission_id": "",
            "environment_id": DEFAULT_ENVIRONMENT_ID,
            "source_mode": "video_file",
            "planning_mode": DEFAULT_PLANNING_MODE,
            "camera_source": DEFAULT_CAMERA_SOURCE,
            "clip_duration_sec": DEFAULT_CLIP_DURATION_SEC,
            "inference_interval_sec": DEFAULT_INFERENCE_INTERVAL_SEC,
            "video_uri": "",
        }

        self.latest_route_request: dict[str, Any] = {}
        self.latest_route_plan: dict[str, Any] = {}
        self.latest_current_step: dict[str, Any] = {}
        self.latest_landmark_detection: dict[str, Any] = {}
        self.latest_mission_state: dict[str, Any] = {}
        self.latest_route_buffer_status: dict[str, Any] = {}

        self.request_pub = self.create_publisher(String, "/llm/request", 10)
        self.response_pub = self.create_publisher(String, "/llm/response", 10)
        self.status_pub = self.create_publisher(String, "/llm/status", 10)
        self.system_prompt_pub = self.create_publisher(String, "/llm/system_prompt", 10)
        self.video_uri_pub = self.create_publisher(String, "/llm/video_uri", 10)
        self.route_request_pub = self.create_publisher(String, ROUTE_REQUEST_TOPIC, 10)

        self.create_subscription(String, "/llm/response", self.on_response, 10)
        self.create_subscription(String, "/llm/status", self.on_status, 10)
        self.create_subscription(String, "/llm/system_prompt", self.on_system_prompt, 10)
        self.create_subscription(String, "/llm/video_uri", self.on_video_uri, 10)
        self.create_subscription(Bool, "/llm/event_flag", self.on_event_flag, 10)
        self.create_subscription(String, ROUTE_PLAN_TOPIC, self.on_route_plan, 10)
        self.create_subscription(String, CURRENT_STEP_TOPIC, self.on_current_step, 10)
        self.create_subscription(
            String,
            LANDMARK_DETECTION_TOPIC,
            self.on_landmark_detection,
            10,
        )
        self.create_subscription(String, MISSION_STATE_TOPIC, self.on_mission_state, 10)
        self.create_subscription(
            String,
            ROUTE_BUFFER_STATUS_TOPIC,
            self.on_route_buffer_status,
            10,
        )

        self.get_logger().info("WebUI ROS bridge started.")
        self.get_logger().info(
            "Subscribed: /llm/response /llm/status /llm/system_prompt /llm/video_uri "
            f"/llm/event_flag {ROUTE_PLAN_TOPIC} {CURRENT_STEP_TOPIC} "
            f"{LANDMARK_DETECTION_TOPIC} {MISSION_STATE_TOPIC} "
            f"{ROUTE_BUFFER_STATUS_TOPIC}"
        )
        self.get_logger().info(
            "Published : /llm/request /llm/response /llm/status /llm/system_prompt "
            f"/llm/video_uri {ROUTE_REQUEST_TOPIC}"
        )

    def push_to_frontend(self, payload: dict):
        asyncio.run_coroutine_threadsafe(
            self.manager.broadcast_json(payload),
            self.loop,
        )

    def _push_json_topic(self, event_type: str, raw_text: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(raw_text)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid JSON on {event_type}: {exc}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warn(f"Ignoring non-object payload on {event_type}")
            return None
        self.push_to_frontend({"type": event_type, "payload": payload})
        return payload

    def current_route_settings_payload(self) -> dict[str, Any]:
        payload = dict(self.route_defaults)
        payload["video_uri"] = self.current_video_uri
        payload["source_mode"] = infer_source_mode(
            self.current_video_uri,
            str(payload.get("source_mode", "")),
        )
        return payload

    def update_route_defaults(self, payload: dict[str, Any]) -> None:
        self.route_defaults["mission_id"] = str(payload.get("mission_id", "")).strip()
        self.route_defaults["environment_id"] = (
            str(payload.get("environment_id", "")).strip() or DEFAULT_ENVIRONMENT_ID
        )
        self.route_defaults["source_mode"] = infer_source_mode(
            str(payload.get("video_uri", self.current_video_uri)),
            str(payload.get("source_mode", "")),
        )
        self.route_defaults["planning_mode"] = (
            str(payload.get("planning_mode", "")).strip() or DEFAULT_PLANNING_MODE
        )
        self.route_defaults["camera_source"] = (
            str(payload.get("camera_source", "")).strip() or DEFAULT_CAMERA_SOURCE
        )
        self.route_defaults["clip_duration_sec"] = float(
            payload.get("clip_duration_sec", DEFAULT_CLIP_DURATION_SEC)
        )
        self.route_defaults["inference_interval_sec"] = float(
            payload.get("inference_interval_sec", DEFAULT_INFERENCE_INTERVAL_SEC)
        )
        self.route_defaults["video_uri"] = str(
            payload.get("video_uri", self.current_video_uri)
        ).strip()

    def publish_route_request(
        self,
        payload: dict[str, Any],
        *,
        remember_mission_id: bool = False,
    ) -> None:
        previous_mission_id = self.route_defaults.get("mission_id", "")
        self.update_route_defaults(payload)
        if not remember_mission_id:
            self.route_defaults["mission_id"] = previous_mission_id
        if payload.get("video_uri"):
            self.current_video_uri = str(payload["video_uri"]).strip()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.route_request_pub.publish(msg)
        status_msg = String()
        status_msg.data = (
            f"已送出 route_request：{payload.get('mission_id', '-')}"
            f" ({payload.get('source_mode', '-')})"
        )
        self.status_pub.publish(status_msg)
        self.latest_route_request = payload
        self.push_to_frontend({"type": "route_request", "payload": payload})
        self.push_to_frontend(
            {"type": "route_defaults", "payload": self.current_route_settings_payload()}
        )

    def on_response(self, msg: String):
        self.push_to_frontend({"type": "ai_reply", "text": msg.data})

    def on_status(self, msg: String):
        self.push_to_frontend({"type": "status", "text": msg.data})

    def on_system_prompt(self, msg: String):
        self.current_system_prompt = msg.data
        self.push_to_frontend({"type": "system_prompt", "text": msg.data})

    def on_video_uri(self, msg: String):
        self.current_video_uri = msg.data
        self.route_defaults["video_uri"] = msg.data
        self.route_defaults["source_mode"] = infer_source_mode(
            msg.data,
            self.route_defaults.get("source_mode", ""),
        )
        self.push_to_frontend({"type": "video_uri", "text": msg.data})
        self.push_to_frontend(
            {"type": "route_defaults", "payload": self.current_route_settings_payload()}
        )

    def on_event_flag(self, msg: Bool):
        if not msg.data:
            return

        event_text = "異常事件觸發！事件：行人跌倒！派遣UGV中"
        status_text = "異常事件：行人跌倒，已派遣 UGV"

        response_msg = String()
        response_msg.data = event_text
        self.response_pub.publish(response_msg)

        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)

        self.push_to_frontend(
            {
                "type": "event",
                "text": event_text,
                "event_type": "行人跌倒",
                "action": "派遣UGV中",
            }
        )

    def on_route_plan(self, msg: String):
        payload = self._push_json_topic("route_plan", msg.data)
        if payload is not None:
            self.latest_route_plan = payload

    def on_current_step(self, msg: String):
        payload = self._push_json_topic("current_step", msg.data)
        if payload is not None:
            self.latest_current_step = payload

    def on_landmark_detection(self, msg: String):
        payload = self._push_json_topic("landmark_detection", msg.data)
        if payload is not None:
            self.latest_landmark_detection = payload

    def on_mission_state(self, msg: String):
        payload = self._push_json_topic("mission_state", msg.data)
        if payload is not None:
            self.latest_mission_state = payload

    def on_route_buffer_status(self, msg: String):
        payload = self._push_json_topic("route_buffer_status", msg.data)
        if payload is not None:
            self.latest_route_buffer_status = payload

    def handle_user_command(self, user_text: str) -> dict[str, Any]:
        result = build_fixed_reply(user_text)

        request_msg = String()
        request_msg.data = user_text
        self.request_pub.publish(request_msg)

        route_request = build_route_request_payload(
            goal_text=user_text,
            mission_id=self.route_defaults.get("mission_id", ""),
            environment_id=self.route_defaults.get(
                "environment_id",
                DEFAULT_ENVIRONMENT_ID,
            ),
            source_mode=self.route_defaults.get("source_mode", ""),
            planning_mode=self.route_defaults.get("planning_mode", DEFAULT_PLANNING_MODE),
            video_uri=self.current_video_uri,
            camera_source=self.route_defaults.get(
                "camera_source",
                DEFAULT_CAMERA_SOURCE,
            ),
            clip_duration_sec=float(
                self.route_defaults.get(
                    "clip_duration_sec",
                    DEFAULT_CLIP_DURATION_SEC,
                )
            ),
            inference_interval_sec=float(
                self.route_defaults.get(
                    "inference_interval_sec",
                    DEFAULT_INFERENCE_INTERVAL_SEC,
                )
            ),
        )
        self.publish_route_request(route_request, remember_mission_id=False)

        response_msg = String()
        response_msg.data = result["reply"]
        self.response_pub.publish(response_msg)

        result["route_request"] = route_request
        return result

    def publish_system_prompt(self, text: str):
        self.current_system_prompt = text
        msg = String()
        msg.data = text
        self.system_prompt_pub.publish(msg)

    def publish_video_uri(self, text: str):
        self.current_video_uri = text
        self.route_defaults["video_uri"] = text
        self.route_defaults["source_mode"] = infer_source_mode(
            text,
            self.route_defaults.get("source_mode", ""),
        )
        msg = String()
        msg.data = text
        self.video_uri_pub.publish(msg)
        self.push_to_frontend(
            {"type": "route_defaults", "payload": self.current_route_settings_payload()}
        )


app = FastAPI()
manager = ConnectionManager()
FRONTEND_DIR = resolve_frontend_dir()

if FRONTEND_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

ros_node: Optional[WebUIROSBridge] = None
ros_thread: Optional[threading.Thread] = None


class CommandRequest(BaseModel):
    text: str


class TextRequest(BaseModel):
    text: str


class RouteRequest(BaseModel):
    goal_text: str
    mission_id: str = ""
    environment_id: str = DEFAULT_ENVIRONMENT_ID
    source_mode: str = ""
    planning_mode: str = DEFAULT_PLANNING_MODE
    video_uri: str = ""
    camera_source: str = DEFAULT_CAMERA_SOURCE
    clip_duration_sec: float = DEFAULT_CLIP_DURATION_SEC
    inference_interval_sec: float = DEFAULT_INFERENCE_INTERVAL_SEC


@app.on_event("startup")
async def startup_event():
    global ros_node, ros_thread

    loop = asyncio.get_running_loop()

    rclpy.init(args=None)
    ros_node = WebUIROSBridge(loop=loop, manager=manager)

    def spin_ros():
        rclpy.spin(ros_node)

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()


@app.on_event("shutdown")
async def shutdown_event():
    global ros_node
    if ros_node is not None:
        ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@app.get("/")
async def index():
    index_path = FRONTEND_DIR / "index.html"
    if not index_path.exists():
        return JSONResponse(
            status_code=500,
            content={"ok": False, "error": f"index.html not found: {index_path}"},
        )
    return FileResponse(str(index_path))


@app.get("/health")
async def health():
    return {
        "ok": True,
        "frontend_dir": str(FRONTEND_DIR),
        "index_exists": (FRONTEND_DIR / "index.html").exists(),
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)

    await websocket.send_json(
        {
            "type": "system",
            "text": "WebSocket 已連線，等待任務、路徑規劃與 landmark 狀態。",
        }
    )

    if ros_node is not None:
        await websocket.send_json({"type": "system_prompt", "text": ros_node.current_system_prompt})
        await websocket.send_json({"type": "video_uri", "text": ros_node.current_video_uri})
        await websocket.send_json(
            {
                "type": "route_defaults",
                "payload": ros_node.current_route_settings_payload(),
            }
        )
        if ros_node.latest_route_request:
            await websocket.send_json(
                {"type": "route_request", "payload": ros_node.latest_route_request}
            )
        if ros_node.latest_route_plan:
            await websocket.send_json(
                {"type": "route_plan", "payload": ros_node.latest_route_plan}
            )
        if ros_node.latest_current_step:
            await websocket.send_json(
                {"type": "current_step", "payload": ros_node.latest_current_step}
            )
        if ros_node.latest_landmark_detection:
            await websocket.send_json(
                {
                    "type": "landmark_detection",
                    "payload": ros_node.latest_landmark_detection,
                }
            )
        if ros_node.latest_mission_state:
            await websocket.send_json(
                {"type": "mission_state", "payload": ros_node.latest_mission_state}
            )
        if ros_node.latest_route_buffer_status:
            await websocket.send_json(
                {
                    "type": "route_buffer_status",
                    "payload": ros_node.latest_route_buffer_status,
                }
            )

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


@app.post("/api/command")
async def api_command(req: CommandRequest):
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}

    text = req.text.strip()
    if not text:
        return {"ok": False, "error": "empty text"}

    result = ros_node.handle_user_command(text)
    task_payload = {
        "asset": result.get("asset", "UGV"),
        "target_location": result.get("target_location", extract_location(text)),
        "route_request_id": result["route_request"]["mission_id"],
        "source_mode": result["route_request"]["source_mode"],
    }

    await manager.broadcast_json(
        {
            "type": "task",
            "task": task_payload,
        }
    )

    return {
        "ok": True,
        "reply": result["reply"],
        "task": task_payload,
        "route_request": result["route_request"],
    }


@app.post("/api/route_request")
async def api_route_request(req: RouteRequest):
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}

    try:
        payload = build_route_request_payload(
            goal_text=req.goal_text,
            mission_id=req.mission_id,
            environment_id=req.environment_id,
            source_mode=req.source_mode,
            planning_mode=req.planning_mode,
            video_uri=req.video_uri.strip() or ros_node.current_video_uri,
            camera_source=req.camera_source,
            clip_duration_sec=req.clip_duration_sec,
            inference_interval_sec=req.inference_interval_sec,
        )
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}

    ros_node.publish_route_request(payload, remember_mission_id=bool(req.mission_id.strip()))
    return {"ok": True, "route_request": payload}


@app.get("/api/route_buffer/status")
async def api_route_buffer_status():
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}
    if not ros_node.latest_route_buffer_status:
        return {"ok": False, "error": "No route buffer status yet"}
    return {"ok": True, "route_buffer_status": ros_node.latest_route_buffer_status}


@app.get("/api/route_buffer/latest")
async def api_route_buffer_latest():
    global ros_node
    if ros_node is None:
        return JSONResponse(status_code=503, content={"ok": False, "error": "ROS node not ready"})

    payload = ros_node.latest_route_buffer_status
    path_text = str(payload.get("local_path", "")).strip()
    if not path_text:
        return JSONResponse(
            status_code=404,
            content={"ok": False, "error": "No persisted live buffer file is available"},
        )

    path = Path(path_text)
    if not path.exists():
        return JSONResponse(
            status_code=404,
            content={"ok": False, "error": f"Live buffer file not found: {path}"},
        )

    return FileResponse(
        str(path),
        media_type=guess_media_type(path),
        filename=path.name,
    )


@app.post("/api/system_prompt")
async def api_system_prompt(req: TextRequest):
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}

    ros_node.publish_system_prompt(req.text.strip())
    return {"ok": True, "text": req.text.strip()}


@app.post("/api/video_uri")
async def api_video_uri(req: TextRequest):
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}

    ros_node.publish_video_uri(req.text.strip())
    return {"ok": True, "text": req.text.strip()}


@app.post("/api/simulate/event")
async def api_simulate_event():
    global ros_node
    if ros_node is None:
        return {"ok": False, "error": "ROS node not ready"}

    ros_node.on_event_flag(Bool(data=True))
    return {"ok": True}
