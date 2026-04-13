from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlparse
from urllib.request import Request, urlopen

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .json_utils import dumps_json, extract_json_object, loads_json
from .prompting import build_route_planner_prompt
from .route_planning import coerce_route_plan_payload
from .schemas import MissionSpec, RouteRequestSpec, SchemaError
from .topics import (
    CAMERA_IMAGE_TOPIC,
    ROUTE_BUFFER_STATUS_TOPIC,
    ROUTE_PLAN_TOPIC,
    ROUTE_REQUEST_TOPIC,
    SET_MISSION_TOPIC,
)

VIDEO_SUFFIXES = {".avi", ".mkv", ".mov", ".mp4", ".webm"}
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".webp"}


class MockRoutePlannerBackend:
    def infer(
        self,
        request: RouteRequestSpec,
        prompt_text: str,
        media_flag: str,
        media_path: str,
    ) -> dict[str, Any]:
        del prompt_text, media_flag, media_path
        target_text = request.goal_text.strip() or "destination"
        return {
            "mission_id": request.mission_id,
            "mission_text": request.goal_text,
            "environment_id": request.environment_id,
            "camera_source": request.camera_source,
            "planning_mode": request.planning_mode,
            "inference_interval_sec": request.inference_interval_sec,
            "steps": [
                {
                    "step_id": 1,
                    "instruction": "Move out from the current position and align with the main corridor.",
                    "visual_goal": "The main corridor fills most of the forward view.",
                    "scene_description": "A forward-facing corridor opens up ahead.",
                    "expected_landmarks": ["corridor"],
                    "primary_landmark": "corridor",
                    "grounding_objects": ["corridor"],
                    "grounding_prompt": "corridor.",
                    "control_primitive": "move_forward_until_recheck",
                    "votes_needed": 2,
                    "confidence_threshold": 0.7,
                    "min_dwell_sec": 1.5,
                    "timeout_sec": 10.0,
                    "fallback": "pause",
                },
                {
                    "step_id": 2,
                    "instruction": "Continue toward the most distinctive waypoint on the route.",
                    "visual_goal": f"A clear waypoint that leads toward {target_text} becomes visible.",
                    "scene_description": "The route should expose a distinctive waypoint or branch.",
                    "expected_landmarks": ["waypoint_marker"],
                    "primary_landmark": "waypoint marker",
                    "grounding_objects": ["waypoint marker"],
                    "grounding_prompt": "waypoint marker.",
                    "control_primitive": "move_forward_until_recheck",
                    "votes_needed": 2,
                    "confidence_threshold": 0.7,
                    "min_dwell_sec": 1.5,
                    "timeout_sec": 12.0,
                    "fallback": "pause",
                },
                {
                    "step_id": 3,
                    "instruction": f"Approach the final destination area for {target_text}.",
                    "visual_goal": f"The destination area for {target_text} is centered ahead.",
                    "scene_description": "The target destination area is visible in front of the robot.",
                    "expected_landmarks": ["destination_area"],
                    "primary_landmark": "destination area",
                    "grounding_objects": ["destination area"],
                    "grounding_prompt": "destination area.",
                    "control_primitive": "approach_target_zone",
                    "votes_needed": 2,
                    "confidence_threshold": 0.75,
                    "min_dwell_sec": 1.5,
                    "timeout_sec": 12.0,
                    "fallback": "stop_and_hold",
                },
            ],
        }


class CosmosCLIBackend:
    def __init__(
        self,
        host: str,
        port: int,
        model: str,
        timeout_sec: float,
    ) -> None:
        self.host = host
        self.port = port
        self.model = model
        self.timeout_sec = timeout_sec
        self.api_url = f"http://{self.host}:{self.port}/v1/chat/completions"
        self.models_url = f"http://{self.host}:{self.port}/v1/models"

    def infer(
        self,
        request: RouteRequestSpec,
        prompt_text: str,
        media_flag: str,
        media_path: str,
    ) -> dict[str, Any]:
        del request
        media_type = "image_url" if media_flag == "--images" else "video_url"
        media_url = self._normalize_media_url(media_path)
        model_name = self.model or self._resolve_model_name()
        payload = {
            "model": model_name,
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are an AMR route planning assistant. "
                        "Return exactly one JSON object and no markdown."
                    ),
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompt_text,
                        },
                        {
                            "type": media_type,
                            media_type: {"url": media_url},
                        },
                    ],
                },
            ],
            "temperature": 0.2,
            "top_p": 0.85,
            "max_completion_tokens": 2048,
            "stream": False,
        }
        raw_response = self._post_json(self.api_url, payload)
        choices = raw_response.get("choices") or []
        if not choices:
            raise RuntimeError("Cosmos server returned no completion choices")
        content = (
            choices[0]
            .get("message", {})
            .get("content", "")
        )
        if not isinstance(content, str) or not content.strip():
            raise RuntimeError("Cosmos server returned empty message content")
        return extract_json_object(content)

    def _normalize_media_url(self, media_path: str) -> str:
        if media_path.startswith(("http://", "https://", "file://")):
            return media_path
        return Path(media_path).expanduser().resolve().as_uri()

    def _resolve_model_name(self) -> str:
        response = self._get_json(self.models_url)
        models = response.get("data") or []
        if not models:
            raise RuntimeError("Cosmos server returned no models")
        model_id = models[0].get("id")
        if not isinstance(model_id, str) or not model_id.strip():
            raise RuntimeError("Cosmos server model list did not contain a usable id")
        return model_id

    def _get_json(self, url: str) -> dict[str, Any]:
        request = Request(url, method="GET")
        try:
            with urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as exc:
            raise RuntimeError(
                f"Cosmos server HTTP {exc.code}: {exc.read().decode('utf-8', errors='ignore')}"
            ) from exc
        except URLError as exc:
            raise RuntimeError(f"Cosmos server connection failed: {exc}") from exc

    def _post_json(self, url: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload).encode("utf-8")
        request = Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as exc:
            raise RuntimeError(
                f"Cosmos server HTTP {exc.code}: {exc.read().decode('utf-8', errors='ignore')}"
            ) from exc
        except URLError as exc:
            raise RuntimeError(f"Cosmos server connection failed: {exc}") from exc


class RoutePlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_route_planner")

        self.declare_parameter("backend_mode", "cosmos_cli")
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 8000)
        self.declare_parameter("model", "")
        self.declare_parameter("backend_timeout_sec", 120.0)
        self.declare_parameter("max_buffer_sec", 20.0)
        self.declare_parameter("video_fps_fallback", 8.0)
        self.declare_parameter("live_request_warmup_timeout_sec", 5.0)
        self.declare_parameter("live_buffer_ready_ratio", 0.85)
        self.declare_parameter("shared_media_dir", "/workspaces/vla_route_media")

        backend_mode = str(self.get_parameter("backend_mode").value)
        if backend_mode == "mock":
            self.backend = MockRoutePlannerBackend()
        else:
            self.backend = CosmosCLIBackend(
                host=str(self.get_parameter("host").value),
                port=int(self.get_parameter("port").value),
                model=str(self.get_parameter("model").value),
                timeout_sec=float(self.get_parameter("backend_timeout_sec").value),
            )

        self.bridge = CvBridge()
        self.frame_buffer: deque[tuple[float, Any]] = deque()
        self.camera_subscription = None
        self.camera_topic = ""
        self.frames_received = 0
        self.last_frame_stamp = 0.0
        self.last_source_frame_stamp = 0.0
        self.pending_live_request: RouteRequestSpec | None = None
        self.pending_live_request_started_at = 0.0
        self.shared_media_dir = Path(str(self.get_parameter("shared_media_dir").value))
        self.shared_media_dir.mkdir(parents=True, exist_ok=True)

        self.route_plan_pub = self.create_publisher(String, ROUTE_PLAN_TOPIC, 10)
        self.route_buffer_status_pub = self.create_publisher(String, ROUTE_BUFFER_STATUS_TOPIC, 10)
        self.set_mission_pub = self.create_publisher(String, SET_MISSION_TOPIC, 10)
        self.create_subscription(String, ROUTE_REQUEST_TOPIC, self._on_route_request, 10)
        self.create_timer(0.2, self._on_pending_live_request_timer)
        self._ensure_camera_subscription(CAMERA_IMAGE_TOPIC)

        self.get_logger().info(
            f"Route planner ready in '{backend_mode}' mode. Listening on {ROUTE_REQUEST_TOPIC}."
        )

    def _ensure_camera_subscription(self, topic: str) -> None:
        normalized_topic = topic.strip() or CAMERA_IMAGE_TOPIC
        if normalized_topic == self.camera_topic and self.camera_subscription is not None:
            return
        if self.camera_subscription is not None:
            self.destroy_subscription(self.camera_subscription)
        self.frame_buffer.clear()
        self.frames_received = 0
        self.last_frame_stamp = 0.0
        self.last_source_frame_stamp = 0.0
        self.camera_topic = normalized_topic
        self.camera_subscription = self.create_subscription(
            Image,
            normalized_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Route planner camera source set to {normalized_topic}")

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        source_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        received_at = self._now()
        if source_stamp <= 0:
            source_stamp = received_at
        self.frame_buffer.append((received_at, frame))
        self.frames_received += 1
        self.last_frame_stamp = received_at
        self.last_source_frame_stamp = source_stamp
        max_buffer_sec = float(self.get_parameter("max_buffer_sec").value)
        while self.frame_buffer and (received_at - self.frame_buffer[0][0]) > max_buffer_sec:
            self.frame_buffer.popleft()

    def _on_route_request(self, msg: String) -> None:
        try:
            payload = loads_json(msg.data, "route_request")
            request = RouteRequestSpec.from_dict(payload)
            if request.source_mode == "live_camera":
                self._ensure_camera_subscription(request.camera_source)
                if not self._has_buffered_live_frames(request):
                    self.pending_live_request = request
                    self.pending_live_request_started_at = self._now()
                    self._publish_route_buffer_status(
                        self._build_route_buffer_status(
                            request,
                            status="warming_up",
                            message="Waiting for buffered live frames.",
                            window_start=self._window_start(request),
                        )
                    )
                    self.get_logger().info(
                        "Queued live route request until camera buffer is ready: "
                        f"{request.mission_id} on {request.camera_source}"
                    )
                    return
            else:
                self.pending_live_request = None
                self.pending_live_request_started_at = 0.0
                self._publish_route_buffer_status(
                    {
                        "mission_id": request.mission_id,
                        "source_mode": request.source_mode,
                        "status": "inactive",
                        "camera_source": request.camera_source,
                        "video_uri": request.video_uri,
                        "message": "Route planner is using video_file input directly.",
                        "updated_at": self._now(),
                    }
                )
            self._execute_route_request(request)
        except (OSError, RuntimeError, SchemaError, ValueError) as exc:
            self.get_logger().error(f"Route planning failed: {exc}")

    def _on_pending_live_request_timer(self) -> None:
        request = self.pending_live_request
        if request is None:
            return
        if self._has_buffered_live_frames(request):
            self.pending_live_request = None
            self.pending_live_request_started_at = 0.0
            try:
                self._execute_route_request(request)
            except (OSError, RuntimeError, SchemaError, ValueError) as exc:
                self.get_logger().error(f"Route planning failed: {exc}")
            return

        now = self._now()
        timeout_sec = float(self.get_parameter("live_request_warmup_timeout_sec").value)
        if (now - self.pending_live_request_started_at) > timeout_sec:
            diagnostics = self._buffer_diagnostics(request)
            self._publish_route_buffer_status(
                self._build_route_buffer_status(
                    request,
                    status="timeout",
                    message=diagnostics,
                    window_start=self._window_start(request),
                )
            )
            self.get_logger().error(
                "Route planning failed: "
                f"{diagnostics}"
            )
            self.pending_live_request = None
            self.pending_live_request_started_at = 0.0

    def _execute_route_request(self, request: RouteRequestSpec) -> None:
        if request.source_mode == "live_camera":
            with tempfile.TemporaryDirectory(
                prefix="vla_route_media_",
                dir=str(self.shared_media_dir),
            ) as tempdir:
                media_flag, media_path = self._prepare_live_media(request, Path(tempdir))
                prompt_text = build_route_planner_prompt(request)
                raw_plan = self.backend.infer(request, prompt_text, media_flag, media_path)
                normalized_plan = coerce_route_plan_payload(raw_plan, request)
        else:
            media_flag, media_path = self._prepare_uri_media(request.video_uri)
            prompt_text = build_route_planner_prompt(request)
            raw_plan = self.backend.infer(request, prompt_text, media_flag, media_path)
            normalized_plan = coerce_route_plan_payload(raw_plan, request)

        MissionSpec.from_dict(normalized_plan)
        output = String()
        output.data = dumps_json(normalized_plan)
        self.route_plan_pub.publish(output)
        self.set_mission_pub.publish(output)
        self.get_logger().info(
            f"Published route plan '{normalized_plan['mission_id']}' with "
            f"{len(normalized_plan['steps'])} steps."
        )

    def _prepare_uri_media(self, video_uri: str) -> tuple[str, str]:
        resolved = self._resolve_uri(video_uri)
        if resolved.startswith("http://") or resolved.startswith("https://"):
            suffix = Path(urlparse(resolved).path).suffix.lower()
        else:
            suffix = Path(resolved).suffix.lower()
        if suffix in IMAGE_SUFFIXES:
            return "--images", resolved
        return "--videos", resolved

    def _prepare_live_media(
        self,
        request: RouteRequestSpec,
        temp_root: Path,
    ) -> tuple[str, str]:
        frames = self._buffered_live_frames(request)
        if not frames:
            raise RuntimeError(self._buffer_diagnostics(request))
        if len(frames) == 1:
            path = temp_root / "frame.jpg"
            cv2.imwrite(str(path), frames[-1][1])
            persisted_path = self.shared_media_dir / "latest_live_buffer.jpg"
            shutil.copy2(path, persisted_path)
            self._publish_route_buffer_status(
                self._build_route_buffer_status(
                    request,
                    status="ready",
                    message="Latest live buffer frame is ready.",
                    media_type="image",
                    frame_count=1,
                    fps=self._estimate_fps([stamp for stamp, _ in frames]),
                    buffer_start_stamp=frames[0][0],
                    buffer_end_stamp=frames[-1][0],
                    local_path=str(persisted_path),
                    file_uri=persisted_path.resolve().as_uri(),
                )
            )
            return "--images", str(persisted_path)

        path = temp_root / "clip_raw.mp4"
        height, width = frames[0][1].shape[:2]
        fps = self._estimate_fps([stamp for stamp, _ in frames])
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height),
        )
        for _, frame in frames:
            writer.write(frame)
        writer.release()
        persisted_path = self.shared_media_dir / "latest_live_buffer.mp4"
        self._encode_previewable_mp4(path, persisted_path)
        self._publish_route_buffer_status(
            self._build_route_buffer_status(
                request,
                status="ready",
                message="Latest live buffer clip is ready.",
                media_type="video",
                frame_count=len(frames),
                fps=fps,
                buffer_span_sec=round(self._buffer_span_sec(frames), 3),
                required_buffer_span_sec=round(self._required_buffer_span_sec(request), 3),
                buffer_start_stamp=frames[0][0],
                buffer_end_stamp=frames[-1][0],
                local_path=str(persisted_path),
                file_uri=persisted_path.resolve().as_uri(),
            )
        )
        return "--videos", str(persisted_path)

    def _estimate_fps(self, stamps: list[float]) -> float:
        if len(stamps) < 2:
            return float(self.get_parameter("video_fps_fallback").value)
        intervals = [
            current - previous
            for previous, current in zip(stamps[:-1], stamps[1:])
            if (current - previous) > 0
        ]
        if not intervals:
            return float(self.get_parameter("video_fps_fallback").value)
        average_interval = sum(intervals) / len(intervals)
        if average_interval <= 0:
            return float(self.get_parameter("video_fps_fallback").value)
        return max(1.0, 1.0 / average_interval)

    def _buffered_live_frames(self, request: RouteRequestSpec) -> list[tuple[float, Any]]:
        window_start = self._window_start(request)
        return [
            (stamp, frame)
            for stamp, frame in self.frame_buffer
            if stamp >= window_start
        ]

    def _has_buffered_live_frames(self, request: RouteRequestSpec) -> bool:
        frames = self._buffered_live_frames(request)
        if not frames:
            return False
        return self._buffer_span_sec(frames) >= self._required_buffer_span_sec(request)

    def _buffer_diagnostics(self, request: RouteRequestSpec) -> str:
        window_start = self._window_start(request)
        return (
            "No buffered frames available on "
            f"{request.camera_source} for live route planning "
            f"(frames_received={self.frames_received}, "
            f"buffer_size={len(self.frame_buffer)}, "
            f"last_frame_stamp={self.last_frame_stamp:.3f}, "
            f"last_source_frame_stamp={self.last_source_frame_stamp:.3f}, "
            f"buffer_span_sec={self._buffer_span_sec(self._buffered_live_frames(request)):.3f}, "
            f"required_buffer_span_sec={self._required_buffer_span_sec(request):.3f}, "
            f"window_start={window_start:.3f})"
        )

    def _build_route_buffer_status(
        self,
        request: RouteRequestSpec,
        *,
        status: str,
        message: str,
        **extra: Any,
    ) -> dict[str, Any]:
        payload = {
            "mission_id": request.mission_id,
            "source_mode": request.source_mode,
            "status": status,
            "camera_source": request.camera_source,
            "clip_duration_sec": float(request.clip_duration_sec),
            "frames_received": self.frames_received,
            "buffer_size": len(self.frame_buffer),
            "last_frame_stamp": round(self.last_frame_stamp, 3),
            "last_source_frame_stamp": round(self.last_source_frame_stamp, 3),
            "updated_at": round(self._now(), 3),
            "message": message,
        }
        payload.update(extra)
        return payload

    def _publish_route_buffer_status(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = dumps_json(payload)
        self.route_buffer_status_pub.publish(msg)

    def _window_start(self, request: RouteRequestSpec) -> float:
        return self._now() - float(request.clip_duration_sec)

    def _buffer_span_sec(self, frames: list[tuple[float, Any]]) -> float:
        if len(frames) < 2:
            return 0.0
        return max(0.0, frames[-1][0] - frames[0][0])

    def _required_buffer_span_sec(self, request: RouteRequestSpec) -> float:
        clip_duration = float(request.clip_duration_sec)
        ratio = float(self.get_parameter("live_buffer_ready_ratio").value)
        return min(clip_duration, max(0.5, clip_duration * ratio))

    def _encode_previewable_mp4(self, input_path: Path, output_path: Path) -> None:
        ffmpeg_path = shutil.which("ffmpeg")
        if ffmpeg_path is None:
            shutil.copy2(input_path, output_path)
            return

        command = [
            ffmpeg_path,
            "-y",
            "-i",
            str(input_path),
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_path),
        ]
        try:
            subprocess.run(
                command,
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            self.get_logger().warn(
                f"Failed to encode browser-friendly live buffer mp4, falling back to raw copy: {exc}"
            )
            shutil.copy2(input_path, output_path)

    def _resolve_uri(self, video_uri: str) -> str:
        parsed = urlparse(video_uri)
        if parsed.scheme == "file":
            path = Path(unquote(parsed.path)).expanduser()
            if not path.exists():
                raise OSError(f"Route planner media path does not exist: {path}")
            return str(path)
        if parsed.scheme in {"http", "https"}:
            return video_uri
        path = Path(video_uri).expanduser()
        if not path.exists():
            raise OSError(f"Route planner media path does not exist: {path}")
        return str(path.resolve())

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
