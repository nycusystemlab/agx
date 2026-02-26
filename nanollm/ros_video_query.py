#!/usr/bin/env python3
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge

# 引入必要的 NanoLLM 模組
from nano_llm.agents.video_query import VideoQuery, ChatQuery
from nano_llm.utils import ArgParser, wrap_text

try:
    import jetson_utils  # 用於 cudaImage -> numpy 的轉換（若輸入是 jetson_utils 的 cudaImage）
    from jetson_utils import cudaMemcpy, cudaToNumpy, cudaDeviceSynchronize
except Exception:
    jetson_utils = None
    cudaMemcpy = None
    cudaToNumpy = None
    cudaDeviceSynchronize = None


def _to_numpy_bgr(img):
    """
    將 NanoLLM/jetson_utils 可能送來的影像格式轉為 numpy BGR (uint8) 供 OpenCV 繪字與 ROS 發佈。
    - 若是 jetson_utils.cudaImage: 嘗試 cudaToNumpy()，並處理 RGBA/RGB -> BGR
    - 若已是 numpy: 盡量轉成 BGR
    """
    # 1) jetson_utils cudaImage
    if jetson_utils is not None:
        try:
            # cudaToNumpy 會得到 HxWxC 的 numpy
            np_img = jetson_utils.cudaToNumpy(img)
            # 多半是 RGBA 或 RGB
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = cv2.cvtColor(np_img, cv2.COLOR_RGBA2BGR)
            elif np_img.ndim == 3 and np_img.shape[2] == 3:
                # jetson_utils 常見是 RGB
                np_img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
            return np_img
        except Exception:
            pass

    # 2) numpy array
    if isinstance(img, np.ndarray):
        np_img = img
        if np_img.ndim == 3 and np_img.shape[2] == 4:
            # 假設是 RGBA
            np_img = cv2.cvtColor(np_img, cv2.COLOR_RGBA2BGR)
        elif np_img.ndim == 3 and np_img.shape[2] == 3:
            # 不確定是 RGB 或 BGR；這裡「不強制」轉換，避免顏色顛倒
            # 若你確定來源是 RGB，可改成 cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
            pass
        return np_img

    raise TypeError(f"Unsupported image type: {type(img)}")


def _overlay_text(img_bgr, text, max_width_ratio=0.95):
    """
    將文字以多行方式畫在影像左上角，避免太長溢出。
    """
    if not text:
        return img_bgr

    h, w = img_bgr.shape[:2]
    x, y = 10, 30
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 2
    line_gap = 8

    # 簡易換行：按空白切詞，累積到超過寬度就換行
    words = text.split()
    lines = []
    cur = ""
    for wd in words:
        test = (cur + " " + wd).strip()
        (tw, th), _ = cv2.getTextSize(test, font, font_scale, thickness)
        if tw <= int(w * max_width_ratio):
            cur = test
        else:
            if cur:
                lines.append(cur)
            cur = wd
    if cur:
        lines.append(cur)

    # 畫背景框（半透明效果用實心矩形 + alpha 混合）
    # 先估計最大行寬與總高
    max_tw = 0
    th = 0
    for ln in lines[:6]:  # 最多顯示 6 行，避免整張被蓋住（可自行調整）
        (tw, _th), _ = cv2.getTextSize(ln, font, font_scale, thickness)
        max_tw = max(max_tw, tw)
        th = _th

    box_h = (th + line_gap) * min(len(lines), 6) + 20
    box_w = max_tw + 20
    x1, y1 = 5, 5
    x2, y2 = min(w - 5, x1 + box_w), min(h - 5, y1 + box_h)

    overlay = img_bgr.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    alpha = 0.5
    img_bgr = cv2.addWeighted(overlay, alpha, img_bgr, 1 - alpha, 0)

    # 畫文字（白字）
    yy = y
    for ln in lines[:6]:
        cv2.putText(img_bgr, ln, (x, yy), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        yy += th + line_gap

    return img_bgr


class NanoLLMRosNode(Node):
    def __init__(self):
        super().__init__('nano_llm_video_query_ros')

        self.desc_pub = self.create_publisher(String, '/nano_llm/description', 10)
        self.img_pub = self.create_publisher(Image, '/nano_llm/annotated_image', 10)
        
        # 新增：訂閱 prompt 修改請求
        self.prompt_sub = self.create_subscription(
            String,
            '/nano_llm/set_prompt',
            self._on_prompt_received,
            10
        )
        
        # 新增：發布當前 prompt
        self.prompt_pub = self.create_publisher(String, '/nano_llm/current_prompt', 10)
        
        # 新增：訂閱辨識頻率設定（單位：秒，例如 0.5 表示每 0.5 秒辨識一次）
        self.inference_interval_sub = self.create_subscription(
            String,
            '/nano_llm/set_inference_interval',
            self._on_inference_interval_received,
            10
        )
        
        # 新增：訂閱是否顯示 overlay 的設定
        self.show_overlay_sub = self.create_subscription(
            String,
            '/nano_llm/set_show_overlay',
            self._on_show_overlay_received,
            10
        )

        self.bridge = CvBridge()

        self._latest_text = ""
        self._current_prompt = ""
        self._new_prompt = None  # 用於儲存待更新的 prompt
        self._inference_interval = 1.0  # 預設每 1 秒辨識一次
        self._new_inference_interval = None
        self._show_overlay = True  # 預設顯示 overlay
        self._new_show_overlay = None
        self._lock = threading.Lock()

        self.get_logger().info(
            "NanoLLM ROS Node Initialized.\n"
            "Publishing:\n"
            "  - /nano_llm/description (std_msgs/String)\n"
            "  - /nano_llm/annotated_image (sensor_msgs/Image)\n"
            "  - /nano_llm/current_prompt (std_msgs/String)\n"
            "Subscribing:\n"
            "  - /nano_llm/set_prompt (std_msgs/String) - Send new prompt here\n"
            "  - /nano_llm/set_inference_interval (std_msgs/String) - Set inference interval in seconds\n"
            "  - /nano_llm/set_show_overlay (std_msgs/String) - 'true'/'false' to show/hide prompt text (output always shown)\n"
        )

    def _on_prompt_received(self, msg: String):
        """處理收到的新 prompt"""
        new_prompt = msg.data.strip()
        if new_prompt:
            with self._lock:
                self._new_prompt = new_prompt
            self.get_logger().info(f"Received new prompt: {new_prompt}")

    def _on_show_overlay_received(self, msg: String):
        """處理是否顯示 overlay 的設定"""
        value = msg.data.strip().lower()
        if value in ('true', '1', 'yes', 'on', 'show'):
            with self._lock:
                self._new_show_overlay = True
            self.get_logger().info("Received: show overlay = True")
        elif value in ('false', '0', 'no', 'off', 'hide'):
            with self._lock:
                self._new_show_overlay = False
            self.get_logger().info("Received: show overlay = False")

    def _on_inference_interval_received(self, msg: String):
        """處理收到的辨識間隔設定"""
        try:
            interval = float(msg.data.strip())
            if interval > 0:
                with self._lock:
                    self._new_inference_interval = interval
                self.get_logger().info(f"Received new inference interval: {interval}s")
        except ValueError:
            self.get_logger().warn(f"Invalid inference interval: {msg.data}")

    def get_new_prompt(self) -> str:
        """取得待更新的 prompt（取完即清除）"""
        with self._lock:
            prompt = self._new_prompt
            self._new_prompt = None
            return prompt

    def get_new_inference_interval(self) -> float:
        """取得待更新的辨識間隔（取完即清除）"""
        with self._lock:
            interval = self._new_inference_interval
            self._new_inference_interval = None
            return interval

    def get_new_show_overlay(self):
        """取得待更新的 show_overlay 設定（取完即清除）"""
        with self._lock:
            value = self._new_show_overlay
            self._new_show_overlay = None
            return value

    def get_show_overlay(self) -> bool:
        """取得目前是否顯示 overlay"""
        with self._lock:
            return self._show_overlay

    def set_show_overlay(self, show: bool):
        """設定是否顯示 overlay"""
        with self._lock:
            self._show_overlay = show

    def get_inference_interval(self) -> float:
        """取得目前的辨識間隔"""
        with self._lock:
            return self._inference_interval

    def set_inference_interval(self, interval: float):
        """設定辨識間隔"""
        with self._lock:
            self._inference_interval = interval

    def set_current_prompt(self, prompt: str):
        """設定並發布當前 prompt"""
        with self._lock:
            self._current_prompt = prompt
        # 發布當前 prompt
        msg = String()
        msg.data = prompt
        self.prompt_pub.publish(msg)

    def set_latest_text(self, text: str):
        with self._lock:
            self._latest_text = text

    def get_latest_text(self) -> str:
        with self._lock:
            return self._latest_text

    def publish_result(self, text: str):
        msg = String()
        msg.data = text
        self.desc_pub.publish(msg)

    def publish_annotated_image(self, cv_bgr: np.ndarray):
        # 這裡統一用 bgr8 發佈
        img_msg = self.bridge.cv2_to_imgmsg(cv_bgr, encoding='bgr8')
        self.img_pub.publish(img_msg)


def main():
    rclpy.init()
    ros_node = NanoLLMRosNode()

    # 解析參數（加入 'web' 啟用 Web UI，'nanodb' 啟用向量資料庫）
    parser = ArgParser(extras=ArgParser.Defaults + ['video_input', 'video_output', 'web', 'nanodb'])
    
    # 新增自訂參數：辨識間隔
    parser.add_argument('--inference-interval', type=float, default=1.0,
                        help='Interval between inferences in seconds (default: 1.0)')
    
    # 新增自訂參數：是否顯示 overlay（prompt 和回應文字）
    parser.add_argument('--show-overlay', type=str, default='true',
                        help='Show text overlay on video (true/false, default: true)')

    try:
        args = parser.parse_args()
    except SystemExit:
        ros_node.destroy_node()
        rclpy.shutdown()
        return
    
    # 設定初始辨識間隔
    inference_interval = getattr(args, 'inference_interval', 1.0)
    ros_node.set_inference_interval(inference_interval)
    
    # 設定初始 show_overlay
    show_overlay_str = getattr(args, 'show_overlay', 'true').lower()
    show_overlay = show_overlay_str in ('true', '1', 'yes', 'on')
    ros_node.set_show_overlay(show_overlay)

    print(f"\n[Info] Loading VideoQuery Agent...")
    print(f"Model: {args.model}")
    print(f"Input: {args.video_input}")
    print(f"Inference interval: {inference_interval}s")
    print(f"Show overlay: {show_overlay}")

    # -----------------------------------------------------------
    # [絕對修正] 參數淨化 (Parameter Sanitization)
    # 移除可能與 VideoQuery/ChatQuery 衝突的參數
    # -----------------------------------------------------------
    agent_kwargs = vars(args).copy()  # 使用 copy 避免修改原始 args

    # 移除自訂參數和可能衝突的參數
    keys_to_remove = ['warmup', 'inference_interval', 'show_overlay']
    for key in keys_to_remove:
        if key in agent_kwargs:
            del agent_kwargs[key]

    # -----------------------------------------------------------
    # 建立自訂的 VideoQuery 子類別，覆寫 on_video 方法
    # 只修改 overlay 顯示邏輯，保持原始推論流程
    # -----------------------------------------------------------
    class CustomVideoQuery(VideoQuery):
        def __init__(self, ros_node_ref, **kwargs):
            self._ros_node = ros_node_ref
            super().__init__(**kwargs)
        
        def on_video(self, image):
            """
            覆寫原始的 on_video：
            - 保持原始推論邏輯不變
            - 只修改 overlay 顯示（可選擇是否顯示 prompt）
            """
            if self.pause_video:
                if not self.pause_image:
                    self.pause_image = cudaMemcpy(image)
                image = cudaMemcpy(self.pause_image)
            
            # 原始推論邏輯，不做任何修改
            if self.auto_refresh or self.prompt != self.last_prompt or self.rag_prompt != self.rag_prompt_last:
                np_image = cudaToNumpy(image)
                cudaDeviceSynchronize()
                
                if self.rag_prompt:
                    prompt = self.rag_prompt + '. ' + self.prompt
                else:
                    prompt = self.prompt
                
                self.llm(['/reset', np_image, prompt])
                
                self.last_prompt = self.prompt
                self.rag_prompt_last = self.rag_prompt
                
                if self.db:
                    if self.db_share_embed:
                        pass
                    else:
                        self.last_image = cudaMemcpy(image)

            # 繪製文字 overlay
            text = self.text.replace('\n', '').replace('</s>', '').strip()
            y = 5
            
            # 只有在 show_overlay=True 時才繪製 prompt 和 RAG 文字
            if self._ros_node.get_show_overlay():
                if self.rag_prompt:
                    y = wrap_text(self.font, image, text='RAG: ' + self.rag_prompt, x=5, y=y, color=(255,172,28), background=self.font.Gray40)
                    
                y = wrap_text(self.font, image, text=self.prompt, x=5, y=y, color=(120,215,21), background=self.font.Gray40)

            # 辨識結果文字始終顯示
            if text:
                y = wrap_text(self.font, image, text=text, x=5, y=y, color=self.font.White, background=self.font.Gray40)
            
            self.video_output(image)

    # 建立 Agent (使用自訂子類別)
    try:
        agent = CustomVideoQuery(ros_node_ref=ros_node, **agent_kwargs)
    except Exception as e:
        print(f"\n[Error] Failed to load VideoQuery: {e}")
        print(f"[Error] Details: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # 定義結果回調（final text）
    def on_final_result(result):
        text = ""
        if hasattr(result, 'text'):
            text = result.text
        elif isinstance(result, str):
            text = result
        elif isinstance(result, list):
            text = " ".join([str(x) for x in result])

        text = text.replace('</s>', '').strip()

        if text:
            ros_node.set_latest_text(text)
            ros_node.publish_result(text)
            print(f">> {text}")

    # 綁定輸出
    if hasattr(agent, 'llm'):
        agent.llm.add(on_final_result, channel=ChatQuery.OutputFinal)
    else:
        print("[Error] Agent structure mismatch: 'agent.llm' not found.")
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # -----------------------------------------------------------
    # 新增：從 video_source 接一個 frame callback
    # 在這裡把最新的文字疊到影像上，並 publish 到 /nano_llm/annotated_image
    # -----------------------------------------------------------
    last_inference_time = [0.0]  # 用 list 來做 mutable closure
    
    def on_video_frame(frame):
        try:
            img_bgr = _to_numpy_bgr(frame)
            text = ros_node.get_latest_text()
            # img_bgr = _overlay_text(img_bgr, text)
            ros_node.publish_annotated_image(img_bgr)
        except Exception as e:
            # 避免刷屏：需要的話你可改成 debug 或加節流
            ros_node.get_logger().warn(f"Failed to publish annotated image: {e}")

    # 這裡假設 video_source 的主要輸出 channel=0 是影像 frame（多數 plugin 預設主輸出為 0）
    if hasattr(agent, 'video_source'):
        try:
            agent.video_source.add(on_video_frame, channel=0)
        except TypeError:
            # 部分版本 add() 可能不接受 named arg channel
            agent.video_source.add(on_video_frame, 0)
    else:
        print("[Error] Agent structure mismatch: 'agent.video_source' not found.")
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # 啟動
    agent.start()
    
    # 設定初始 auto_refresh = False，改由我們的 timer 控制
    if hasattr(agent, 'auto_refresh'):
        agent.auto_refresh = False
    
    # 發布初始 prompt
    initial_prompt = agent.prompt if hasattr(agent, 'prompt') else "Describe the image concisely."
    ros_node.set_current_prompt(initial_prompt)
    
    print("\n" + "="*60)
    print("[Info] Agent running!")
    print("="*60)
    print(f"[Info] WebRTC Video Output: http://<Jetson_IP>:8554/output")
    print(f"[Info] Web UI (built-in):   https://<Jetson_IP>:8050")
    print("="*60)
    print(f"[Info] Current prompt: {initial_prompt}")
    print(f"[Info] Inference interval: {ros_node.get_inference_interval()}s")
    print(f"[Info] Show overlay: {ros_node.get_show_overlay()}")
    print("-"*60)
    print("[Info] ROS 2 Topics:")
    print("  - /nano_llm/set_prompt - Change prompt")
    print("  - /nano_llm/set_inference_interval - Set interval (seconds)")
    print("  - /nano_llm/set_show_overlay - 'true'/'false'")
    print("-"*60)
    print("Press Ctrl+C to exit.")

    # -----------------------------------------------------------
    # 使用 timer 來檢查是否有新的 prompt 需要更新
    # 同時控制辨識頻率
    # -----------------------------------------------------------
    last_trigger_time = [time.time()]
    
    def check_updates():
        current_time = time.time()
        
        # 檢查是否有新 prompt
        new_prompt = ros_node.get_new_prompt()
        if new_prompt and hasattr(agent, 'prompt'):
            # 重置 agent 的文字輸出
            if hasattr(agent, 'text'):
                agent.text = ""
            
            agent.prompt = new_prompt
            ros_node.set_current_prompt(new_prompt)
            print(f"[Info] Prompt updated to: {new_prompt}")
            
            # prompt 改變時立即觸發推論
            if hasattr(agent, 'auto_refresh'):
                agent.auto_refresh = True
            last_trigger_time[0] = current_time
        
        # 檢查是否有新的辨識間隔設定
        new_interval = ros_node.get_new_inference_interval()
        if new_interval:
            ros_node.set_inference_interval(new_interval)
            print(f"[Info] Inference interval updated to: {new_interval}s")
        
        # 檢查是否有新的 show_overlay 設定
        new_show_overlay = ros_node.get_new_show_overlay()
        if new_show_overlay is not None:
            ros_node.set_show_overlay(new_show_overlay)
            if new_show_overlay:
                print("[Info] Overlay enabled")
            else:
                print("[Info] Overlay disabled")
        
        # 控制辨識頻率：只有間隔夠長時才觸發 auto_refresh
        interval = ros_node.get_inference_interval()
        if current_time - last_trigger_time[0] >= interval:
            if hasattr(agent, 'auto_refresh'):
                agent.auto_refresh = True
            last_trigger_time[0] = current_time
        else:
            # 間隔不夠，關閉 auto_refresh
            if hasattr(agent, 'auto_refresh'):
                agent.auto_refresh = False
    
    # 每 50ms 檢查一次
    update_timer = ros_node.create_timer(0.05, check_updates)

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[Info] Shutting down...")
        update_timer.cancel()
        try:
            agent.stop()
        except Exception:
            pass
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
