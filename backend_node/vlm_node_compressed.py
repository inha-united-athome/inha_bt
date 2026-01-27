#!/usr/bin/env python3
import time
import json
import os
import glob
import base64
import requests
from io import BytesIO
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from inha_interfaces.action import Vlm

ACTION_NAME = "/vlm/query"
IMAGE_TOPIC = "/camera/camera/color/image_raw/compressed"   # 필요하면 여기만 바꾸면 됨


class VLMActionServer(Node):
    """
    규칙 (고정)
    - mode 1: text + prompt  -> LLM text only
    - mode 2: live image(compressed) + prompt (+ optional text) -> LLM vision
    - mode 3: context_json (+ optional prompt) -> task routing (최소 people_intro 지원)
    """

    def __init__(self):
        super().__init__("vlm_action_server")

        # ---- image (CompressedImage) ----
        self.bridge = CvBridge()
        self.latest_image_bgr = None   # cv2 bgr 이미지
        self.last_img_stamp = None     # ROS stamp (builtin_interfaces/Time)
        self.last_img_time = 0.0       # time.time() 기준 수신 시각
        self.last_img_bytes = 0
        self.last_img_format = ""
        self.img_count = 0
        self._fps_t0 = time.time()
        self._fps_cnt = 0
        self.img_fps = 0.0

        # QoS: 카메라 퍼블리셔가 보통 best_effort(sensor_data)라서 이걸로 맞추는게 안전
        self.create_subscription(
            CompressedImage,
            IMAGE_TOPIC,
            self.image_cb,
            qos_profile_sensor_data
        )

        # ---- debug topic ----
        self.debug_pub = self.create_publisher(String, "/vlm/debug", 10)
        self.create_timer(1.0, self.timer_debug)  # 1초마다 수신 상태 로그

        # ---- LLM ----
        self.llm_api_url = "http://192.168.50.189:11111"
        self.llm_api_key = "ollama"
        self.llm_api_model_name = "llava:7b"

        # Busy (동시 goal 방지)
        self._busy = False

        # ActionServer
        self._as = ActionServer(
            self,
            Vlm,
            ACTION_NAME,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"[VLM] Action Server started: {ACTION_NAME}")
        self.get_logger().info(f"[VLM] Subscribing: {IMAGE_TOPIC} (CompressedImage, sensor_data QoS)")

    # ------------------ Debug timer ------------------
    def timer_debug(self):
        # 최근 이미지 수신 상태를 주기적으로 찍어서 "정말 받고 있는지" 눈으로 확인
        now = time.time()
        age_ms = (now - self.last_img_time) * 1000.0 if self.last_img_time > 0 else -1.0

        msg = String()
        msg.data = (
            f"img_count={self.img_count} fps={self.img_fps:.1f} "
            f"age_ms={age_ms:.0f} bytes={self.last_img_bytes} fmt='{self.last_img_format}'"
        )
        self.debug_pub.publish(msg)

        # 너무 시끄럽지 않게 info는 1초마다
        self.get_logger().info("[VLM] " + msg.data)

    # ------------------ ROS ------------------
    def image_cb(self, msg: CompressedImage):
        try:
            # 메타 업데이트
            self.img_count += 1
            self.last_img_stamp = msg.header.stamp
            self.last_img_time = time.time()
            self.last_img_bytes = len(msg.data)
            self.last_img_format = msg.format or ""

            # FPS 계산
            self._fps_cnt += 1
            dt = self.last_img_time - self._fps_t0
            if dt >= 1.0:
                self.img_fps = self._fps_cnt / dt
                self._fps_cnt = 0
                self._fps_t0 = self.last_img_time

            # CompressedImage -> cv2(bgr8)
            self.latest_image_bgr = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

        except Exception as e:
            self.get_logger().warn(f"[VLM] image convert failed: {e}")

    # ------------------ Action callbacks ------------------
    def goal_cb(self, goal_req: Vlm.Goal) -> GoalResponse:
        if self._busy:
            self.get_logger().warn("[VLM] Rejecting goal: busy")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[VLM] Goal: mode={goal_req.mode} timeout_ms={goal_req.timeout_ms} "
            f"text_len={len(goal_req.text)} prompt_len={len(goal_req.prompt)} ctx_len={len(goal_req.context_json)}"
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("[VLM] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self._busy = True

        g: Vlm.Goal = goal_handle.request
        mode = int(g.mode)
        text = g.text or ""
        prompt = g.prompt or ""
        ctx = g.context_json or ""
        timeout_ms = int(g.timeout_ms) if g.timeout_ms is not None else 0

        start = time.time()

        def check_cancel_timeout():
            if goal_handle.is_cancel_requested:
                raise RuntimeError("canceled")
            if timeout_ms > 0 and (time.time() - start) * 1000.0 > timeout_ms:
                raise RuntimeError("timeout")

        fb = Vlm.Feedback()
        res = Vlm.Result()

        try:
            fb.state, fb.progress = "started", 0.0
            goal_handle.publish_feedback(fb)
            check_cancel_timeout()

            if mode == 1:
                fb.state, fb.progress = "mode1_text", 0.2
                goal_handle.publish_feedback(fb)
                caption = self.llm_text(text, prompt)

            elif mode == 2:
                # mode2 핵심 디버그: "이미지를 받고 있나?"
                fb.state, fb.progress = "mode2_wait_image", 0.2
                goal_handle.publish_feedback(fb)

                # (1) 이미지 대기: 최대 wait_ms 동안만 기다림 (무한대기 방지)
                wait_ms = 3000  # 필요하면 늘려
                t0 = time.time()
                while self.latest_image_bgr is None:
                    check_cancel_timeout()
                    if (time.time() - t0) * 1000.0 > wait_ms:
                        raise RuntimeError(
                            f"no_image_received_within_{wait_ms}ms "
                            f"(check topic name/QoS/domain)."
                        )
                    time.sleep(0.02)

                # (2) stale 체크: "최근 프레임이 너무 오래전이면" 실패/대기
                stale_ms = 1500
                age_ms = (time.time() - self.last_img_time) * 1000.0
                if age_ms > stale_ms:
                    raise RuntimeError(
                        f"image_is_stale age_ms={age_ms:.0f} > {stale_ms}ms "
                        f"(publisher stopped? network? QoS?)"
                    )

                self.get_logger().info(
                    f"[VLM][mode2] got image: age_ms={age_ms:.0f}, bytes={self.last_img_bytes}, "
                    f"fmt='{self.last_img_format}', fps={self.img_fps:.1f}"
                )

                fb.state, fb.progress = "mode2_vision", 0.5
                goal_handle.publish_feedback(fb)

                caption = self.llm_live_image(self.latest_image_bgr, prompt, text)

            elif mode == 3:
                fb.state, fb.progress = "mode3_people_intro", 0.2
                goal_handle.publish_feedback(fb)

                try:
                    data = json.loads(ctx) if ctx else {}
                except json.JSONDecodeError as e:
                    raise RuntimeError(f"invalid context_json: {e}")

                caption = self.people_intro_from_context(data, text, prompt)

            else:
                raise RuntimeError(f"unsupported mode: {mode}")

            check_cancel_timeout()

            res.success = True
            res.generated_text = caption
            res.result_json = ""
            res.error_message = ""

            fb.state, fb.progress = "done", 1.0
            goal_handle.publish_feedback(fb)

            goal_handle.succeed()
            return res

        except Exception as e:
            msg = str(e)
            self.get_logger().error(f"[VLM] execute failed: {msg}")

            if msg == "canceled":
                goal_handle.canceled()
            else:
                goal_handle.abort()

            res.success = False
            res.generated_text = ""
            res.result_json = ""
            res.error_message = msg
            return res

        finally:
            self._busy = False

    # ======================================================
    # LLM HTTP
    # ======================================================
    def call_llm(self, payload: dict) -> str:
        url = self.llm_api_url.rstrip("/") + "/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.llm_api_key}",
        }

        t0 = time.time()
        try:
            self.get_logger().info(f"[LLM] POST {url} model={payload.get('model')}")

            # (중요) connect/read timeout 분리
            r = requests.post(url, headers=headers, json=payload, timeout=(3, 60))
            dt = (time.time() - t0) * 1000.0
            self.get_logger().info(f"[LLM] HTTP {r.status_code} in {dt:.0f}ms, len={len(r.text)}")

            # 에러면 body도 찍어서 서버가 뭐라하는지 확인
            if r.status_code != 200:
                self.get_logger().error(f"[LLM] non-200 body: {r.text[:500]}")
                r.raise_for_status()

            j = r.json()

            # (중요) 응답 포맷 다양하게 대응 + 진단 로그
            out = None
            if isinstance(j, dict):
                # OpenAI 스타일
                try:
                    out = j["choices"][0]["message"]["content"]
                except Exception:
                    pass

                # Ollama 스타일(많이 나옴)
                if out is None:
                    try:
                        out = j["message"]["content"]
                    except Exception:
                        pass

                # 다른 흔한 키
                if out is None:
                    out = j.get("response") or j.get("content")

            if not out:
                self.get_logger().error(f"[LLM] unexpected json keys={list(j.keys()) if isinstance(j, dict) else type(j)}")
                self.get_logger().error(f"[LLM] json head: {str(j)[:800]}")
                raise RuntimeError("llm_bad_response_format")

            return (out or "").replace("\n", " ").strip()

        except Exception as e:
            dt = (time.time() - t0) * 1000.0
            raise RuntimeError(f"llm_call_failed after {dt:.0f}ms: {e}")

    # ======================================================
    # mode 1: text + prompt
    # ======================================================
    def llm_text(self, text: str, prompt: str) -> str:
        if not prompt:
            prompt = "Answer concisely."

        system = (
            "You will receive an utterance and an instruction. "
            "Follow the instruction strictly and output only the answer."
        )
        user = f"UTTERANCE: {text}\nINSTRUCTION: {prompt}"

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "max_tokens": 120,
            "temperature": 0.1,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload)

    # ======================================================
    # mode 2: live image + prompt (+ optional text)
    # ======================================================
    def llm_live_image(self, bgr_img, prompt: str, text: str = "") -> str:
        if not prompt:
            prompt = "Describe the image in one short sentence."

        user_text = prompt if not text else f"{prompt}\nTEXT: {text}"

        pil = PILImage.fromarray(bgr_img[:, :, ::-1])  # BGR->RGB
        buf = BytesIO()
        pil.save(buf, format="JPEG")
        b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": "Answer concisely in English."},
                {"role": "user", "content": [
                    {"type": "text", "text": user_text},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ]},
            ],
            "max_tokens": 200,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload)

    # ======================================================
    # mode 3: context_json routing (최소)
    # ======================================================
    def people_intro_from_context(self, data: dict, text: str, prompt: str) -> str:
        saved_dir = (data.get("saved_dir") or "").strip()
        person_name = (data.get("person_name") or "").strip() or "the guest"
        favorite_drink = (data.get("favorite_drink") or "").strip()

        b64 = self.load_latest_image_b64(saved_dir)

        if not prompt:
            prompt = (
                "Introduce this person to another guest who is listening. "
                "Use the name and (if provided) favorite drink, and include one short visual trait from the face. "
                "Output EXACTLY ONE natural English sentence. Do not mention photo or image."
            )

        extra = (text or "").strip()

        system = (
            "You are a robot host at a party. "
            "You will be shown a face-only photo of a person and given their name and favorite drink. "
            "Your job is to introduce that person to another guest who is listening. "
            "Output EXACTLY ONE natural English sentence. "
            "Do not mention 'photo' or 'image'. "
            "Do not guess age, nationality, or ethnicity."
        )

        info_lines = [f"NAME: {person_name}"]
        if favorite_drink:
            info_lines.append(f"FAVORITE_DRINK: {favorite_drink}")
        if extra:
            info_lines.append(f"EXTRA_TEXT: {extra}")

        user_text = "\n".join(info_lines) + "\nINSTRUCTION: " + prompt

        user_content = [{"type": "text", "text": user_text}]
        if b64:
            user_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user_content},
            ],
            "max_tokens": 80,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }

        out = self.call_llm(payload).strip()

        # 안전장치: 혹시 2문장 나오면 첫 문장만
        if "." in out:
            out = out.split(".")[0].strip() + "."
        return out

    def load_latest_image_b64(self, saved_dir: str) -> str:
        if not saved_dir or not os.path.exists(saved_dir):
            self.get_logger().warn(f"[VLM] Directory not found: {saved_dir}")
            return ""

        files = glob.glob(os.path.join(saved_dir, "*.jpg"))
        if not files:
            self.get_logger().warn(f"[VLM] No jpg files in: {saved_dir}")
            return ""

        latest_file = max(files, key=os.path.getmtime)

        try:
            with open(latest_file, "rb") as f:
                return base64.b64encode(f.read()).decode("utf-8")
        except Exception as e:
            self.get_logger().error(f"[VLM] Failed to encode image: {e}")
            return ""


def main(args=None):
    rclpy.init(args=args)
    node = VLMActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
