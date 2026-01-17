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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inha_interfaces.action import Vlm

ACTION_NAME = "/vlm/query"
IMAGE_TOPIC = "/camera/camera/color/image_raw"


class VLMActionServer(Node):
    """
    규칙 (고정)
    - mode 1: text + prompt  -> LLM text only
    - mode 2: live image + prompt (+ optional text) -> LLM vision
    - mode 3: context_json (+ optional prompt) -> task routing (최소 people_intro 지원)
    """

    def __init__(self):
        super().__init__("vlm_action_server")

        # ---- image ----
        self.bridge = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, 10)

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

    # ------------------ ROS ------------------
    def image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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
                fb.state, fb.progress = "mode2_wait_image", 0.2
                goal_handle.publish_feedback(fb)

                while self.latest_image is None:
                    check_cancel_timeout()
                    time.sleep(0.02)

                fb.state, fb.progress = "mode2_vision", 0.5
                goal_handle.publish_feedback(fb)

                caption = self.llm_live_image(self.latest_image, prompt, text)

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
        try:
            r = requests.post(url, headers=headers, json=payload, timeout=60)
            r.raise_for_status()
            j = r.json()
            out = j["choices"][0]["message"]["content"]
            return (out or "").replace("\n", " ").strip()
        except Exception as e:
            # 서버 쪽에서도 실패를 "문자열"로 반환하면 디버깅이 쉬움
            raise RuntimeError(f"llm_call_failed: {e}")

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

        # text는 optional 파라미터로만 붙임
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
        """
        context_json:
        {
        "saved_dir": "/home/thor/face_ws/src/face_data/minho",
        "person_name": "minho",
        "favorite_drink": "cola"
        }

        Returns: exactly ONE natural English sentence introducing the person
        to someone else (not the person in the photo).
        """

        saved_dir = (data.get("saved_dir") or "").strip()
        person_name = (data.get("person_name") or "").strip() or "the guest"
        favorite_drink = (data.get("favorite_drink") or "").strip()

        b64 = self.load_latest_image_b64(saved_dir)  # 없으면 "" 반환한다고 가정

        # prompt 없으면 기본 prompt 사용
        if not prompt:
            prompt = (
                "Introduce this person to another guest who is listening. "
                "Use the name and (if provided) favorite drink, and include one short visual trait from the face. "
                "Output EXACTLY ONE natural English sentence. Do not mention photo or image."
            )

        # text는 optional extra context (예: 상황/장소/톤)
        extra = (text or "").strip()

        # ---- system/user 구성 (한 번에 intro 생성) ----
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

        # 폴더 내의 모든 jpg 파일 목록 가져오기
        files = glob.glob(os.path.join(saved_dir, "*.jpg"))
        if not files:
            self.get_logger().warn(f"[VLM] No jpg files in: {saved_dir}")
            return ""

        # 가장 최근에 수정된 파일 찾기
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
