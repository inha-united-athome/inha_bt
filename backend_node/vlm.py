import time
import json
import os, glob, base64, requests, logging
from io import BytesIO
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inha_interfaces.action import Vlm  

log = logging.getLogger("VLMActionServer")
log.setLevel(logging.INFO)

ACTION_NAME = "/vlm/query"


class VLMActionServer(Node):
    def __init__(self):
        super().__init__("vlm_action_server")

        self.bridge = CvBridge()
        self.latest_image = None
        self._img_mx = False  # 간단히; 필요하면 threading.Lock 쓰면 됨

        # LLM API 세팅 (기존 유지)
        self.llm_api_url = "http://192.168.50.189:11111"
        self.llm_api_key = "ollama"
        self.llm_api_model_name = "llava:7b"

        # 카메라 구독(기존 유지)
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        # Busy (동시 goal 방지)
        self._busy = False

        # ✅ 액션 서버 생성 (이 이름이 곧 "서버 액션 이름")
        self._as = ActionServer(
            self,
            Vlm,
            ACTION_NAME,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"VLM Action Server started: {ACTION_NAME}")

    # ------------------ ROS 콜백 ------------------
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    # ------------------ Action callbacks ------------------
    def goal_cb(self, goal_req: Vlm.Goal) -> GoalResponse:
        if self._busy:
            self.get_logger().warn("Rejecting goal: server is busy")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Goal received: mode={goal_req.mode}, timeout_ms={goal_req.timeout_ms}, "
            f"prompt_len={len(goal_req.prompt)}, ctx_len={len(goal_req.context_json)}"
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self._busy = True
        goal: Vlm.Goal = goal_handle.request

        mode = int(goal.mode)
        prompt = goal.prompt or ""
        context_json = goal.context_json or ""
        timeout_ms = int(goal.timeout_ms) if goal.timeout_ms is not None else 0

        start = time.time()

        fb = Vlm.Feedback()
        fb.state = "started"
        fb.progress = 0.0
        goal_handle.publish_feedback(fb)

        res = Vlm.Result()
        try:
            # cancel/timeout 체크 헬퍼
            def check_cancel_timeout():
                if goal_handle.is_cancel_requested:
                    raise RuntimeError("canceled")
                if timeout_ms > 0:
                    if (time.time() - start) * 1000.0 > timeout_ms:
                        raise RuntimeError("timeout")

            check_cancel_timeout()

            if mode == 1:
                fb.state = "text_processing"
                fb.progress = 0.1
                goal_handle.publish_feedback(fb)

                caption = self.process_text(prompt)

            elif mode in (2, 3):
                fb.state = "waiting_image"
                fb.progress = 0.1
                goal_handle.publish_feedback(fb)

                # 이미지가 필요하면 기다림(타임아웃/캔슬 반영)
                while self.latest_image is None:
                    check_cancel_timeout()
                    time.sleep(0.02)

                fb.state = "image_processing"
                fb.progress = 0.3
                goal_handle.publish_feedback(fb)

                caption = self.process_image(mode, prompt)

            elif mode == 4:
                fb.state = "people_intro"
                fb.progress = 0.2
                goal_handle.publish_feedback(fb)

                # context_json에 person1/person2 들어오는 걸 기대
                try:
                    data = json.loads(context_json) if context_json else {}
                except json.JSONDecodeError as e:
                    raise RuntimeError(f"invalid context_json: {e}")

                caption = self._process_people_intro(data)

            else:
                raise RuntimeError(f"unsupported mode: {mode}")

            check_cancel_timeout()

            res.success = True
            res.generated_text = caption
            res.result_json = ""
            res.error_message = ""

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

    # ------------------ 기존 로직을 "함수화"해서 재사용 ------------------
    def process_text(self, text_prompt: str) -> str:
        if not text_prompt:
            return "Text prompt is not set."

        system_prompt = (
            "You are a word extractor. Explain only the prompt below clearly and concisely in English. "
            "You will receive a sentence and an additional instruction describing what to extract or answer from that sentence. "
            "Follow the instruction strictly and output only the required word or phrase. "
            "For example: "
            "'My name is Alisa' and 'Only get name' -> Answer: 'Alisa'. "
            "'I like the cola most' and 'Get his favorite drink' -> Answer: 'cola'."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": text_prompt},
        ]

        payload = {
            "model": self.llm_api_model_name,
            "messages": messages,
            "max_tokens": 120,
            "temperature": 0.1,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload).strip()

    def process_image(self, mode: int, text_prompt: str) -> str:
        if self.latest_image is None:
            return "이미지를 수신하지 못했습니다."

        # mode별 프롬프트 구성
        if mode == 2:
            drink_name = (text_prompt or "drink").strip()

            system_prompt = (
                "You are an AI assistant who guides a customer to the location of their ordered drink shown in the image. "
                f"Look at the image and tell the customer where the {drink_name} is located. "
                "Answer in short, polite English, similar in length and style to: "
                "'Thank you for waiting. The cola is on the table next to the menu order paper.' "
                "Describe where the drink is relative to clear objects in the scene (table, counter, tray...). "
                "Do not refer directly to the customer (do not use 'you' or 'your'). "
                "At the end, append exactly once: "
                "' Please follow me and try to stay close behind me.' "
                "Keep it concise, 1-2 sentences."
            )
            user_prompt = f"Where is the {drink_name} in the image?"

        elif mode == 3:
            system_prompt = (
                "You are an AI assistant who guides a guest to an available empty seat shown in the image. "
                "Answer in short, polite English, similar to: "
                "'Thank you for waiting. An empty seat is available at the table next to the window.' "
                "Describe where the empty seat is relative to clear objects. "
                "Do not refer directly to the guest (do not use 'you' or 'your'). "
                "Keep it concise, 1-2 sentences."
            )
            user_prompt = "Where is an available empty seat in the image?"

        else:
            return f"unsupported image mode: {mode}"

        # 이미지 -> base64
        pil_image = PILImage.fromarray(self.latest_image[:, :, ::-1])  # BGR->RGB
        buffer = BytesIO()
        pil_image.save(buffer, format="JPEG")
        base64_image = base64.b64encode(buffer.getvalue()).decode("utf-8")

        messages = [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}},
                ],
            },
        ]

        payload = {
            "model": self.llm_api_model_name,
            "messages": messages,
            "max_tokens": 200,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload).strip()

    # ------------------ LLM 호출 ------------------
    def call_llm(self, json_payload):
        full_api_url = self.llm_api_url
        if not full_api_url.endswith("/"):
            full_api_url += "/"

        # LM Studio / Ollama 호환
        full_api_url += "v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.llm_api_key}",
        }

        try:
            response = requests.post(full_api_url, headers=headers, json=json_payload, timeout=60)
            response.raise_for_status()
            completion_json = response.json()
            result = completion_json["choices"][0]["message"]["content"]
            return result.replace("\n", " ")
        except Exception as e:
            return f"오류: API 호출 실패 ({e})"

    # ------------------ mode=4 (people intro) 기존 로직 유지 ------------------
    def _process_people_intro(self, data: dict) -> str:
        person1 = data.get("person1") or {}
        person2 = data.get("person2") or {}

        if not person1 and not person2:
            return ""

        if person1 and person2:
            return self._introduce_two_people_vlm(person1, person2)

        only = person1 if person1 else person2
        return self._introduce_one_person_vlm(only)

    def _introduce_two_people_vlm(self, p1: dict, p2: dict) -> str:
        id1 = p1.get("person_id", 999999)
        id2 = p2.get("person_id", 999999)
        first, second = (p1, p2) if id1 <= id2 else (p2, p1)

        first_name   = first.get("name", "the first guest")
        second_name  = second.get("name", "the second guest")
        first_drink  = first.get("favorite_drink", "")
        second_drink = second.get("favorite_drink", "")

        desc_first  = self._describe_person_with_vlm(first)
        desc_second = self._describe_person_with_vlm(second)

        parts = []
        parts.append(
            f"Hello again! Let's take a moment to introduce our guests. The guest who arrived first is {first_name}. "
            f"{desc_first}"
        )
        if first_drink:
            parts.append(f"{first_name} usually enjoys {first_drink}.")
        parts.append(f"The guest who arrived later is {second_name}. {desc_second}")
        if second_drink:
            parts.append(f"{second_name} usually enjoys {second_drink}.")

        common_line = self._describe_common_traits_with_vlm(first, second)
        if common_line:
            parts.append(common_line)

        return " ".join(p.strip() for p in parts if p.strip())

    def _introduce_one_person_vlm(self, person: dict) -> str:
        name = person.get("name", "the guest")
        drink = person.get("favorite_drink", "")
        desc = self._describe_person_with_vlm(person)

        parts = [f"Let me introduce {name}. {desc}"]
        if drink:
            parts.append(f"{name} usually enjoys {drink}.")
        return " ".join(p.strip() for p in parts if p.strip())

    def _load_representative_image(self, saved_dir: str) -> str:
        try:
            files = glob.glob(os.path.join(saved_dir, "*.jpg")) + glob.glob(os.path.join(saved_dir, "*.png"))
            if not files:
                return ""
            files.sort(key=lambda f: os.path.getmtime(f))
            latest_file = files[-1]
            with open(latest_file, "rb") as f:
                img_bytes = f.read()
            return base64.b64encode(img_bytes).decode("utf-8")
        except Exception:
            return ""

    def _describe_person_with_vlm(self, person: dict) -> str:
        name = person.get("name", "the guest")
        saved_dir = person.get("saved_dir", "")
        if not saved_dir:
            return f"{name} is our guest."

        base64_image = self._load_representative_image(saved_dir)
        if not base64_image:
            return f"{name} is our guest."

        system_prompt = (
            "You will receive a face-only cropped photograph of one person. "
            "Describe only the person's face and appearance in exactly one short English sentence. "
            "Focus on stable traits such as hairstyle, hair color, facial features, expression, "
            "and any visible clothing near the face. Do not mention background. One sentence only."
        )
        user_prompt = f"This is a face-only cropped photo of a guest named {name}. Describe the guest in one short sentence."

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": [
                {"type": "text", "text": user_prompt},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
            ]}
        ]

        payload = {
            "model": self.llm_api_model_name,
            "messages": messages,
            "max_tokens": 80,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload).strip()

    def _describe_common_traits_with_vlm(self, p1: dict, p2: dict) -> str:
        saved_dir1 = p1.get("saved_dir", "")
        saved_dir2 = p2.get("saved_dir", "")
        if not saved_dir1 or not saved_dir2:
            return ""

        b1 = self._load_representative_image(saved_dir1)
        b2 = self._load_representative_image(saved_dir2)
        if not b1 or not b2:
            return ""

        system_prompt = (
            "You will receive two face-only cropped photographs. "
            "If there is at least one obvious common trait, answer with exactly one short sentence starting with "
            "'Both of our guests ...'. If none, answer exactly 'NONE'."
        )
        user_prompt = "Say what they clearly have in common, if anything."

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": [
                {"type": "text", "text": user_prompt},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b1}"}},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b2}"}},
            ]}
        ]

        payload = {
            "model": self.llm_api_model_name,
            "messages": messages,
            "max_tokens": 80,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }

        out = self.call_llm(payload).strip()
        return "" if out.upper() == "NONE" else out


def main(args=None):
    rclpy.init(args=args)
    node = VLMActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
