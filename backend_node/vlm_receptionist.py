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

        # LLM API
        self.llm_api_url = "http://192.168.50.189:11111"
        self.llm_api_key = "ollama"
        self.llm_api_model_name = "llava:7b"

        # image topic
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self._busy = False

        self._as = ActionServer(
            self,
            Vlm,
            ACTION_NAME,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"VLM Action Server started: {ACTION_NAME}")

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

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
        instruction = goal.prompt or ""          # ✅ instruction only
        context_json = goal.context_json or ""   # ✅ contains heard_text etc.
        timeout_ms = int(goal.timeout_ms) if goal.timeout_ms is not None else 0

        start = time.time()

        def check_cancel_timeout():
            if goal_handle.is_cancel_requested:
                raise RuntimeError("canceled")
            if timeout_ms > 0 and (time.time() - start) * 1000.0 > timeout_ms:
                raise RuntimeError("timeout")

        fb = Vlm.Feedback()
        res = Vlm.Result()

        try:
            # parse context
            try:
                ctx = json.loads(context_json) if context_json else {}
            except Exception as e:
                raise RuntimeError(f"invalid context_json: {e}")

            heard_text = (ctx.get("heard_text") or "").strip()

            fb.state = "started"
            fb.progress = 0.0
            goal_handle.publish_feedback(fb)

            check_cancel_timeout()

            # ---------------- mode 45: extract name & favorite drink ----------------
            if mode == 45:
                fb.state = "mode1_text_extract"
                fb.progress = 0.2
                goal_handle.publish_feedback(fb)

                data = self.llm_extract_name_drink(heard_text, instruction)

                name = (data.get("name") or "").strip()
                drink = (data.get("favorite_drink") or "").strip()

                ok = (len(name) > 0)  # 최소 name은 있어야 성공으로 보는게 안전

                res.success = bool(ok)
                res.generated_text = ""
                res.result_json = json.dumps(
                    {"name": name, "favorite_drink": drink},
                    ensure_ascii=False
                )
                res.error_message = "" if ok else "extract_failed"

                if ok:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return res

            # ---------------- mode 56: empty seat check (image) ----------------
            # elif mode == 56:
            #     fb.state = "mode2_waiting_image"
            #     fb.progress = 0.1
            #     goal_handle.publish_feedback(fb)

            #     while self.latest_image is None:
            #         check_cancel_timeout()
            #         time.sleep(0.02)

            #     fb.state = "mode2_image_processing"
            #     fb.progress = 0.4
            #     goal_handle.publish_feedback(fb)

            #     # returns dict: {"has_empty_seat": bool, "desc": str}
            #     out = self.vlm_check_empty_seat(instruction)

            #     has_seat = bool(out.get("has_empty_seat", False))
            #     desc = (out.get("desc") or "").strip()

            #     if has_seat and desc:
            #         res.success = True
            #         res.generated_text = desc
            #         res.result_json = json.dumps({"has_empty_seat": True}, ensure_ascii=False)
            #         res.error_message = ""
            #         goal_handle.succeed()
            #     else:
            #         res.success = False
            #         res.generated_text = ""
            #         res.result_json = json.dumps({"has_empty_seat": False}, ensure_ascii=False)
            #         res.error_message = "no_empty_seat"
            #         goal_handle.abort()
            #     return res

            elif mode == 56:
                fb.state = "mode56_check_empty_seat"
                fb.progress = 0.1
                goal_handle.publish_feedback(fb)

                # ✅ 이미지 있으면 VLM, 없으면 fallback
                if self.latest_image is None:
                    # 없는대로 진행: "없다고 가정" or "확인 불가" 중 택1
                    res.success = True  # ✅ 파이프라인 계속 돌리려면 succeed가 편함
                    res.generated_text = "I couldn't get the camera image, so I can't confirm an empty seat right now."
                    res.result_json = json.dumps({"has_empty_seat": False, "reason": "no_image"}, ensure_ascii=False)
                    res.error_message = ""
                    goal_handle.succeed()
                    return res

                fb.state = "mode56_image_processing"
                fb.progress = 0.4
                goal_handle.publish_feedback(fb)

                out = self.vlm_check_empty_seat(instruction)
                has_seat = bool(out.get("has_empty_seat", False))
                desc = (out.get("desc") or "").strip()

                res.success = True
                if has_seat and desc:
                    res.generated_text = desc
                    res.result_json = json.dumps({"has_empty_seat": True}, ensure_ascii=False)
                else:
                    res.generated_text = ""  # 혹은 "No empty seat detected."
                    res.result_json = json.dumps({"has_empty_seat": False}, ensure_ascii=False)

                res.error_message = ""
                goal_handle.succeed()
                return res

            # ---------------- mode 67: next utterance (text) ----------------
            elif mode == 67:
                fb.state = "mode3_next_utterance"
                fb.progress = 0.2
                goal_handle.publish_feedback(fb)

                line = self.llm_next_utterance(heard_text, instruction).strip()
                ok = (len(line) > 0)

                res.success = bool(ok)
                res.generated_text = line if ok else ""
                res.result_json = ""
                res.error_message = "" if ok else "empty_utterance"

                if ok:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return res

            
            # ---------------- mode 78: introduce ONE person from Mapstore context ---------------
            elif mode == 78:    
                fb.state = "mode78_intro_person"
                fb.progress = 0.2
                goal_handle.publish_feedback(fb)

                # ctx 예시:
                # {"person": {"name":"Alex","saved_dir":"/home/.../face_data/Alex","favorite_drink":"cola"}}
                p = (ctx.get("person") or {})
                name = (p.get("name") or "").strip()
                saved_dir = (p.get("saved_dir") or "").strip()
                drink = (p.get("favorite_drink") or "").strip()

                if not name:
                    raise RuntimeError("missing_person_name")

                line = self.introduce_one_person(name, saved_dir, drink, instruction).strip()
                ok = (len(line) > 0)

                res.success = bool(ok)
                res.generated_text = line if ok else ""
                res.result_json = ""
                res.error_message = "" if ok else "intro_failed"

                if ok:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return res


            else:
                raise RuntimeError(f"unsupported mode: {mode}")

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

    # ---------------- mode1: JSON-only extractor ----------------
    def llm_extract_name_drink(self, heard_text: str, instruction: str) -> dict:
        # instruction 예: "Extract name and favorite drink"
        system_prompt = (
            "You are an information extractor.\n"
            "Return ONLY valid JSON with exactly these keys:\n"
            "  - name (string)\n"
            "  - favorite_drink (string)\n"
            "If missing, use empty string.\n"
            "No extra words, no markdown.\n"
            "Example: {\"name\":\"Minho\",\"favorite_drink\":\"cola\"}"
        )

        user_prompt = f"sentence: {heard_text}\nrule: {instruction}"

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "max_tokens": 120,
            "temperature": 0.1,
            "top_p": 0.9,
            "stream": False,
        }

        out = self.call_llm(payload).strip()
        try:
            return json.loads(out)
        except Exception:
            return {"name": "", "favorite_drink": ""}

    # ---------------- mode3: one-line next utterance ----------------
    def llm_next_utterance(self, heard_text: str, instruction: str) -> str:
        system_prompt = (
            "You are a service robot speaking to a host.\n"
            "Generate exactly ONE short natural English sentence the robot should say next.\n"
            "No quotes, no explanation, one sentence only."
            "Please be polite."
        )
        user_prompt = f"heard: {heard_text}\ncontext: {instruction}"

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "max_tokens": 60,
            "temperature": 0.4,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload).replace("\n", " ").strip()

    # ---------------- mode2: JSON-only empty-seat checker ----------------
    def vlm_check_empty_seat(self, style_instruction: str) -> dict:
        if self.latest_image is None:
            return {"has_empty_seat": False, "desc": ""}

        # ✅ style_instruction은 "설명 스타일" 힌트로만 사용
        system_prompt = (
            "Look at the image and decide if there is at least one clearly available empty seat.\n"
            "Return ONLY valid JSON exactly in this format:\n"
            "{\"has_empty_seat\": true/false, \"desc\": \"...\"}\n"
            "Rules:\n"
            "- If has_empty_seat is false, desc MUST be empty string.\n"
            "- If true, desc MUST be exactly one short English sentence describing where the empty seat is.\n"
            "- No extra keys. No markdown. No additional text."
        )
        if style_instruction:
            system_prompt += f"\nStyle hint (only if true): {style_instruction}"

        user_prompt = "Check whether there is an available empty seat."

        # bgr -> rgb
        pil_image = PILImage.fromarray(self.latest_image[:, :, ::-1])
        buf = BytesIO()
        pil_image.save(buf, format="JPEG")
        b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": [
                    {"type": "text", "text": user_prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ]},
            ],
            "max_tokens": 120,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }

        out = self.call_llm(payload).strip()
        try:
            return json.loads(out)
        except Exception:
            return {"has_empty_seat": False, "desc": ""}


    # ---------------- mode78 helpers ----------------
    def introduce_one_person(self, name: str, saved_dir: str, drink: str, style_hint: str) -> str:
        """
        - saved_dir에 얼굴 이미지가 있으면: VLM으로 외형 1문장 묘사 + 소개문 생성
        - 이미지가 없으면: 텍스트 기반으로만 짧게 소개
        """
        desc = ""
        b64 = self._load_representative_image(saved_dir) if saved_dir else ""
        if b64:
            desc = self._describe_person_with_vlm(name, b64, style_hint)

        parts = []
        if desc:
            parts.append(f"Let me introduce {name}. {desc}")
        else:
            parts.append(f"Let me introduce {name}.")

        if drink:
            parts.append(f"{name} usually enjoys {drink}.")

        return " ".join(p.strip() for p in parts if p.strip())

    def _load_representative_image(self, saved_dir: str) -> str:
        try:
            if not saved_dir or not os.path.isdir(saved_dir):
                return ""
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

    def _describe_person_with_vlm(self, name: str, base64_image: str, style_hint: str) -> str:
        system_prompt = (
            "You will receive a face-only cropped photograph of one person. "
            "Describe only the person's face and appearance in exactly one short English sentence. "
            "Focus on stable traits such as hairstyle, hair color, facial features, expression, "
            "and any visible clothing near the face. Do not mention background. One sentence only."
        )
        if style_hint:
            system_prompt += f" Style hint: {style_hint}"

        user_prompt = f"This is a face-only cropped photo of a guest named {name}. Describe the guest in one short sentence."

        payload = {
            "model": self.llm_api_model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": [
                    {"type": "text", "text": user_prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}},
                ]},
            ],
            "max_tokens": 80,
            "temperature": 0.2,
            "top_p": 0.9,
            "stream": False,
        }
        return self.call_llm(payload).replace("\n", " ").strip()



    # ---------------- LLM call ----------------
    def call_llm(self, json_payload):
        url = self.llm_api_url.rstrip("/") + "/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.llm_api_key}",
        }
        response = requests.post(url, headers=headers, json=json_payload, timeout=60)
        response.raise_for_status()
        j = response.json()
        return j["choices"][0]["message"]["content"].replace("\n", " ")


def main(args=None):
    rclpy.init(args=args)
    node = VLMActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
