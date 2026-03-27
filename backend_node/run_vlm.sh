#!/bin/bash

# 종료 시 실행될 캐시 및 프로세스 정리 함수
cleanup() {
    echo -e "\n[시스템 정리 중] 남아있는 프로세스와 캐시를 삭제합니다..."
    
    # 1. 파이썬 스크립트 종료
    pkill -f "vlm_thor.py"
    
    # 2. 도커 컨테이너 강제 종료 및 삭제 (캐시/VRAM 반환)
    docker rm -f vllm_thor_server >/dev/null 2>&1
    
    # 3. vLLM 및 Triton 관련 로컬 찌꺼기 캐시 파일 삭제
    rm -rf ~/.cache/vllm
    rm -rf ~/.triton
    
    echo "[정리 완료] 캐시와 프로세스가 모두 정상적으로 지워졌습니다!"
}

# Ctrl+C (SIGINT) 또는 스크립트 종료 시 무조건 cleanup 함수가 실행되도록 설정
trap cleanup EXIT INT TERM

echo "1. vLLM 도커 컨테이너를 실행합니다..."
# --name vllm_thor_server 옵션을 추가하여 종료 시 확실하게 컨테이너를 끄도록 지정했습니다.
docker run --runtime nvidia --rm --name vllm_thor_server --net=host \
  --shm-size=4g \
  -v /home/thor/bt_ws/src/backend_node/models/qwen3_vl:/workspace \
  ghcr.io/nvidia-ai-iot/vllm:latest-jetson-thor \
  python3 -m vllm.entrypoints.openai.api_server \
  --model /workspace/Qwen3-VL-8B-Instruct \
  --quantization fp8 \
  --gpu-memory-utilization 0.3 \
  --max-model-len 4096 \
  --limit-mm-per-prompt '{"image": 1}' \
  --trust-remote-code &

echo "서버 초기화 대기 중 (15초)..."
sleep 15

echo "2. vlm_thor.py 스크립트를 실행합니다..."
python3 vlm_thor.py &

# 두 프로세스가 끝날 때까지 대기
wait
