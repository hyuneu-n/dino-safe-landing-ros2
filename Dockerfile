# Dockerfile — 재현용 이미지 (CONTEXT.md 결정 2: 시연은 내 PC 네이티브, 재현성은 Docker).
#
# ⚠️ 이 이미지는 이 세션(WSL, docker CLI 없음)에서 build/run 테스트를 못 했다.
#    README.md "Docker 검증" 절차대로 실제 Docker가 있는 머신에서 한 번 확인할 것.
#
# GUI(Gazebo client, RViz)는 뺐다 — 헤드리스(gzserver만)로 미션 1회를 끝까지 돌려서
# 로그/프레임이 정상적으로 나오는지 보는 게 목적. 눈으로 보는 시연은 네이티브 설치로.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        python3-pip \
        curl \
    && rm -rf /var/lib/apt/lists/*

# torch는 CPU wheel로 (이미지 이식성 우선). GPU 쓰고 싶으면 README "GPU 사용" 절 참고.
RUN pip3 install --no-cache-dir \
        torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir \
        numpy pillow opencv-python-headless onnxruntime

ENV HOME=/root
WORKDIR /root/safe_landing
COPY . .

# run_*.sh 스크립트들이 "$HOME/gz_worlds/*.world"를 참조하므로 맞춰줌 (네이티브 설치와 동일 관례)
RUN mkdir -p /root/gz_worlds \
    && cp worlds/*.world worlds/*.sdf /root/gz_worlds/ 2>/dev/null || true

# DINOv2 가중치 + OpenLander ONNX를 빌드 타임에 미리 받아둠 — "설치하면 동작"이 진짜가
# 되려면 데모 중에 인터넷이 필요하면 안 됨. (torch.hub 캐시는 /root/.cache/torch/hub)
RUN python3 -c "import torch; torch.hub.load('facebookresearch/dinov2', 'dinov2_vits14', trust_repo=True)" \
    && bash eval/models/download_openlander.sh

CMD ["bash", "docker-entrypoint.sh"]
