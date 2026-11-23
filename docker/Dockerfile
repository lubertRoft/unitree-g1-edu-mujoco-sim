# G1 29-DoF MuJoCo sim + emulated onboard controller (rt/arm_sdk).
FROM ubuntu:24.04

ARG UNITREE_MUJOCO_REF=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
ARG UNITREE_RL_GYM_REF=276801e46c5d433564f24658bac64f254b7d2d4b
ARG UNITREE_SDK2_PYTHON_REF=814556d15970dd2ecf1c9984e845ca02ab07e206
ARG CYCLONEDDS_REF=0.10.2

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates git build-essential cmake \
        python3 python3-venv python3-dev python3-pip \
        libgl1 libglx0 libegl1 libglvnd0 libglfw3 libxrender1 libxext6 libx11-6 \
        mesa-utils x11-utils \
    && rm -rf /var/lib/apt/lists/*

# Fetch a repo at an exact commit (shallow).
RUN printf '#!/bin/sh\nset -e\nmkdir -p "$3" && cd "$3" && git init -q && git remote add origin "$1" && git fetch -q --depth 1 origin "$2" && git checkout -q FETCH_HEAD\n' \
        > /usr/local/bin/git-pin && chmod +x /usr/local/bin/git-pin

# CycloneDDS C library (needed by the python bindings).
# 0.10.2 aborts ("buffer overflow detected") under Ubuntu 24.04's default
# _FORTIFY_SOURCE hardening, so it is built with fortification disabled.
RUN git clone -q --depth 1 --branch ${CYCLONEDDS_REF} https://github.com/eclipse-cyclonedds/cyclonedds /tmp/cyclonedds \
    && cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DBUILD_IDLC=ON \
       -DCMAKE_C_FLAGS="-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0" \
    && cmake --build /tmp/cyclonedds/build --target install -j"$(nproc)" \
    && rm -rf /tmp/cyclonedds
ENV CYCLONEDDS_HOME=/opt/cyclonedds

RUN git-pin https://github.com/unitreerobotics/unitree_mujoco.git ${UNITREE_MUJOCO_REF} /opt/unitree_mujoco \
 && git-pin https://github.com/unitreerobotics/unitree_rl_gym.git ${UNITREE_RL_GYM_REF} /opt/unitree_rl_gym \
 && git-pin https://github.com/unitreerobotics/unitree_sdk2_python.git ${UNITREE_SDK2_PYTHON_REF} /opt/unitree_sdk2_python

ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH=$VIRTUAL_ENV/bin:$PATH

RUN pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch \
 && pip install --no-cache-dir mujoco numpy pygame pyyaml pytest opencv-python-headless \
 && pip install --no-cache-dir cyclonedds==0.10.2 \
 && pip install --no-cache-dir --no-deps -e /opt/unitree_sdk2_python

# DDS pinned to loopback (domain id is chosen in code: 1).
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# NVIDIA GL passthrough (libs are injected by the container toolkit / CDI).
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display

COPY sim /app/sim
COPY controller /app/controller
COPY docker/entrypoint.py /app/entrypoint.py
ENV PYTHONPATH=/app:/app/sim:/opt/unitree_mujoco/simulate_python \
    UNITREE_MUJOCO_DIR=/opt/unitree_mujoco \
    RL_GYM_DIR=/opt/unitree_rl_gym \
    PYTHONUNBUFFERED=1
WORKDIR /app
ENTRYPOINT ["python3", "/app/entrypoint.py"]
