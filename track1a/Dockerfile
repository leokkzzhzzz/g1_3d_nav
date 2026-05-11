FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── System deps ────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    git build-essential cmake \
    libpcl-dev libeigen3-dev \
    ros-noetic-gtsam \
    ros-noetic-pcl-ros \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-eigen-conversions \
    ros-noetic-rviz \
    ros-noetic-rqt-graph \
    ros-noetic-map-server \
    python3-pip \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Livox-SDK2 (C library) ─────────────────────────────────
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Livox-SDK2

# ── Entrypoint ─────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
