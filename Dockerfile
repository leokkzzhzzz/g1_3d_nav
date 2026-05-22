FROM hongtu-fastlio2:noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    liblapacke liblapacke-dev libopenblas-dev \
    ros-noetic-roslint \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-costmap-2d \
    ros-noetic-nav-core \
    ros-noetic-move-base \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install catkin_pkg

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
