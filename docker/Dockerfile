# syntax=docker/dockerfile:1
ARG ROS_IMAGE=ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
FROM ${ROS_IMAGE} AS controller-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ca-certificates unzip python3-catkin-tools \
      libeigen3-dev libspdlog-dev ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
      ros-noetic-std-srvs && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/kinova_ws
RUN curl -fL --retry 3 https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip \
      -o /tmp/kortex.zip && \
    echo "105da907c336ba322357a284898bcbd3bebe01dc222cef4f9549f3b69aedf160  /tmp/kortex.zip" | sha256sum -c - && \
    mkdir -p /opt/kortex_api && unzip -q /tmp/kortex.zip -d /opt/kortex_api && \
    rm /tmp/kortex.zip
COPY src/robot_controller src/robot_controller
COPY src/kinova_ros_control src/kinova_ros_control
ARG BUILD_JOBS=2
RUN source /opt/ros/noetic/setup.bash && \
    catkin config --install --extend /opt/ros/noetic \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DKORTEX_ROOT=/opt/kortex_api && \
    catkin build --no-status --jobs ${BUILD_JOBS} --parallel-packages 1

FROM ${ROS_IMAGE} AS cameras-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      build-essential cmake ca-certificates python3-catkin-tools libeigen3-dev \
      libusb-1.0-0-dev libudev-dev libssl-dev pkg-config \
      ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-nav-msgs \
      ros-noetic-nodelet ros-noetic-tf ros-noetic-ddynamic-reconfigure \
      ros-noetic-diagnostic-updater ros-noetic-message-generation \
      ros-noetic-compressed-image-transport && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/camera_ws
RUN mkdir -p third_party/librealsense src/realsense-ros && \
    curl -fL --retry 3 https://codeload.github.com/realsenseai/librealsense/tar.gz/refs/tags/v2.51.1 \
      -o /tmp/librealsense.tar.gz && \
    echo "f03b2bf6d52c665120dd0b961fe4553867c2a6eddb5d1898e123f9eb81a91536  /tmp/librealsense.tar.gz" | sha256sum -c - && \
    tar -xzf /tmp/librealsense.tar.gz --strip-components=1 -C third_party/librealsense && \
    curl -fL --retry 3 https://codeload.github.com/realsenseai/realsense-ros/tar.gz/refs/tags/2.3.2 \
      -o /tmp/realsense-ros.tar.gz && \
    echo "18c0f90eeea2b64889388e2e441221931e220d1fea06fe6eff8d70442c456459  /tmp/realsense-ros.tar.gz" | sha256sum -c - && \
    tar -xzf /tmp/realsense-ros.tar.gz --strip-components=1 -C src/realsense-ros && \
    sed -i '/RS405_PID/s/0x0b0c/0x0b5b/' src/realsense-ros/realsense2_camera/include/constants.h && \
    rm /tmp/librealsense.tar.gz /tmp/realsense-ros.tar.gz
ARG BUILD_JOBS=2
RUN cmake -S third_party/librealsense -B third_party/librealsense-build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
      -DBUILD_TOOLS=ON -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_WITH_TM2=OFF \
      -DIMPORT_DEPTH_CAM_FW=OFF -DBUILD_WITH_CUDA=OFF -DCHECK_FOR_UPDATES=OFF && \
    cmake --build third_party/librealsense-build --parallel ${BUILD_JOBS} && \
    cmake --install third_party/librealsense-build && ldconfig
COPY --from=controller-builder /opt/kinova_ws/install /opt/kinova_ws/install
COPY --from=controller-builder /opt/kortex_api /opt/kortex_api
RUN source /opt/ros/noetic/setup.bash && \
    catkin config --install --extend /opt/kinova_ws/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build realsense2_camera --no-status --jobs ${BUILD_JOBS} --parallel-packages 1

FROM ${ROS_IMAGE} AS runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      libspdlog1 ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-std-srvs \
      libusb-1.0-0 libudev1 libssl1.1 \
      ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-nav-msgs \
      ros-noetic-nodelet ros-noetic-tf ros-noetic-ddynamic-reconfigure \
      ros-noetic-diagnostic-updater ros-noetic-compressed-image-transport \
      && rm -rf /var/lib/apt/lists/*
COPY --from=controller-builder /opt/kinova_ws/install /opt/kinova_ws/install
COPY --from=controller-builder /opt/kortex_api /opt/kortex_api
COPY --from=cameras-builder /usr/local /usr/local
COPY --from=cameras-builder /opt/camera_ws/install /opt/camera_ws/install
COPY docker/entrypoint.sh /kinova_entrypoint.sh
RUN ldconfig && chmod +x /kinova_entrypoint.sh && \
    echo "source /opt/camera_ws/install/setup.bash" >> /etc/bash.bashrc
WORKDIR /opt/kinova_ws
ENTRYPOINT ["/kinova_entrypoint.sh"]
CMD ["sleep", "infinity"]
