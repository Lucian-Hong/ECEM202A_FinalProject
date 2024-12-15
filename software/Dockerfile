# Set the base image
FROM ubuntu:22.04

# Set the ROS distribution to Humble (or your desired distribution)
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

RUN locale  # check for UTF-8
RUN apt update && apt install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y

RUN apt install -y ros-humble-desktop

RUN apt install ros-$ROS_DISTRO-image-tools
RUN apt install ros-$ROS_DISTRO-vision-msgs
RUN apt install ros-$ROS_DISTRO-ament-cmake
RUN apt install ros-$ROS_DISTRO-tf-transformations -y
RUN apt install python3-pip clang -y
RUN pip install torchvision
RUN pip install ultralytics

RUN apt update && apt install -y git

# Clone the repository into /ros2_ws directory
RUN git clone --recurse-submodules https://github.com/JL013001/go2_ros2_sdk_ecem202a.git /ros2_ws/go2_sdk/src
RUN git clone --recurse-submodules https://github.com/JL013001/m-explore-ros2-ecem202a.git /ros2_ws/m-explore/src

# Set the working directory to the 'src' directory inside the cloned repo
WORKDIR /ros2_ws/go2_sdk/src

# Install Python dependencies from the requirements.txt in the 'src' directory
RUN pip install --no-cache-dir -r requirements.txt

# Install rustup and install the specific version 1.79.0
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    /root/.cargo/bin/rustup install 1.79.0 && \
    /root/.cargo/bin/rustup default 1.79.0

# Add the Cargo and Rust binaries to the PATH
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /ros2_ws/go2_sdk
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash"
RUN apt install -y python3-rosdep python3-colcon-common-extensions
RUN rosdep init && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN pip install "numpy<1.24"

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

RUN apt-get update && apt-get install -y \
    build-essential \
    libasound-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    espeak \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

ENV ALSA_CARD=2

RUN pip install --no-cache-dir \
    speechrecognition \
    pyttsx3 \
    pyaudio \
    vosk

COPY ./ost.yaml /root/ost.yaml
COPY ./yolov8s-worldv2.pt ./yolov8s-worldv2.pt
COPY ./vosk-model-small-en-us-0.15 ./vosk-model-small-en-us-0.15

WORKDIR /ros2_ws/m-explore
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

RUN /bin/bash
