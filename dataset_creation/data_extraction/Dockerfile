FROM ros:humble-ros-base
ARG DEBIAN_FRONTEND=noninteractive

# Install OpenCV and dependencies
RUN apt-get update \
    && apt-get install -y \
       python3-pip \
       python3-opencv \
       ros-humble-cv-bridge \
       ffmpeg libsm6 libxext6 
RUN pip install pillow scipy numpy==1.25 matplotlib python3-tk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* 

# Source the ROS workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc