FROM ros:humble-ros-base

# Install OpenCV, CV_Bridge and their dependencies
RUN apt-get update \
    && apt-get install python3-pip -y \
    && pip python3-opencv -y \
    && apt-get install ros-noetic-cv-bridge -y \
    && apt-get install ffmpeg libsm6 libxext6  -y 

# Source the ROS workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc