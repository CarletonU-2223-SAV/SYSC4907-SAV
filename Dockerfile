# Set up a container with the project
FROM ros:noetic

# Install Python and other dependencies
RUN apt-get update
RUN apt-get install python3 python3-pip -y
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install ros-noetic-cv-bridge  -y

# Copy dependencies file and install them
COPY dependencies.txt .
RUN pip install --upgrade pip
RUN pip install -r requirements.txt

# Download and run AirSim
RUN bash wget https://github.com/microsoft/AirSim/releases/download/v1.8.1/AirSimNH.zip
RUN bash unzip AirSimNH.zip

# Copy project files
COPY . /ros_ws
WORKDIR /ros_ws
RUN bash ./AirSimNH/LinuxNoEditor/AirSimNH.sh -- headless
RUN bash ./opt/ros/noetic/setup.bash
RUN bash catkin_make
RUN bash roslaunch central_control full_system.launch
