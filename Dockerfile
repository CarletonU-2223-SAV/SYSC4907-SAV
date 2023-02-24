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

# Copy project files
COPY . /ros_ws
WORKDIR /ros_ws
RUN bash bash_scripts/run_and_check_tests.sh
