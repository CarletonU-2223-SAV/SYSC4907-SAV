# Set up a container with the project
FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# Install Python and other dependencies
RUN apt-get update
RUN apt-get install python3 python3-pip git ros-noetic-catkin -y

# Copy dependencies file and install them
COPY dependencies.txt .
RUN pip3 install --upgrade pip
RUN pip3 install msgpack-rpc-python
RUN pip3 install --ignore-installed pyyaml
RUN pip3 install --no-deps -r dependencies.txt

# Copy project files
COPY . /ros_ws
WORKDIR /ros_ws
