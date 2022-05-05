FROM ros:noetic
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y git \
	python3-pip \
	python3-catkin-tools \
	ros-noetic-cv-bridge \
	python3-opengl \
	ros-noetic-ros-numpy \
	ros-noetic-rviz \
      	&& rm -rf /var/lib/apt/lists/*

WORKDIR /home/catkin_ws/src

RUN git clone https://github.com/mrudorfer/burg-toolkit.git \
	&& git clone https://github.com/v4r-tuwien/burg_scene_visualizer.git

WORKDIR /home/catkin_ws/src/burg-toolkit
RUN python3 -m pip install --upgrade pip
RUN pip3 install --upgrade setuptools wheel
RUN pip3 install .

WORKDIR /home/catkin_ws/
RUN . /opt/ros/noetic/setup.bash && catkin build
