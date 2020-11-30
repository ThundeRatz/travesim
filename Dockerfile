FROM ros:noetic-robot-focal

# Needed to run 'source' cmd
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install gazebo11 \
  libgazebo11 \
  ros-noetic-gazebo-ros --no-install-recommends -y

RUN mkdir -p /catkin_ws/src/project
WORKDIR /catkin_ws

RUN /ros_entrypoint.sh catkin_make

COPY package.xml /catkin_ws/src/project/
COPY CMakeLists.txt /catkin_ws/src/project/

RUN source devel/setup.bash && apt update --fix-missing && rosdep install vss_simulation -y

COPY . /catkin_ws/src/project/

CMD source devel/setup.bash && roslaunch vss_simulation simulation_robot.launch gui:=false
