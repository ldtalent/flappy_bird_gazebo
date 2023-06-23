# flappy_bird_gazebo
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)
![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)

A ROS1 package designed to play the flappy bird game in Gazebo.

<p align=center>
<img src="https://user-images.githubusercontent.com/45683974/236690701-362b65d1-8e90-448b-912d-c8eba77f547a.gif"/>
</p>

## Installation

* Install `yaml-cpp`:

	```bash
	sudo apt install libyaml-cpp-dev
	```

* Install Python dependencies using pip3:

	```bash
	pip3 install -r requirements.txt
	```

* Finally, install all the dependencies using `rosdep`:

	```bash
	rosdep install --from-paths $ROS_WS/src --ignore-src -r -y
	```

* Build the workspace:

	```bash
	catkin build flappy_bird_gazebo
	```

## Usage

* To launch the game and control using arrow keys (up and down), run:

	```bash
	roslaunch flappy_bird_gazebo flappy_bird.launch
	```

* Optional: configurable parameters such as obstacle gap, inter-obstacle distance, etc., can be controlled using [sim_params.yaml](config/sim_params.yaml) file.

## Standardization

* The entire package is formatted against [roslint](http://wiki.ros.org/roslint) and [catkin_lint](https://github.com/fkie/catkin_lint).

## Blog

* Check out our [article](https://blog.ldtalentwork.com/2023/06/21/simulating-flappy-bird-in-gazebo-using-ros/) on how we developed this repository and the hurdles we had to go through in doing so.

###### 💾 EOF
