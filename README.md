# Utilizing LLMs as a Task Planning Agent for Robotics

This repository is the code for [Utilizing LLMs as a Task Planning Agent for Robotics](https://hlfshell.ai/posts/llm-task-planner/); it is a group project that acted as the capstone project for my Robotics Engineering Masters @ WPI.

This project's aim was to demonstrate that not only can LLMs be used to drive robotic task planning for complex tasks given a set of natural language objectives, but also demonstrate some contextual understanding that goes beyond information known about the world state.

# Environment Setup

[This blogpost](https://hlfshell.ai/posts/repeatable-ros-environments/) explains the why behind the environment setup. In this repos reference `docs/quickstart.md` for setting up your environment and installing the necessary dependencies.

Note that we assume that the following environment variables are available if you are utilizing those services:

* `OPENAI_API_KEY` - the OpenAPI provider key
* `GOOGLE_API_KEY` - an API key or service account key for accessing PaLM2 API endpoints for GCP
* `ELEVENLABS_API_KEY` - for AI thoughts to voice; optional

# Building code

In `ros_ws` in your chosen environment run `colcon build` to build the code. Note that you'll likely have to also run `pip install -r requirements.txt` to setup your Python environment. If you are using a python virtual environment be sure to activate it when running launch files as well.

Note that sometimes there are issues with your Python virtual environments and ROS2 base Python packages. If this occurs you may need to do additional work linking the library modules into your virtual environment.

# Running the Planner

To run all appropriate code, create N??? terminal windows and run the following in order. For each terminal, source your environment with:

```
cd ros_ws/
source install/setup.bash
```

First, we launch the environment, robot, navigation backend, RVIZ, and Gazebo

```
ros2 launch aws-robomaker-small-house-world house.map.launch.py
```

Then we load the `query_services` module to run the state service. If you wish to use a prepared database of known items, copy `state_db.db.fresh` to `state_db.db` within your working directory.

```
ros2 launch query_services state_query_launch.py
```

We then launch `litterbug` service to populate the world with items and manage simulated vision and interaction:

```
ros2 launch litterbug litterbug.launch.py
```

We can lauch the AI service itself to launch the task planner:

```
ros2 launch planner ai.launch.py
```

Once running, you may also wish to stream important control information as it occurs within the AI system. To do this, run the ai_streamer software from the `ros_ws/src/planner` directory:

```
cd ros_ws/src/planner
python ai_streamer.py
```

To trigger an objective to be planned and executed, you can use the `ros2 service call` command to the `/objective` endpoint.

```
 ros2 topic pub /objective my_package/msg/Objective 'id: "01", objective: "I'm kind of hungry"'
 ```