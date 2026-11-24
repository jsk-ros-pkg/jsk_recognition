# GPT4V VQA

This repository offers a ROS Node with GPT4V model.

## Installation

```bash
catkin build gpt4v_vqa
```

## Usage

```bash
roslaunch gpt4v_vqa vqa.launch api_key:=<YOUR_API_KEY> VQA_INPUT_IMAGE:=<IMAGE TOPIC>
```

And from other terminal

```bash
$ rosrun gpt4v_vqa vqa_interpreter.py
```

## Nodes

### gpt4v_vqa

This node is a ROS wrapper for GPT4V model. Its behavior is similar to [VQA node](../jsk_perception/node_scripts/vqa_node.py). But there is a difference that this node does not support continuous inference. This node use API only when action server is called.

#### Subscribed Topics

* **`~image`** ([sensor_msgs/Image])

    The image used for VQA as default image.

#### Action Servers

* **`~inference_server`** ([jsk_recognition_msgs/VQATaskAction])

    The action server for VQA.