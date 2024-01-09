# GPT4V VQA

This repository offers a ROS Node with GPT4V model.

## Installation

```bash
catkin build gpt4v_vqa
```

## Usage

```bash
roslaunch gpt4v_vqa vqa.launch api_key:=<YOUR_API_KEY>
```

## Nodes

### gpt4v_vqa

#### Subscribed Topics

* **`~image`** ([sensor_msgs/Image])

    The image used for VQA as default image.

#### Action Servers

* **`~inference_server`** ([jsk_recognition_msgs/VQATaskAction])

    The action server for VQA.