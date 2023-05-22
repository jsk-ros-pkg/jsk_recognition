Sound Classification
====================

ROS package to classify sound stream.

## Contents
- [Setup](#Setup)
- [Usage](#Usage)
- [Quick demo](#Quick-Demo)

## Setup

1. [Install ROS](http://wiki.ros.org/ROS/Installation). Available OS:
     - Ubuntu 16.04 (?)
     - Ubuntu 18.04

1. Create workspace
    ```bash
    mkdir ~/sound_classification_ws/src -p
    cd ~/sound_classification_ws/src
    git clone https://github.com/jsk-ros-pkg/jsk_recognition.git
    rosdep install --from-paths . --ignore-src -y -r
    cd ..
    catkin build sound_classification
    source ~/sound_classification_ws/devel/setup.bash
    ```

1. Install other packages.
    - cuda and cupy are needed for chainer. See [installation guide of JSK](../doc/install_chainer_gpu.rst)
    - Using GPU is highly recommended.

## Usage

1. Check and specify your microphone parameters.
    - In particular, `device`, `n_channel`, `bitdepth` and `mic_sampling_rate` need to be known.
    - The example bash commands to get these params are below:
        ```bash
        # For device. In this example, card 0 and device 0, so device:="hw:0,0"
        $ arecord -l
        \**** List of CAPTURE Hardware Devices ****
        card 0: PCH [HDA Intel PCH], device 0: ALC293 Analog [ALC293 Analog]
        Subdevices: 1/1
        Subdevice #0: subdevice #0
        ```
        ```bash
        # For n_channel, bitdepth and sample_rate,
        # Note that sources means input (e.g. microphone) and sinks means output (e.g. speaker)
        $ pactl list short sources
        1       alsa_input.pci-0000_00_1f.3.analog-stereo       module-alsa-card.c      s16le 2ch 44100Hz       SUSPENDED
        ```
    - Pass these params to each launch file as arguments when launching (e.g., `device:=hw:0,0 n_channel:=2 bitdepth:=16 mic_sampling_rate:=44100`).
    - If you use `/audio` topic from other computer and do not want to publish `/audio`, set `use_microphone:=false` at each launch file when launching.

1. Save environmental noise to `train_data/noise.npy`.
    - By subtracting noise, spectrograms become clear.
    - During this script, you must not give any sound to the sensor.
    - You should update noise data everytime before sound recognition, because environmental sound differs everytime.
    - 30 noise samples are enough.
        ```bash
        $ roslaunch sound_classification save_noise.launch
        ```

1. Publish audio -> spectrum -> spectrogram topics.
     - You can set the max/min frequency to be included in the spectrum by `high_cut_freq`/`low_cut_freq` args in `audio_to_spectrogram.launch`.
     - If `gui:=true`, spectrum and spectrogram are visualized.
        ```bash
        $ roslaunch sound_classification audio_to_spectrogram.launch gui:=true
        ```
     - Here is an example spectrogram at quiet environment.
         - Horiozntal axis is time [Hz]
         - Vertical axis is frequency [Hz]

         |Spectrogram w/o noise subtraction|Spectrogram w/ noise subtraction|
         |---|---|
         |![](https://user-images.githubusercontent.com/19769486/86824253-d9e6df80-c0c8-11ea-8946-ca1367c1b1b0.png)|![](https://user-images.githubusercontent.com/19769486/86824246-d81d1c00-c0c8-11ea-8c13-dc9660e89ea0.png)|

1. Collect spectrogram you would like to classify.

    - When the volume exceeds the `threshold`, save the spectrogram at `train_data/original_spectrogram/TARGET_CLASS`.
    - You can use rosbag and stream as sound sources.

    1. Rosbag version (Recommended)
        - I recommend to use rosbag to collect spectrograms. The rosbag makes it easy to use `save_sound.launch` with several parameters.
        - In `target_class:=TARGET_CLASS`, you can set the class name of your target sound.
        - By using `use_rosbag:=true` and `filename:=PATH_TO_ROSBAG`, you can save spectrograms from rosbag.
        - By default, rosbag is paused at first. Press 'Space' key on terminal to start playing rosbag. When rosbag ends, press 'Ctrl-c' to terminate.
        - The newly saved spectrograms are appended to existing spectrograms.
        - You can change threshold of sound saving by `threshold:=xxx`. The smaller the value is, the more easily sound is saved.
            ```bash
            # Save audio to rosbag
            $ roslaunch sound_classification record_audio_rosbag.launch filename:=PATH_TO_ROSBAG
            ```
            ```bash
            # play rosbag and collecting data
            $ export ROS_MASTER_URI=http://localhost:11311
            $ roslaunch sound_classification save_sound.launch use_rosbag:=true \
              filename:=PATH_TO_ROSBAG target_class:=TARGET_CLASS threshold:=0.5
            ```
        - By setting `threshold:=0` and `save_when_sound:=false`, you can collect spectrogram of "no sound".
            ```bash
            # play rosbag and collecting no-sound data
            $ export ROS_MASTER_URI=http://localhost:11311
            $ roslaunch sound_classification save_sound.launch use_rosbag:=true \
              filename:=PATH_TO_ROSBAG target_class:=no_sound threshold:=0 save_when_sound:=false
            ```

    1. Stream version (Not Recommended)
        - You can collect spectrogram directly from audio topic stream.
        - Do not use `use_rosbag:=true`. The other args are the same as the rosbag version. Please see above.
            ```bash
            $ roslaunch sound_classification save_sound.launch \
            save_when_sound:=true target_class:=TARGET_CLASS threshold:=0.5 save_data_rate:=5
            ```

1. Create dateaset for chainer from saved spectrograms.
    - Some data augmentation is executed.
    - `--number 30` means to use maximum 30 images for each class in dataset.
        ```bash
        $ rosrun sound_classification create_dataset.py --number 30
        ```

6. Visualize dataset.
    - You can use `train` arg for train dataset (augmented dataset), `test` arg for test dataset.
    - The spectrograms in the dataset are visualized in random order.
        ```bash
        $ rosrun sound_classification visualize_dataset.py test # train/test
        ```

1. Train with dataset.
    - Default model is `NIN` (Recommended).
    - If you use `vgg16`, pretrained weights of VGG16 is downloaded to `scripts/VGG_ILSVRC_16_layers.npz` at the first time you run this script.
        ```bash
        $ rosrun sound_classification train.py --epoch 30
        ```

1. Classify sounds.
    - It takes a few seconds for the neural network weights to be loaded.
    - `use_rosbag:=true` and `filename:=PATH_TO_ROSBAG` is available if you classify sound with rosbag.
        ```bash
        $ roslaunch sound_classification classify_sound.launch
        ```
    - You can fix class names' color in classification result image by specifying order of class names like below:
        ```xml
        <rosparam>
          target_names: [none, other, chip_bag]
        </rosparam>
        ```
    - Example classification result:
        |no_sound|applause|voice|
        |---|---|---|
        |![](https://user-images.githubusercontent.com/19769486/86828259-ed487980-c0cd-11ea-9f51-7ccb0bc93321.png)|![](https://user-images.githubusercontent.com/19769486/86828249-ec174c80-c0cd-11ea-854c-da1b0fa08e33.png)|![](https://user-images.githubusercontent.com/19769486/86828260-ede11000-c0cd-11ea-9b4e-94ee7b1c1a5f.png)|


## Quick demo

Sound classification demo with your laptop's built-in microphone. You can create dataset from rosbag files in `sample_rosbag/` directory.

#### Classification example gif
![demo](https://user-images.githubusercontent.com/19769486/86830468-b2941080-c0d0-11ea-97f4-45d3e496d059.gif)

#### Commands

1. [Setup](#Setup) environment and write Microphone settings (1. of [Usage](#Usage) section)

1. Save environmental noise
    ```bash
    $ roslaunch sound_classification save_noise.launch
    ```

1. Collect spectrograms from sample rosbags. Press 'Space' to start rosbag.
    - For no_sound class
        ```bash
        $ roslaunch sound_classification save_sound.launch use_rosbag:=true \
        filename:=$(rospack find sound_classification)/sample_rosbag/no_sound.bag \
        target_class:=no_sound threshold:=0 save_when_sound:=false
        ```
    - For applause class
        ```bash
        $ roslaunch sound_classification save_sound.launch use_rosbag:=true \
        filename:=$(rospack find sound_classification)/sample_rosbag/applause.bag \
        target_class:=applause threshold:=0.5
        ```
    - For voice class
        ```bash
        $ roslaunch sound_classification save_sound.launch use_rosbag:=true \
        filename:=$(rospack find sound_classification)/sample_rosbag/voice.bag \
        target_class:=voice threshold:=0.5
        ```

4. Create dataset
    ```bash
    $ rosrun sound_classification create_dataset.py --number 30
    ```

5. Train (takes ~10 minites)
    ```bash
    $ rosrun sound_classification train.py --epoch 20
    ```

6. Classify sound
    ```bash
    $ roslaunch sound_classification classify_sound.launch
    ```
