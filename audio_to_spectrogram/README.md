# audio_to_spectrogram

This package converts audio data to spectrum and spectrogram data.

# Usage
By following command, you can publish audio, spectrum and spectrogram topics. Please set correct args for your microphone configuration, such as mic\_sampling\_rate or bitdepth.

```bash
roslaunch audio_to_spectrogram audio_to_spectrogram.launch
```

Here is an example using rosbag with 300Hz audio.
```bash
roslaunch audio_to_spectrogram sample_audio_to_spectrogram.launch
```

|Audio Amplitude|Spectrum|Spectrogram|
|---|---|---|
|<img src="docs/images/audio_amplitude.jpg" width="429">|![](https://user-images.githubusercontent.com/19769486/82075694-9a7ac300-9717-11ea-899c-db6119a76d52.png)|![](https://user-images.githubusercontent.com/19769486/82075685-96e73c00-9717-11ea-9abc-e6e74104d666.png)|

# Scripts

## audio_to_spectrum.py
  A script to convert audio to spectrum.

  - ### Publishing topics

    - `~spectrum` (`jsk_recognition_msgs/Spectrum`)

      Spectrum data calculated from audio by FFT.

  - ### Subscribing topics
    - `audio` (`audio_common_msgs/AudioData`)

      Audio stream data from microphone. The audio format must be `wave`.

  - ### Parameters
    - `~mic_sampling_rate` (`Int`, default: `16000`)

      Sampling rate [Hz] of microphone. Namely, sampling rate of audio topic.

    - `~fft_sampling_period` (`Double`, default: `0.3`)

      Period [s] to sample audio data for one FFT.

    - `~n_channel` (`Int`, default: `1`)

      Number of channel of microphone.

    - `~target_channel` (`Int`, default: `0`)

      Target channel.

    - `~bitdepth` (`Int`, default: `16`)

      Number of bits per audio data.

    - `~high_cut_freq` (`Int`, default: `800`)

      Threshold to limit the maximum frequency of the output spectrum.

    - `~low_cut_freq` (`Int`, default: `1`)

      Threshold to limit the minimum frequency of the output spectrum.

## spectrum_to_spectrogram.py
  A script to convert spectrum to spectrogram.

  - ### Publishing topics
    - `~spectrogram` (`sensor_msgs/Image`)

      Spectrogram data, which is concatenation of spectrum in time series. Image format is 32FC1.

  - ### Subscribing topics
    - `~spectrum` (`jsk_recognition_msgs/Spectrum`)

      Spectrum data calculated from audio by FFT.

  - ### Parameters
    - `~image_height` (`Int`, default: `300`)

      Number of vertical (frequency axis) pixels in output spectrogram.

    - `~image_width` (`Int`, default: `300`)

      Number of horizontal (time axis) pixels in output spectrogram.

    - `~spectrogram_period` (`Double`, default: `5`)

      Period [s] to store spectrum data to create one spectrogram topic.

    - `~publish_rate` (`Double`, default: `image_width / spectrogram_period`)

      Publish rate [Hz] of spectrogram topic.

## audio_amplitude_plot.py

  A script to publish audio amplitude plot image.

![](docs/images/audio_amplitude.jpg)

  - ### Publishing topics

    - `~output/viz` (`sensor_msgs/Image`)

      Audio amplitude plot image.

  - ### Subscribing topics

    - `~audio` (`audio_common_msgs/AudioData`)

      Audio stream data from microphone. The audio format must be `wave`.

  - ### Parameters
    - `~mic_sampling_rate` (`Int`, default: `16000`)

      Sampling rate [Hz] of microphone. Namely, sampling rate of audio topic.

    - `~n_channel` (`Int`, default: `1`)

      Number of channel of microphone.

    - `~target_channel` (`Int`, default: `0`)

      Target channel.

    - `~bitdepth` (`Int`, default: `16`)

      Number of bits per audio data.

    - `~maximum_amplitude` (`Int`, default: `10000`)

      Maximum range of amplitude to plot.

    - `~window_size` (`Double`, default: `10.0`)

      Window size of sound input to plot.

    - `~rate` (`Double`, default: `10.0`)

      Publish rate [Hz] of audio amplitude image topic.

## spectrum_plot.py

  A script to publish frequency vs amplitude plot image.

![](docs/images/spectrum.jpg)

  - ### Publishing topics

    - `~output/viz` (`sensor_msgs/Image`)

      Frequency vs amplitude plot image.

  - ### Subscribing topics

    - `~spectrum` (`jsk_recognition_msgs/Spectrum`)

      Spectrum data calculated from audio by FFT.

  - ### Parameters
    - `~plot_amp_min` (`Double`, default: `0.0`)

      Minimum value of amplitude in plot

    - `~plot_amp_max` (`Double`, default: `20.0`)

      Maximum value of amplitude in plot

    - `~queue_size` (`Int`, default: `1`)

      Queue size of spectrum subscriber
