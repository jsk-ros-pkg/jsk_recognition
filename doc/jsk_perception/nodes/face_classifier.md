# FaceClassifier
![](image/face_classifier_sample.jpg)

Recognize and classify faces in a image.
This is ROS wrapper of [ageitgey's dlib based face_reconigion](https://github.com/ageitgey/face_recognition).

## How to use

To run sample,

```bash
```

## `face_classifier.py` ROS Interface

### Subscriber

* `~input` (`sensor_msgs/Image`)

    Input image topic

### Publisher

* `~output/rects` (`jsk_recognition_msgs/RectArray`)

    2D bounding boxes of recognized faces

* `~output/labels` (`jsk_recognition_msgs/LabelsArray`)

    Labels for each bounding boxes.

### Parameters

* `~known_person_image_dir` (`String`, required)

    Directory where known person images are placed. Directory structure must be like below.

```
<known_person_image_dir>/
    <person_1>/
        <person_1_face-1>.jpg
        <person_1_face-2>.jpg
        .
        .
        <person_1_face-n>.jpg
    <person_2>/
        <person_2_face-1>.jpg
        <person_2_face-2>.jpg
        .
        .
        <person_2_face-n>.jpg
    .
    .
    <person_n>/
        <person_n_face-1>.jpg
        <person_n_face-2>.jpg
        .
        .
        <person_n_face-n>.jpg
```

* `~fitting_method` (`String`, deafult: `svm`)

    Fitting methods for classification. Options are `svm` or `knn`.
