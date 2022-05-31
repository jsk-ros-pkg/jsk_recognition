aws_auto_checkin_app.py
=======================

What is this?
-------------

.. image:: https://d1.awsstatic.com/Solutions/Solutions%20Category%20Template%20Draft/Solution%20Architecture%20Diagrams/auto-check-in-app-architecture.8baa84b79c2294d035c7b9cee323d7c9ba53a43a.png

Face recognition using Amazon Rekognition, see
https://aws.amazon.com/solutions/implementations/auto-check-in-app/
for more info.

Subscribing Topic
-----------------


* ``~image`` (``sensor_msgs/Image``)

  Raw image.

* ``~face_roi`` (``opencv_apps/FaceArrayStamped``)

  Rectangles on the face of input image. Use ROI value.

.. code-block::

        msg.faces[].face.x      : X coordinates of the center of the face image in the ~image input
        msg.faces[].face.y      : Y coordinates of the center of the face image in the ~image input
        msg.faces[].face.width  : Width of the face image
        msg.faces[].face.height : Height of the face iamge

Publishing Topic
----------------

* ``~face_name`` (``opencv_apps/FaceArrayStamped``)

  Publish recognized face name as well as face image. The face.{x,y,width,height} corresponds to input `face_roi`, that means x, y is the center of face rectangle.

Parameters
----------

* ``~use_window`` (Bool, default: ``False``)

  Show input image on the window, if it is true.

* ``~env_path`` (String, default: ``env.json``)

  Json file for environment variables to run aws auto-checkin app. You
  can find how to generate this file on
  https://aws.amazon.com/jp/builders-flash/202004/auto-checkin-app/.
  In addition to that, you need to add "UserName" and "UserPassword"

.. code-block:: json

  {
    "Region": "%%REGION%%",
    "ApiEndpoint" : "%%REST_API_ID%%.execute-api.%%REGION%%.amazonaws.com/prod/rekognize_face",
    "CognitoUserPoolId": "%%COGNITO_USER_POOL_ID%%",
    "CognitoUserPoolClientId": "%%COGNITO_USER_POOL_CLIENT_ID%%",
    "FaceAreaThreshold": 1e4,
    "FaceMarginRatio": 0.2,
    "FaceSimilarityThreshold": 90,
    "CroppedImageWidth": 540,
    "CroppedImageHeight": 540,
    "NameTtlSec": 10,
    "UseDeepLeaningForDetector": true,
    "UserName": "%%YOUR_USER_NAME%%",
    "UserPassword": "%%YOUR_PASSWORD%%"
  }

Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_aws_auto_checkin_app.launch use_window:=true


For JSK user, Download `env.json` file from
[Gdrive](https://drive.google.com/file/d/1WUrRxPtT0ZuRx-IqjGwDBqeR5vZVTkB1/view?usp=sharing)
and put this under `/tmp` directory to run sample code.

To add new people to face database, add face image file to [Amazon
S3](https://console.aws.amazon.com/s3),
`auto-check-in-gapp-register...` buckets
