aws_auto_checkin_app.py
=======================

What is this?
-------------

.. image:: https://d1.awsstatic.com/Solutions/Solutions%20Category%20Template%20Draft/Solution%20Architecture%20Diagrams/auto-check-in-app-architecture.8baa84b79c2294d035c7b9cee323d7c9ba53a43a.png

Face recognition using Amazon Rekognition, see
`SearchFacesByImage <https://docs.aws.amazon.com/rekognition/latest/APIReference/API_SearchFacesByImage.html>`_ and `Auto Check-In App <https://aws.amazon.com/solutions/implementations/auto-check-in-app/>`_ for more info.  

Subscribing Topic
-----------------


* ``image/compressed`` (``sensor_msgs/CompressedImage``)

  Input compressed image. (only when ``~image_transport`` is ``compressed``.)
  
  This topic name is resolved from ``image``.

* ``image`` (``sensor_msgs/Image``)

  Input raw image. (only when ``~image_transport`` is ``raw``.)

* ``face_roi`` (``opencv_apps/FaceArrayStamped``)

  Rectangles on the face of input image. Use ROI value.

.. code-block::

        msg.faces[].face.x      : X coordinates of the center of the face image in the ~image input
        msg.faces[].face.y      : Y coordinates of the center of the face image in the ~image input
        msg.faces[].face.width  : Width of the face image
        msg.faces[].face.height : Height of the face iamge

Publishing Topic
----------------

* ``face_name`` (``opencv_apps/FaceArrayStamped``)

  Publish recognized face name as well as face image. The face.{x,y,width,height} corresponds to input `face_roi`, that means x, y is the center of face rectangle.

* ``~output/rects`` (``jsk_recognition_msgs/RectArray``)

  Rectangles of matched faces.

* ``~output/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Detected face class labels and probabilities.

* ``~image`` (``sensor_msgs/Image``)

  Passthourgh input image. This message contains face detected image
  when ``~always_publish`` is false.

Parameters
----------

* ``~use_window`` (Bool, default: ``False``)

  Show input image on the window, if it is true.

* ``~approximate_sync`` (Bool, default: ``True``)

  Approximately synchronize inputs if it's true.

* ``~queue_size`` (Int, default: ``100``)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.

* ``~slop`` (Float, default: ``1.0``)

  Maximum allowed time for approximate synchronization in [sec].

  This parameter is enabled only when ``~approximate_sync`` is True.

* ``~classifier_name`` (String, default: ``rospy.get_name()``)

  Classifier name written to ``classifier`` field of ``~output/class``.

* ``~image_transport`` (String, default: ``compressed``)

  Set ``raw`` to subscribe raw image.

* ``~env_path`` (String, default: ``env.json``)

  Json file for environment variables.

  See `DynamoDB <https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/dynamodb.html>`_ for how to obtain DynamoDB Table name and `Rekognition <https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/rekognition.html>`_ for CollectionId.

.. code-block:: json

  {
    "Region": "%%REGION%%",
    "DynamodbTable": "%%DYNAMODB_TABLE%%",
    "CollectionId": "%%COLLECTION_ID%%",
    "FaceAreaThreshold": 1e4,
    "FaceSimilarityThreshold": 90,
    "MaxFaces": 1
  }

* ``~aws_credentials_path`` (String, Default: ``aws.json``)

  Json file for aws access key and secret key to run AWS Rekognition.
  See `Set up an AWS account and create an IAM user
  <https://docs.aws.amazon.com/rekognition/latest/dg/setting-up.html>`_
  for how to obtain keys.

.. code-block:: json

  {
      "aws_access_key_id" : "####################",
      "aws_secret_access_key" : "********************"
  }

* ``~always_publish`` (Bool, Default: ``True``)

  Set false to publish only when face is detected.

Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_aws_auto_checkin_app.launch use_aws_face_detection:=true

If ``use_aws_face_detection`` is true, AWS face detection API is used. Otherwise OpenCV face detection is used.
AWS face detection is more accurate.

For JSK user, Download ``env.json`` file and ``aws.json`` from
`Gdrive <https://drive.google.com/drive/folders/10kVoswI3EgDG4x1tTW0iSQkqUwsJkOqg?usp=sharing>`_
and put these under ``/tmp`` directory to run sample code.

To add new people to face database, add face image file to
`Amazon S3 <https://console.aws.amazon.com/s3>`_,
`auto-check-in-gapp-register...` buckets

For more info about managing the face collection, see `Searching faces in a collection <https://docs.aws.amazon.com/rekognition/latest/dg/collections.html>`_.
