sklearn_classifier.py
=====================

What is this?
-------------

Load trained classifier model and classify with input vector array.


Subscribing Topic
-----------------

* ``~input`` (``jsk_recognition_msgs/VectorArray``)

  Input vector array to do classification.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClassificationResult``)

  Classification result.


Parameters
----------

* ``~clf_path`` (type: ``String``, required)

  Trained classifier path. Currently we support ``*.pkl.gz`` file.
  To train the classifier, use ``jsk_perception/sklearn_classifier_trainer.py``.


Sample
------

See `here <../bof_object_recognition.html>`_.
