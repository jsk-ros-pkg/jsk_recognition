Contributions
=============

``jsk_recognition`` welcomes your contributions.
We maintain this project on `Github`_.

.. _Github: http://github.com/jsk-ros-pkg/jsk_recognition


Issues
------

If you find a problem, go to `Issues`_ page and report it.

.. _Issues: https://github.com/jsk-ros-pkg/jsk_recognition/issues


Pull request
------------

If you want to fix a bug or add new feature, go to `Pull requests`_ page and make PR.

.. _Pull requests: https://github.com/jsk-ros-pkg/jsk_recognition/pulls


Contribution guideline
----------------------

- Letter size of arguments in ``launch`` file
    - Small letter arguments:
        - Users can change them on command line.
        - Example: ``roslaunch jsk_perception sample_ssd_object_detector.launch gpu:=0``
    - Capital letter arguments:
        - Users are not recommended to change them on command line.
        - Users should change their values in a launch file with ``<include>`` tag.
