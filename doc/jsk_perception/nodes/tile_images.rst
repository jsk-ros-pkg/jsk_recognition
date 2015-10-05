tile_images.py
==============

What is this?
-------------

.. image:: ./images/tile_images.png

Publish an image topic by tiling image topics specified.


Subscribing Topic
-----------------
The subscribing topics should be specified with rosparam.
See **Parameters**.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Tiled image.


Parameters
----------

* ``input_topics`` (type: ``StringArray``, required)

  input topic names should be specified like::

    <node name="tile_images" pkg="jsk_perception" type="tile_images.py" output="screen">
      <rosparam>
        input_topics: [img1/output, img2/output, img3/output, img4/output]
      </rosparam>
    </node>


CLI
---
::

    rosrun jsk_perception tile_images.py _input_topics:='[img1/output, img2/output]'


Sample
------
::

    roslaunch jsk_perception tile_images.launch
