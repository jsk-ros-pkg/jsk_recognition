tile_image.py
==============

What is this?
-------------

.. image:: ./images/tile_image.png

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

    <node name="tile_image" pkg="jsk_perception" type="tile_image.py" output="screen">
      <rosparam>
        input_topics: [img1/output, img2/output, img3/output, img4/output]
      </rosparam>
    </node>

* ``no_sync`` (type: ``Bool``, optional)

  set ``no_sync`` parameter true if you do not want to synchronize timestamps of ``input_topics``

CLI
---
::

    rosrun jsk_perception tile_image.py _input_topics:='[img1/output, img2/output]'


Sample
------
::

    roslaunch jsk_perception tile_image.launch
