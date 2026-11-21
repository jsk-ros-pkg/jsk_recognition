"""Exercise the node_scripts on ROS 2: publish to each node and check what it
publishes."""
import os
import runpy
import time
import unittest

# a field of a wrong type raises in the test instead of aborting at publish
os.environ.setdefault('ROS_PYTHON_CHECK_FIELDS', '1')

from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import ModelCoefficientsArray
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from pcl_msgs.msg import ModelCoefficients
from pcl_msgs.msg import PointIndices
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


NODE_SCRIPTS = os.path.join(os.path.dirname(__file__), '..', 'node_scripts')


def node_class(script, name):
    return runpy.run_path(os.path.join(NODE_SCRIPTS, script))[name]


class TestNodeScriptsROS2(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.client = self.add_node(rclpy.create_node('client'))

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()

    def add_node(self, node):
        self.executor.add_node(node)
        self.nodes.append(node)
        return node

    def params(self, **kwargs):
        return [Parameter(k, value=v) for k, v in kwargs.items()]

    def run_node(self, node, inputs, out_topic, out_type, seconds=10.0):
        """Publish inputs [(topic, msg)] until node publishes on out_topic,
        return the first output."""
        if node not in self.nodes:
            self.add_node(node)
        received = []
        self.client.create_subscription(
            out_type, out_topic, received.append, 10)
        pubs = [(self.client.create_publisher(type(msg), topic, 10), msg)
                for topic, msg in inputs]
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not received:
            for pub, msg in pubs:
                pub.publish(msg)
            for _ in range(4):
                self.executor.spin_once(timeout_sec=0.05)
        self.assertTrue(received, 'nothing on {}'.format(out_topic))
        return received[0]

    def stamped(self, msg, sec=1):
        msg.header.frame_id = 'frame'
        msg.header.stamp.sec = sec
        return msg

    def test_rect_array_to_polygon_array(self):
        cls = node_class('rect_array_to_polygon_array.py',
                         'RectArrayToPolygonArray')
        rects = self.stamped(RectArray(
            rects=[Rect(x=1, y=2, width=3, height=4)]))
        out = self.run_node(
            cls(), [('/rect_array_to_polygon_array/input', rects)],
            '/rect_array_to_polygon_array/output', PolygonArray)
        self.assertEqual(out.header.frame_id, 'frame')
        self.assertEqual(
            [(p.x, p.y) for p in out.polygons[0].polygon.points],
            [(1, 2), (1, 6), (4, 6), (4, 2)])

    def test_image_16uc1_to_32fc1(self):
        cls = node_class('image_16uc1_to_32fc1.py', 'Image16UC1To32FC1')
        import numpy as np
        img = self.stamped(Image(
            height=1, width=2, encoding='16UC1', step=4,
            data=np.array([0, 32768], dtype=np.uint16).tobytes()))
        out = self.run_node(
            cls(), [('/image_16uc1_to_32fc1/input', img)],
            '/image_16uc1_to_32fc1/output', Image)
        self.assertEqual(out.encoding, '32FC1')
        values = np.frombuffer(bytes(out.data), dtype=np.float32)
        np.testing.assert_allclose(values, [0.0, 0.5])

    def test_add_bounding_box_array(self):
        cls = node_class('add_bounding_box_array.py', 'AddBoundingBoxArray')
        node = cls(parameter_overrides=self.params(topics=['/a', '/b']))
        a = self.stamped(BoundingBoxArray(boxes=[BoundingBox(label=1)]))
        b = self.stamped(BoundingBoxArray(
            boxes=[BoundingBox(label=2), BoundingBox(label=3)]))
        out = self.run_node(node, [('/a', a), ('/b', b)],
                            '/add_bounding_box_array/output',
                            BoundingBoxArray)
        self.assertEqual([box.label for box in out.boxes], [1, 2, 3])

    def test_add_cluster_indices(self):
        cls = node_class('add_cluster_indices.py', 'AddClusterIndices')
        node = cls(parameter_overrides=self.params(topics=['/a', '/b']))
        a = self.stamped(ClusterPointIndices(
            cluster_indices=[PointIndices(indices=[0, 1])]))
        b = self.stamped(ClusterPointIndices(
            cluster_indices=[PointIndices(indices=[2])]))
        out = self.run_node(node, [('/a', a), ('/b', b)],
                            '/add_cluster_indices/output',
                            ClusterPointIndices)
        self.assertEqual([list(c.indices) for c in out.cluster_indices],
                         [[0, 1], [2]])

    def test_rect_array_to_cluster_point_indices(self):
        cls = node_class('rect_array_to_cluster_point_indices.py',
                         'RectArrayToClusterPointIndices')
        node = cls(parameter_overrides=self.params(img_height=3, img_width=4))
        rects = self.stamped(RectArray(
            rects=[Rect(x=1, y=1, width=2, height=2)]))
        out = self.run_node(
            node, [('/rect_array_to_cluster_point_indices/input', rects)],
            '/rect_array_to_cluster_point_indices/output',
            ClusterPointIndices)
        self.assertEqual(list(out.cluster_indices[0].indices),
                         [5, 6, 9, 10])

    def test_polygon_array_to_polygon(self):
        cls = node_class('polygon_array_to_polygon.py',
                         'PolygonArrayToPolygon')
        node = cls(parameter_overrides=self.params(index=1))
        polygons = self.stamped(PolygonArray(polygons=[
            PolygonStamped(), self.stamped(PolygonStamped(), sec=2)]))
        polygons.polygons[1].polygon.points = [Point32(x=1.0)]
        out = self.run_node(
            node, [('/polygon_array_to_polygon/input', polygons)],
            '/polygon_array_to_polygon/output', PolygonStamped)
        self.assertEqual(out.header.stamp.sec, 2)
        self.assertEqual(out.polygon.points[0].x, 1.0)

    def test_pose_array_to_pose(self):
        cls = node_class('pose_array_to_pose.py', 'PoseArrayToPose')
        node = cls(parameter_overrides=self.params(index=1))
        poses = self.stamped(PoseArray(poses=[Pose(), Pose()]))
        poses.poses[1].position.x = 1.0
        out = self.run_node(node, [('/pose_array_to_pose/input', poses)],
                            '/pose_array_to_pose/output', PoseStamped)
        self.assertEqual(out.header.frame_id, 'frame')
        self.assertEqual(out.pose.position.x, 1.0)

    def test_polygon_array_to_box_array(self):
        cls = node_class('polygon_array_to_box_array.py',
                         'PolygonArrayToRectBoxArray')
        node = cls(parameter_overrides=self.params(thickness=0.01))
        polygon = self.stamped(PolygonStamped())
        polygon.polygon.points = [
            Point32(x=0.0, y=0.0, z=1.0), Point32(x=2.0, y=0.0, z=1.0),
            Point32(x=2.0, y=1.0, z=1.0), Point32(x=0.0, y=1.0, z=1.0)]
        polygons = self.stamped(PolygonArray(polygons=[polygon]))
        coeffs = self.stamped(ModelCoefficientsArray(coefficients=[
            ModelCoefficients(values=[0.0, 0.0, 1.0, -1.0])]))
        out = self.run_node(
            node, [('/polygon_array_to_box_array/input/polygons', polygons),
                   ('/polygon_array_to_box_array/input/coefficients',
                    coeffs)],
            '/polygon_array_to_box_array/output/boxes', BoundingBoxArray)
        box = out.boxes[0]
        self.assertAlmostEqual(box.pose.position.x, 1.0, places=5)
        self.assertAlmostEqual(box.pose.position.y, 0.5, places=5)
        self.assertAlmostEqual(box.pose.position.z, 1.0, places=5)
        # which side of the rectangle is x depends on shapely
        self.assertEqual(
            sorted([round(box.dimensions.x, 5), round(box.dimensions.y, 5)]),
            [1.0, 2.0])
        self.assertAlmostEqual(box.dimensions.z, 0.01, places=5)

    def test_bounding_box_array_publisher(self):
        cls = node_class('bounding_box_array_publisher.py',
                         'BoundingBoxArrayPublisher')
        node = rclpy.create_node(
            'bounding_box_array_publisher', parameter_overrides=self.params(
                frame_id='frame', rate=10.0,
                boxes='[{position: [1, 2, 3], dimension: [0.1, 0.2, 0.3],'
                      ' rotation: [0, 0, 1.5707963], label: 4}]'))
        cls(node)
        out = self.run_node(node, [],
                            '/bounding_box_array_publisher/output',
                            BoundingBoxArray)
        box = out.boxes[0]
        self.assertEqual(out.header.frame_id, 'frame')
        self.assertEqual((box.pose.position.x, box.pose.position.y,
                          box.pose.position.z), (1.0, 2.0, 3.0))
        self.assertAlmostEqual(box.pose.orientation.z, 0.70710678, places=5)
        self.assertAlmostEqual(box.dimensions.z, 0.3)
        self.assertEqual(box.label, 4)

    def test_polygon_array_publisher(self):
        cls = node_class('polygon_array_publisher.py',
                         'PolygonArrayPublisher')
        node = rclpy.create_node(
            'polygon_array_publisher', parameter_overrides=self.params(
                frame_id='frame', publish_rate=10.0, publish_coeffs=True,
                polygons='[{points: [[0, 0, 1], [1, 0, 1], [0, 1, 1]],'
                         ' label: 2}]'))
        cls(node)
        out = self.run_node(node, [], '/polygon_array_publisher/output',
                            PolygonArray)
        self.assertEqual(out.header.frame_id, 'frame')
        self.assertEqual(len(out.polygons[0].polygon.points), 3)
        self.assertEqual(list(out.labels), [2])
        coeffs = self.run_node(
            node, [], '/polygon_array_publisher/output/coefficients',
            ModelCoefficientsArray)
        self.assertEqual(
            [round(v, 5) for v in coeffs.coefficients[0].values],
            [0.0, 0.0, 1.0, -1.0])


if __name__ == '__main__':
    unittest.main()
