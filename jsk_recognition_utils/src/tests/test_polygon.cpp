#include "jsk_recognition_utils/geo/polygon.h"
#include <gtest/gtest.h>

// Polygon on ROS 1 and ROS 2: built from the messages, decomposed into
// triangles by pcl::EarClippingPatched

static geometry_msgs::Polygon makePolygon(const std::vector<std::pair<float, float> >& xy,
                                          float z)
{
  geometry_msgs::Polygon msg;
  for (size_t i = 0; i < xy.size(); i++) {
    geometry_msgs::Point32 p;
    p.x = xy[i].first;
    p.y = xy[i].second;
    p.z = z;
    msg.points.push_back(p);
  }
  return msg;
}

static double sumOfAreas(const std::vector<jsk_recognition_utils::Polygon::Ptr>& triangles)
{
  double area = 0;
  for (size_t i = 0; i < triangles.size(); i++) {
    area += triangles[i]->area();
  }
  return area;
}

TEST(Polygon, square)
{
  std::vector<std::pair<float, float> > xy;
  xy.push_back(std::make_pair(0.0f, 0.0f));
  xy.push_back(std::make_pair(1.0f, 0.0f));
  xy.push_back(std::make_pair(1.0f, 1.0f));
  xy.push_back(std::make_pair(0.0f, 1.0f));
  jsk_recognition_utils::Polygon polygon
    = jsk_recognition_utils::Polygon::fromROSMsg(makePolygon(xy, 0.5));
  EXPECT_EQ(4u, polygon.getVertices().size());
  Eigen::Vector3f normal = polygon.getNormal();
  EXPECT_NEAR(0.0, normal[0], 1e-6);
  EXPECT_NEAR(0.0, normal[1], 1e-6);
  EXPECT_NEAR(1.0, normal[2], 1e-6);
  std::vector<jsk_recognition_utils::Polygon::Ptr> triangles
    = polygon.decomposeToTriangles();
  ASSERT_EQ(2u, triangles.size());
  for (size_t i = 0; i < triangles.size(); i++) {
    EXPECT_EQ(3u, triangles[i]->getVertices().size());
    EXPECT_NEAR(1.0, triangles[i]->getNormal()[2], 1e-6);
  }
  EXPECT_NEAR(1.0, sumOfAreas(triangles), 1e-6);
}

TEST(Polygon, lShape)
{
  // concave, counterclockwise
  std::vector<std::pair<float, float> > xy;
  xy.push_back(std::make_pair(0.0f, 0.0f));
  xy.push_back(std::make_pair(2.0f, 0.0f));
  xy.push_back(std::make_pair(2.0f, 1.0f));
  xy.push_back(std::make_pair(1.0f, 1.0f));
  xy.push_back(std::make_pair(1.0f, 2.0f));
  xy.push_back(std::make_pair(0.0f, 2.0f));
  jsk_recognition_utils::Polygon polygon
    = jsk_recognition_utils::Polygon::fromROSMsg(makePolygon(xy, 0.0));
  Eigen::Vector3f normal = polygon.getNormal();
  EXPECT_NEAR(1.0, normal[2], 1e-6);
  std::vector<jsk_recognition_utils::Polygon::Ptr> triangles
    = polygon.decomposeToTriangles();
  ASSERT_EQ(4u, triangles.size());
  for (size_t i = 0; i < triangles.size(); i++) {
    EXPECT_NEAR(1.0, triangles[i]->getNormal()[2], 1e-6);
  }
  EXPECT_NEAR(3.0, sumOfAreas(triangles), 1e-6);
  // no triangle covers the notch at (1.5, 1.5)
  for (size_t i = 0; i < triangles.size(); i++) {
    EXPECT_FALSE(triangles[i]->isInside(Eigen::Vector3f(1.5, 1.5, 0)));
  }
}

TEST(Polygon, fromPolygonArray)
{
  std::vector<std::pair<float, float> > xy;
  xy.push_back(std::make_pair(0.0f, 0.0f));
  xy.push_back(std::make_pair(1.0f, 0.0f));
  xy.push_back(std::make_pair(0.0f, 1.0f));
  jsk_recognition_msgs::PolygonArray msg;
  msg.polygons.resize(2);
  msg.polygons[0].polygon = makePolygon(xy, 0.0);
  msg.polygons[1].polygon = makePolygon(xy, 1.0);
  std::vector<jsk_recognition_utils::Polygon::Ptr> polygons
    = jsk_recognition_utils::Polygon::fromROSMsg(msg);
  ASSERT_EQ(2u, polygons.size());
  EXPECT_TRUE(polygons[0]->isTriangle());
  EXPECT_NEAR(1.0, polygons[1]->getVertices()[0][2], 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
