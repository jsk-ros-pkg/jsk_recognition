// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/octomap_server_contact.h"
#include <algorithm>

using namespace octomap;
using octomap_msgs::Octomap;

namespace jsk_pcl_ros
{
  OctomapServerContact::OctomapServerContact(const ros::NodeHandle &privateNh)
  : OctomapServer(privateNh),
    DiagnosticNodelet("OctomapServerContact"),
    m_octreeContact(NULL),
    m_publishUnknownSpace(false),
    m_publishFrontierSpace(false),
    m_offsetVisualizeUnknown(0),
    m_maxRangeProximity(0.05),
    m_occupancyMinX(-std::numeric_limits<double>::max()),
    m_occupancyMaxX(std::numeric_limits<double>::max()),
    m_occupancyMinY(-std::numeric_limits<double>::max()),
    m_occupancyMaxY(std::numeric_limits<double>::max()),
    m_useContactSurface(true)
  {
    delete m_octree;
    m_octree = NULL;
    m_octree = new OcTreeContact(m_res);
    m_octreeContact = dynamic_cast<OcTreeContact*>(m_octree);
    if (!m_octreeContact) {
      ROS_ERROR("Could not convert OcTreeContact from OcTree");
      assert(m_octreeContact);
    }

    m_useHeightMap = false;

    privateNh.param("publish_unknown_space", m_publishUnknownSpace, m_publishUnknownSpace);
    privateNh.param("offset_vis_unknown", m_offsetVisualizeUnknown, m_offsetVisualizeUnknown);
    privateNh.param("sensor_model/max_range_proximity", m_maxRangeProximity, m_maxRangeProximity);
    privateNh.param("publish_frontier_space", m_publishFrontierSpace, m_publishFrontierSpace);

    privateNh.param("occupancy_min_x", m_occupancyMinX,m_occupancyMinX);
    privateNh.param("occupancy_max_x", m_occupancyMaxX,m_occupancyMaxX);
    privateNh.param("occupancy_min_y", m_occupancyMinY,m_occupancyMinY);
    privateNh.param("occupancy_max_y", m_occupancyMaxY,m_occupancyMaxY);

    privateNh.param("use_contact_surface", m_useContactSurface,m_useContactSurface);

    double r, g, b, a;
    privateNh.param("color_unknown/r", r, 0.5);
    privateNh.param("color_unknown/g", g, 0.5);
    privateNh.param("color_unknown/b", b, 0.7);
    privateNh.param("color_unknown/a", a, 1.0);
    m_colorUnknown.r = r;
    m_colorUnknown.g = g;
    m_colorUnknown.b = b;
    m_colorUnknown.a = a;
    privateNh.param("color_frontier/r", r, 1.0);
    privateNh.param("color_frontier/g", g, 0.0);
    privateNh.param("color_frontier/b", b, 0.0);
    privateNh.param("color_frontier/a", a, 1.0);
    m_colorFrontier.r = r;
    m_colorFrontier.g = g;
    m_colorFrontier.b = b;
    m_colorFrontier.a = a;

    m_unknownPointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_unknown_point_cloud_centers", 1, m_latchedTopics);
    m_umarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("unknown_cells_vis_array", 1, m_latchedTopics);

    m_pointProximitySub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "proximity_in", 5);
    m_tfPointProximitySub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointProximitySub, m_tfListener, m_worldFrameId, 5);
    m_tfPointProximitySub->registerCallback(boost::bind(&OctomapServerContact::insertProximityCallback, this, _1));

    m_frontierPointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_frontier_point_cloud_centers", 1, m_latchedTopics);
    m_fromarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("frontier_cells_vis_array", 1, m_latchedTopics);

    m_contactSensorSub.subscribe(m_nh, "contact_sensors_in", 2);
    m_tfContactSensorSub.reset(new tf::MessageFilter<jsk_recognition_msgs::ContactSensorArray> (
                                 m_contactSensorSub, m_tfListener, m_worldFrameId, 2));
    m_tfContactSensorSub->registerCallback(boost::bind(&OctomapServerContact::insertContactSensorCallback, this, _1));

    initContactSensor(privateNh);
  }

  OctomapServerContact::~OctomapServerContact() {
  }

  void OctomapServerContact::initContactSensor(const ros::NodeHandle &privateNh) {
    double defaultPadding, defaultScale;
    privateNh.param<double>("self_see_defaultPadding", defaultPadding, .01);
    privateNh.param<double>("self_see_defaultScale", defaultScale, 1.0);
    std::vector<robot_self_filter::LinkInfo> links;

    if (!privateNh.hasParam("self_see_links")) {
      ROS_WARN("No links specified for self filtering.");
    }
    else {
      XmlRpc::XmlRpcValue sslVals;;
      privateNh.getParam("self_see_links", sslVals);
      if (sslVals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Self see links need to be an array");
      }
      else {
        if (sslVals.size() == 0) {
          ROS_WARN("No values in self see links array");
        }
        else {
          for (int i = 0; i < sslVals.size(); i++) {
            robot_self_filter::LinkInfo li;
            if (sslVals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              ROS_WARN("Self see links entry %d is not a structure.  Stopping processing of self see links",i);
              break;
            }
            if (!sslVals[i].hasMember("name")) {
              ROS_WARN("Self see links entry %d has no name.  Stopping processing of self see links",i);
              break;
            }
            li.name = std::string(sslVals[i]["name"]);
            if (!sslVals[i].hasMember("padding")) {
              ROS_DEBUG("Self see links entry %d has no padding.  Assuming default padding of %g",i,defaultPadding);
              li.padding = defaultPadding;
            }
            else {
              li.padding = sslVals[i]["padding"];
            }
            if (!sslVals[i].hasMember("scale")) {
              ROS_DEBUG("Self see links entry %d has no scale.  Assuming default scale of %g",i,defaultScale);
              li.scale = defaultScale;
            }
            else {
              li.scale = sslVals[i]["scale"];
            }
            links.push_back(li);
          }
        }
      }
    }
    m_selfMask = boost::shared_ptr<robot_self_filter::SelfMaskNamedLink>(new robot_self_filter::SelfMaskNamedLink(m_tfListener, links));
  }

  void OctomapServerContact::insertProximityCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    // ROS_INFO("insertProximityCallback is called");

    ros::WallTime startTime = ros::WallTime::now();

    //
    // ground filtering in base frame
    //
    PCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try {
      m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    pc.header = pc.header;

    insertScanProximity(sensorToWorldTf.getOrigin(), pc);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu pts, %f sec)", pc.size(), total_elapsed);

    publishAll(cloud->header.stamp);
  }

  void OctomapServerContact::insertScanProximity(const tf::Point& sensorOriginTf, const PCLPointCloud& pc) {
    point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
        || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
      {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
      }

#ifdef COLOR_OCTOMAP_SERVER
    unsigned char* colors = new unsigned char[3];
#endif

    // instead of direct scan insertion, compute update to filter ground:
    KeySet free_cells, occupied_cells;

    // all other points: free on ray, occupied on endpoint:
    for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it) {
      point3d point(it->x, it->y, it->z);
      // maxrange check
      if ((m_maxRangeProximity < 0.0) || ((point - sensorOrigin).norm() <= m_maxRangeProximity) ) {

        // free cells
        if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }
        // occupied endpoint
        OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)){
          occupied_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an occupied node
          const int rgb = *reinterpret_cast<const int*>(&(it->rgb)); // TODO: there are other ways to encode color than this one
          colors[0] = ((rgb >> 16) & 0xff);
          colors[1] = ((rgb >> 8) & 0xff);
          colors[2] = (rgb & 0xff);
          m_octree->averageNodeColor(it->x, it->y, it->z, colors[0], colors[1], colors[2]);
#endif
        }
      } else {// ray longer than maxrange:;
        point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRangeProximity;
        if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (m_octree->coordToKeyChecked(new_end, endKey)){
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
          } else{
            ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
          }
        }
      }
    }

    // mark free cells only if not seen occupied in this cloud
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
      if (occupied_cells.find(*it) == occupied_cells.end()){
        m_octree->updateNode(*it, false);
      }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
      m_octree->updateNode(*it, true);
    }

    // TODO: eval lazy+updateInner vs. proper insertion
    // non-lazy by default (updateInnerOccupancy() too slow for large maps)
    //m_octree->updateInnerOccupancy();
    octomap::point3d minPt, maxPt;
    ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

    // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
    //   if (m_maxTreeDepth < 16)
    //   {
    //      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
    //      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
    //      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
    //      m_updateBBXMin = tmpMin;
    //      m_updateBBXMax = tmpMax;
    //   }

    // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
    minPt = m_octree->keyToCoord(m_updateBBXMin);
    maxPt = m_octree->keyToCoord(m_updateBBXMax);
    ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
    ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

    if (m_compressMap)
      m_octree->prune();
  }

  void OctomapServerContact::insertContactSensor(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg) {
    std::vector<jsk_recognition_msgs::ContactSensor> datas = msg->datas;

    // setup tf transformation between octomap and each link
    {
      std_msgs::Header tmpHeader;
      tmpHeader.frame_id = m_worldFrameId;
      tmpHeader.stamp = msg->header.stamp;
      if(!m_selfMask->assumeFrame(tmpHeader)) {
        ROS_ERROR_STREAM("failed tf transformation in insertContactSensor");
        return;
      }
    }

    // clamp min and max points  cf. https://github.com/OctoMap/octomap/issues/146
    point3d pmin_raw( m_occupancyMinX, m_occupancyMinY, m_occupancyMinZ );
    point3d pmax_raw( m_occupancyMaxX, m_occupancyMaxY, m_occupancyMaxZ );
    point3d pmin = m_octree->keyToCoord(m_octree->coordToKey(pmin_raw));
    point3d pmax = m_octree->keyToCoord(m_octree->coordToKey(pmax_raw));
    float diff[3];
    unsigned int steps[3];
    double resolution = m_octreeContact->getResolution();
    for (int i = 0; i < 3; ++i) {
      diff[i] = pmax(i) - pmin(i);
      steps[i] = floor(diff[i] / resolution);
      // std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
    }

    // loop for grids of octomap
    if (m_useContactSurface) {
      std::vector< std::vector<bool> > containFlag(datas.size(), std::vector<bool>(8));
      point3d p = pmin;
      for (unsigned int x = 0; x < steps[0]; ++x) {
        p.x() += resolution;
        for (unsigned int y = 0; y < steps[1]; ++y) {
          p.y() += resolution;
          for (unsigned int z = 0; z < steps[2]; ++z) {
            // std::cout << "querying p=" << p << std::endl;
            p.z() += resolution;
            // loop for vertices of each gird
            point3d vertexOffset(-resolution/2.0, -resolution/2.0, -resolution/2.0);
            point3d vertex;
            for (int i = 0; i < 2; i++) {
              if (i == 1) { vertexOffset.z() += resolution; }
              for (int j = 0; j < 2; j++) {
                if (j == 1) { vertexOffset.y() += resolution; }
                for (int k = 0; k < 2; k++) {
                  if (k == 1) { vertexOffset.x() += resolution; }
                  vertex = p + vertexOffset;
                  // std::cout << "vertex = " << vertex << std::endl;
                  // loop for each body link
                  for (int l=0; l<datas.size(); l++) {
                    if (m_selfMask->getMaskContainmentforNamedLink(vertex(0), vertex(1), vertex(2), datas[l].link_name) == robot_self_filter::INSIDE) {
                      // std::cout << "inside vertex = " << vertex << std::endl;
                      containFlag[l][i+j*2+k*4] = true;
                    }
                    else {
                      containFlag[l][i+j*2+k*4] = false;
                    }
                  }
                }
                vertexOffset.x() -= resolution;
              }
              vertexOffset.y() -= resolution;
            }

            // update probability of grid
            std::vector<bool> containFlagLinkSum(8, false);
            std::vector<bool> containFlagVerticesSum(datas.size(), false);
            std::vector<bool> containFlagVerticesProd(datas.size(), true);
            bool insideFlag = false;
            bool surfaceFlag = false;
            for (int l = 0; l < datas.size(); l++) {
              for (int i = 0; i < 8; i++) {
                if (containFlag[l][i]) {
                  containFlagLinkSum[i] = true;
                  containFlagVerticesSum[l] = true;
                }
                else {
                  containFlagVerticesProd[l] = false;
                }
              }
            }
            insideFlag = (std::find(containFlagLinkSum.begin(), containFlagLinkSum.end(), false) == containFlagLinkSum.end()); // when all elements is true
            for (int l = 0; l < datas.size(); l++) {
              if (containFlagVerticesSum[l] && !(containFlagVerticesProd[l]) ) {
                if (datas[l].contact) {
                  surfaceFlag = true;
                }
              }
            }
            if (insideFlag) { // inside
              octomap::OcTreeKey pKey;
              if (m_octreeContact->coordToKeyChecked(p, pKey)) {
                m_octreeContact->updateNode(pKey, m_octreeContact->getProbMissContactSensorLog());
                // std::cout << "find inside grid and find key. p = " << vertex << std::endl;
              }
            }
            else if (surfaceFlag) { // surface
              octomap::OcTreeKey pKey;
              if (m_octreeContact->coordToKeyChecked(p, pKey)) {
                m_octreeContact->updateNode(pKey, m_octreeContact->getProbHitContactSensorLog());
                // std::cout << "find surface grid and find key. p = " << vertex << std::endl;
              }
            }
          }
          p.z() = pmin.z();
        }
        p.y() = pmin.y();
      }
    }
    else {
      point3d vertexOffset(-resolution/2.0, -resolution/2.0, -resolution/2.0);
#pragma omp parallel for
      for (unsigned int cnt = 0; cnt < steps[0] * steps[1] * steps[2]; ++cnt) {
        // get grid center
        point3d p;
        {
          unsigned int id[3];
          id[0] = cnt / (steps[1] * steps[2]);
          id[1] = (cnt % (steps[1] * steps[2])) / steps[2];
          id[2] = (cnt % (steps[1] * steps[2])) % steps[2];
          p.x() = pmin(0) + resolution * id[0];
          p.y() = pmin(1) + resolution * id[1];
          p.z() = pmin(2) + resolution * id[2];
        }
        point3d vertex;
        vertex = p + vertexOffset;
        // loop for each body link
        for (int l=0; l<datas.size(); l++) {
          if (m_selfMask->getMaskContainmentforNamedLink(vertex(0), vertex(1), vertex(2), datas[l].link_name) == robot_self_filter::INSIDE) {
            octomap::OcTreeKey pKey;
            if (m_octreeContact->coordToKeyChecked(p, pKey)) {
#pragma omp critical
              m_octreeContact->updateNode(pKey, m_octreeContact->getProbMissContactSensorLog());
            }
            break;
          }
        }
      }
    }
    m_octreeContact->updateInnerOccupancy();
  }

  void OctomapServerContact::insertContactSensorCallback(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg) {
    NODELET_INFO("insert contact sensor");
    insertContactSensor(msg);

    publishAll(msg->header.stamp);
  }

  void OctomapServerContact::publishAll(const ros::Time& rostime) {
    ros::WallTime startTime = ros::WallTime::now();
    size_t octomapSize = m_octreeContact->size();
    // TODO: estimate num occ. voxels for size of arrays (reserve)
    if (octomapSize <= 1) {
      ROS_WARN("Nothing to publish, octree is empty");
      return;
    }

    bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
    bool publishUnknownMarkerArray = m_publishUnknownSpace && (m_latchedTopics || m_umarkerPub.getNumSubscribers() > 0);
    bool publishFrontierMarkerArray = m_publishFrontierSpace && (m_latchedTopics || m_fromarkerPub.getNumSubscribers() > 0);
    bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
    bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
    bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
    bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
    m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

    // init markers for free space:
    visualization_msgs::MarkerArray freeNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    freeNodesVis.markers.resize(m_treeDepth+1);

    geometry_msgs::Pose pose;
    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // init markers:
    visualization_msgs::MarkerArray occupiedNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(m_treeDepth+1);

    // init pointcloud:
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    // call pre-traversal hook:
    handlePreNodeTraversal(rostime);

    // now, traverse all leafs in the tree:
    for (OcTree::iterator it = m_octreeContact->begin(m_maxTreeDepth), end = m_octreeContact->end(); it != end; ++it) {
      bool inUpdateBBX = isInUpdateBBX(it);

      // call general hook:
      handleNode(it);
      if (inUpdateBBX) {
        handleNodeInBBX(it);
      }

      if (m_octreeContact->isNodeOccupied(*it)) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        if (x > m_occupancyMinX && x < m_occupancyMaxX && y > m_occupancyMinY && y < m_occupancyMaxY && z > m_occupancyMinZ && z < m_occupancyMaxZ) {
          double size = it.getSize();
          double x = it.getX();
          double y = it.getY();

          // Ignore speckles in the map:
          if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())) {
            ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
            continue;
          } // else: current octree node is no speckle, send it out

          handleOccupiedNode(it);
          if (inUpdateBBX) {
            handleOccupiedNodeInBBX(it);
          }

          //create marker:
          if (publishMarkerArray) {
            unsigned idx = it.getDepth();
            assert(idx < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            if (m_useHeightMap) {
              double minX, minY, minZ, maxX, maxY, maxZ;
              m_octreeContact->getMetricMin(minX, minY, minZ);
              m_octreeContact->getMetricMax(maxX, maxY, maxZ);

              double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
              occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
            }
          }

          // insert into pointcloud:
          if (publishPointCloud) {
            pclCloud.push_back(pcl::PointXYZ(x, y, z));
          }

        }
      } else { // node not occupied => mark as free in 2D map if unknown so far
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        if (x > m_occupancyMinX && x < m_occupancyMaxX && y > m_occupancyMinY && y < m_occupancyMaxY && z > m_occupancyMinZ && z < m_occupancyMaxZ) {
          handleFreeNode(it);
          if (inUpdateBBX)
            handleFreeNodeInBBX(it);

          if (m_publishFreeSpace) {
            double x = it.getX();
            double y = it.getY();

            //create marker for free space:
            if (publishFreeMarkerArray) {
              unsigned idx = it.getDepth();
              assert(idx < freeNodesVis.markers.size());

              geometry_msgs::Point cubeCenter;
              cubeCenter.x = x;
              cubeCenter.y = y;
              cubeCenter.z = z;

              freeNodesVis.markers[idx].points.push_back(cubeCenter);
            }
          }
        }
      }
    }

    // call post-traversal hook:
    handlePostNodeTraversal(rostime);

    // finish MarkerArray:
    if (publishMarkerArray) {
      for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {
        double size = m_octreeContact->getNodeSize(i);

        occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
        occupiedNodesVis.markers[i].header.stamp = rostime;
        occupiedNodesVis.markers[i].ns = m_worldFrameId;
        occupiedNodesVis.markers[i].id = i;
        occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].scale.x = size;
        occupiedNodesVis.markers[i].scale.y = size;
        occupiedNodesVis.markers[i].scale.z = size;
        occupiedNodesVis.markers[i].color = m_color;


        if (occupiedNodesVis.markers[i].points.size() > 0) {
          occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else {
          occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }
      }

      m_markerPub.publish(occupiedNodesVis);
    }


    // finish FreeMarkerArray:
    if (publishFreeMarkerArray) {
      for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i) {
        double size = m_octreeContact->getNodeSize(i);

        freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
        freeNodesVis.markers[i].header.stamp = rostime;
        freeNodesVis.markers[i].ns = m_worldFrameId;
        freeNodesVis.markers[i].id = i;
        freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        freeNodesVis.markers[i].scale.x = size;
        freeNodesVis.markers[i].scale.y = size;
        freeNodesVis.markers[i].scale.z = size;
        freeNodesVis.markers[i].color = m_colorFree;


        if (freeNodesVis.markers[i].points.size() > 0) {
          freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else {
          freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }
      }

      m_fmarkerPub.publish(freeNodesVis);
    }


    // publish unknown grid as marker
    if (publishUnknownMarkerArray) {
      visualization_msgs::MarkerArray unknownNodesVis;
      unknownNodesVis.markers.resize(1);

      point3d_list unknownLeaves;
      double offset = m_offsetVisualizeUnknown;
      point3d pMin(m_occupancyMinX + offset, m_occupancyMinY + offset, m_occupancyMinZ + offset);
      point3d pMax(m_occupancyMaxX - offset, m_occupancyMaxY - offset, m_occupancyMaxZ - offset);

      m_octreeContact->getUnknownLeafCenters(unknownLeaves, pMin, pMax);
      pcl::PointCloud<pcl::PointXYZ> unknownCloud;

      for (point3d_list::iterator it = unknownLeaves.begin(); it != unknownLeaves.end(); it++) {
        float x = (*it).x();
        float y = (*it).y();
        float z = (*it).z();
        unknownCloud.push_back(pcl::PointXYZ(x, y, z));

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;
        if (m_useHeightMap) {
          double minX, minY, minZ, maxX, maxY, maxZ;
          m_octreeContact->getMetricMin(minX, minY, minZ);
          m_octreeContact->getMetricMax(maxX, maxY, maxZ);
          double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
          unknownNodesVis.markers[0].colors.push_back(heightMapColor(h));
        }
        unknownNodesVis.markers[0].points.push_back(cubeCenter);
      }

      double size = m_octreeContact->getNodeSize(m_maxTreeDepth);
      unknownNodesVis.markers[0].header.frame_id = m_worldFrameId;
      unknownNodesVis.markers[0].header.stamp = rostime;
      unknownNodesVis.markers[0].ns = m_worldFrameId;
      unknownNodesVis.markers[0].id = 0;
      unknownNodesVis.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
      unknownNodesVis.markers[0].scale.x = size;
      unknownNodesVis.markers[0].scale.y = size;
      unknownNodesVis.markers[0].scale.z = size;
      unknownNodesVis.markers[0].color = m_colorUnknown;

      if (unknownNodesVis.markers[0].points.size() > 0) {
        unknownNodesVis.markers[0].action = visualization_msgs::Marker::ADD;
      }
      else {
        unknownNodesVis.markers[0].action = visualization_msgs::Marker::DELETE;
      }
      m_umarkerPub.publish(unknownNodesVis);

      // publish unknown grid as pointcloud
      sensor_msgs::PointCloud2 unknownRosCloud;
      pcl::toROSMsg (unknownCloud, unknownRosCloud);
      unknownRosCloud.header.frame_id = m_worldFrameId;
      unknownRosCloud.header.stamp = rostime;
      m_unknownPointCloudPub.publish(unknownRosCloud);

      // publish frontier grid as marker
      if (publishFrontierMarkerArray) {
        visualization_msgs::MarkerArray frontierNodesVis;
        frontierNodesVis.markers.resize(1);
        pcl::PointCloud<pcl::PointXYZ> frontierCloud;
        double resolution = m_octreeContact->getResolution();
        // how many resolution-size grids are in one edge
        int x_num = int(((m_occupancyMaxX - m_occupancyMinX) / resolution));
        int y_num = int(((m_occupancyMaxY - m_occupancyMinY) / resolution));
        int z_num = int(((m_occupancyMaxZ - m_occupancyMinZ) / resolution));
        std::vector< std::vector< std::vector<int> > > check_unknown(x_num, std::vector< std::vector<int> >(y_num, std::vector<int>(z_num)));
        std::vector< std::vector< std::vector<int> > > check_occupied(x_num, std::vector< std::vector<int> >(y_num, std::vector<int>(z_num)));
        std::vector< std::vector< std::vector<int> > > check_frontier(x_num, std::vector< std::vector<int> >(y_num, std::vector<int>(z_num)));

        for (int i = 0; i < x_num; i++) {
          for (int j = 0; j < y_num; j++) {
            for (int k = 0; k < z_num; k++) {
              check_unknown[i][j][k] = 0;
              check_occupied[i][j][k] = 0;
              check_frontier[i][j][k] = 0;
            }
          }
        }

        // for all unknown grids, store its information to array
        for (point3d_list::iterator it_unknown = unknownLeaves.begin();
             it_unknown != unknownLeaves.end();
             it_unknown++) {
          // get center of unknown grids
          double x_unknown = it_unknown->x();
          double y_unknown = it_unknown->y();
          double z_unknown = it_unknown->z();
          int x_index = int(std::round((x_unknown - m_occupancyMinX) / resolution - 1));
          int y_index = int(std::round((y_unknown - m_occupancyMinY) / resolution - 1));
          int z_index = int(std::round((z_unknown - m_occupancyMinZ) / resolution - 1));
          check_unknown[x_index][y_index][z_index] = 1;
        }

        // for all occupied grids, store its information to array
        for (int idx = 0; idx < occupiedNodesVis.markers.size(); idx++) {
          double size_occupied = occupiedNodesVis.markers[idx].scale.x;
          for (int id = 0; id < occupiedNodesVis.markers[idx].points.size(); id++) {
            double x_occupied = occupiedNodesVis.markers[idx].points[id].x;
            double y_occupied = occupiedNodesVis.markers[idx].points[id].y;
            double z_occupied = occupiedNodesVis.markers[idx].points[id].z;
            int x_min_index = std::round((x_occupied - (size_occupied / 2.0) - m_occupancyMinX) / resolution);
            int y_min_index = std::round((y_occupied - (size_occupied / 2.0) - m_occupancyMinY) / resolution);
            int z_min_index = std::round((z_occupied - (size_occupied / 2.0) - m_occupancyMinZ) / resolution);
            for (int i = x_min_index; i < x_min_index + int(size_occupied/resolution); i++) {
              for (int j = y_min_index; j < y_min_index + int(size_occupied/resolution); j++) {
                for (int k = z_min_index; k < z_min_index + int(size_occupied/resolution); k++) {
                  check_occupied[i][j][k] = 1;
                }
              }
            }
          }
        }

        // for all grids except occupied and unknown, (NOTE there are grids which are not free, nor occupied, nor unknown)
        // check whether they are frontier, namely, adjecent to unknown grids
        // NOTE all unknown grids are displaced half from the other grids
        geometry_msgs::Point cubeCenter;
        for (int i = 0; i < x_num; i++) {
          for (int j = 0; j < y_num; j++) {
            for (int k = 0; k < z_num-1; k++) {
              for (int l = -1; l <= 1; l++) {
                if ( i+l < 0 || x_num <= i+l ) continue;
                for (int m = -1; m <= 1; m++) {
                  if ( j+m < 0 || y_num <= j+m ) continue;
                  for (int n = -1; n <= 1; n++) {
                    if (  k+n < 0 || z_num <= k+n ) continue;
                    if (l == 0 && m == 0 && n== 0) continue;
                    if (check_unknown[i+l][j+m][k+n] == 1 && check_unknown[i][j][k] == 0 && check_occupied[i][j][k] == 0 && check_frontier[i][j][k] == 0) {
                      check_frontier[i][j][k] = 1;
                      cubeCenter.x = (i+0.5)*resolution + m_occupancyMinX;
                      cubeCenter.y = (j+0.5)*resolution + m_occupancyMinY;
                      cubeCenter.z = (k+0.5)*resolution + m_occupancyMinZ;
                      if (m_useHeightMap) {
                        double minX, minY, minZ, maxX, maxY, maxZ;
                        m_octreeContact->getMetricMin(minX, minY, minZ);
                        m_octreeContact->getMetricMax(maxX, maxY, maxZ);
                        double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                        frontierNodesVis.markers[0].colors.push_back(heightMapColor(h));
                      }
                      frontierNodesVis.markers[0].points.push_back(cubeCenter);
                    }
                  }
                }
              }
            }
          }
        }

        // publish frontier grid as marker
        double size = m_octreeContact->getNodeSize(m_maxTreeDepth);
        frontierNodesVis.markers[0].header.frame_id = m_worldFrameId;
        frontierNodesVis.markers[0].header.stamp = rostime;
        frontierNodesVis.markers[0].ns = m_worldFrameId;
        frontierNodesVis.markers[0].id = 0;
        frontierNodesVis.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
        frontierNodesVis.markers[0].scale.x = size;
        frontierNodesVis.markers[0].scale.y = size;
        frontierNodesVis.markers[0].scale.z = size;
        frontierNodesVis.markers[0].color = m_colorFrontier;

        if (frontierNodesVis.markers[0].points.size() > 0) {
          frontierNodesVis.markers[0].action = visualization_msgs::Marker::ADD;
        }
        else {
          frontierNodesVis.markers[0].action = visualization_msgs::Marker::DELETE;
        }

        m_fromarkerPub.publish(frontierNodesVis);

        // publish frontier grid as pointcloud
        sensor_msgs::PointCloud2 frontierRosCloud;
        pcl::toROSMsg (frontierCloud, frontierRosCloud);
        frontierRosCloud.header.frame_id = m_worldFrameId;
        frontierRosCloud.header.stamp = rostime;
        m_frontierPointCloudPub.publish(frontierRosCloud);
      }
    }

    // finish pointcloud:
    if (publishPointCloud) {
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg (pclCloud, cloud);
      cloud.header.frame_id = m_worldFrameId;
      cloud.header.stamp = rostime;
      m_pointCloudPub.publish(cloud);
    }

    if (publishBinaryMap) {
      publishBinaryOctoMap(rostime);
    }

    if (publishFullMap) {
      publishFullOctoMap(rostime);
    }

    double totalElapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Map publishing in OctomapServer took %f sec", totalElapsed);
  }

  void OctomapServerContact::onInit(void)
  {
    DiagnosticNodelet::onInit();
    onInitPostProcess();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OctomapServerContact, nodelet::Nodelet);
