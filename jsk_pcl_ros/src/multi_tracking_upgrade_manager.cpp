#include "ros/ros.h"
#include "pcl/pcl_base.h"
#include "pcl/point_types.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include "jsk_pcl_ros/SetPointCloud2.h"
#include "jsk_pcl_ros/SetWorkingState.h"
#include "jsk_pcl_ros/SetWorkingStateAndTopic.h"
#include "jsk_pcl_ros/SetTrackerObjectId.h"
#include "jsk_pcl_ros/SetMainObjectId.h"
#include "jsk_pcl_ros/TrackerInfoArray.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_rviz_plugins/PointCloud2WithId.h>

#include <boost/smart_ptr/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm_ext/erase.hpp>

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <sstream>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <pcl_ros/transforms.h>

#include <jsk_debug_tools/jsk_debug_tools.h>
using namespace std;

class TrackingObjectInfo{
public:
  //static --- seem not move
  //moving --- seem to move
  //halt   --- waited
  enum TrackingObjectState{STATIC=0, DYNAMIC=1};
  enum TrackingObjectCameraRelation{OUT_OF_CAMERA=0, IN_CAMERA=1};
  enum TrackingObjectWorkingState{HALT=0,WORK=1};
  enum TrackingObjectProperty{MUTABLE=1,NOT_MUTABLE=2,UNKNOWN=3};
  enum TrackingObjectOclusion{IN_OCLUSION=1, OUT_OF_OCLUSION=0};

  typedef boost::shared_ptr<TrackingObjectInfo> Ptr;
  typedef boost::shared_ptr<const TrackingObjectInfo> ConstPtr;

  TrackingObjectInfo(double in_out_camera_x_threshold,double in_out_camera_y_threshold, std::string frame_id, std::string camera_origin_frame, bool on_pr2):frame_id_(frame_id),in_out_camera_x_threshold_(in_out_camera_x_threshold),in_out_camera_y_threshold_(in_out_camera_y_threshold),counter_(0),tos_(STATIC),top_(UNKNOWN),tocr_(IN_CAMERA),statical_counter_(0),cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),STATIC_THRESHOLD(100), CENTROID_ERROR_THRESHOLD(0.02),too_(OUT_OF_OCLUSION),camera_origin_frame_(camera_origin_frame), on_pr2_(on_pr2){
    std::stringstream ss;
    ss << "tracking_object_tmp_id" << rand();
    object_id_name_ = ss.str();
    ROS_INFO_STREAM("Create new tracking object " << object_id_name_);
  }


  TrackingObjectInfo(double in_out_camera_x_threshold, double in_out_camera_y_threshold, std::string object_id_name, std::string frame_id, std::string camera_origin_frame, bool on_pr2):frame_id_(frame_id),in_out_camera_x_threshold_(in_out_camera_x_threshold),in_out_camera_y_threshold_(in_out_camera_y_threshold),counter_(0),object_id_name_(object_id_name),tos_(STATIC),top_(UNKNOWN),tocr_(IN_CAMERA),statical_counter_(0),cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),STATIC_THRESHOLD(100), CENTROID_ERROR_THRESHOLD(0.01),too_(OUT_OF_OCLUSION),camera_origin_frame_(camera_origin_frame), on_pr2_(on_pr2){
    //    ROS_INFO("Create new tracking object id_name: %s" , object_id_name.c_str());
  }

  //check states and update some tracking info
  void update(std::string frame_id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& update_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_diff, double x_abs_max, double y_abs_max, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>::Ptr& search_octree){
    //check oclusions before update cloud
    //cloud frame should not be camera_link
    bool check_oclusion_result = true;
#if 0
    if(search_octree->getLeafCount () > 0)
      check_oclusion_result = check_oclusions(search_octree);
#endif

    if(check_oclusion_result){
      //check static or dynamic
      //update centroid here
      check_static(frame_id, update_cloud);

      //check in camera or out camera
      //which will use centroid
      check_in_range(x_abs_max, y_abs_max);
    }
  }

  void check_static(std::string frame_id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& update_cloud){
    Eigen::Vector4f c;
    //Check it is static or dynamic
    if(cloud_->points.size() != 0){
      pcl::compute3DCentroid<pcl::PointXYZRGBA> (*update_cloud, c);
      Eigen::Vector4f centroid_diff = c - cloud_centroid_;
      if(centroid_diff.norm() < CENTROID_ERROR_THRESHOLD){
        statical_counter_++;
        if(statical_counter_ > STATIC_THRESHOLD){
          tos_ = STATIC;
        }
      }else{
        //reset STATIC
        statical_counter_ = 0;
        tos_ = DYNAMIC;
        //update cloud
        setCloudCentroid(c);
        setFrameId(frame_id);
        setPointCloud(update_cloud);
      }
    }else{
      pcl::compute3DCentroid<pcl::PointXYZRGBA> (*update_cloud, c);
      //update cloud
      setCloudCentroid(c);
      setFrameId(frame_id);
      setPointCloud(update_cloud);
    }
  }

  void renewROSPointCloud(sensor_msgs::PointCloud2 update_cloud_pc){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(update_cloud_pc, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *cloud);

    renewPointCloud(cloud, update_cloud_pc.header.frame_id);
  }

  void renewPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& update_cloud, std::string frame_id){
    Eigen::Vector4f c;
    pcl::compute3DCentroid<pcl::PointXYZRGBA> (*update_cloud, c);
    setCloudCentroid(c);
    setFrameId(frame_id);
    setPointCloud(update_cloud);
  }

  void check_in_range(double x_abs_max, double y_abs_max){
    Eigen::Vector4f centroid = getCloudCentroid();
    tf::StampedTransform transform;

    std::string object_tf = "tracker_tf_"+getObjectId();
    transform.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
    transform.setRotation(tf::createIdentityQuaternion());
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), getFrameId(), object_tf));

    double x_z_c = centroid[0]/centroid[2];
    double y_z_c = centroid[1]/centroid[2];

    //ROS_INFO("x_z %f  y_z %f x %f y %f z %f", x_z_c, y_z_c, centroid[0], centroid[1], centroid[2]);

    static tf::TransformListener tf_listener;
    try{
      tf_listener.waitForTransform( camera_origin_frame_, object_tf, ros::Time(0), ros::Duration(1.0) );
      tf_listener.lookupTransform( camera_origin_frame_, object_tf, ros::Time(0), transform);
      double x_z = transform.getOrigin().y()/transform.getOrigin().x();
      double y_z = transform.getOrigin().z()/transform.getOrigin().x();

      if(on_pr2_){
        x_z = transform.getOrigin().x()/transform.getOrigin().z();
        y_z = transform.getOrigin().y()/transform.getOrigin().z();
      }

      //      ROS_INFO("Frame %s x_z : %f y_z :  %f  boarder_min_x %f broader_max_y %f", (getFrameId()).c_str(), x_z, y_z,  - x_abs_max + in_out_camera_x_threshold_, y_abs_max - in_out_camera_x_threshold_);
      //      ROS_INFO("y_min : %f y_max : %f y_z %f", - y_abs_max + in_out_camera_y_threshold_, y_abs_max - in_out_camera_y_threshold_, y_z);

      if ( - x_abs_max + in_out_camera_x_threshold_ >  x_z  ||  x_abs_max - in_out_camera_x_threshold_ < x_z ||
                                                                                                         - y_abs_max + in_out_camera_y_threshold_ > y_z   ||  y_abs_max - in_out_camera_y_threshold_ < y_z )
        tocr_ = OUT_OF_CAMERA;
      else
        tocr_ = IN_CAMERA;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }

  bool check_oclusions(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>::Ptr& search_octree){
    //look camera origin
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    //camera center
    tf_listener.waitForTransform( camera_origin_frame_, getFrameId(), ros::Time(0), ros::Duration(1.0) );
    tf_listener.lookupTransform( camera_origin_frame_, getFrameId(), ros::Time(0), transform);

    Eigen::Vector3f origin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    Eigen::Vector3f direction( cloud_centroid_[0] - origin[0], cloud_centroid_[1] - origin[1], cloud_centroid_[2] - origin[2]);
    std::vector< int > k_indices;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>::AlignedPointTVector point_vector;
    search_octree->getIntersectedVoxelCenters(origin, direction, point_vector, 1);
    BOOST_FOREACH(pcl::PointXYZRGBA pt, point_vector){
      Eigen::Vector3f result_direction( pt.x - origin[0], pt.y - origin[1], pt.z - origin[2]);
      //      PUBLISH_DEBUG_TF_XYZ("debug_oclusion_point", pcl::PointXYZRGBA, pt, getFrameId());
      // tf::Transform _transform;
      // _transform.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));
      // _transform.setRotation(tf::createIdentityQuaternion());
      // br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
      //                                       getFrameId(), "debug_oclusions"));
      //check it is close to camera
      ROS_INFO("result_direction.norm() %lf /  direction.norm() %lf = %lf @ %s", result_direction.norm(), direction.norm(), result_direction.norm()/direction.norm(),getObjectId().c_str());
      if(result_direction.norm()/direction.norm() < 0.7){
        ROS_ERROR("IN_OCLUSION");
        too_ = IN_OCLUSION;
        return false;
      }else{
        too_ = OUT_OF_OCLUSION;
        return true;
      }
    }
  }


  void print(){
    ROS_INFO(" id:  %s", object_id_name_.c_str());
  };

  void setState(TrackingObjectState tos){tos_ = tos;};
  void setObjectId(std::string object_id_name){object_id_name_ = object_id_name;};
  void setFrameId(std::string frame_id){frame_id_ = frame_id;};
  void setCameraOriginFrame(std::string camera_origin_frame){camera_origin_frame_ = camera_origin_frame;};
  void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){cloud_ = cloud;pointcloud_num=cloud->points.size();};
  void setPointCloudNum(std::size_t num){pointcloud_num=num;};
  void setCloudCentroid(Eigen::Vector4f cloud_centroid){cloud_centroid_ = cloud_centroid;};
  void setProperty(TrackingObjectProperty top){top_ = top;};

  TrackingObjectState getState(){return tos_;}
  TrackingObjectProperty getProperty(){return top_;};
  std::string getObjectId(){return object_id_name_;};
  std::string getFrameId(){return frame_id_;};
  std::string getCameraOriginFrame(){return camera_origin_frame_;};
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(){return cloud_;};
  Eigen::Vector4f getCloudCentroid(){return cloud_centroid_;};
  TrackingObjectCameraRelation getCameraRelation(){return tocr_;};
  TrackingObjectOclusion getOclusion(){return too_;};
  std::size_t getPointCloudNum(){return pointcloud_num;};
  void setCounter(int counter){counter_=counter;};
  int getStaticalCounter(){return statical_counter_;};
private:
  int counter_;
  int statical_counter_;
  std::size_t pointcloud_num;
  double in_out_camera_x_threshold_;
  double in_out_camera_y_threshold_;
  bool on_pr2_;
  tf::TransformBroadcaster br;

  std::string object_id_name_;
  std::string frame_id_;
  std::string camera_origin_frame_;

  TrackingObjectState tos_;
  TrackingObjectProperty top_;
  TrackingObjectCameraRelation tocr_;
  TrackingObjectOclusion too_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  Eigen::Vector4f cloud_centroid_;

  const double CENTROID_ERROR_THRESHOLD;
  const int    STATIC_THRESHOLD;
};


class TrackerInfo{
public:

  typedef boost::shared_ptr<TrackerInfo> Ptr;
  typedef boost::shared_ptr<const TrackerInfo> ConstPtr;

  enum TrackerInfoState{WORKING, HALT};
  TrackerInfo(int tracker_id):state_(HALT), has_tracker_model_(false), tracker_id_(tracker_id), tracker_time_stamp_(ros::Time::now()){
  };

  int getId(){return tracker_id_;};
  TrackerInfoState getState(){return state_;};

  void setTrackerInfoState(TrackerInfoState state){state_=state;};
  TrackerInfoState getTrackerInfoState(){return state_;};

  void setTrackerTimeStamp(ros::Time update_time){tracker_time_stamp_=update_time;};
  ros::Time getTrackerTimeStamp(){return tracker_time_stamp_;};

  void setTrackingObjectId(std::string tracking_object_id){tracking_object_id_=tracking_object_id;};
  std::string getTrackingObjectId(){return tracking_object_id_;};

  TrackerInfoState state_;
  bool has_tracker_model_;
  int tracker_id_;
  std::string tracking_object_id_;
  ros::Time tracker_time_stamp_;
};


//------------------------------::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-----------------------------------//
//------------------------------::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-----------------------------------//
//------------------------------::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-----------------------------------//
//------------------------------::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-----------------------------------//



class ParticleFilterTrackerManager{
public:
  ParticleFilterTrackerManager():INIT_ID(1),SEGMENTATION_MIN_SIZE(20),target_id_(INIT_ID),n_("~"),CAMERA_X_ABS_MAX(0.411428571),CAMERA_Y_ABS_MAX(0.5685715),counter_(0),prev_tracker_mutable_id_(-1){
    ROS_INFO("Initializing multi tracker");

    //Get Node Param
    n_.param("tracker_num", tracker_num_, 4);
    n_.param("tracker_name", tracker_name_, std::string("/pcl_nodelet/particle_filter_tracker"));
    n_.param("simple_mutable_model_name", simple_mutable_model_name_, std::string("/mutable_model_tracking"));

    n_.param("tracker_service_name", tracker_service_name_, std::string("/renew_model"));
    n_.param("tracker_result_name", tracker_result_name_, std::string("/track_result"));
    n_.param("main_object_id", main_object_id_, std::string(""));
    n_.param("cluster_tolerance", cluster_tolerance_, 0.03);
    n_.param("in_out_camera_x_threshold", in_out_camera_x_threshold_, 0.1);
    n_.param("in_out_camera_y_threshold", in_out_camera_y_threshold_, 0.2);
    n_.param("octree_resolution", octree_resolution_, 0.1);
    n_.param("camera_origin_frame", camera_origin_frame_, std::string("openni_rgb_optical_frame"));
    n_.param("on_pr2", on_pr2_, true);
    n_.param("downsampling_grid_size", downsampling_grid_size_, 0.05);


    //Create TrackerInfo Instances
    for(int i = 0; i < tracker_num_; i++){
      TrackerInfo::Ptr tracker_info(new TrackerInfo(i+1));
      tracker_info_vector_.push_back(tracker_info);
    }


    //initialize oclustion check octree searcher
    search_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>(octree_resolution_));

    //Set subscriber
    sub1_ = n_.subscribe("input_cloud1", 1, &ParticleFilterTrackerManager::cloud1_cb, this);
    sub2_ = n_.subscribe("input_cloud2", 1, &ParticleFilterTrackerManager::cloud2_cb, this);
    sub3_ = n_.subscribe("input_cloud3", 1, &ParticleFilterTrackerManager::cloud3_cb, this);
    sub4_ = n_.subscribe("input_cloud4", 1, &ParticleFilterTrackerManager::cloud4_cb, this);

    sub_ = n_.subscribe("renew_set_input_cloud", 1, &ParticleFilterTrackerManager::register_new_cloud_cb, this);

    raw_sub_ = n_.subscribe("raw_input_cloud", 1, &ParticleFilterTrackerManager::raw_cloud_cb, this);

    srv_ = n_.advertiseService("exchange_tracker_info", &ParticleFilterTrackerManager::exchange_cb, this);

    srv2_ = n_.advertiseService("update_tracker_info", &ParticleFilterTrackerManager::update_cb, this);

    srv3_ = n_.advertiseService("set_main_object_id", &ParticleFilterTrackerManager::set_mainobject_cb, this);

    sub_octree_result_ = n_.subscribe("input_octree_result", 1, &ParticleFilterTrackerManager::octreeResultCb,this);
    octree_result_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);


    pub_ = n_.advertise<jsk_pcl_ros::TrackerInfoArray>("tracker_info_array", 1);;
  }


  bool set_mainobject_cb(jsk_pcl_ros::SetMainObjectId::Request &req,
                         jsk_pcl_ros::SetMainObjectId::Response &res)
  {
    ROS_ERROR("Get request :SetMainObjectid");
    ROS_ERROR("            :main_object_id %s",  req.main_object_id.c_str());

    if(req.main_object_id == ""){
      main_object_id_ = req.main_object_id;
      prev_tracker_mutable_id_ = -1;
      //      send_boot_mutable_model(-1, false);
      return true;
    }else{
      main_object_id_ = req.main_object_id;
      typedef std::pair<int,TrackingObjectInfo::Ptr> P;
      BOOST_FOREACH(P toii, tracker_and_object_info_){
        if( toii.second->getObjectId() == main_object_id_){
          send_boot_mutable_model(toii.first, true);
        }
      }
      return true;
    }
  }


  bool update_cb(jsk_pcl_ros::SetTrackerObjectId::Request &req,
                 jsk_pcl_ros::SetTrackerObjectId::Response &res)
  {
    ROS_INFO("Get request :Multi_tracking_upgrade_manager");
    ROS_INFO("            :tracker_id %d object_id %s", int(req.tracker_id), req.new_object_id.c_str());

    int tracker_id(req.tracker_id);
    std::string target_object_id(req.new_object_id);
    int target_object_property(req.object_property);
    //1.update object id by tracker id
    //2.update cloud     by tracker id
    //3.update cloud     by object  id

    //1.2
    if(tracker_id > 0){
      ROS_INFO("trakcer_id > 0");
      //1 object_id rename
      if(target_object_id != ""){
        std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_map = tracker_and_object_info_.find(tracker_id);
        if(target_object_map != tracker_and_object_info_.end()){
          ROS_INFO("hit");
          TrackingObjectInfo::Ptr target_object_info = target_object_map->second;
          target_object_info->setObjectId(target_object_id);
          if(target_object_property != 0)
            target_object_info->setProperty((TrackingObjectInfo::TrackingObjectProperty)target_object_property);
          res.id = tracker_id;
          return true;
        }else{
          ROS_ERROR("The pair of TRACKER_ID %d doesn't exist in tracker_and_object_info", tracker_id);
          return false;
        }
      }
      //2 update cloud or property
      else{
        std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_map = tracker_and_object_info_.find(tracker_id);
        if(target_object_map != tracker_and_object_info_.end()){
          TrackingObjectInfo::Ptr target_object_info = target_object_map->second;

          if(target_object_info->getProperty() == TrackingObjectInfo::NOT_MUTABLE){
            ROS_ERROR("This is not MUTABLE MODEL %s", target_object_info->getObjectId().c_str());
            return false;
          }

          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_resized_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
          if(req.cloud.fields.size() > 0){
            resizeTrackerModel(req.cloud, target_resized_cloud);
            if(target_resized_cloud->points.size() > 0)
              target_object_info->renewPointCloud(target_resized_cloud, req.cloud.header.frame_id);
          }

          if(target_object_property != 0)
            target_object_info->setProperty((TrackingObjectInfo::TrackingObjectProperty)target_object_property);

          send_pointcloud_to_tracker_service(tracker_id, target_object_info->getPointCloud());
          res.id = tracker_id;
          ROS_INFO("hit2");
          return true;
        }else{
          ROS_ERROR("The pair of TRACKER_ID %d doesn't exist in tracker_and_object_info", tracker_id);
          return false;
        }
      }
    }
    //3 update property
    else{
      ROS_INFO("trakcer_id > 0");
      //search in tracker_and_object_info
      typedef std::pair<int,TrackingObjectInfo::Ptr> P;
      BOOST_FOREACH(P toii, tracker_and_object_info_){
        if( toii.second->getObjectId() == target_object_id){
          if(req.cloud.fields.size() > 0){
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_resized_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            resizeTrackerModel(req.cloud, target_resized_cloud);
            if(target_resized_cloud->points.size() > 0)
              toii.second->renewPointCloud(target_resized_cloud, req.cloud.header.frame_id);
          }

          if(target_object_property != 0)
            toii.second->setProperty((TrackingObjectInfo::TrackingObjectProperty)target_object_property);
          ROS_INFO("No.%d tracker's object changed : object_id %s", toii.first, target_object_id.c_str());

          send_pointcloud_to_tracker_service(toii.first, toii.second->getPointCloud());
          res.id = toii.first;
          return true;
        }
      }

      //Update object_id's cloud if object_cloud exist
      //Check in object_info_vector
      for(int i = 0; i < tracker_object_info_vector_.size();i++){
        if(tracker_object_info_vector_[i]->getObjectId() == target_object_id){
          ROS_INFO("Found in tracker_object_info_vector %s", target_object_id.c_str());
          TrackingObjectInfo::Ptr tmp_object_info = tracker_object_info_vector_[i];
          if(req.cloud.fields.size() > 0){
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_resized_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            resizeTrackerModel(req.cloud, target_resized_cloud);
            if(target_resized_cloud->points.size() > 0)
              tmp_object_info->renewPointCloud(target_resized_cloud, req.cloud.header.frame_id);
          }
          if(target_object_property != 0)
            tmp_object_info->setProperty((TrackingObjectInfo::TrackingObjectProperty)target_object_property);

          res.id = -1;
          return true;
        }
      }

      //if came here it is error
      ROS_ERROR("There aren't such object_id %s", target_object_id.c_str());
      res.id = -2;
      return false;
    }
  }


  void resizeTrackerModel(const sensor_msgs::PointCloud2& cloud_msg,
                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_raw_cloud_downsampled){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_raw_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    convert_to_pcl(cloud_msg, target_raw_cloud);
    gridSampleApprox (target_raw_cloud, *target_raw_cloud_downsampled, downsampling_grid_size_);
  }



  //If requested object_id is exist, pick from object list and set to tracker
  //Else rename the now tracking object id to requested one
  bool exchange_cb(jsk_pcl_ros::SetTrackerObjectId::Request &req,
                   jsk_pcl_ros::SetTrackerObjectId::Response &res)
  {
    ROS_INFO("Get request :Multi_tracking_upgrade_manager");
    ROS_INFO("            :tracker_id %d object_id %s", int(req.tracker_id), req.new_object_id.c_str());
    int tracker_id(req.tracker_id);
    std::string target_object_id(req.new_object_id);
    int target_object_property(req.object_property);


    //Basically user shouldn't set tracker id
    //Because it should be unvisial for user
    if( tracker_id <= 0 ){
      int target_tracker_id = getNewCloudTrackerId(-1);
      //Update object_id's cloud if object_cloud exist
      //Check in object_info_vector
      for(int i = 0; i < tracker_object_info_vector_.size();i++){
        if(tracker_object_info_vector_[i]->getObjectId() == target_object_id){
          ROS_INFO("Found in tracker_object_info_vector %s", target_object_id.c_str());
          TrackingObjectInfo::Ptr tmp_object_info = tracker_object_info_vector_[i];
          
          //remove target object
          boost::range::remove_erase(tracker_object_info_vector_, tracker_object_info_vector_[i]);
          
          //move current target to tracker_object_info_vector
          tracker_object_info_vector_.push_back((*(tracker_and_object_info_.find(target_tracker_id))).second);
          //remove the target pair
          tracker_and_object_info_.erase(target_tracker_id);
          
          //add to object
          tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( target_tracker_id, tmp_object_info) );

          send_pointcloud_to_tracker_service(target_tracker_id, req.cloud);

          return true;
        }
      }

      ROS_ERROR("There are no object_id so make new one..");
      //If there are no object_id, make new object and cloud
      //THIS IS DEFAULT MODE
      std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_map = tracker_and_object_info_.find(target_tracker_id);
      if(target_object_map != tracker_and_object_info_.end()){
        TrackingObjectInfo::Ptr target_object_info = target_object_map->second;
        tracker_object_info_vector_.push_back(target_object_info);
        //delete old object info
        tracker_and_object_info_.erase(target_object_map);
      }

      ROS_INFO("Set new Cloud!!");
      TrackingObjectInfo::Ptr new_target_object_info(new TrackingObjectInfo( in_out_camera_x_threshold_ , in_out_camera_y_threshold_ , target_object_id, req.cloud.header.frame_id, camera_origin_frame_, on_pr2_));
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_resized_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      resizeTrackerModel(req.cloud, target_resized_cloud);

      if(target_object_property != 0)
        new_target_object_info->setProperty((TrackingObjectInfo::TrackingObjectProperty)target_object_property);
      new_target_object_info->renewPointCloud(target_resized_cloud, req.cloud.header.frame_id);
      tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( target_tracker_id , new_target_object_info ) );

      send_pointcloud_to_tracker_service(target_tracker_id, target_resized_cloud, new_target_object_info->getProperty());

      res.id = target_tracker_id;
      return true;
    }
    //this is main for debug
    else{
      for(int i = 0; i < tracker_object_info_vector_.size();i++){
        if(tracker_object_info_vector_[i]->getObjectId() == target_object_id){
          TrackingObjectInfo::Ptr tmp_object_info = tracker_object_info_vector_[i];

          ROS_INFO("Exchange object within tracker_object_info_vector's ELEMENT");
          ROS_INFO("about                                         :%s", target_object_id.c_str());
          //remove target object
          boost::range::remove_erase(tracker_object_info_vector_, tracker_object_info_vector_[i]);

          //move current target to tracker_object_info_vector
          tracker_object_info_vector_.push_back((*(tracker_and_object_info_.find(tracker_id))).second);
          //remove the target pair
          tracker_and_object_info_.erase(tracker_id);

          //add to object
          tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( tracker_id, tmp_object_info) );

          send_pointcloud_to_tracker_service(tracker_id, tmp_object_info->getPointCloud());

          res.id = tracker_id;
          return true;
        }
      }
    }

    return true;
  }


  //These are object tree change detector calback
  void octreeResultCb(const sensor_msgs::PointCloud2 &pc){
    octree_result_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(pc, pcl_pc);
    {
      boost::mutex::scoped_lock lock (mtx_);
      pcl::fromPCLPointCloud2 (pcl_pc, *octree_result_cloud_);
    }
  }

  void raw_cloud_cb(const sensor_msgs::PointCloud2& cloud_msg){
    if(cloud_msg.fields.size() <= 0)
      return;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_raw_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PCLPointCloud2 pcl_pc;
    if(frame_id_ != "" && cloud_msg.header.frame_id != "" && frame_id_ != cloud_msg.header.frame_id){
      static tf::TransformListener tf_listener_;
      sensor_msgs::PointCloud2 cloud_msg_transformed;
      ROS_INFO("raww_cloud_transform");
      pcl_ros::transformPointCloud(frame_id_, cloud_msg, cloud_msg_transformed, tf_listener_);
      ROS_INFO("raww_cloud_transform end");
      pcl_conversions::toPCL(cloud_msg_transformed, pcl_pc);
      pcl::fromPCLPointCloud2 (pcl_pc, *target_raw_cloud);
    }else{
      pcl_conversions::toPCL(cloud_msg, pcl_pc);
      pcl::fromPCLPointCloud2 (pcl_pc, *target_raw_cloud);
    }

    //make octree for oclustion check
    search_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>(octree_resolution_));
    search_octree_->setInputCloud (target_raw_cloud);
    search_octree_->addPointsFromInputCloud ();
  }

  //These are particlefilter cloud callback
  void cloud1_cb(const sensor_msgs::PointCloud2& cloud_msg){
    (tracker_info_vector_[0])->setTrackerInfoState(TrackerInfo::WORKING);
    //check time
    if((tracker_info_vector_[0])->getTrackerTimeStamp() > cloud_msg.header.stamp){
      (tracker_info_vector_[0])->setTrackerTimeStamp(cloud_msg.header.stamp);
      ROS_ERROR("timestamp is OLD!! cloud1_cb in multi_tracking_upgrade_manager");
      return;
    }
    cloud_cb(1, cloud_msg);
  }
  void cloud2_cb(const sensor_msgs::PointCloud2& cloud_msg){
    (tracker_info_vector_[1])->setTrackerInfoState(TrackerInfo::WORKING);
    //check time
    if((tracker_info_vector_[1])->getTrackerTimeStamp() > cloud_msg.header.stamp){
      (tracker_info_vector_[1])->setTrackerTimeStamp(cloud_msg.header.stamp);
      ROS_ERROR("timestamp is OLD!! cloud2_cb in multi_tracking_upgrade_manager");
      return;
    }
    cloud_cb(2, cloud_msg);
  }
  void cloud3_cb(const sensor_msgs::PointCloud2& cloud_msg){
    (tracker_info_vector_[2])->setTrackerInfoState(TrackerInfo::WORKING);
    //check time
    if((tracker_info_vector_[2])->getTrackerTimeStamp() > cloud_msg.header.stamp){
      (tracker_info_vector_[2])->setTrackerTimeStamp(cloud_msg.header.stamp);
      ROS_ERROR("timestamp is OLD!! cloud3_cb in multi_tracking_upgrade_manager");
      return;
    }
    cloud_cb(3, cloud_msg);
  }
  void cloud4_cb(const sensor_msgs::PointCloud2& cloud_msg){
    (tracker_info_vector_[3])->setTrackerInfoState(TrackerInfo::WORKING);
    //check time
    if((tracker_info_vector_[3])->getTrackerTimeStamp() > cloud_msg.header.stamp){
      (tracker_info_vector_[3])->setTrackerTimeStamp(cloud_msg.header.stamp);
      ROS_ERROR("timestamp is OLD!! cloud4_cb in multi_tracking_upgrade_manager");
      return;
    }
    cloud_cb(4, cloud_msg);
  }

  void gridSampleApprox (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result, double leaf_size)
  {
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
  }

  //These are from rviz user interactive callback
  void register_new_cloud_cb(const jsk_rviz_plugins::PointCloud2WithId& cloud_msg)
  {
    //    ROS_INFO("cloud_cb id : %d tracker_num : %d tracker_name : %s tracker_sercice_name : %s", cloud_msg.id, tracker_num_, tracker_name_.c_str(), tracker_service_name_.c_str());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_object_cloud_raw(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_object_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    convert_to_pcl(cloud_msg.cloud, target_object_cloud_raw);

    gridSampleApprox (target_object_cloud_raw, *target_object_cloud, downsampling_grid_size_);


    int target_id = cloud_msg.id;
    if(target_id <= 0)
      target_id = getNewCloudTrackerId(target_id);

    {
      boost::mutex::scoped_lock lock (mtx_);
      std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_map = tracker_and_object_info_.find(target_id);

      //Check
      if(target_object_map != tracker_and_object_info_.end()){
        TrackingObjectInfo::Ptr target_object_info = target_object_map->second;
        //        target_object_info->setObjectId(std::string("old_")+target_object_info->getObjectId());
        tracker_object_info_vector_.push_back(target_object_info);
        //delete old object info
        tracker_and_object_info_.erase(target_object_map);

        std::stringstream ss;
        if(cloud_msg.object_id != ""){
          ss << cloud_msg.object_id;
        }else{
          ss << std::string("tracker_object") << int(target_id);
        }
        TrackingObjectInfo::Ptr new_target_object_info(new TrackingObjectInfo( in_out_camera_x_threshold_ , in_out_camera_y_threshold_ , ss.str(), cloud_msg.cloud.header.frame_id, camera_origin_frame_, on_pr2_));

        new_target_object_info->renewPointCloud(target_object_cloud, cloud_msg.cloud.header.frame_id);
        tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( target_id , new_target_object_info ) );
      }
      //Boot New Tracker
      else{
        std::stringstream ss;
        if(cloud_msg.object_id != ""){
          ss << cloud_msg.object_id;
        }else{
          ss << std::string("tracker_object") << int(target_id);
        }
        addNewTrackerObject(target_id, target_object_cloud, ss.str(), cloud_msg.cloud.header.frame_id);
      }
    }

    //Safety
    {
      boost::mutex::scoped_lock lock (mtx_);
      target_id_ = target_id;
      if(target_id_ > tracker_num_ || target_id_ < 0){
        target_id_ = INIT_ID;
        ROS_INFO("Given tracker_id is invalid! %d", target_id_);
      }
      send_pointcloud_to_tracker_service(target_id_, cloud_msg.cloud);
      target_id_++;
    }
    //    ROS_INFO("%s : %d", __func__, __LINE__);
  };

  //int is tracker id , second had pointcloud infomations
  void send_pointcloud_to_tracker_service(int target_id, const sensor_msgs::PointCloud2& cloud, TrackingObjectInfo::TrackingObjectProperty mutable_model = TrackingObjectInfo::UNKNOWN){
    if(tracker_and_object_info_.find(target_id)->second->getObjectId() == main_object_id_){
      ROS_INFO("send boot_mutable %d : %s", target_id,  main_object_id_.c_str());
      send_boot_mutable_model(target_id, mutable_model);
    }

    //register latest time
    sensor_msgs::PointCloud2 publish_cloud(cloud);
    ros::Time now = ros::Time::now();
    publish_cloud.header.stamp = now;
    tracker_info_vector_[target_id-1]->setTrackerTimeStamp(now);

    ros::ServiceClient client = n_.serviceClient<jsk_pcl_ros::SetPointCloud2>(tracker_name_+boost::lexical_cast<std::string>(target_id)+tracker_service_name_);
    jsk_pcl_ros::SetPointCloud2 srv;
    srv.request.cloud = publish_cloud;
    ROS_INFO("send service to tracker %d", target_id);
    if (!client.call(srv))
      ROS_ERROR("Failed to call service in multi manager");
  }

  //int is tracker id , second had pointcloud infomations
  void send_pointcloud_to_tracker_service(int target_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud, TrackingObjectInfo::TrackingObjectProperty mutable_model = TrackingObjectInfo::UNKNOWN){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*pcl_cloud, cloud);

    send_pointcloud_to_tracker_service(target_id, cloud, mutable_model);
  }


  void convert_to_pcl(const sensor_msgs::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_object_cloud)
  {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(cloud_msg, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *target_object_cloud);
    frame_id_ = cloud_msg.header.frame_id;
    std::vector<int> indices;
    target_object_cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*target_object_cloud, *target_object_cloud, indices);
  }

  void cloud_cb(int tracker_id, const sensor_msgs::PointCloud2& cloud_msg){
#if 1
    debugPrint();
#endif


    counter_++;
    std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_info = tracker_and_object_info_.find(tracker_id);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_object_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    convert_to_pcl(cloud_msg, target_object_cloud);

    //check if it is new thing
    if(target_object_info != tracker_and_object_info_.end()){
      TrackingObjectInfo::Ptr tmp_target_object_info = target_object_info->second;
      std::size_t point_num = tmp_target_object_info->getPointCloudNum();
      if(point_num != target_object_cloud->points.size()){
        ROS_ERROR("POINT_CLOUD DOES NOT SEEM TO BE UPDATED YET point_num: %ld (%s %ld)  target_object_cloud : %ld  at %d tracker", point_num, tmp_target_object_info->getObjectId().c_str(), (tmp_target_object_info->getPointCloud())->points.size(), target_object_cloud->points.size(), tracker_id);
        tmp_target_object_info->setPointCloudNum((tmp_target_object_info->getPointCloud())->points.size());//TODO
        sensor_msgs::PointCloud2 new_pointcloud2;
        pcl::toROSMsg(*tmp_target_object_info->getPointCloud(), new_pointcloud2);
        new_pointcloud2.header.frame_id = tmp_target_object_info->getFrameId();
        send_pointcloud_to_tracker_service(tracker_id, new_pointcloud2);
        ROS_ERROR("RESET AGAIN");
        return;
      }
    }
    //IF IT IS NOT EXIST
    else{
      ROS_ERROR("Ignore tracker_id which is not in pair");
      return;
    }

    //update those which are not traked is in camera or out of camera
    updateNotTrackedObject();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_target_object_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    bool divide_diff = false;
    bool update_diff = false;
    //If there should be new object
    //I think the candidate should be 2 clusters at most...
    divide_diff = divideCheck( tracker_id , cloud_msg.header.frame_id,  target_object_cloud, old_target_object_cloud);

    //Update target infomation and stop tracking when it is static
    update_diff = updateCurrentTarget(target_object_info, cloud_msg, tracker_id, old_target_object_cloud);

    //Update tracker_infos;
    updateTrackerInfos();

    //If there are in_camera object and have space for tracking object in trackers, boot it and send request
    reviveInCameraObject();

    //set send results
    //    ROS_INFO("%d id tracker will be Updated ?", tracker_id);
    if(tracker_info_vector_[tracker_id-1]->getTrackerInfoState() == TrackerInfo::WORKING){
      //If there any change
      if(divide_diff || update_diff){
        sensor_msgs::PointCloud2 old_pointcloud2;
        pcl::toROSMsg(*old_target_object_cloud, old_pointcloud2);
        old_pointcloud2.header.frame_id = cloud_msg.header.frame_id;
        send_pointcloud_to_tracker_service(tracker_id , old_pointcloud2);
      }
    }

    publishTrackerInfo();
  }


  void updateTrackerInfos()
  {
    for(int i = 1; i <= tracker_num_ ; i++){
      if(tracker_and_object_info_.find(i) == tracker_and_object_info_.end())
        {
          tracker_info_vector_[i - 1]->setTrackerInfoState(TrackerInfo::HALT);
        }
    }
  }

  void reviveInCameraObject()
  {
    //chech exist trackers
    std::vector<int> candidate_tracker;
    for(int i = 0; i < tracker_num_;i++){
      if(tracker_info_vector_[i]->getTrackerInfoState() == TrackerInfo::HALT){
        candidate_tracker.push_back(i);
      }
    }

    //assign in camera object
    std::vector<int> candidate_object;
    for(int i = 0; i < tracker_object_info_vector_.size();i++)
      {
        if(tracker_object_info_vector_[i]->getCameraRelation() == TrackingObjectInfo::IN_CAMERA &&
           tracker_object_info_vector_[i]->getOclusion() == TrackingObjectInfo::OUT_OF_OCLUSION){
          candidate_object.push_back(i);
        }
      }

    ROS_INFO("candidate_tracker : %ld candidate_object : %ld", candidate_tracker.size() , candidate_object.size());
    //if there are candidates in both vector assigne some thing to trackers
    if(candidate_tracker.size() && candidate_object.size()){
      BOOST_FOREACH(int index, candidate_object){
        if(candidate_tracker.size () <= 0)
          break;


        int tracker_id = candidate_tracker.back() + 1;
        TrackingObjectInfo::Ptr tmp_tracker_info = tracker_object_info_vector_[index];

        ROS_ERROR("REVIVE!!");
        ROS_ERROR("  tracker %d   --->   %s", tracker_id, tmp_tracker_info->getObjectId().c_str());
        tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( tracker_id, tmp_tracker_info ) );
        boost::range::remove_erase(tracker_object_info_vector_, tracker_object_info_vector_[index]);
        ROS_ERROR("REVIVE Success!!");

        sensor_msgs::PointCloud2 new_pointcloud2;
        pcl::toROSMsg(*tmp_tracker_info->getPointCloud(), new_pointcloud2);
        ROS_INFO("toROSMsg");
        new_pointcloud2.header.frame_id = tmp_tracker_info->getFrameId();
        ROS_INFO("toROSMsg2");
        send_pointcloud_to_tracker_service(tracker_id, new_pointcloud2);
        ROS_ERROR("REVIVE send_pointcloud_to_tracker Success!!");
        candidate_tracker.pop_back();
      }
    }
  }

  void debugPrint(){
    if(counter_%3 == 0){
      ROS_INFO("Debug Print------------------------------------------------------------------");
      ROS_INFO("    tracker_object_info_vector_.size() : %ld", tracker_object_info_vector_.size());
      
      ROS_INFO("    tracker_object_info_vector elements:");
      std::string oclusion = "OUT_OF_OCLUSION";
      for(int i = 0; i < tracker_object_info_vector_.size();i++){
        if(tracker_object_info_vector_[i]->getOclusion() == TrackingObjectInfo::IN_OCLUSION){
          oclusion = std::string("IN_OCLUSION");
        }else{
          oclusion = std::string("OUT_OF_OCLUSION");
        }
        if(tracker_object_info_vector_[i]->getCameraRelation() == TrackingObjectInfo::IN_CAMERA){
          ROS_INFO("        %s : IN_CAMERA       | %ld %ld %s", (tracker_object_info_vector_[i]->getObjectId()).c_str(), tracker_object_info_vector_[i]->getPointCloudNum(),tracker_object_info_vector_[i]->getPointCloud()->points.size(), oclusion.c_str());
        }else{
          ROS_INFO("        %s : OUT_OF_CAMERA   | %ld %ld %s", (tracker_object_info_vector_[i]->getObjectId()).c_str(), tracker_object_info_vector_[i]->getPointCloudNum(),tracker_object_info_vector_[i]->getPointCloud()->points.size(), oclusion.c_str());
        }
      }
      
      ROS_INFO("    tracker_info_vector_.size()        : %ld", tracker_info_vector_.size());
      ROS_INFO("    tracker_and_object_info_.size()    : %ld", tracker_and_object_info_.size());
      
      typedef std::pair<int,TrackingObjectInfo::Ptr> P;
      BOOST_FOREACH(P toii, tracker_and_object_info_){
        if(toii.second->getOclusion() == TrackingObjectInfo::IN_OCLUSION){
          oclusion = std::string("IN_OCLUSION");
        }else{
          oclusion = std::string("OUT_OF_OCLUSION");
        }
        if(toii.second->getCameraRelation() == TrackingObjectInfo::IN_CAMERA){
          ROS_INFO("        tracker %d : %s : IN_CAMERA      | %ld %ld %s", toii.first, (toii.second->getObjectId()).c_str(), toii.second->getPointCloudNum(), toii.second->getPointCloud()->points.size() , oclusion.c_str());
        }else{
          ROS_INFO("        tracker %d : %s : OUT_OF_CAMERA  | %ld %ld %s", toii.first, (toii.second->getObjectId()).c_str(), toii.second->getPointCloudNum(), toii.second->getPointCloud()->points.size(), oclusion.c_str());
        }
    }

    BOOST_FOREACH(TrackerInfo::Ptr p , tracker_info_vector_){
      if(p->getTrackerInfoState() == TrackerInfo::HALT){
        ROS_INFO("tracker_id : %d is HALT", p->getId());
      }else{
        ROS_INFO("tracker_id : %d is WORKING", p->getId());
      }
    }

    ROS_INFO("END Debug Print--------------------------------------------------------------");
    }
  }

  void updateNotTrackedObject()
  {
    static int alpha = 0;
    for(int i = 0; i < tracker_object_info_vector_.size();i++){
      (tracker_object_info_vector_[i])->check_in_range(CAMERA_X_ABS_MAX, CAMERA_Y_ABS_MAX);
#if 0
      if(tracker_object_info_vector_[i]->getCameraRelation() == TrackingObjectInfo::IN_CAMERA)
        {
          boost::mutex::scoped_lock lock (mtx_);
          if(search_octree_->getLeafCount () > 0)
            (tracker_object_info_vector_[i])->check_oclusions(search_octree_);
        }
#endif

      if(tracker_and_object_info_.size() == 0  && alpha % 100 == 0){
        alpha = 0;
        ROS_INFO("check:");
        if(tracker_object_info_vector_[i]->getCameraRelation() == TrackingObjectInfo::IN_CAMERA){
          ROS_INFO("%s :  INCAMERA", tracker_object_info_vector_[i]->getObjectId().c_str());
        }
        else{
          ROS_INFO("%s :  NOCAMERA", tracker_object_info_vector_[i]->getObjectId().c_str());
        }
      }
    }
    alpha++;
  }

  bool updateCurrentTarget(std::map<int, TrackingObjectInfo::Ptr>::iterator &target_object_info,
                           const sensor_msgs::PointCloud2& cloud_msg,
                           int tracker_id,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& old_target_object_cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);
    //check there are correct
    if(target_object_info != tracker_and_object_info_.end()){
      (*target_object_info).second->update(cloud_msg.header.frame_id, old_target_object_cloud, octree_result_cloud_, CAMERA_X_ABS_MAX, CAMERA_Y_ABS_MAX, search_octree_);

#if 0
      //when dynamic
      if((*target_object_info).second->getState() == TrackingObjectInfo::STATIC){
        send_boot_tracker_service(tracker_id, false);
      }
#endif
      //When target become out of camera
      if((*target_object_info).second->getCameraRelation() == TrackingObjectInfo::OUT_OF_CAMERA ){
        ROS_ERROR("Current target %d out of camera", tracker_id);
        //manageTrackerSetting(tracker_id, old_target_object_cloud);
        TrackingObjectInfo::Ptr tmp_tracker_object_info(new TrackingObjectInfo(*((*(tracker_and_object_info_.find(tracker_id))).second)));
        ROS_ERROR("Current target %d out of camera2", tracker_id);

        //remove the target pair
        tracker_and_object_info_.erase(tracker_id);
        ROS_ERROR("Current target %d out of camera3", tracker_id);

        //move current target to tracker_object_info_vector
        tracker_object_info_vector_.push_back(tmp_tracker_object_info);
        ROS_ERROR("Current target %d out of camera4", tracker_id);

        send_boot_tracker_service(tracker_id, false, tmp_tracker_object_info->getObjectId());
        ROS_ERROR("Current target %d out of camera", tracker_id);

        return true;
      }

      //When target become in oclusions
      if((*target_object_info).second->getOclusion() == TrackingObjectInfo::IN_OCLUSION ){
        ROS_ERROR("Current target %d IN_OCLUSIONS", tracker_id);
        //manageTrackerSetting(tracker_id, old_target_object_cloud);
        TrackingObjectInfo::Ptr tmp_tracker_object_info(new TrackingObjectInfo(*((*(tracker_and_object_info_.find(tracker_id))).second)));
        //remove the target pair
        tracker_and_object_info_.erase(tracker_id);

        //move current target to tracker_object_info_vector
        tracker_object_info_vector_.push_back(tmp_tracker_object_info);
        send_boot_tracker_service(tracker_id, false, tmp_tracker_object_info->getObjectId());
        return true;
      }

      return false;
    }
    //add New object_info and tracker should be the designated one
    else{
      std::stringstream ss;
      ss << std::string("tracker_object") << tracker_id;
      //      int assigned_tracker_id = getNewCloudTrackerId(tracker_id);
      addNewTrackerObject(tracker_id, old_target_object_cloud, ss.str(), cloud_msg.header.frame_id);
      return true;
    }
  }


  void send_boot_mutable_model(int tracker_id, TrackingObjectInfo::TrackingObjectProperty mutable_model)
  {
    bool mutable_model_state;
    if(mutable_model == TrackingObjectInfo::NOT_MUTABLE){
      mutable_model_state = false;
      ROS_INFO("send boot_mutable %d : FALSE", tracker_id);
    }else{
      mutable_model_state = true;
      ROS_INFO("send boot_mutable %d : TRUE", tracker_id);
    }
    send_boot_mutable_model(tracker_id, mutable_model_state);
  }

  void send_boot_mutable_model(int tracker_id, bool bool_wake)
  {
    if(prev_tracker_mutable_id_ == tracker_id && bool_wake){
      ROS_INFO("previous send...");
      return;
    }

    ros::ServiceClient client = n_.serviceClient<jsk_pcl_ros::SetWorkingStateAndTopic>(simple_mutable_model_name_+"/reset_state");
    jsk_pcl_ros::SetWorkingStateAndTopic ws;
    ws.request.state.data = bool_wake;
    if(bool_wake){
      ws.request.topic_name = tracker_name_+boost::lexical_cast<std::string>(tracker_id)+tracker_result_name_;
      prev_tracker_mutable_id_ = tracker_id;
    }else{
      prev_tracker_mutable_id_ = -1;
    }

    if (client.call(ws)){
      ROS_INFO("Working State Success at simple");
      if(bool_wake){
        ROS_INFO("Set True %d at simple_model", tracker_id);
      }
      else{
        ROS_INFO("Set False %d at simple_model", tracker_id);
      }
    }
    else
      ROS_ERROR("Failed Working State service");
  }


  void send_boot_tracker_service(int tracker_id, bool bool_wake, std::string prev_object_id)
  {
    if(bool_wake == false && main_object_id_ == prev_object_id){
      ROS_INFO("send boot mutble model false");
      send_boot_mutable_model(tracker_id, false);
    }
    ROS_INFO("send boot mutble model true2");
    //    ROS_INFO("send boot mutble model true");
    ros::ServiceClient client = n_.serviceClient<jsk_pcl_ros::SetWorkingState>(tracker_name_+boost::lexical_cast<std::string>(tracker_id)+"/reset_state");
    jsk_pcl_ros::SetWorkingState ws;
    ws.request.state.data = bool_wake;
    ROS_INFO("send boot mutble model true3");
    if (client.call(ws)){
      (tracker_info_vector_[tracker_id-1])->setTrackerInfoState(TrackerInfo::HALT);
      ROS_INFO("Working State Success");
      if(bool_wake){
        ROS_INFO("Set True %d", tracker_id);
        tracker_info_vector_[tracker_id-1]->setTrackerInfoState(TrackerInfo::WORKING);
      }
      else{
        ROS_INFO("Set False %d", tracker_id);
        tracker_info_vector_[tracker_id-1]->setTrackerInfoState(TrackerInfo::HALT);
      }
    }
    else
      ROS_ERROR("Failed Working State service");
  }

  bool divideCheck( int tracker_id, std::string frame_id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_object_cloud,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& old_target_object_cloud )
  {
    //update target object info . Add new object if need here.
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanSegment (target_object_cloud, cluster_indices);

    if(cluster_indices.size() > 1){
      ROS_ERROR("DIVIDE!");
      boost::mutex::scoped_lock lock (mtx_);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
      for(std::size_t i = 1; i < cluster_indices.size() ; i++){
        temp_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);

        extractSegmentCluster (target_object_cloud, cluster_indices, int (i), *temp_cloud);


        std::map<int, TrackingObjectInfo::Ptr>::iterator target_object_map = tracker_and_object_info_.find(tracker_id);
        TrackingObjectInfo::Ptr target_object_info = target_object_map->second;

        std::stringstream ss;
        ss << target_object_info->getObjectId() << std::string("_divided") << ros::Time::now().toNSec();

        std::string target_object_id = ss.str();
        int assigned_tracker_id = getNewCloudTrackerId(tracker_id);
        addNewTrackerObject(assigned_tracker_id, temp_cloud, target_object_id, frame_id);

        //send_pointcloud_to_tracker_service for new object
        sensor_msgs::PointCloud2 new_pointcloud2;
        pcl::toROSMsg(*temp_cloud, new_pointcloud2);
        new_pointcloud2.header.frame_id = frame_id;
        send_pointcloud_to_tracker_service(assigned_tracker_id, new_pointcloud2);
        ROS_INFO("NEW TRACKER_OBJECT %d -> %d", tracker_id, assigned_tracker_id);
      }
      extractSegmentCluster ( target_object_cloud , cluster_indices, 0, *old_target_object_cloud);
      return true;
    }else{
      pcl::copyPointCloud(*target_object_cloud, *old_target_object_cloud);
      return false;
    }
  }

  int getNewCloudTrackerId(int tracker_id)
  {
    //first check the halted TrackerInfo
    int counter = 0;
    BOOST_FOREACH(TrackerInfo::Ptr tip, tracker_info_vector_){
      if(tip->getTrackerInfoState() == TrackerInfo::HALT){
        //wake up tracker
        //boot is automatically executed in tracker itself.
        //        send_boot_tracker_service(counter, true);
        return (counter+1);
      }
      counter++;
    }

    //find which is most long tracking by comparing static_counter_
    typedef std::pair<int,TrackingObjectInfo::Ptr> P;
    int id = -1;
    int max_counter = 0;
    BOOST_FOREACH(P toii, tracker_and_object_info_){
      if(toii.first == tracker_id)
        continue;
      if(toii.second->getStaticalCounter() > max_counter)
        id = toii.first;
    }
    if(id != -1)
      return id;

    //if there are not good candidate. send random... but it counln't be like that
    return (tracker_id+rand())%tracker_num_+1;
  }


  //Execute  euclidean segment and retrun indices.
  void euclideanSegment (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                         std::vector<pcl::PointIndices> &cluster_indices)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (SEGMENTATION_MIN_SIZE);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
  }

  void extractSegmentCluster (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                              const std::vector<pcl::PointIndices> cluster_indices,
                              const int segment_index,
                              pcl::PointCloud<pcl::PointXYZRGBA> &result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (std::size_t i = 0; i < segmented_indices.indices.size (); i++)
      {
        pcl::PointXYZRGBA point = cloud->points[segmented_indices.indices[i]];
        result.points.push_back (point);
      }
    result.width = pcl::uint32_t (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }

  //For finding other object which is not tracked now.
  void manageTrackerSetting(int tracker_id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& old_target_object_cloud){
    //check in_camera object except tracking object
    for(int i = 0; i < tracker_object_info_vector_.size();i++){
      if(tracker_object_info_vector_[i]->getCameraRelation() == TrackingObjectInfo::IN_CAMERA){
        ROS_ERROR("Change Tracker %d target %s -> %s",tracker_id,(((*(tracker_and_object_info_.find(tracker_id))).second)->getObjectId()).c_str() ,(tracker_object_info_vector_[i]->getObjectId()).c_str());
        //get from tracker_object_info
        TrackingObjectInfo::Ptr tmp_object_info = tracker_object_info_vector_[i];
        //remove target object
        //        tracker_object_info_vector_.erase(std::remove(tracker_object_info_vector_.begin(), tracker_object_info_vector_.end(), tracker_object_info_vector_.begin[i]), v.end());
        boost::range::remove_erase(tracker_object_info_vector_, tracker_object_info_vector_[i]);

        TrackingObjectInfo::Ptr tmp_tracker_object_info(new TrackingObjectInfo(*((*(tracker_and_object_info_.find(tracker_id))).second)));
        //remove the target pair
        tracker_and_object_info_.erase(tracker_id);

        //move current target to tracker_object_info_vector
        tracker_object_info_vector_.push_back(tmp_tracker_object_info);

        //add to object
        tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( tracker_id, tmp_object_info) );

        //copy pointcloud to old_target_object_cloud
        pcl::copyPointCloud(*(tmp_object_info->getPointCloud()), *old_target_object_cloud);
        return;
      }
    }

    //If there are no more tracking object candidate
    //remove the target pair and register it to tracker_object_info_vector_
    //remove the target pair
    TrackingObjectInfo::Ptr tmp_tracker_object_info = tracker_and_object_info_.find(tracker_id)->second;
    tracker_and_object_info_.erase(tracker_id);
    tracker_object_info_vector_.push_back(tmp_tracker_object_info);

    //Stop the tracker
    //   send_boot_tracker_service(tracker_id, false);
  }

  void addNewTrackerObject(int tracker_id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, std::string target_object_id, std::string frame_id)
  {
    TrackingObjectInfo::Ptr  new_target_object_info( new TrackingObjectInfo( in_out_camera_x_threshold_, in_out_camera_y_threshold_ , target_object_id, frame_id, camera_origin_frame_, on_pr2_));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_resized_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    gridSampleApprox (cloud, *target_resized_cloud, downsampling_grid_size_);

    new_target_object_info->renewPointCloud(target_resized_cloud, frame_id);
    tracker_and_object_info_.insert( map<int, TrackingObjectInfo::Ptr >::value_type( tracker_id, new_target_object_info ) );
    //Below will be automatically done by cloud_cbx
    //    tracker_info_vector_[tracker_id-1]->setTrackerInfoState(TrackerInfo::WORKING);
  }

  void publishTrackerInfo(){
    jsk_pcl_ros::TrackerInfoArray tia;
    typedef std::pair<int,TrackingObjectInfo::Ptr> P;
    BOOST_FOREACH(P toii, tracker_and_object_info_){
      jsk_pcl_ros::TrackerInfo ti;
      ti.tracker_id = toii.first;
      ti.object_id = toii.second->getObjectId();
      tia.tracker_infos.push_back(ti);

      Eigen::Vector4f centroid = toii.second->getCloudCentroid();
      pcl::PointXYZ tmp_center;
      tmp_center.x = centroid[0];
      tmp_center.y = centroid[1];
      tmp_center.z = centroid[2];
      PUBLISH_DEBUG_TF_XYZ(br_, toii.second->getObjectId(), pcl::PointXYZ, tmp_center, toii.second->getFrameId());
    }

    pub_.publish(tia);
  }


  void run(){
    ros::Rate loop_rate(30);

    while(ros::ok()){
      if(tracker_and_object_info_.size() == 0 && tracker_object_info_vector_.size() != 0){
        static int tmp_counter = 0;
        if(tmp_counter%10 == 0){
          ROS_INFO("check:");
          updateNotTrackedObject();
          //Update tracker_infos;
          updateTrackerInfos();
          //If there are in_camera object and have space for tracking object in trackers, boot it and send request
          reviveInCameraObject();
        }
        debugPrint();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Subscriber raw_sub_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;
  ros::Subscriber sub_octree_result_;
  ros::ServiceServer srv_;
  ros::ServiceServer srv2_;
  ros::ServiceServer srv3_;
  boost::mutex mtx_;
  ros::Publisher pub_;

  tf::TransformBroadcaster br_;

  int tracker_num_;
  int prev_tracker_mutable_id_;
  int counter_;
  std::string frame_id_;
  std::string tracker_name_;
  std::string simple_mutable_model_name_;
  std::string tracker_service_name_;
  std::string tracker_result_name_;
  std::string input_cloud_name_;
  std::string camera_origin_frame_;
  std::string main_object_id_;

  int target_id_;

  bool on_pr2_;

  double cluster_tolerance_;
  double CAMERA_X_ABS_MAX;
  double CAMERA_Y_ABS_MAX;
  double in_out_camera_x_threshold_;
  double in_out_camera_y_threshold_;
  double octree_resolution_;
  double downsampling_grid_size_;

  map<int,TrackingObjectInfo::Ptr> tracker_and_object_info_;
  vector<TrackingObjectInfo::Ptr> tracker_object_info_vector_;
  vector<TrackerInfo::Ptr> tracker_info_vector_;

  const int INIT_ID;
  const int SEGMENTATION_MIN_SIZE;
  //const int SEGMENTATION_MAX_SIZE;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr octree_result_cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>::Ptr search_octree_;

};

int main (int argc, char** argv) {
  ros::init(argc, argv, "send_renew_model");

  ParticleFilterTrackerManager pm;
  pm.run();

  return 0;
}
