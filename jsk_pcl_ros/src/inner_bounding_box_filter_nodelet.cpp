/*
 * inner_bounding_box_filter_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_pcl_ros/inner_bounding_box_filter.h>

namespace jsk_pcl_ros
{
  void InnerBoundingBoxFilter::onInit()
  {
    ConnectionBasedNodelet::onInit();

    // param
    pnh_->param("approximate_sync", approximate_sync_, false);

    // dynamic reconfigure
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&InnerBoundingBoxFilter::configCallback, this, _1, _2);
    srv_->setCallback(f);

    // diagnostics setup
    diagnostic_updater_.reset(
      new jsk_recognition_utils::TimeredDiagnosticUpdater(*pnh_, ros::Duration(1.0)));
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName() + "::InnerBoundingBoxFilter",
      boost::bind(&InnerBoundingBoxFilter::updateDiagnostic, this, _1));
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1.0 / vital_rate));
    diagnostic_updater_->start();

    // publisher
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void InnerBoundingBoxFilter::subscribe()
  {
    processed_msg_num_ = 0;
    sub_pc_.subscribe(*pnh_, "input_cloud", 1);
    sub_bbox_.subscribe(*pnh_, "input_box", 1);
    if (approximate_sync_) {
      approx_sync_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      approx_sync_->connectInput(sub_pc_, sub_bbox_);
      approx_sync_->registerCallback(
        boost::bind(&InnerBoundingBoxFilter::filter, this, _1, _2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_pc_, sub_bbox_);
      sync_->registerCallback(
        boost::bind(&InnerBoundingBoxFilter::filter, this, _1, _2));
    }
  }

  void InnerBoundingBoxFilter::unsubscribe()
  {
    sub_pc_.unsubscribe();
    sub_bbox_.unsubscribe();
  }

  void InnerBoundingBoxFilter::filter(
    const sensor_msgs::PointCloud2::ConstPtr & pc_msg,
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &bbox_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    vital_checker_->poke();

    int point_size = pc_msg->width * pc_msg->height;
    jsk_recognition_msgs::ClusterPointIndices cluster_indices;
    pcl_msgs::PointIndices indices;
    indices.header = pc_msg->header;
    cluster_indices.header = pc_msg->header;

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(*pc_msg, cloud_xyz);

    for (int i = 0; i < bbox_msg->boxes.size(); ++i)
    {
      BoundingBox b(bbox_msg->boxes[i], padding_rate_);
      for (int j = 0; j < point_size; ++j)
      {
        if (b.innerPoint(cloud_xyz.points[i]))
          indices.indices.push_back(j);
      }
    }
    cluster_indices.cluster_indices.push_back(indices);
    pub_indices_.publish(cluster_indices);
    ++processed_msg_num_;
  }

  void InnerBoundingBoxFilter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    JSK_NODELET_DEBUG_STREAM("padding_rate is changed " << padding_rate_ << " -> " << config.padding_rate);
    padding_rate_ = config.padding_rate;
  }

  void InnerBoundingBoxFilter::updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "InnerBoundingBoxFilter running");
      stat.add("Number of processed messages", processed_msg_num_);
      jsk_recognition_utils::addDiagnosticBooleanStat("Approximate Sync", approximate_sync_, stat);
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::InnerBoundingBoxFilter, nodelet::Nodelet);
