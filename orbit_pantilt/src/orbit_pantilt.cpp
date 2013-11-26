////////////////////////////////////////////////////////////////////////////////
//
// JSK Logicool Orbit/Sphere control node
//

#include <ros/ros.h>
#include <ros/names.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <orbit_pantilt/JointCommand.h>

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

#include <linux/videodev2.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "orbit_pantilt/OrbitPanTiltConfig.h"

class OrbitPanTilt {
private:
    ros::NodeHandle nh_;

    // dynamic reconfigure
    typedef dynamic_reconfigure::Server<orbit_pantilt::OrbitPanTiltConfig> ReconfigureServer;
    ReconfigureServer reconfigure_server_;

    //
    int fd_;
    std::string device_;
    double pan_, tilt_; // [rad]
    int pan_ratio_, tilt_ratio_;

public:
    int set_ext_ctrls(int fd, int id, int value) {
        struct v4l2_ext_control xctrls[1];
        struct v4l2_ext_controls ctrls;

        xctrls[0].id = id;
        xctrls[0].value = value;

        ctrls.count = 1;
        ctrls.controls = xctrls;

        if (-1 == ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ctrls)) {
            throw std::runtime_error("ioctl failed");
            return 0;
        }
        return 1;
    }

    void config_cb (orbit_pantilt::OrbitPanTiltConfig &config, uint32_t level)
    {
        if ( config.pan_reset ) {
            if (!set_ext_ctrls(fd_, V4L2_CID_PAN_RESET, 1))
                throw std::runtime_error(device_ + " does not support set pan reset");
            config.pan_reset = false;
            config.pan = 0;
            pan_ = 0;
        }
        if ( config.tilt_reset ) {
            if (!set_ext_ctrls(fd_, V4L2_CID_TILT_RESET, 1))
                throw std::runtime_error(device_ + " does not support set tilt reset");
            config.tilt_reset = false;
            config.tilt = 0;
            tilt_ = 0;
        }

        printf("pan  %d %d\n", pan_, config.pan);
        if ( pan_ != config.pan ) {
            if (!set_ext_ctrls(fd_, V4L2_CID_PAN_RELATIVE, (pan_-config.pan)*pan_ratio_))
                throw std::runtime_error(device_ + " does not support set pan relative");
            pan_ = config.pan;
        }
        printf("tilt %d %d\n", tilt_, config.tilt);
        if ( tilt_ != config.tilt ) {
            if (!set_ext_ctrls(fd_, V4L2_CID_TILT_RELATIVE, (tilt_-config.tilt)*tilt_ratio_))
                throw std::runtime_error(device_ + " does not support set tilt relative");
            tilt_ = config.tilt;
        }
    }

    double min_max_angle(double raw_data_rad, double min_deg, double max_deg) {
      double min_rad = min_deg * M_PI / 180.0;
      double max_rad = max_deg * M_PI / 180.0;
      if (raw_data_rad < min_rad) { return min_rad; }
      if (raw_data_rad > max_rad) { return max_rad; }
      return raw_data_rad;
    }

    void command_cb(const orbit_pantilt::JointCommandConstPtr& msg) {
      double msgpan = min_max_angle(msg->pan, -94, 94);
      double msgtilt = min_max_angle(msg->tilt, -51, 51);
      if ( fabs(msgpan - pan_) > 0.5 * M_PI / 180.0 ) {
        if (!set_ext_ctrls(fd_, V4L2_CID_PAN_RELATIVE, round((msgpan-pan_)*pan_ratio_*180.0/M_PI)))
            throw std::runtime_error(device_ + " does not support set pan relative");
        pan_ = msgpan;
        nh_.setParam("pan", pan_*180.0/M_PI);
      }
      if ( fabs(msgtilt - tilt_) > 0.5 * M_PI / 180.0 ) {
        if (!set_ext_ctrls(fd_, V4L2_CID_TILT_RELATIVE, round((msgtilt-tilt_)*tilt_ratio_*180.0/M_PI)))
            throw std::runtime_error(device_ + " does not support set tilt relative");
        tilt_ = msgtilt;
        nh_.setParam("tilt", tilt_*180.0/M_PI);
      }
    }

    bool pan_reset(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res) {
        if (!set_ext_ctrls(fd_, V4L2_CID_PAN_RESET, 1))
            throw std::runtime_error(device_ + " does not support set pan reset");
        pan_ = 0;
        nh_.setParam("pan", 0);
        return true;
    }
    bool tilt_reset(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res) {
        if (!set_ext_ctrls(fd_, V4L2_CID_TILT_RESET, 1))
            throw std::runtime_error(device_ + " does not support set tilt reset");
        tilt_ = 0;
        nh_.setParam("tilt", 0);
        return true;
    }


    OrbitPanTilt() : nh_("~"), pan_(0), tilt_(0)
    {

        //
        device_ = "/dev/video0";

        nh_.getParam("device", device_);
        ROS_INFO("device : %s", device_.c_str());

        pan_ratio_ = 40;
        tilt_ratio_ = 40;
        nh_.getParam("pan_ratio", pan_ratio_);
        nh_.getParam("tilt_ratio", tilt_ratio_);

        ReconfigureServer::CallbackType f = boost::bind(&OrbitPanTilt::config_cb, this, _1, _2);
        reconfigure_server_.setCallback(f);

        ros::Publisher joint_state_pub = nh_.advertise<sensor_msgs::JointState> ("joint_states", 1);
        ros::Subscriber joint_command_sub = nh_.subscribe<orbit_pantilt::JointCommand> ("pan_tilt_command", 1, &OrbitPanTilt::command_cb, this);
        ros::ServiceServer pan_reset_srv  = nh_.advertiseService("pan_reset", &OrbitPanTilt::pan_reset, this);
        ros::ServiceServer tilt_reset_srv  = nh_.advertiseService("tilt_reset", &OrbitPanTilt::tilt_reset, this);

        printf("opening %s\n", device_.c_str());
        if ((fd_ = open(device_.c_str(), O_RDWR)) == -1)
            throw std::runtime_error("couldn't open " + device_);

        v4l2_format fmt;
        v4l2_capability cap;

        memset(&fmt, 0, sizeof(v4l2_format));
        memset(&cap, 0, sizeof(v4l2_capability));
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
            throw std::runtime_error("couldn't query " + device_);
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
            throw std::runtime_error(device_ + " does not support capture");
        if (!(cap.capabilities & V4L2_CAP_STREAMING))
            throw std::runtime_error(device_ + " does not support streaming");

        if (!set_ext_ctrls(fd_, V4L2_CID_PAN_RESET, 1))
            throw std::runtime_error(device_ + " does not support set pan reset");
        nh_.setParam("pan", 0);
        sleep(3);
        if (!set_ext_ctrls(fd_, V4L2_CID_TILT_RESET, 1))
            throw std::runtime_error(device_ + " does not support set tilt reset");
        nh_.setParam("tilt", 0);
        sleep(3);

        ros::Rate loop_rate(1);
        while( ros::ok() ) {
            sensor_msgs::JointState msg;

            msg.header.stamp = ros::Time::now();
            msg.name.push_back("pan");
            msg.name.push_back("tilt");
            msg.position.push_back(pan_);
            msg.position.push_back(tilt_);

            joint_state_pub.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        if ( fd_ > 0 ) {
            close(fd_);
        }
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "orbit_pantilt");

  try {
      OrbitPanTilt orbit_pantilt;
  }
  catch (std::runtime_error &ex) {
      printf("ERROR: could not set some settings.  \n %s \n", ex.what());
  }
  return 0;
}
