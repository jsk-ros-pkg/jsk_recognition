#include <ros/ros.h>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

#include "jsk_perception/WhiteBalance.h"
#include "jsk_perception/WhiteBalancePoints.h"

class WhiteBalanceConverter {

private:
  ros::NodeHandle nh_;

  ros::ServiceServer service_img_;
  ros::ServiceServer service_pts_;

  double cmat[12]; // convert_matrix
  double pmat[48]; // paramteter matrix
  //param
  int queue_size_;

  typedef union
  {
    struct /*anonymous*/
    {
      unsigned char Blue; // Blue channel
      unsigned char Green; // Green channel
      unsigned char Red; // Red channel
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
  } RGBValue;

public:
  WhiteBalanceConverter() : nh_ () {
    ros::NodeHandle pnh("~");

    if (pnh.hasParam("parameter_matrix")) {
      XmlRpc::XmlRpcValue param_val;
      pnh.getParam("parameter_matrix", param_val);
      // ROS_INFO("MATRIX %d", param_val.size());
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray && param_val.size() == 48) {
        for ( int i = 0; i < 48; i++) {
          pmat[i] = param_val[i];
        }
      }
      for ( int i = 0; i < 12; i++ ) {
        ROS_INFO("[%f, %f, %f, %f]", pmat[4*i + 0], pmat[4*i + 1], pmat[4*i + 2], pmat[4*i + 3]);
      }
    }

    service_img_ = pnh.advertiseService(pnh.resolveName("convert_image"),
                                        &WhiteBalanceConverter::imageCallback, this);
    service_pts_ = pnh.advertiseService(pnh.resolveName("convert_points"),
                                        &WhiteBalanceConverter::pointsCallback, this);
  }

  bool imageCallback(jsk_perception::WhiteBalance::Request &req,
                     jsk_perception::WhiteBalance::Response &res) {
    makeConvertMatrix(req.reference_color[0], req.reference_color[1], req.reference_color[2]);

    return true;
  }

  bool pointsCallback(jsk_perception::WhiteBalancePoints::Request &req,
                      jsk_perception::WhiteBalancePoints::Response &res) {
    makeConvertMatrix(req.reference_color[0], req.reference_color[1], req.reference_color[2]);

    int rgb_offset = -1;
    for (size_t i = 0; i < req.input.fields.size(); i++) {
      if ( req.input.fields[i].name == "rgb" ) {
        rgb_offset = req.input.fields[i].offset;
        break;
      }
    }
    //ROS_INFO("offst = %d", rgb_offset);

    if ( rgb_offset < 0 ) {
      ROS_WARN("point cloud didn't have rgb field");
      return false;
    }

    res.output = req.input;
    int size = req.input.height * req.input.width;
    int step = req.input.point_step;
    unsigned char *sdata = (unsigned char *) &(req.input.data[0]);
    unsigned char *ddata = (unsigned char *) &(res.output.data[0]);

    for(int i = 0; i < size; i++, sdata += step, ddata += step) {
      RGBValue rgb;
      rgb.float_value = *((float *)(sdata + rgb_offset));
      int r = rgb.Red; int g = rgb.Green; int b = rgb.Blue;
      convertColor(r, g, b);
      rgb.long_value = 0;
      rgb.Red = (unsigned char)r;
      rgb.Green = (unsigned char)g;
      rgb.Blue = (unsigned char)b;
      *((float *)(ddata + rgb_offset)) = rgb.float_value;
    }

    return true;
  }

  inline void convertColor (int &r, int &g, int &b) {
    double rr = ( cmat[0] * r ) + ( cmat[1] * g ) + (  cmat[2] * b ) +  cmat[3]*256;
    double gg = ( cmat[4] * r ) + ( cmat[5] * g ) + (  cmat[6] * b ) +  cmat[7]*256;
    double bb = ( cmat[8] * r ) + ( cmat[9] * g ) + ( cmat[10] * b ) + cmat[11]*256;

    r = round(rr);
    g = round(gg);
    b = round(bb);

    if(r > 255) r = 255; if (r < 0) r = 0;
    if(g > 255) g = 255; if (g < 0) g = 0;
    if(b > 255) b = 255; if (b < 0) b = 0;
  }

  void makeConvertMatrix(float refr, float refg, float refb) {
    for(int i = 0; i < 12; i++) {
      cmat[i] =
        ( pmat[4*i + 0] * refr ) +
        ( pmat[4*i + 1] * refg ) +
        ( pmat[4*i + 2] * refb ) +
        pmat[4*i + 3] ;
    }
    ROS_INFO("%f %f %f %f", cmat[0], cmat[1], cmat[2], cmat[3]);
    ROS_INFO("%f %f %f %f", cmat[4], cmat[5], cmat[6], cmat[7]);
    ROS_INFO("%f %f %f %f", cmat[8], cmat[9], cmat[10], cmat[11]);

    Eigen::Matrix4d mat;
    mat(0,0) = cmat[0]; mat(0,1) = cmat[1]; mat(0,2) = cmat[2];  mat(0,3) = cmat[3];
    mat(1,0) = cmat[4]; mat(1,1) = cmat[5]; mat(1,2) = cmat[6];  mat(1,3) = cmat[7];
    mat(2,0) = cmat[8]; mat(2,1) = cmat[9]; mat(2,2) = cmat[10]; mat(2,3) = cmat[11];
    mat(3,0) = 0.0;     mat(3,1) = 0.0;     mat(3,2) = 0.0;      mat(3,3) = 1.0;

    Eigen::Matrix4d imat = mat.inverse();
    cmat[0] = imat(0, 0); cmat[1] = imat(0, 1); cmat[2]  = imat(0, 2); cmat[3]  = imat(0, 3);
    cmat[4] = imat(1, 0); cmat[5] = imat(1, 1); cmat[6]  = imat(1, 2); cmat[7]  = imat(1, 3);
    cmat[8] = imat(2, 0); cmat[9] = imat(2, 1); cmat[10] = imat(2, 2); cmat[11] = imat(2, 3);

    ROS_INFO("%f %f %f %f", cmat[0], cmat[1], cmat[2], cmat[3]);
    ROS_INFO("%f %f %f %f", cmat[4], cmat[5], cmat[6], cmat[7]);
    ROS_INFO("%f %f %f %f", cmat[8], cmat[9], cmat[10], cmat[11]);
  }
};

int main (int argc, char** argv)
{
  // ROS init
  ros::init (argc, argv, "white_balance_converter");

  WhiteBalanceConverter p;
  ros::spin ();

  return (0);
}
