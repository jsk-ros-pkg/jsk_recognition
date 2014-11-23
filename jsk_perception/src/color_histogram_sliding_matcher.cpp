#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <rospack/rospack.h>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_perception/Rect.h>
#include <jsk_perception/ColorHistogramSlidingMatcherConfig.h>

using namespace std;
using namespace ros;

class MatcherNode
{
  boost::mutex _mutex;
  ros::NodeHandle _node;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _subImage;
  ros::Subscriber _subInfo;
  ros::Publisher _pubBestRect;
  ros::Publisher _pubBestPolygon; //for jsk_pcl_ros/src/pointcloud_screenpoint_nodelet.cpp
  std::vector< cv::Mat > template_images;
  std::vector< std::vector<unsigned int> > template_vecs;
  std::vector< std::vector< std::vector<unsigned int> > >hsv_integral;  
  int standard_height, standard_width;
  double coefficient_thre;
  bool show_result_;
  inline double calc_coe(std::vector< unsigned int > &a, std::vector<unsigned int> &b){
    unsigned long sum_a=0, sum_b=0;
    double coe=0;
    for(int k=0, size=a.size(); k<size; ++k){
      sum_a += a[k]; sum_b+= b[k];
    }
    for(int k=0; k<256; ++k){
      coe +=std::min((double)a[k]/sum_a, (double)b[k]/sum_b);
    }
    return coe;
  }
  typedef struct BOX{
    int x, y, dx, dy;
    BOX(int _x, int _y, int _dx, int _dy){
      x=_x, y=_y, dx=_dx, dy=_dy;
    }
  } box;
  inline bool point_in_box(int x, int y, box a_box){
    return a_box.x <= x && a_box.x+a_box.dx >= x && a_box.y <= y && a_box.y+a_box.dy >= y;
  }
  inline bool in_box(int x, int y, int dx, int dy, box a_box){
    return point_in_box(x, y, a_box) || point_in_box(x+dx, y, a_box) || point_in_box(x, y+dy, a_box) || point_in_box(x+dx, y+dy, a_box);
  }
  inline bool in_boxes(int x, int y, int dx, int dy,  std::vector<box> boxes){
    for (int i=0; i<boxes.size(); ++i){
      if(in_box(x, y, dx, dy, boxes[i])){
	return true;
      }
    }
    return false;
  }
public:
  MatcherNode() : _it(_node)
  {
    _pubBestRect = _node.advertise< jsk_perception::Rect >("best_rect", 1);
    _pubBestPolygon = _node.advertise< geometry_msgs::PolygonStamped >("best_polygon", 1);
    _subImage = _it.subscribe("image", 1, &MatcherNode::image_cb, this);
    ros::NodeHandle local_nh("~");
    std::string template_filename;
    std::string default_template_file_name;
    try {
#ifdef ROSPACK_EXPORT
      rospack::ROSPack rp;
      rospack::Package *p = rp.get_pkg("jsk_perception");
      if (p!=NULL) default_template_file_name = p->path + std::string("/sample/opencv-logo2.png");
#else
      rospack::Rospack rp;
      std::vector<std::string> search_path;
      rp.getSearchPathFromEnv(search_path);
      rp.crawl(search_path, 1);
      std::string path;
      if (rp.find("jsk_perception",path)==true) default_template_file_name = path + std::string("/sample/opencv-logo2.png");
#endif
    } catch (std::runtime_error &e) {}
    local_nh.param("template_filename", template_filename, default_template_file_name);
    local_nh.param("standard_width", standard_width, 12);
    local_nh.param("standard_height", standard_height, 12);
    local_nh.param("coefficient_threshold", coefficient_thre, 0.67);
    local_nh.param("show_result", show_result_, true);
    cv::Mat template_image=cv::imread(template_filename);
    if(template_add(template_image))template_images.push_back(template_image);
  }
  bool template_add(cv::Mat template_image){
    if(template_image.cols==0){
      ROS_INFO("template_error size==0");
      return false;
    }
    cv::Mat template_hsv_image; 
    cv::cvtColor(template_image, template_hsv_image, CV_BGR2HSV);
    std::vector<unsigned int> template_vec(256);
    for (int k=0; k<256; ++k){
      template_vec[k]=0;
    }
    for (int i = 0; i<template_image.rows; ++i){
      for (int j = 0; j<template_image.cols; ++j) {
	uchar h=template_hsv_image.data[i*template_hsv_image.step+j*3+0];
	uchar s=template_hsv_image.data[i*template_hsv_image.step+j*3+1];
	int index_h=0;
	int index_s=0;
	while(h>15)
	  {
	    h-=16;++index_h;
	  }
	while(s>15)
	  {
	    s-=16;++index_s;
	  }
	++template_vec[index_h*16 + index_s];
      }
    }
    template_vecs.push_back(template_vec);
    ROS_INFO("template vec was set successfully");
    return true;
  }
  
  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    boost::mutex::scoped_lock lock(_mutex);
    if(template_vecs.size()==0){
      ROS_INFO("template vec is empty");
      return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image=cv_ptr->image.clone();
    ROS_INFO("mat made");
    if(hsv_integral.size()==0||(image.cols!=hsv_integral.size() && image.rows!=hsv_integral[0].size())){
      ROS_INFO("before memory size was changed");
      hsv_integral.resize(image.cols);
      for(int i=0; i<image.cols; ++i){
	hsv_integral[i].resize(image.rows);
	for(int j=0; j<image.rows; ++j){
	  hsv_integral[i][j].resize(255);
	}
      }
      ROS_INFO("memory size was changed");
    }
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);
    unsigned char hsv_mat[image.cols][image.rows];
    for (size_t i = 0, image_rows=image.rows; i<image_rows; ++i){
      for (size_t j = 0, image_cols=image.cols; j<image_cols; ++j) {
	uchar h=hsv_image.data[i*hsv_image.step+j*3+0];
	uchar s=hsv_image.data[i*hsv_image.step+j*3+1];
	int index_h=0;
	int index_s=0;
	while(h>15)
	  {
	    h-=16;++index_h;
	  }
	while(s>15)
	{
	  s-=16;++index_s;
	}
	hsv_mat[j][i] = index_h*16+index_s;
      }
    }
    
#ifdef _OPENMP
#pragma omp parallel for
#endif 
    for (size_t k =0; k<16*16; ++k){
      if (k == hsv_mat[0][0]){
	hsv_integral[0][0][k]=1;
      }
      else{
	hsv_integral[0][0][k]=0;
      }
      for (size_t j=1, image_cols=image.cols; j<image_cols; ++j){
	if(k == hsv_mat[j][0]){
	  hsv_integral[j][0][k] = hsv_integral[j-1][0][k]+1;
	}
	else{
	  hsv_integral[j][0][k] = hsv_integral[j-1][0][k];
	}
      }
      for (int i=1,image_rows=image.rows; i<image_rows; ++i){
	unsigned int s=0;
	for (int j=0, image_cols=image.cols; j<image_cols; ++j){
	  if(k == hsv_mat[j][i]){
	    ++s;
	  }
	  hsv_integral[j][i][k] = hsv_integral[j][i-1][k]+ s;
	}
      }
    } 
    ROS_INFO("integral histogram made");
    for(size_t template_index=0; template_index<template_vecs.size(); ++template_index){
      std::vector<unsigned int> template_vec = template_vecs[template_index];
      double max_coe = 0;
      int index_i=-1, index_j=-1;
      float max_scale=-1;  
      // for(float scale=1; scale<image.cols/32; scale*=1.2){
      std::vector<box> boxes;
      for(float scale=image.cols/32; scale > 1; scale/=1.2){
	int dot = (int)(scale*2);
	int width_d = (int)(scale*standard_width);
	int height_d = (int)(scale*standard_height);
	//ROS_INFO("%d %d %d",dot, width_d, height_d);
	for(size_t i=0, image_rows=image.rows; i+height_d < image_rows; i+=dot){
	  for(size_t j=0, image_cols=image.cols; j+width_d < image_cols; j+=dot){	
	    //if in box continue;
	    std::vector <unsigned int> vec(256);
	    for(size_t k=0; k<256; ++k){
	      vec[k] = hsv_integral[j+width_d][i+height_d][k] 
		- hsv_integral[j][i+height_d][k] 
		- hsv_integral[j+width_d][i][k] 
		+ hsv_integral[j][i][k];
	      //ROS_INFO("inte %d",hsv_integral[j+width_d][i+height_d][k]);
	    }
	    double temp_coe = calc_coe(template_vec, vec);
	    //if larger than 0.7
	    if(temp_coe > coefficient_thre){
	      if(in_boxes(j, i, width_d, height_d, boxes)){
		continue;
	      }
	      boxes.push_back(box(j, i, width_d, height_d));
	  
	      if (temp_coe > max_coe){
		max_coe = temp_coe;
		index_i=i; index_j=j; max_scale = scale;
	      }
	    }
	  }
	}
      }
      for(int i=0; i<boxes.size(); ++i){
	ROS_INFO("x: %d, y: %d, dx:%d, dy:%d", boxes[i].x, boxes[i].y, boxes[i].dx, boxes[i].dy);
	cv::rectangle(image, cv::Point(boxes[i].x, boxes[i].y), cv::Point(boxes[i].x+boxes[i].dx, boxes[i].y+boxes[i].dy), cv::Scalar(200, 0, 0), 3, 4);
      }
      ROS_INFO("y:%d x:%d coe:%f scale:%f", index_i, index_j, max_coe, max_scale);
      // best result;
      cv::rectangle(image, cv::Point(index_j, index_i), cv::Point(index_j+(max_scale*standard_width), index_i+(max_scale*standard_height)), cv::Scalar(0, 0, 200), 3, 4);
      jsk_perception::Rect rect;
      rect.x=index_j, rect.y=index_i, rect.width=(int)(max_scale*standard_width); rect.height=(int)(max_scale*standard_height);
      _pubBestRect.publish(rect);
      geometry_msgs::PolygonStamped polygon_msg;
      polygon_msg.header = msg_ptr->header;
      polygon_msg.polygon.points.resize(2);
      polygon_msg.polygon.points[0].x=index_j;
      polygon_msg.polygon.points[0].y=index_i;
      polygon_msg.polygon.points[1].x=(int)(max_scale*standard_width)+index_j;
      polygon_msg.polygon.points[1].y=(int)(max_scale*standard_height)+index_i;
      _pubBestPolygon.publish(polygon_msg);
      if(show_result_){
	cv::imshow("result", image);
	cv::imshow("template", template_images[template_index]);
	cv::waitKey(20);
      }
    } 
  }
  void dyn_conf_callback(jsk_perception::ColorHistogramSlidingMatcherConfig &config, uint32_t level){
    boost::mutex::scoped_lock lock(_mutex);
    standard_height = config.standard_height;
    standard_width = config.standard_width;
    coefficient_thre = config.coefficient_threshold;
    if(level==1){ // size changed
      
    }
  }
};

inline double calc_coe(unsigned int * a, unsigned int* b){
  unsigned long sum_a=0, sum_b=0;
  double coe=0;
  for(int k=0; k<256; ++k){
    sum_a += a[k]; sum_b+= b[k];
  }
  for(int k=0; k<256; ++k){
    coe +=std::min((double)a[k]/sum_a, (double)b[k]/sum_b);
  }
  return coe;
}

typedef struct BOX{
  int x, y, dx, dy;
  BOX(int _x, int _y, int _dx, int _dy){
    x=_x, y=_y, dx=_dx, dy=_dy;
  }
} box;
inline bool point_in_box(int x, int y, box a_box){
  return a_box.x <= x && a_box.x+a_box.dx >= x && a_box.y <= y && a_box.y+a_box.dy >= y;
}
inline bool in_box(int x, int y, int dx, int dy, box a_box){
  return point_in_box(x, y, a_box) || point_in_box(x+dx, y, a_box) || point_in_box(x, y+dy, a_box) || point_in_box(x+dx, y+dy, a_box);
}
inline bool in_boxes(int x, int y, int dx, int dy,  std::vector<box> boxes){
  for (int i=0; i<boxes.size(); ++i){
    if(in_box(x, y, dx, dy, boxes[i])){
      return true;
    }
  }
  return false;
  
}

int main(int argc, char **argv){
  ros::init (argc, argv, "ColorHistogramSlidingMatcher");

  MatcherNode matcher;

  dynamic_reconfigure::Server<jsk_perception::ColorHistogramSlidingMatcherConfig> server;
  dynamic_reconfigure::Server<jsk_perception::ColorHistogramSlidingMatcherConfig>::CallbackType f;
  f = boost::bind(&MatcherNode::dyn_conf_callback, &matcher, _1, _2);
  server.setCallback(f);  
  ros::spin();
  return 0;
}
