// -*- mode: c++ -*-
#ifndef __COLOR_FILTER_H__
#define __COLOR_FILTER_H__

#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

namespace pcl
{
  template <typename PointT>
  class RGBColorFilter : public Filter<PointT>
  {
  };

  template <>
  class RGBColorFilter<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud< PointType > PointCloud;

  protected:
    int r_min_, r_max_, b_min_, b_max_, g_min_, g_max_;
    ConditionalRemoval<PointType> filter_instance_;

  public:
    RGBColorFilter () : filter_instance_ (true)
    {
      r_min_ = b_min_ = g_min_ = 0;
      r_max_ = b_max_ = g_max_ = 255;
      filter_name_ = "RGBColorFilter";
      updateCondition();
    }
    virtual ~RGBColorFilter (){ };

    inline void setRedMin (int r_min) {r_min_ = r_min;}
    inline void setRedMax (int r_max) {r_max_ = r_max;}
    inline void setGreenMin (int g_min) {g_min_ = g_min;}
    inline void setGreenMax (int g_max) {g_max_ = g_max;}
    inline void setBlueMin (int b_min) {b_min_ = b_min;}
    inline void setBlueMax (int b_max) {b_max_ = b_max;}
    inline int getRedMin() { return r_min_; }
    inline int getRedMax() { return r_max_; }
    inline int getGreenMin() { return g_min_; }
    inline int getGreenMax() { return g_max_; }
    inline int getBlueMin() { return b_min_; }
    inline int getBlueMax() { return b_max_; }
    inline void updateCondition()
    {
      pcl::ConditionBase<PointType>::Ptr condp (new pcl::ConditionOr<PointType> ());

      if ( r_max_ >= r_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("r", pcl::ComparisonOps::GE, r_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("r", pcl::ComparisonOps::LE, r_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("r", pcl::ComparisonOps::LE, r_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("r", pcl::ComparisonOps::GE, r_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      if ( g_max_ >= g_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("g", pcl::ComparisonOps::GE, g_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("g", pcl::ComparisonOps::LE, g_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("g", pcl::ComparisonOps::LE, g_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("g", pcl::ComparisonOps::GE, g_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      if ( b_max_ >= b_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("b", pcl::ComparisonOps::GE, b_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("b", pcl::ComparisonOps::LE, b_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedRGBComparison<PointType> ("b", pcl::ComparisonOps::LE, b_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedRGBComparison<PointType> ("b", pcl::ComparisonOps::GE, b_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      filter_instance_.setCondition (condp);
    }

    inline virtual void applyFilter (sensor_msgs::PointCloud2 &output)
    {
      PointCloud tmp_in;
      fromROSMsg<PointType> (*input_, tmp_in);
      filter_instance_.setInputCloud (tmp_in.makeShared());
      filter_instance_.setIndices (indices_);

      PointCloud tmp_out;
      filter_instance_.filter (tmp_out);
      pcl::IndicesConstPtr ids_c = filter_instance_.getRemovedIndices();
      pcl::IndicesPtr ids (new std::vector<int>);
      ids->resize (ids_c->size());
      for (size_t i = 0; i < ids_c->size(); i++)
          (*ids)[i] = (*ids_c)[i];

      pcl::ExtractIndices< sensor_msgs::PointCloud2 > ei;
      ei.setInputCloud (input_);
      ei.setIndices (ids);
      ei.filter (output);
    }
  };

  template <typename PointT>
  class HSVColorFilter : public Filter<PointT>
  {
  };

  template <>
  class HSVColorFilter<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud< PointType > PointCloud;

  protected:
    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    bool use_h_;
    ConditionalRemoval<PointType> filter_instance_;

  public:
    HSVColorFilter () : h_min_(-128), h_max_(127), s_min_(0), s_max_(255),
                        v_min_(0), v_max_(255), use_h_(true),
                        filter_instance_(true)
    {
      filter_name_ = "HSVColorFilter";
      updateCondition();
    }
    virtual ~HSVColorFilter () {};

    inline void setHueMin (int h_min) {h_min_ = h_min;}
    inline void setHueMax (int h_max) {h_max_ = h_max;}
    inline void setSaturationMin (int s_min) {s_min_ = s_min;}
    inline void setSaturationMax (int s_max) {s_max_ = s_max;}
    inline void setValueMin (int v_min) {v_min_ = v_min;}
    inline void setValueMax (int v_max) {v_max_ = v_max;}
    inline int getHueMin() { return h_min_; }
    inline int getHueMax() { return h_max_; }
    inline int getSaturationMin() { return s_min_; }
    inline int getSaturationMax() { return s_max_; }
    inline int getValueMin() { return v_min_; }
    inline int getValueMax() { return v_max_; }
    inline void setUseHue (bool use_h) { use_h_ = use_h; }
    inline bool getUseHue (void) { return use_h_; }
    inline void updateCondition()
    {
      pcl::ConditionBase<PointType>::Ptr condp (new pcl::ConditionOr<PointType> ());

      if ( h_max_ >= h_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("h", pcl::ComparisonOps::GE, h_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("h", pcl::ComparisonOps::LE, h_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("h", pcl::ComparisonOps::LE, h_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("h", pcl::ComparisonOps::GE, h_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      if ( s_max_ >= s_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("s", pcl::ComparisonOps::GE, s_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("s", pcl::ComparisonOps::LE, s_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("s", pcl::ComparisonOps::LE, s_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("s", pcl::ComparisonOps::GE, s_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      if ( v_max_ >= v_min_ )
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("i", pcl::ComparisonOps::GE, v_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("i", pcl::ComparisonOps::LE, v_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }
      else
      {
        pcl::ConditionBase<PointType>::Ptr cond (new pcl::ConditionAnd<PointType> ());
        pcl::ComparisonBase<PointType>::Ptr
          le (new pcl::PackedHSIComparison<PointType> ("i", pcl::ComparisonOps::LE, v_max_));
        pcl::ComparisonBase<PointType>::Ptr
          ge (new pcl::PackedHSIComparison<PointType> ("i", pcl::ComparisonOps::GE, v_min_));
        cond->addComparison (le);
        cond->addComparison (ge);
        condp->addCondition(cond);
      }

      filter_instance_.setCondition (condp);
    }

    virtual inline void applyFilter (sensor_msgs::PointCloud2 &output)
    {
      PointCloud tmp_in;
      fromROSMsg<PointType> (*input_, tmp_in);
      filter_instance_.setInputCloud (tmp_in.makeShared());
      filter_instance_.setIndices (indices_);

      PointCloud tmp_out;
      filter_instance_.filter (tmp_out);
      pcl::IndicesConstPtr ids_c = filter_instance_.getRemovedIndices();
      pcl::IndicesPtr ids (new std::vector<int>);
      ids->resize (ids_c->size());
      for (size_t i = 0; i < ids_c->size(); i++)
          (*ids)[i] = (*ids_c)[i];

      pcl::ExtractIndices< sensor_msgs::PointCloud2 > ei;
      ei.setInputCloud (input_);
      ei.setIndices (ids);
      ei.filter (output);
    }
  };
}

#endif
