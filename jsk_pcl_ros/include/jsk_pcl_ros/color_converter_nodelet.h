// -*- mode: c++ -*-
#ifndef COLOR_CONVERTER_NODELET_H_
#define COLOR_CONVERTER_NODELET_H_

#include "color_converter.h"

#include "pcl_ros/features/feature.h"

//#include "jsk_pcl_ros/ColorFilterConfig.h"


namespace pcl_ros
{
    namespace sync_policies = message_filters::sync_policies;

    template <typename InPointT, typename OutPointT> class ColorConverter
        : public Feature
    {
        using PCLNodelet::isValid;
        // overwrite types
    public:
        typedef typename pcl::PointCloud<InPointT> PointCloudIn;
        typedef typename PointCloudIn::Ptr PointCloudInPtr;
        typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
        typedef typename pcl::PointCloud<OutPointT> PointCloudOut;
        
        inline void
        input_callback (const PointCloudInConstPtr &input)
            {
                PointIndices indices;
                indices.header.stamp = input->header.stamp;
                PointCloudIn cloud;
                cloud.header.stamp = input->header.stamp;
                nf_pc_.add (cloud.makeShared ());
                nf_pi_.add (boost::make_shared<PointIndices> (indices));
            }
        
    protected:
        message_filters::Subscriber<PointCloudIn> sub_input_filter_;
        
        
    private:
        // overwrite for PointHSV, PointRGB
        
        boost::shared_ptr <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloudIn, PointCloudIn, PointIndices> > > sync_input_surface_indices_a_;
        boost::shared_ptr <message_filters::Synchronizer<sync_policies::ExactTime<PointCloudIn, PointCloudIn, PointIndices> > > sync_input_surface_indices_e_;
        message_filters::PassThrough<PointIndices> nf_pi_;
        message_filters::PassThrough<PointCloudIn> nf_pc_;
        
        void
        input_surface_indices_callback2 (const PointCloudInConstPtr &cloud, 
                                         const PointCloudInConstPtr &cloud_surface,
                                         const PointIndicesConstPtr &indices)
            {
                // No subscribers, no work
                if (pub_output_.getNumSubscribers () <= 0)
                    return;
                // If indices given...
                IndicesPtr vindices;
                if (indices && !indices->header.frame_id.empty ())
                    vindices = boost::make_shared < std::vector<int> > (indices->indices);
                computePublish (cloud, cloud_surface, vindices);
            }
        
        // dummy implementation of pure virtual functions of Feature class! 
        void emptyPublish (const Feature::PointCloudInConstPtr &cloud)
            {
                NODELET_ERROR("emptyPublish of Feature class is called. it might be a bug");
            }
        
        void computePublish (const Feature::PointCloudInConstPtr &cloud,
                             const Feature::PointCloudInConstPtr &surface,
                             const Feature::IndicesPtr &indices)
            {
                NODELET_ERROR("computePublish of Feature class is called. it might be a bug");
            }

        void emptyPublish (const PointCloudInConstPtr &cloud)
            {
                PointCloudOut output;
                output.header = cloud->header;
                pub_output_.publish (output.makeShared ());
            }

        virtual void
        onInit()
            {
                PCLNodelet::onInit ();
                use_surface_ = false; // we dont use it

                childInit (*pnh_);
                
                // If we're supposed to look for PointIndices (indices) or PointCloud (surface) messages
                if (use_indices_)
                {
                    // Create the objects here
                    if (approximate_sync_)
                        sync_input_surface_indices_a_ =
                            boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloudIn, PointCloudIn, PointIndices> > >(max_queue_size_);
                    else
                        sync_input_surface_indices_e_ = boost::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloudIn, PointCloudIn, PointIndices> > >(max_queue_size_);
                    
                    // Subscribe to the input using a filter
                    sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
                    // If indices are enabled, subscribe to the indices
                    sub_indices_filter_.subscribe (*pnh_, "indices", max_queue_size_);
                    sub_input_filter_.registerCallback (bind (&ColorConverter::input_callback, this, _1));
                    
                    // surface not enabled, connect the input-indices duo and register
                    if (approximate_sync_)
                        sync_input_surface_indices_a_->connectInput (sub_input_filter_, nf_pc_, sub_indices_filter_);
                    else
                        sync_input_surface_indices_e_->connectInput (sub_input_filter_, nf_pc_, sub_indices_filter_);
                    
                    // Register callbacks
                    if (approximate_sync_)
                        sync_input_surface_indices_a_->registerCallback (bind (&ColorConverter::input_surface_indices_callback2, this, _1, _2, _3));
                    else
                        sync_input_surface_indices_e_->registerCallback (bind (&ColorConverter::input_surface_indices_callback2, this, _1, _2, _3));
                }
                else
                    //Subscribe in an old fashion to input only (no filters)
                    sub_input_ =
                        pnh_->subscribe<PointCloudIn> ("input",
                                                       max_queue_size_,
                                                       bind (&ColorConverter::input_surface_indices_callback2, this, _1, PointCloudInConstPtr (), PointIndicesConstPtr ()));
            }
        
        void input_surface_indices_callback
        (const PointCloudInConstPtr &cloud,
         const PointCloudInConstPtr &cloud_surface,
         const PointIndicesConstPtr &indices)
            {
                // No subscribers, no work
                if (pub_output_.getNumSubscribers () <= 0)
                    return;
                
                // If indices given...
                IndicesPtr vindices;
                if (indices && !indices->header.frame_id.empty ())
                    vindices = boost::make_shared < std::vector<int> >
                        (indices->indices);
            }
        
        virtual inline bool
        childInit (ros::NodeHandle& nh)
            {
                pub_output_ = nh.advertise<PointCloudOut> ("output",
                                                           max_queue_size_);
                return true;
            }
        
        virtual void
        computePublish (const PointCloudInConstPtr &cloud,
                        const PointCloudInConstPtr &surface,
                        const IndicesPtr &indices) = 0;
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    class RGB2HSVColorConverter :
        public ColorConverter<pcl::PointRGB, pcl::PointHSV>
    {
    public:
        void
        computePublish (const PointCloudInConstPtr &cloud,
                        const PointCloudInConstPtr &surface,
                        const IndicesPtr &indices);
        
        RGB2HSVColorConverter() { }
        ~RGB2HSVColorConverter() { }
    protected:
        pcl::RGB2HSVConverter<pcl::PointRGB, pcl::PointHSV> impl_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class HSV2RGBColorConverter :
        public ColorConverter<pcl::PointHSV, pcl::PointRGB>
    {
    public:
        void
        computePublish (const PointCloudInConstPtr &cloud,
                        const PointCloudInConstPtr &surface,
                        const IndicesPtr &indices);
        HSV2RGBColorConverter() { }
        ~HSV2RGBColorConverter() { }
    protected:
        pcl::HSV2RGBConverter<pcl::PointHSV, pcl::PointRGB> impl_;
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
}

#endif
