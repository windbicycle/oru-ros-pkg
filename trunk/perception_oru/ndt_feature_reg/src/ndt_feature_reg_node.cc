#include <cstdio>
#include <fstream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <Eigen/Eigen>
#include <fstream>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <depth_camera.h>

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_feature_reg/ndt_frame_proc.h>
#include <ndt_feature_reg/ndt_frame_viewer.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace lslgeneric;
using namespace ndt_feature_reg;

cv::Mat generateLookupTable(const pcl::PointCloud<pcl::PointXYZ> &pointcloud)
{
    cv::Mat lookup_table(cv::Size(pointcloud.width, pointcloud.height), CV_64FC3);
    if (pointcloud.is_dense)
    {

    }
    else
    {
        assert(false);
    }
    return lookup_table;
}

void subsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ> &orig, pcl::PointCloud<pcl::PointXYZ> &sub, int step)
{
    uint32_t x = 0;
    while (x < orig.width)
    {
        uint32_t y = 0;
        while (y*step < orig.height)
        {
            sub.push_back(orig.at(x*step, y*step));
            y += step;
        }
        x += step;
    }
}



class NDTFeatureRegNode
{
protected:
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::Image> intensity_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> points2_sub_;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync_;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> > sync2_;

    sensor_msgs::CvBridge _imBridge;
    bool visualize_;

    tf::TransformBroadcaster tf_;
    tf::TransformListener tf_listener_;

    ros::Publisher pub_points2_;

    boost::mutex m_;
    lslgeneric::DepthCamera<pcl::PointXYZ> cameraparams_;
    bool camerasetup_;
    bool skip_matching_;
    bool estimate_di_;
    bool match_full_;
    bool match_no_association_;
    int support_size_;
    double max_var_;
    double current_res_;
    int max_nb_frames_;

    int camera_nb_;
    bool set_initial_pose_;
    bool publish_cloud_;
    int subsample_step_;

    ros::Time prev_timestamp_;
    NDTFrameProc<pcl::PointXYZ>* proc;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> global_transform_;
    void TransformEigenToTF(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &k, tf::Transform &t)
    {
        t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
        t.setBasis(btMatrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
    };

    void setupCamera(bool isFloat)
    {
        double fx, fy, cx, cy, ds, scale;
        std::vector<double> dist(5);
        switch (camera_nb_)
        {
        case 1:
            fx = 517.3;
            fy = 516.5;
            cx = 318.6;
            cy = 255.3;
            dist[0] = 0.2624;
            dist[1] = -0.9531;
            dist[2] = -0.0054;
            dist[3] = 0.0026;
            dist[4] = 1.1633;
            ds = 1.035; // Depth scaling factor.
            scale = 0.0002;
            break;
        case 2:
            fx = 520.9;
            fy = 521.0;
            cx = 325.1;
            cy = 249.7;
            dist[0] = 0.2312;
            dist[1] = -0.7849;
            dist[2] = -0.0033;
            dist[3] = -0.0001;
            dist[4] = 0.9172;
            ds = 1.031;
            scale = 0.0002;
            break;
        default:
            cerr << "unknown camera number : " << camera_nb_ << endl;
        }
        cameraparams_ = lslgeneric::DepthCamera<pcl::PointXYZ>(fx,fy,cx,cy,dist,ds/*scale*/,isFloat);
    }

public:
    // Constructor
    NDTFeatureRegNode(ros::NodeHandle comm_nh, ros::NodeHandle param_nh) : sync_(2), sync2_(2)
    {
        bool xyzi_data;
        param_nh.param("xyzi_data", xyzi_data, false);

        if (!xyzi_data)
        {
            rgb_sub_.subscribe(nh_, "rgb", 1);
            depth_sub_.subscribe(nh_, "depth", 2);
            sync_.connectInput(rgb_sub_, depth_sub_);
            sync_.registerCallback( boost::bind(&NDTFeatureRegNode::rgbDepthCb, this, _1, _2) );
        }
        else
        {
            intensity_sub_.subscribe(nh_, "intensity", 1);
            points2_sub_.subscribe(nh_, "points2", 2);
            sync2_.connectInput(intensity_sub_, points2_sub_);
            sync2_.registerCallback( boost::bind(&NDTFeatureRegNode::intensityPoints2Cb, this, _1, _2) );
        }

        pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2>("reg_points2", 15);

        param_nh.param("visualize", visualize_, false);

        param_nh.param("skip_matching", skip_matching_, false);
        param_nh.param("estimate_di", estimate_di_, false);
        param_nh.param("match_full", match_full_, false);
        param_nh.param("match_no_association", match_no_association_, false);
        param_nh.param("support_size", support_size_, 10);
        param_nh.param("max_var", max_var_, 0.3);
        param_nh.param("current_res", current_res_, 0.2);
        param_nh.param("max_nb_frames", max_nb_frames_, 10);

        param_nh.param("camera_nb", camera_nb_, 1);

        double max_inldist_xy, max_inldist_z;
        int nb_ransac;

        param_nh.param("max_inldist_xy", max_inldist_xy, 0.02);
        param_nh.param("max_inldist_z", max_inldist_z, 0.05);
        param_nh.param("nb_ransac", nb_ransac, 1000);

        proc = new NDTFrameProc<pcl::PointXYZ>(nb_ransac, max_inldist_xy, max_inldist_z);

        double detector_thresh;
        param_nh.param("detector_thresh", detector_thresh, 400.);
        //proc->detector = new cv::SurfFeatureDetector( detector_thresh );
        proc->detector = cv::FeatureDetector::create("SURF");

        param_nh.param("img_scale", proc->img_scale, 0.25);
        param_nh.param("trim_factor", proc->trim_factor, 1.);
        param_nh.param("non_mean", proc->non_mean, false);
        param_nh.param("windowed_matching2", proc->pe.windowed, false);
        param_nh.param("max_kp_dist", proc->pe.maxDist, 10.);
        param_nh.param("min_kp_dist", proc->pe.minDist, 0.);
        param_nh.param("set_initial_pose", set_initial_pose_, false);

        param_nh.param("publish_cloud", publish_cloud_, false);
        param_nh.param("subsample_step", subsample_step_, 5);

        camerasetup_ = false;
        global_transform_.setIdentity();
    }

    ~NDTFeatureRegNode()
    {
        delete proc;
    }

    void process(cv::Mat &rgb_img, cv::Mat &depth_img, const ros::Time &current_timestamp)
    {
        NDTFrame<pcl::PointXYZ> *frame = new NDTFrame<pcl::PointXYZ>();
        frame->img = rgb_img;
        frame->depth_img = depth_img;
        frame->supportSize = support_size_;
        frame->maxVar = max_var_;
        frame->current_res = current_res_;
        frame->cameraParams = cameraparams_;
        proc->addFrameIncremental(frame, skip_matching_, estimate_di_,match_full_,match_no_association_);

        if (visualize_)
            viewKeypointMatches(proc, 10);


        proc->trimNbFrames(max_nb_frames_);

        global_transform_ =  global_transform_ * proc->transformVector.back();
        tf::Transform transform;
        tf::Transform global_transform;
        TransformEigenToTF(proc->transformVector.back(), transform);
        TransformEigenToTF(global_transform_, global_transform);
        //	       ros::Time current_timestamp = msg_rgb->header.stamp;

        tf_.sendTransform(tf::StampedTransform(transform, current_timestamp, "world", "ndt_feature_reg"));
        tf_.sendTransform(tf::StampedTransform(global_transform, current_timestamp, "world", "global_ndt_feature_reg"));

        std::string gt_frame = "/openni_rgb_optical_frame";
        if (publish_cloud_)
        {
            pcl::PointCloud<pcl::PointXYZ> depthcloud, sub_depthcloud;
            cameraparams_.convertDepthImageToPointCloud(frame->depth_img,depthcloud);
            // Subsample the pointcloud to be able to visualize longer sequences.
            subsamplePointCloud(depthcloud, sub_depthcloud, subsample_step_);

            sensor_msgs::PointCloud2 pcloud;
            pcl::toROSMsg(sub_depthcloud,pcloud);
            pcloud.header.stamp = ros::Time::now();
            pcloud.header.frame_id = "global_ndt_feature_reg";
            pub_points2_.publish(pcloud);
        }

        try
        {
            tf::StampedTransform gt_transform;

            tf_listener_.waitForTransform(gt_frame, prev_timestamp_,
                                          gt_frame, current_timestamp,
                                          "/world", ros::Duration(2.0));

            tf_listener_.lookupTransform(gt_frame, prev_timestamp_,
                                         gt_frame, current_timestamp,
                                         "/world", gt_transform);

            tf_.sendTransform(tf::StampedTransform(gt_transform, current_timestamp, "world", "gt_reg"));

            if (set_initial_pose_)
            {
                tf::StampedTransform glb_gt_transform;
                tf_listener_.lookupTransform("/world", gt_frame, current_timestamp, glb_gt_transform);
                TransformTFToEigen(glb_gt_transform, global_transform_);
                set_initial_pose_ = false;
            }

            prev_timestamp_ = current_timestamp;
        }
        catch ( tf::TransformException & ex )
        {
            ROS_ERROR( "%s",
                       ex.what() );
        }

    }

    void intensityPoints2Cb(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::PointCloud2::ConstPtr& msg_pts)
    {
        m_.lock();

        ROS_INFO("Got intensity and points2 pair...");
        cv::Mat intensity(_imBridge.imgMsgToCv(msg_img, "mono8"));
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg (*msg_pts, pc);

        // Currently lacking support to handle pointcloud directly convert it to a depth image... also we don't have any calibration data from the Fotonic camera in the same format but can derrive this from the 3D depth data.

        if (!camerasetup_)
        {
            cameraparams_.setLookupTable(generateLookupTable(pc));
            camerasetup_ = true;
        }
        m_.unlock();
    }

    void rgbDepthCb(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
    {
        m_.lock();
        ROS_INFO("Got image pair...");
        cv::Mat rgb_img(_imBridge.imgMsgToCv(msg_rgb, "bgr8")); // mono8
        cv::Mat depth_img(_imBridge.imgMsgToCv(msg_depth)); //, "mono16"));

        if (!camerasetup_)
        {
            setupCamera(depth_img.depth() == CV_32F);
            camerasetup_ = true;
        }

        process(rgb_img, depth_img, msg_rgb->header.stamp);
        m_.unlock();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_feature_reg_node");
    ros::NodeHandle comm_nh ("camera"); // for topics, services
    ros::NodeHandle param_nh ("~");     // for parameters

    NDTFeatureRegNode n(comm_nh, param_nh);
    ros::spin();

    return 0;
}
