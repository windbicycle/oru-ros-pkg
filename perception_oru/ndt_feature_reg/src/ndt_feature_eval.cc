// Evaluation using TUM data set of feature f2f registration.
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_feature_reg/ndt_frame_proc.h>
#include <ndt_feature_reg/ndt_frame_viewer.h>

#include <ndt_matcher_d2d_feature.h>
#include <pointcloud_utils.h>

#include <cv.h>
#include <highgui.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace ndt_feature_reg;
using namespace pcl;
using namespace cv;
using namespace std;
using namespace lslgeneric;

namespace po = boost::program_options;

inline
std::vector<std::string> split_string(const std::string &str, const std::string &separator)
{
    std::string::size_type it = 0;
    std::vector<std::string> values;
    bool stop = false;
    while (!stop)
    {
        std::string::size_type start = it;
        it = str.find(separator, it+1); // Gets the next comma.
        if (it == std::string::npos) // didn't find any new comma, return the remaining string.
        {
            values.push_back(str.substr(start, std::string::npos));
            stop = true;
        }
        else
        {
            it++; // We want the character after the comma.
            values.push_back(str.substr(start, it-start-1));
        }
    }
    return values;
}


vector<pair<string,string> > load_associations(const string &association_file, vector<string> &timestamps, int skip_nb)
{
    // Generated by associate.py rgb.txt depth.txt (always this order)
    vector<pair<string, string> > ret;
    std::ifstream file(association_file.c_str());
    std::string line;
    bool skip = false;
    int skip_counter = 0;
    while(std::getline (file,line))
    {
        vector<string> splits = split_string(line, std::string(" "));
        if (splits.size() < 4)
        {
            cerr << "couldn't parse : " << line << endl;
            continue;
        }
        if (!skip)
        {
            ret.push_back(pair<string,string>(splits[1], splits[3])); // rgb, depth
            timestamps.push_back(splits[0]); // Timestamp for the rgb.
        }

        if (skip_counter > skip_nb)
        {
            skip = false;
            skip_counter = 0;
        }
        else
        {
            skip = true;
            skip_counter++;
        }
    }
    return ret;
}




int main(int argc, char** argv)
{
    cout << "--------------------------------------------------" << endl;
    cout << "Evaluations of f2f registration" << endl;
    cout << "--------------------------------------------------" << endl;

    string association_name;
    string output_name;
    string times_name;
    double scale, max_inldist_xy, max_inldist_z;
    Eigen::Matrix<double,6,1> pose_increment_v;
    int nb_ransac;
    int nb_points;
    size_t support_size;
    size_t max_nb_frames;
    double max_var;
    double current_res;
    int delay_vis;
    double img_scale;
    double detector_thresh;
    int skip_nb;
    int freiburg_camera_nb;
    double max_kp_dist, min_kp_dist;
    double trim_factor;
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("debug", "print debug output")
    ("visualize", "visualize the output")
    ("association", po::value<string>(&association_name), "association file name (generated by association.py script)")
    ("output", po::value<string>(&output_name), "file name of the estimation output (to be used with the evaluation python script)")
    ("time_output", po::value<string>(&times_name), "file name for saving the per-frame processing time (to be used with the evaluation python script)")
    ("scale", po::value<double>(&scale)->default_value(0.0002), "depth scale (depth = scale * pixel value)")
    ("max_inldist_xy", po::value<double>(&max_inldist_xy)->default_value(0.02), "max inlier distance in xy related to the camera plane (in meters)")
    ("max_inldist_z", po::value<double>(&max_inldist_z)->default_value(0.05), "max inlier distance in z - depth (in meters)")
    ("nb_ransac", po::value<int>(&nb_ransac)->default_value(1000), "max number of RANSAC iterations")
    ("nb_points", po::value<int>(&nb_points)->default_value(21), "number of surrounding points")
    ("support_size", po::value<size_t>(&support_size)->default_value(10), "width of patch around each feature in pixels")
    ("max_var", po::value<double>(&max_var)->default_value(0.3), "max variance of ndt cells")
    ("current_res", po::value<double>(&current_res)->default_value(0.2), "resolution of ndt cells if doing full matching")
    ("skip_matching", "skip the ndt matching step")
    ("match_full", "skip the visual features step and perform only ndt matching")
    ("match_no_association", "skip the feature association step and match directly")
    ("estimate_di", "don't compute the ndt explicitly but estimate from neighbourhood depth")
    ("max_nb_frames", po::value<size_t>(&max_nb_frames)->default_value(10), "max number of frames to be keept in the processing step")
    ("delay_vis", po::value<int>(&delay_vis)->default_value(10), "param to opencv function waitKey (if visualize is used)")
    ("img_scale", po::value<double>(&img_scale)->default_value(1.), "if the rgb image should be scaled when feature are detected and extracted - mainly to boost the speed")
    ("detector_thresh", po::value<double>(&detector_thresh)->default_value(400.), "playaround parameter, used to test different detectorvalues")
    ("skip_nb", po::value<int>(&skip_nb)->default_value(0), "how many intermediate frames should be skipped between the registration is runned")
    ("old_reg_code", "use the old registration code - for debugging only")
    ("freiburg_camera_nb", po::value<int>(&freiburg_camera_nb)->default_value(1), "the camera used for the evaluation 1-> freiburg camera #1, 2-> freiburg camera #2")
    ("max_kp_dist", po::value<double>(&max_kp_dist)->default_value(10.), "max distance of keypoints used")
    ("min_kp_dist", po::value<double>(&min_kp_dist)->default_value(0.), "min distance of keypoints used")
    ("windowed_matching", "limit the search of the keypoint matches to a local region (will not work that good if the skip_nb is large)")
    ("trim_factor", po::value<double>(&trim_factor)->default_value(1.), "only utilize the trimfactor number of keypoints with the closest NDT distance - similar idea as in trimmed ICP")
    ("non_mean", "if the mean used in NDT matching (including the RANSAC step) are computed direclty from the closest depth image pixel")
    ;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("output") || !vm.count("association"))
    {
        cout << "Missing association / output file names.\n";
        cout << desc << "\n";
        return 1;
    }
    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }
    bool debug = vm.count("debug");
    bool visualize = vm.count("visualize");
    bool skip_matching = vm.count("skip_matching");
    bool match_full = vm.count("match_full");
    bool match_no_association = vm.count("match_no_association");
    bool estimate_di = vm.count("estimate_di");
    bool old_reg_code = vm.count("old_reg_code");
    bool windowed_matching = vm.count("windowed_matching");
    bool non_mean = vm.count("non_mean");

    if (times_name == "")
    {
        times_name = output_name + ".times";
    }

    vector<string> timestamps;
    vector<pair<string,string> > associations = load_associations(association_name, timestamps, skip_nb);
    if (debug)
    {
        for (size_t i = 0; i < associations.size(); i++)
        {
            cout << "associations : " << associations[i].first << " <-> " << associations[i].second << endl;
        }
        cout << "loaded : " << associations.size() << " associations" << endl;
    }

    double fx, fy, cx, cy, ds;
    std::vector<double> dist(5);
    switch (freiburg_camera_nb)
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
        break;
    default:
        cerr << "unknown camera number : " << freiburg_camera_nb << endl;
        return 1;
    }


    double t1, t2;

    lslgeneric::DepthCamera<pcl::PointXYZ> cameraparams(fx,fy,cx,cy,dist,ds*scale,false);

    typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> EigenTransform;
    std::vector<EigenTransform, Eigen::aligned_allocator<EigenTransform> > transformVector;

    std::ofstream file_times(times_name.c_str(), ios::out);
    if (!file_times)
    {
        cerr << "Failed to open times file : " << times_name << endl;
        return -1;
    }

    file_times<<"times = [";
//#define PAIR_REG
//#ifdef PAIR_REG
    if (old_reg_code)
    {
        for (size_t i = 1; i < associations.size(); i++)
        {
            std::cout << "iter " << i << "(" << associations.size() << ")" << std::endl;

            NDTFrame<pcl::PointXYZ> *frame1 = new NDTFrame<pcl::PointXYZ>();
            frame1->img = cv::imread(associations[i-1].first, 0);
            frame1->depth_img = cv::imread(associations[i-1].second, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
            if (i == 1)
                cameraparams.setupDepthPointCloudLookUpTable(frame1->depth_img.size());
            frame1->cameraParams = cameraparams;
            frame1->supportSize = support_size;

            NDTFrame<pcl::PointXYZ> *frame2 = new NDTFrame<pcl::PointXYZ>();
            frame2->img = cv::imread(associations[i].first, 0);
            frame2->depth_img = cv::imread(associations[i].second, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
            frame2->cameraParams = cameraparams;
            frame2->supportSize = support_size;

            frame1->current_res = current_res;
            frame2->current_res = current_res;
            NDTFrameProc<pcl::PointXYZ> proc(nb_ransac, max_inldist_xy, max_inldist_z);
            proc.detector = cv::FeatureDetector::create("SURF");
            //( detector_thres
            proc.img_scale = img_scale;

            double t1 = getDoubleTime();
            proc.addFrame(frame1);
            proc.addFrame(frame2);
            proc.processFrames(skip_matching,estimate_di,match_full,match_no_association);
            double t2 = getDoubleTime();

            if (visualize)
                viewKeypointMatches(&proc, delay_vis);

            if (i == 1)
                transformVector.push_back(proc.transformVector[0]);
            transformVector.push_back(proc.transformVector[1]);

            file_times << t2-t1 <<", ";

            //	   delete frame1;
            //	   delete frame2;
        }
    }
    else
    {
        // ----------------------------------------------------------------
//#else
        NDTFrameProc<pcl::PointXYZ> proc(nb_ransac, max_inldist_xy, max_inldist_z);
        proc.detector = cv::FeatureDetector::create("SURF");
        //proc.detector = new cv::SurfFeatureDetector( detector_thresh );
        proc.img_scale = img_scale;
        proc.trim_factor = trim_factor;
        proc.non_mean = non_mean;
        proc.pe.windowed = windowed_matching;
        proc.pe.maxDist = max_kp_dist;
        proc.pe.minDist = min_kp_dist;
        // Loop through all the associations pairs
        for (size_t i = 0; i < associations.size(); i++)
        {
            std::cout << "iter " << i << "(" << associations.size() << ")" << std::endl;
            NDTFrame<pcl::PointXYZ> *frame = new NDTFrame<pcl::PointXYZ>();
            frame->img = cv::imread(associations[i].first, 0);
            frame->depth_img = cv::imread(associations[i].second, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
            frame->supportSize = support_size;
            frame->maxVar = max_var;
            frame->current_res = current_res;
            if (i == 0)
                cameraparams.setupDepthPointCloudLookUpTable(frame->depth_img.size());
            frame->cameraParams = cameraparams;
            //frame->supportSize = 10;

            double t1 = getDoubleTime();
            proc.addFrameIncremental(frame, skip_matching, estimate_di,match_full,match_no_association);
            double t2 = getDoubleTime();

            if (visualize)
                viewKeypointMatches(&proc, delay_vis);
            proc.trimNbFrames(max_nb_frames);

            file_times << t2-t1 <<", ";
        }
        transformVector = proc.transformVector;
    }
    file_times<<"];\n";
//#endif
    // Write out the transformations
    assert(associations.size() == transformVector.size());
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> global_transform;
    global_transform.setIdentity();


    std::ofstream file(output_name.c_str(), ios::out);
    if (!file)
    {
        cerr << "Failed to create file : " << output_name << endl;
        return -1;
    }

    for (size_t i = 0; i < transformVector.size(); i++)
    {
        global_transform = global_transform * transformVector[i];
        Eigen::Quaternion<double> tmp(global_transform.rotation());
        cout << timestamps[i] << " " << global_transform.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << endl;
        file << timestamps[i] << " " << global_transform.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << endl;
    }
    file.close();

    return 0;
}
