#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
//#include "pcl/common/transform.hpp"

#include <random>

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);
//float noise(float mean, float std_dev);

int main(int argc, char**argv) {
    if(argc < 3) {
        cout << "Usage: " << argv[0] << " <object> <scene> [iterations]" << endl;
        return 0;
    }
    
    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene_pre(new PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);

    loadPCDFile(argv[1], *object);
    loadPCDFile(argv[2], *scene);

    //Rotate object to align with world
    Eigen::Matrix4f _aligned = Eigen::Matrix4f::Identity();
    //_aligned (0, 0) = 0.999613;
    //_aligned (0, 1) = -0.0638307;
    //_aligned (0, 2) = 0.00749443;
    //_aligned (0, 3) = 0.0;
    //_aligned (1, 0) = -0.0543234;
    _aligned (1, 1) = std::cos(M_PI/8); //0.775433;
    _aligned (1, 2) = -std::sin(M_PI/8); //-0.629089;
    //_aligned (1, 3) = 0.0;
    //_aligned (2, 0) = -0.0343439;
    _aligned (2, 1) = std::sin(M_PI/8); //0.628195;
    _aligned (2, 2) = std::cos(M_PI/8); //0.777297;
    _aligned (2, 3) = -1.0;
    //_aligned (2, 3) = 0.0;
    transformPointCloud(*object, *object, _aligned);

    Eigen::Matrix4f _poseInv; //= pose * _aligned.inverse();
    // transformPointCloud(pose, pose, _aligned.inverse());

    Eigen::Matrix4f _center = Eigen::Matrix4f::Identity();
    _center(0, 3) = 0.000;
    _center(1, 3) = 0.007;
    _center(2, 3) = -1.337;

    Eigen::Matrix4f _frontRight = Eigen::Matrix4f::Identity();
    _frontRight(0, 3) = -0.275;
    _frontRight(1, 3) = 0.067;
    _frontRight(2, 3) = -1.31;

    Eigen::Matrix4f _backLeft = Eigen::Matrix4f::Identity();
    _backLeft(0, 3) = 0.298;
    _backLeft(1, 3) = -0.092;
    _backLeft(2, 3) = -1.384;

    // Virker med scan_center, garanteret også hvor der er objekter med. så skal comparison filteret bare ændres tilbage igen.
    // Vi skal i forvejen bruge en transform fra scanner, her skal vi så bruge yderligere en fra denne transform
    
    {
        pcl::ScopeTime t("Cloud filtering");
        // build the condition
        pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, -0.37)));
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, 0.37)));
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, -0.2)));
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, 0.15)));
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, -1.5)));
        range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, 0.0)));

        // build the filter
        pcl::ConditionalRemoval<PointT> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(scene);
        
        //condrem.setKeepOrganized(true);

        // apply filter
        condrem.filter(*scene); // Removes everything from the scene besides the pick area

        //downsample both clouds to have the same leafsize
        pcl::VoxelGrid<PointT> avg;
        avg.setInputCloud(scene);
        avg.setLeafSize(0.01f, 0.01f, 0.01f);
        avg.filter(*scene);

        avg.setInputCloud(object);
        avg.setLeafSize(0.01f, 0.01f, 0.01f);
        avg.filter(*object);

    }
    //std::cout << "Scene er: " << scene->size() << std::endl;
    //std::cout << "object er: " << object->size() << std::endl;
            
    //std::ofstream scanErr("center_error_2.csv");
    //scanErr << "std_dev, error\n";

    for(int i = 0; i <= 25; i++)      // Loop to increase the noise
    {
        float std_dev = 0.001*i;
        cout << endl << "------------------------------------------------" << endl;
        cout << "Standart deviation: " << 0.001*i << endl;
        float mean = 0;
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();    // Gives the best randomness
        std::default_random_engine rand_gen(seed);
        std::normal_distribution<float> noise(mean, std_dev);
        
        for(size_t j = 0; j < scene->size(); j++){
            scene->at(j).x = scene->at(j).x + noise(rand_gen);
            scene->at(j).y = scene->at(j).y + noise(rand_gen);
            scene->at(j).z = scene->at(j).z + noise(rand_gen);
        }


    // Show
    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();     
        // comment to run entire noise
    }
    
    // Compute surface normals
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(20);
        //ne.setRadiusSearch(0.02);   //Radius for normals needs to be smaller than the radius for estimating features, according to pcl!!
        
        ne.setInputCloud(object);
        ne.compute(*object);
        
        ne.setInputCloud(scene);
        ne.compute(*scene);
    }
    
    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");
        
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(0.05);    //from 0.05  //0.04 med k = 10 normals og downsample til 0.01 virker op til 0.002 std_dev
        
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    // Find feature matches
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    // Show matches
    {
        // PCLVisualizer v("Matches");
        // v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        // v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        // v.addCorrespondences<PointT>(object, scene, corr, 1);
        // v.spin();
        // comment to run entire noise
    }
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 5000;
    const float thressq = 0.01 * 0.01;
    
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            //if((i + 1) % 100 == 0)
            //    cout << "\t" << i+1 << endl;
            // comment to run entire noise
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);

            // Apply pose
            transformPointCloud(*object, *object_aligned, T);
            
            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);
            
            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;
            
            // Update result
            if(penaltyi < penalty) {
                //cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
            }
        }
        
        transformPointCloud(*object, *object_aligned, pose);
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing
    
    // Show result
    {
        // PCLVisualizer v("After global alignment");
        // v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        // v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        // v.spin();
        // comment to do entire noise data
    }
    
    // ICP
    {
        cout << "Starting ICP" << endl;
        ScopeTime t("ICP");
        for(size_t i = 0; i < iter; ++i) {
            // 1) Find closest points
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Threshold and create indices for object/scene and compute RMSE
            vector<int> idxobj;
            vector<int> idxscn;
            for(size_t j = 0; j < idx.size(); ++j) {
                if(distsq[j][0] <= thressq) {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }
            
            // 2) Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);
            
            // 3) Apply pose
            transformPointCloud(*object_aligned, *object_aligned, T);
            
            // 4) Update result
            pose = T * pose;
        }
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    }
    
       // Inverse Transformation to 25DScanner
        //_poseInv = _aligned.transpose() * pose;
        _poseInv = pose * _aligned;
        
        
        float dist = sqrt(pow(_center(0, 3) - _poseInv(0, 3), 2) + pow(_center(1, 3) - _poseInv(1, 3), 2) + pow(_center(2, 3) - _poseInv(2, 3), 2));
        cout << "Pose from scanner: " << _poseInv << endl;
        cout << "Current error: " << dist << endl;

        //scanErr << 0.001 * i << "," << dist << "\n";

        {
        //PCLVisualizer v("After local alignment");
        //v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        //v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        //v.spin();
        // comment to do entire noise
    }
    }

    //scanErr.close();

    return 0;
}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}
