#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <functional>

// Point Cloud Library
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


using namespace rws;

using namespace cv;

using namespace std::placeholders;

typedef pcl::PointNormal PointT;
typedef pcl::Histogram<153> FeatureT;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")

public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    
private slots:

    void btnPressed();
    void timer();
    void getImage();
    void get25DImage();
    void stateChangedListener(const rw::kinematics::State& state);

    bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
    void createPathRRTConnect(Q from, Q to,  double extend, double maxTime);
	void printProjectionMatrix(std::string frameName);

private:
    inline float dist_sq(const FeatureT& query, const FeatureT& target);
    void nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq);
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    //std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, State& state);
    void _3DTo3DPointCloud(pcl::PointCloud<PointT>::Ptr cloudIn, pcl::PointCloud<PointT>::Ptr object);
    QTimer* _timer;
    QTimer* _timer25D;
    
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    SerialDevice::Ptr _device;
    Device::Ptr _gripper;
    QPath _path;
    QPath _pathGoal;
    int _step;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
