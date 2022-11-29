#include "SamplePlugin.hpp"

#include <filesystem>

SamplePlugin::SamplePlugin () : RobWorkStudioPlugin ("SamplePluginUI", QIcon ((std::filesystem::path(__FILE__).parent_path()/"pa_icon.png").c_str()))
{
    setupUi (this);

    _timer = new QTimer (this);
    connect (_timer, SIGNAL (timeout ()), this, SLOT (timer ()));

    // now connect stuff from the ui component
    connect (_btn_im, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_scan, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn0, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn1, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_spinBox, SIGNAL (valueChanged (int)), this, SLOT (btnPressed ()));

    _framegrabber = NULL;

    _cameras    = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin ()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize ()
{
    log ().info () << "INITALIZE"
                   << "\n";

    getRobWorkStudio ()->stateChangedEvent ().add (
        std::bind (&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell

    std::filesystem::path wc_path (__FILE__);
    wc_path          = wc_path.parent_path() / "../../Project_WorkCell/Scene.wc.xml";
	std::cout << "wc path: " << wc_path << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_path.string ());
    getRobWorkStudio ()->setWorkCell (wc);
}

void SamplePlugin::open (WorkCell* workcell)
{
    log ().info () << "OPEN"
                   << "\n";
    _wc    = workcell;
    _state = _wc->getDefaultState ();

    log ().info () << workcell->getFilename () << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame ("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame ("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "BackgroundImage", _bgRender, bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame (_cameras[0]);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap ().has ("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber             = new GLFrameGrabber (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber->init (gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame (_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap ().has ("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap ().get< std::string > ("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D          = new GLFrameGrabber25D (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber25D->init (gldrawer);
            }
        }

        _device = _wc->findDevice<SerialDevice> ("UR-6-85-5-A");
        
        //const SerialDevice::Ptr device = _wc->findDevice<SerialDevice>("UR-6-85-5-A");
        
        _step   = -1;
    }
}

void SamplePlugin::close ()
{
    log ().info () << "CLOSE"
                   << "\n";

    // Stop the timer
    _timer->stop ();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame ("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame ("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("BackgroundImage", bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc           = NULL;
}

Mat SamplePlugin::toOpenCVImage (const Image& img)
{
    Mat res (img.getHeight (), img.getWidth (), CV_8SC3);
    res.data = (uchar*) img.getImageData ();
    return res;
}

//
std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);
}


void SamplePlugin::btnPressed ()
{
    QObject* obj = sender ();
    if (obj == _btn0) {
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        Q from (6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);

/*******************************************************************************************************************/        
        MovableFrame::Ptr bottleFrame = _wc->findFrame< MovableFrame > ("Bottle");
        if (bottleFrame.isNull ()) {
            RW_THROW ("COULD not find movable frame Bottle ... check model");
        }

        std::vector< Q > collisionFreeSolutions;
        // create Collision Detector
        rw::proximity::CollisionDetector detector (_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
        
        
        //til heatmap:
        //Mat heatmap (, img.getWidth (), CV_8SC3);
        
        //Dette for loop til at regne antal solutions
        //tjek 1 fra siden, 1 fra toppen for hver 30 base pos til hver 5 flaske pos.

        for (double rollAngle = 0; rollAngle < 360.0; rollAngle += 1.0) {    // for every degree around the roll axis

            //bottleFrame->moveTo (Transform3D<> (Vector3D<> (bottleFrame->getTransform (_state).P ()), RPY<> (rollAngle * Deg2Rad, 0, 1.571)), _state);
            
            //bottleFrame->moveTo (Transform3D(Vector3D(0.0, 0.474, 0.110), RPY<> (rollAngle * Deg2Rad, 0, 1.571)), _state);  //Virker når vi hardcoder bottle, getTransform tager udgangspunkt i bordet
            
            //Hvor kun z værdien er hardcoded. ville være bedre bare at finde transform fra WORLD
            bottleFrame->moveTo (Transform3D(Vector3D<> (bottleFrame->getTransform (_state).P()[0], bottleFrame->getTransform (_state).P()[1], 0.11), RPY<>(rollAngle * Deg2Rad, 0, 1.571)), _state);
            
            std::vector< Q > solutions = getConfigurations("Bottle", "GraspTCP", _device, _wc, _state);

            for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
                _device->setQ (solutions[i], _state);

                if (!detector.inCollision (_state)) {
                    collisionFreeSolutions.push_back (solutions[i]);    // save it
                    //getRobWorkStudio ()->setState (state);
                    break;                                              // we only need one
                }
            }
        }
        //Transform3D(Vector3D(0, 0.474, 0.21), Rotation3D(-0.000203673, -0.000203673, -1, -1, 4.14828e-08, 0.000203673, -0, 1, -0.000203673))

        std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;

        
        //kun til RRTConnect mellem from og to for at bygge path. vi kan resette path her i stedet for i RRTConnect for at lave den fulde path.
        //bottleFrame->attachTo
        //bruger from Q, to Q, extend, maxTime
        createPathRRTConnect (from, collisionFreeSolutions.at(1), extend, maxTime);
        
    }
    else if (obj == _btn1) {
        log ().info () << "Button 1\n";
        std::cout << endl << "_btn1 pressed!\n";
        // Toggle the timer on and off
        if (!_timer->isActive ()) {
            _timer->start (100);    // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    }
    else if (obj == _spinBox) { //måske slideren?
        log ().info () << "spin value:" << _spinBox->value () << "\n";
    }
    else if (obj == _btn_im) {  //Get image knappen
        getImage ();
    }
    else if (obj == _btn_scan) {    //Get scan knappen
        get25DImage ();
    }
}

void SamplePlugin::_3DTo3DPointCloud(pcl::PointCloud<PointT>::Ptr cloudIn, pcl::PointCloud<PointT>::Ptr object)
{
    pcl::PointCloud<PointT>::Ptr cloud = cloudIn;
    pcl::PointCloud<PointT>::Ptr bottle_cloud = object;

    {                                                                                                                                            //  Viser den første iteration af clouds
        pcl::visualization::PCLVisualizer viewer("Initial visualizer");                                                                          // denne er vidst multithreaded
        viewer.addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 255, 255), "Scan");             // tilføjer med farven hvid
        viewer.addPointCloud<PointT>(bottle_cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(bottle_cloud, 255, 0, 0), "Bottle"); // tilføjer med farven rød
        viewer.spin();                                                                                                                           // for at vise på en anden tråd?
    }

    // Compute surface normals
    {
        pcl::ScopeTime t("Surface normals");
        pcl::NormalEstimation<PointT, PointT> ne;
        ne.setKSearch(10);

        ne.setInputCloud(bottle_cloud);
        ne.compute(*bottle_cloud);

        ne.setInputCloud(cloud);
        ne.compute(*cloud);
    }

    // Compute shape features
    pcl::PointCloud<FeatureT>::Ptr object_features(new pcl::PointCloud<FeatureT>);
    pcl::PointCloud<FeatureT>::Ptr scene_features(new pcl::PointCloud<FeatureT>);
    {
        pcl::ScopeTime t("Shape features");

        pcl::SpinImageEstimation<PointT, PointT, FeatureT> spin;
        spin.setRadiusSearch(0.05);

        spin.setInputCloud(bottle_cloud);
        spin.setInputNormals(bottle_cloud);
        spin.compute(*object_features);

        spin.setInputCloud(cloud);
        spin.setInputNormals(cloud);
        spin.compute(*scene_features);
    }

    // Find feature matches
    pcl::Correspondences corr(object_features->size());
    {
        pcl::ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    // Show matches
    {
        pcl::visualization::PCLVisualizer v("Matches");
        v.addPointCloud<PointT>(bottle_cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(bottle_cloud, 0, 255, 0), "object");
        v.addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 0, 0),"scene");
        v.addCorrespondences<PointT>(bottle_cloud, cloud, corr, 1);
        v.spin();
    }

}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}

void nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq) {
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

void SamplePlugin::get25DImage ()
{
    if (_framegrabber25D != NULL) {
        for (size_t i = 0; i < _cameras25D.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame (_cameras25D[i]);    // "Camera");
            _framegrabber25D->grab (cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage ());

            std::ofstream output (_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth () << "\n";
            output << "HEIGHT " << img->getHeight () << "\n";
            output << "POINTS " << img->getData ().size () << "\n";
            output << "DATA ascii\n";
            for (const auto& p_tmp : img->getData ()) {
                rw::math::Vector3D< float > p = p_tmp;
                output << p (0) << " " << p (1) << " " << p (2) << "\n";
            }
            output.close ();
        }

        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr bottle_cloud (new pcl::PointCloud<PointT>);

        if (pcl::io::loadPCDFile<PointT> ("Scanner25D.pcd", *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file Scanner25D.pcd \n");
        }
        
        if (pcl::io::loadPCDFile<PointT> ("Bottle.pcd", *bottle_cloud) == -1) //* load the bottle point cloud
        {
            PCL_ERROR ("Couldn't read file Bottle.pcd \n");
        }

        _3DTo3DPointCloud(cloud, bottle_cloud);
    }
}

void SamplePlugin::getImage ()
{
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);

            cv::imwrite (_cameras[i] + ".png", imflip_mat);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left");
}

void SamplePlugin::timer ()
{
    if (0 <= _step && (size_t) _step < _path.size ()) {
        _device->setQ (_path.at (_step), _state);
        getRobWorkStudio ()->setState (_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener (const State& state)
{
    _state = state;
}

bool SamplePlugin::checkCollisions (Device::Ptr device, const State& state,
                                    const CollisionDetector& detector, const Q& q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ (q, testState);
    colFrom = detector.inCollision (testState, &data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin (); it != fps.end (); it++) {
            cerr << (*it).first->getName () << " " << (*it).second->getName () << endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect (Q from, Q to, double extend, double maxTime)
{
    _device->setQ (from, _state);
    getRobWorkStudio ()->setState (_state);
    CollisionDetector detector (_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    PlannerConstraint constraint = PlannerConstraint::make (&detector, _device, _state);
    QSampler::Ptr sampler        = QSampler::makeConstrained (QSampler::makeUniform (_device),
                                                       constraint.getQConstraintPtr ());
    QMetric::Ptr metric          = MetricFactory::makeEuclidean< Q > ();
    QToQPlanner::Ptr planner =
        RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear ();     /************************flyt clear til knap funk, så begge paths kan lægges sammen. så kan pos for første goal bruges til rrt til næste og evt til at attache bottle***************************/
    if (!checkCollisions (_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions (_device, _state, detector, to))         
        cout << to << " is in colission!" << endl;
    ;
    Timer t;
    t.resetAndResume ();
    planner->query (from, to, _path, maxTime);
    t.pause ();

    if (t.getTime () >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if (_path.size () == 2) {    // The interpolated path between Q start and Q goal is collision
                                 // free. Set the duration with respect to the desired velocity
        LinearInterpolator< Q > linInt (from, to, duration);
        QPath tempQ;
        for (int i = 0; i < duration + 1; i++) {
            tempQ.push_back (linInt.x (i));
        }

        _path = tempQ;
    }
}

void SamplePlugin::printProjectionMatrix (std::string frameName)
{
    Frame* cameraFrame = _wc->findFrame (frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap ().has ("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix< double, 3, 4 > KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
            Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
            Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e () << std::endl;
        }
    }
}
