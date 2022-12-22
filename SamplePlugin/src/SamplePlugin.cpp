#include "SamplePlugin.hpp"

#include <filesystem>

//typedef pcl::PointNormal PointT;
//typedef pcl::Histogram<153> FeatureT;

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
    //wc_path          = wc_path.parent_path() / "../../WorkCellObstacle/Scene.wc.xml";
    //switch comment to load other workcell
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
       
        
        /***************linear interpol****************************/
        std::vector<Q> points = createPointsList();
        std::vector<LinearInterpolator<Q>> interps = linInterpol(points);
        TimedStatePath timePath;

        std::ofstream linfile("linear.csv");
        linfile << "t; x; dx; ddx\n";
        for(size_t i = 0; i < interps.size(); i++){
            for(double k = 0; k < interps.at(i).duration() ; k += 0.05){
                linfile << k << ";" << interps.at(i).x(k)[0] << " " << interps.at(i).x(k)[1] << " " << interps.at(i).x(k)[2] << " " << interps.at(i).x(k)[3] << " " << interps.at(i).x(k)[4] << " " << interps.at(i).x(k)[5];
                linfile <<  ";" << interps.at(i).dx(k)[0] << " " << interps.at(i).dx(k)[1] << " " << interps.at(i).dx(k)[2] << " " << interps.at(i).dx(k)[3] << " " << interps.at(i).dx(k)[4] << " " << interps.at(i).dx(k)[5];
                linfile << ";" << interps.at(i).ddx(k)[0] << " " << interps.at(i).ddx(k)[1] << " " << interps.at(i).ddx(k)[2] << " " << interps.at(i).ddx(k)[3] << " " << interps.at(i).ddx(k)[4] << " " << interps.at(i).ddx(k)[5] << "\n";
            }
        }
        linfile.close();
        
        

        // std::ofstream RRTTime("rrt_time.csv");
        // RRTTime << "iteration, time";
        // for(size_t j = 0; j < interps.size(); j++){
        //     for(double k = 0; k <= interps.at(j).duration(); k += 0.05){
        //         _device->setQ(interps.at(j).x(k), _state);
        //         timePath.push_back(TimedState (k, _state));
        //     }
        // }

        /*******************Parabolic blend********************************/

        std::ofstream parabolfile("parabolic.csv");
        parabolfile << "x, dx, ddx\n";

        double t_blend = 0.5;

        for(double k = 0; k < interps.at(0).duration() - t_blend; k += 0.05){
            _device->setQ(interps.at(0).x(k), _state);
            timePath.push_back(TimedState (k, _state));
            parabolfile << interps.at(0).x(k) << "," << interps.at(0).dx(k) << "," << interps.at(0).ddx(k) << "\n";
        }
        for(size_t i = 1; i < interps.size(); i++){
            ParabolicBlend<Q> parab(&interps.at(i-1), &interps.at(i), t_blend);

            for(double k = 0; k < 2 * t_blend; k += 0.05){
                _device->setQ(parab.x(k), _state);
                timePath.push_back(TimedState (k, _state));
                parabolfile << parab.x(k) << "," << parab.dx(k) << "," << parab.ddx(k) << "\n";
            }
            for(double k = t_blend; k < interps.at(i).duration() - t_blend; k += 0.05){
                _device->setQ(interps.at(i).x(k), _state);
                timePath.push_back(TimedState (k, _state));
                parabolfile << interps.at(i).x(k) << "," << interps.at(i).dx(k) << "," << interps.at(i).ddx(k) << "\n";
            }
        }
        for(double k = interps.at(interps.size()-1).duration() - t_blend; k < interps.at(interps.size()-1).duration(); k += 0.05){
            _device->setQ(interps.at(interps.size()-1).x(k), _state);
            parabolfile << interps.at(interps.size()-1).x(k) << "," << interps.at(interps.size()-1).dx(k) << "," << interps.at(interps.size()-1).ddx(k) << "\n";
            timePath.push_back(TimedState (k, _state));
        }

        parabolfile.close();
        PathLoader::storeTimedStatePath(*_wc, timePath, "./linearvis.rwplay");

        
        /********************************************/
        /**************************************************RRT connect testing*****************************************************************/
        //runRRT();
    }
    else if (obj == _btn1) {
        log ().info () << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive ()) {
            _timer->start (100);    // run 10 Hz
            _step = 0;
        }
        else{
            _step = 0;
        }
        
        std::ofstream RRTpath("rrt_path.csv");
        RRTpath << "x, y, z\n";
        //for(size_t l = 0; l < _path.size(); l++){
        //    _device->setQ (_path.at (l), _state);
        //    RRTpath << _device->getEnd()->getTransform(_state).P()[0] << "," << _device->getEnd()->getTransform(_state).P()[1] << "," << _device->getEnd()->getTransform(_state).P()[2] << "\n";
       // }
        RRTpath.close();
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
        //std::ofstream RRTout;
        //RRTout.open("rrt_path.csv", ios_base::app);
        //RRTout << _device->baseTend(_state).P()[0] << "," << _device->baseTend(_state).P()[1] << "," << _device->baseTend(_state).P()[2] << "\n";
        //RRTout.close();
        if(_step == pathPosAtBott){
            
            MovableFrame::Ptr bottleFrame = _wc->findFrame< MovableFrame > ("Bottle");
            bottleFrame->attachTo(_wc->findFrame< FixedFrame > ("GraspTCP"), _state);
            bottleFrame->moveTo(Transform3D(Vector3D<>(0.000, 0.473, 0.11), RPY<>(0, 0, 1.571)), _state);
        }
       
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
            _path.push_back (linInt.x (i));
        }

        //_path = tempQ;
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

TimedStatePath SamplePlugin::createPathP2PParabol(std::vector<LinearInterpolator<Q>> interpols)     //ændre til at spise vec af interps. find collisions
{
    TimedStatePath timeStatePath;
    for(size_t i = 0; i < interpols.size()-1; i++){
        ParabolicBlend<Q> parabol(interpols.at(i), interpols.at(i+1), 0.5);
        for(double k = 0; k < parabol.duration(); k += 0.05){
            _device->setQ(parabol.x(k), _state);
            timeStatePath.push_back(TimedState (k, _state));
        }
    }
    return timeStatePath;

}

std::vector<LinearInterpolator<Q>> SamplePlugin::linInterpol(std::vector<Q> points){
    std::vector<LinearInterpolator<Q>> interps;
    
    for (size_t i = 0; i < points.size() - 1; i++)
    {
        double minDur = 0.0001;
        double maxDur = 10;

        while (!withinLimit(_device, LinearInterpolator<Q>(points[i], points[i + 1], maxDur)))
        {
            maxDur += 10;
        }

        double duration = (maxDur + minDur) / 2;
        for (size_t j = 0; j < 100; j++)
        {
            if (withinLimit(_device, LinearInterpolator<Q>(points[i], points[i + 1], duration)))
            {
                maxDur = duration;
            }
            else
            {
                minDur = duration;
            }
            duration = (maxDur + minDur) / 2;
        }
        LinearInterpolator<Q> interp(points[i], points[i + 1], maxDur);
        interps.push_back(interp);
    }
    return interps;
}

bool SamplePlugin::withinLimit (Device::Ptr device, LinearInterpolator< Q > interp)
{
    Q velLimit   = device->getVelocityLimits ();
    Q accelLimit = device->getAccelerationLimits ();

    for (double t = 0; t < interp.duration (); t += 0.05) {
        for (size_t i = 0; i < device->getDOF (); i++) {
            if (abs (interp.dx (t)[i]) > velLimit[i]) {
                return false;
            }
            if (abs (interp.ddx (t)[i]) > accelLimit[i]) {
                return false;
            }
        }
    }
    return true;
}


void SamplePlugin::runRRT(){
    double extend = 0.15;
    double maxTime = 60;
    Q _from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
    MovableFrame::Ptr goalFrame = _wc->findFrame<MovableFrame>("25DResult");
    if (goalFrame.isNull())
    {
        RW_THROW("COULD not find movable frame 25DResult ... check model");
    }
    // std::ofstream RRTTime("rrt_time.csv");
    // RRTTime << "iteration, time";
    // for (int k = 0; k <= 50; k++)
    // {
    //const clock_t begin_time = clock();
    _device->setQ(_from, _state);
    MovableFrame::Ptr bottleFrame = _wc->findFrame<MovableFrame>("Bottle");
    if (bottleFrame.isNull())
    {
        RW_THROW("COULD not find movable frame Bottle ... check model");
    }

    bottleFrame->attachTo(_wc->findFrame<FixedFrame>("Table"), _state);
    bottleFrame->moveTo(Transform3D(Vector3D<>(0.000, 0.473, 0.11), RPY<>(0, 0, 1.571)), _state);

    std::vector<Q> collisionFreeSolutions;
    // create Collision Detector
    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // for (double rollAngle = 0; rollAngle < 360.0; rollAngle += 1.0) {    // for every degree around the roll axis

    goalFrame->moveTo(Transform3D(Vector3D<>(bottleFrame->getTransform(_state).P()[0], bottleFrame->getTransform(_state).P()[1], 0.11), RPY<>(M_PI, 0, 1.571)), _state);

    std::vector<Q> solutions;

    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);

    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            collisionFreeSolutions.push_back(solutions[i]); // save it
            break;                                          // we only need one
        }
    }
    //}

    std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size() << " collision-free inverse kinematics solutions!"
              << std::endl;

    Q goal = collisionFreeSolutions[0];

    _path.clear();

    createPathRRTConnect(_from, goal, extend, maxTime);
    cout << "Bottle found, trying goal!\n";

    pathPosAtBott = _path.size();
    bottleFrame->attachTo(_wc->findFrame<FixedFrame>("GraspTCP"), _state);
    bottleFrame->moveTo(Transform3D(Vector3D<>(0.000, 0.473, 0.11), RPY<>(0, 0, 1.571)), _state);
    getRobWorkStudio()->setState(_state);
    goalFrame->moveTo(Transform3D(Vector3D<>(0.29, -0.5, 0.13), RPY<>(0, 0, 1.571)), _state);

    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);

    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            collisionFreeSolutions.push_back(solutions[i]); // save it
            break;                                          // we only need one
        }
    }
    Q goal2 = collisionFreeSolutions[1];

    createPathRRTConnect(goal, goal2, extend, maxTime);

    bottleFrame->attachTo(_wc->findFrame<FixedFrame>("Table"), _state);
    bottleFrame->moveTo(Transform3D(Vector3D<>(0.000, 0.473, 0.11), RPY<>(0, 0, 1.571)), _state);

    _device->setQ(_from, _state);
    getRobWorkStudio()->setState(_state);
    //float time_spent = float(clock() - begin_time) / CLOCKS_PER_SEC;

    // cout << "RRTConnect number: " << k << " took: " << time_spent << " seconds!\n";
    //            RRTTime << k << "," << time_spent << "\n";
    //        }
    //        RRTTime.close();
}

std::vector<Q> SamplePlugin::createPointsList()
{
    MovableFrame::Ptr bottleFrame = _wc->findFrame<MovableFrame>("Bottle");
    if (bottleFrame.isNull())
    {
        RW_THROW("COULD not find movable frame Bottle ... check model");
    }

    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    MovableFrame::Ptr goalFrame = _wc->findFrame<MovableFrame>("25DResult");
    if (goalFrame.isNull())
    {
        RW_THROW("COULD not find movable frame 25DResult ... check model");
    }
    std::vector<Q> points;
    std::vector<Q> solutions;
    Q home(-1.571, -1.572, -1.572, -1.572, 1.571, 0);
    points.push_back(home);

    goalFrame->moveTo(Transform3D(Vector3D<>(0.0, 0.473, 0.30), RPY<>(0, 0, 3.1)), _state);
    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            points.push_back(solutions[i]); // save it
            break;                          // we only need one
        }
    }

    goalFrame->moveTo(Transform3D(Vector3D<>(0.0, 0.473, 0.21), RPY<>(0, 0, 3.1)), _state);
    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            points.push_back(solutions[i]); // save it
            break;                          // we only need one
        }
    }
    
    goalFrame->moveTo(Transform3D(Vector3D<>(0.0, 0.473, 0.30), RPY<>(0, 0, 3.1)), _state);
    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            points.push_back(solutions[i]); // save it
            break;                          // we only need one
        }
    }

    goalFrame->moveTo(Transform3D(Vector3D<>(0.29, -0.5, 0.30), RPY<>(0, 0, 3.1)), _state);
    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            points.push_back(solutions[i]); // save it
            break;                          // we only need one
        }
    }

    goalFrame->moveTo(Transform3D(Vector3D<>(0.29, -0.5, 0.21), RPY<>(0, 0, 3.1)), _state);
    solutions = getConfigurations("25DResult", "GraspTCP", _device, _wc, _state);
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        _device->setQ(solutions[i], _state);
        if (!detector.inCollision(_state))
        {
            points.push_back(solutions[i]); // save it
            break;                          // we only need one
        }
    }

    points.push_back(home);
    return points;
}