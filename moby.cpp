#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/Camera>
#include <osg/NodeCallback>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osg/Notify>
#include <Moby/RigidBody.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Quatd.h>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <Moby/GravityForce.h>
#include <Moby/Simulator.h>
#include <Moby/BoxPrimitive.h>
#include <Moby/Visualizable.h>
#include <iostream>

using namespace Moby;
using namespace Ravelin;


int
main( int, char ** )
{


    // RIGID BODY
    boost::shared_ptr< RigidBody > mBody( new RigidBody () );
    Ravelin::SpatialRBInertiad J;

    // parameters required
    Ravelin::SForced force(2,0,0,0,0,0);
    Ravelin::SVelocityd velocity(0.1,0,0,0,0,0);
    Ravelin::SMomentumd momentum(1,1,1,1,1,1);
    Ravelin::Origin3d trans(0,0,0.5);
    Ravelin::Matrix3d I(1,0.1,0.1,0.1,1,0,0.1,0,0.5);
    Ravelin::VectorNd gc(6);
    

    // set inertial parameters
    Quatd quat(0,0,0,1);
    Vector3d origin(0,0,0);
    boost::shared_ptr<Pose3d> pose(new Pose3d());
    pose->x = origin;
    pose->q = quat;
    J.m = 0.5;
    J.h;
    J.J = I;
    J.pose = pose ;
    mBody->set_inertia(J);
    mBody->set_enabled(true);


    boost::shared_ptr< BoxPrimitive > collision( new BoxPrimitive () ); // register collision
    boost::shared_ptr< CollisionGeometry > geom( new CollisionGeometry() );
    geom->set_single_body(mBody);
    geom->set_geometry(collision);
    
    mBody->geometries.push_back(geom); // do not if it is required
    
    // Create the viewer and set its scene data to our scene
    // graph created above.
    osgViewer::Viewer viewer;
    viewer.setSceneData(collision->create_visualization());
    if (!viewer.getSceneData())
        return( 1 );

   mBody->set_inertia(J);
   mBody->set_enabled(true);
   mBody->set_velocity(velocity);
   mBody->set_generalized_coordinates(DynamicBody::eEuler,gc);

    while(true){

	mBody->add_force(force);
	mBody->sum_forces();
	mBody->calc_fwd_dyn();
	//boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        Pose3d P = *mBody->get_pose();
	P.update_relative_pose(GLOBAL);
	std::cout << P << std::endl;

    }
    return( viewer.run() );
}


   

    
