#include <Moby/RigidBody.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Quatd.h>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <Moby/GravityForce.h>
#include <iostream>

using namespace Moby;
using namespace Ravelin;

int main ( int argc, char *argv[] ){

    boost::shared_ptr<RigidBody> mBody(new RigidBody());   // register body
    Ravelin::SpatialRBInertiad J; 

    // parameters required
    Ravelin::SForced force(2,0,0,0,0,0);
    Ravelin::SVelocityd velocity(0.1,0,0,0,0,0);
    Ravelin::SMomentumd momentum(1,1,1,1,1,1);
    Ravelin::Origin3d trans(0,0,0.5);
    Ravelin::Matrix3d I(1,0.1,0.1,0.1,1,0,0.1,0,0.5);
    Ravelin::VectorNd ga(6);
    //ga.resize(6);

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

    // set body properites
    mBody->set_inertia(J);
    mBody->set_enabled(true);
    mBody->set_velocity(velocity);
    while(true){
	
	mBody->add_force(force);
	//mBody->sum_forces();
	mBody->calc_fwd_dyn();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	//mBody->set_accel(ga);
	std::cout <<  "Quat :: "<< mBody->get_gc_pose()->q << " Trans :: " << mBody->get_gc_pose()->x << std::endl;
	//std::cout << mBody->get_transform() << std::endl;
	
    }

    std::cout << mBody->get_mass() << std::endl;
   

    return 0;
}
