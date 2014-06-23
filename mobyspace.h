#ifndef OPENRAVE_MOBY_SPACE
#define OPENRAVE_MOBY_SPACE

#include "plugindefs.h"
#include <Moby/GravityForce.h>



class MobySpace : public boost::enable_shared_from_this<MobySpace>
{
	inline boost::weak_ptr<MobySpace> weak_space() {
        	return shared_from_this();
    	}
public:

    // information about the kinematics of the body
    class KinBodyInfo : public UserData
    {
public:
        

        KinBodyInfo()   {
           
        }
        virtual ~KinBodyInfo() {
            Reset();
        }

        void Reset()
        {
           
        }

       

private:
       
    };

typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;

public :
      /// default constructor
      MobySpace(EnvironmentBasePtr penv):_penv(penv){};
      virtual ~MobySpace(){};
      KinBodyInfoPtr InitKinBody( KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), bool blockode=true ){

          EnvironmentMutex::scoped_lock lock(pbody->GetEnv()->GetMutex());


       }
       std::pair<KinBodyInfoPtr, bool> GetCreateInfo(KinBodyConstPtr pbody, bool blockode=true) {
       KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
        bool bcreated = false;
        if( !pinfo ) {
            //pinfo = InitKinBody(pbody, KinBodyInfoPtr(), blockode);
            //pbody->SetUserData(_userdatakey, pinfo);
            bcreated = true;
        }
        return std::make_pair(pinfo,bcreated);
    }
   
      void Synchronize(){
	boost::mutex::scoped_lock lockode(_mutex);
	vector<KinBodyPtr> vbodies;
	_penv->GetBodies(vbodies);
	FOREACHC(itbody, vbodies) {
		//KinBodyInfoPtr pinfo = GetCreateInfo(*itbody, false).first;
		//BOOST_ASSERT( pinfo->GetBody() == *itbody );
		//_Synchronize(pinfo,false);
	}


      };


private :

     mutable boost::mutex _mutex;
     EnvironmentBasePtr _penv;
     std::string _userdatakey;
	


};

#endif
