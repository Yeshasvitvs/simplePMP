#include <simplePMPModule.h>
#include<yarp/os/ResourceFinderOptions.h>
#include<yarp/os/ResourceFinder.h>


using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[])
{
    std::cout << "simplePMP Module adapted from PMPModule of DARWIN Project!" << std::endl;

    yarp::os::Network yarp;
    yarp::os::ResourceFinder rf;//TODO Learn to use RF for Module Configuration
    rf.setVerbose();
    rf.setDefaultConfigFile("/home/yeshi/qt/simplePMP/simplePMPConfig.ini");
    //rf.setDefaultContext("simplePMP");
    rf.configure(argc,argv);

    //Getting the EndEffector Position in Cartesian Space using iCub Cartesian Controller
    /*yarp::os::Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote","/icub/cartesianController/left_arm");
    option.put("local","/PMP/left_arm");

    yarp::dev::PolyDriver ClientCartCtrl(option);
    yarp::dev::ICartesianControl *icart=NULL;

    if(ClientCartCtrl.isValid()){
        ClientCartCtrl.view(icart);
    }else{
        std::cout << "Device not available. Here are the known devices: " << std::endl;
        std::cout << yarp::dev::Drivers::factory().toString().c_str() << std::endl;
        return 1;
    }*/


#if DEBUG_CODE>0
    ConstString rName=rf.find("robot").asString();
    std:cout<< "Robot Name: " << rName << std::endl;
    ConstString rpName=rf.find("robotPartName").asString();
    std::cout << "Robot Part Name: " << rpName << std::endl;
#endif

    simplePMPModule pmpiCubClient;
    pmpiCubClient.configure(rf);
    //while(true);
    return 0;
}

