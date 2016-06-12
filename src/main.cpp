#include <simplePMPModule.h>
#include <yarp/dev/all.h>


using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[])
{
    std::cout << "simplePMP Module adapted from PMPModule of DARWIN Project!" << std::endl;

    yarp::os::Network yarp;
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("/home/yeshi/qt/simplePMP/simplePMPConfig.ini");
    //rf.setDefaultContext("simplePMP");
    rf.configure(argc,argv); //This configures the RF using data from command line


    //This Created a Module of pmpiCubClient
    simplePMPModule pmpiCubClient;
    pmpiCubClient.configure(rf);//This Runs the Module and Thread Once

    //This Runs the Module and Thread Indefinetely
    /*if (!pmpiCubClient.runModule(rf))
    {
        cerr<<"Error module did not start"<<endl;
        return 1;
    }*/


    return 0;
}

//Currently Not Using Cartesian Controller
int getCartController(){

    //Getting the EndEffector Position in Cartesian Space using iCub Cartesian Controller
    yarp::os::Property option;
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
    }


}
