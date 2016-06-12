#ifndef SIMPLEPMPMODULE_H
#define SIMPLEPMPMODULE_H

#include<iostream>
#include<yarp/os/RFModule.h>
#include<yarp/os/ResourceFinderOptions.h>
#include<yarp/os/ResourceFinder.h>
#include<yarp/os/Network.h>
#include<yarp/os/Thread.h>

#include<simplePMPThread.h>

using namespace std;
using namespace yarp::os;

class simplePMPModule : public RFModule{

    simplePMPThread *rThread;//Pointer to simplePMP Thread which is Configured in configure() and closed in close()

    std::string moduleName;
    std::string robotName;
    std::string partName;

    bool verboseTerm, verboseFile;//TODO Learn to use them


public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool updateModule();
    double getPeriod();
    bool close();

};





#endif // SIMPLEPMPMODULE_H

