#ifndef SIMPLEPMPMODULE_H
#define SIMPLEPMPMODULE_H

#include<simplePMPThread.h>

class simplePMPModule:public yarp::os::RFModule {


    std::string moduleName;                  // name of the module - PMP
    std::string robotName;                   // name of the robot - icubGazeboSim
    std::string robotPortName;               // name of robot port - ??
    std::string robotPartName;               //Which part of the robot to be read for joints

    std::string jointPortName;               // Port to deal with which joint angles are to be read
    std::string angleRead;                   //Port Connected to Joint Port Name

    std::string inputPortName;               // name of the input port for events - Port that recieves the target positions in extrinsic space
    std::string outputPortName;              // name of output port - Port that sends commands to the Arms
    std::string handlerPortName;             // name of handler port - ??
    std::string configFile;                  // name of the configFile that the resource Finder will seek - ??
    yarp::os::Port handlerPort;              // a port to handle messages - ??

    simplePMPThread *rThread;             // pointer to a new thread to be created and started in configure() and stopped in close()

public:

public:
    /**
    *  configure all the tutorial parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf);

    /**
    *  interrupt, e.g., the ports
    */
    bool interruptModule();

    /**
    *  close and shut down the tutorial
    */
    bool close();
    /**
    *  Function to share robotName to the thread
    */
    void setRobotName(char );
    /**
    *  to respond through rpc port
    * @param command reference to bottle given to rpc port of module, alongwith parameters
    * @param reply reference to bottle returned by the rpc port in response to command
    * @return bool flag for the success of response else termination of module
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
    *  unimplemented
    */
    double getPeriod();

    /**
    *  unimplemented
    */
    bool updateModule();

};





#endif // SIMPLEPMPMODULE_H

