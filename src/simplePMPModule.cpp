
#include "simplePMPModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/*
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 *  equivalent of the "open" method.
 */

bool simplePMPModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name",
                           Value("/PMP"),
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str()); //This creates a port with given name

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName               = rf.check("robot",
                           Value("icubGazeboSim"),
                           "Robot name (string)").asString();


    //printf("robotName in Module %s \n", robotName.c_str());
    //robotPortName         = "/" + robotName + "/head";


    inputPortName           = rf.check("inputPortName",
                            Value("/target:i"),
                            "Input port name (string)").asString();
    setName(inputPortName.c_str());

    robotPartName           = rf.check("robotPortName", Value("right_arm"),"Robot Part Name").asString();

    jointPortName           = "/" + robotName + "/" + robotPartName + "/" + "state:o"; //This is to read the joint angles of the arm/robot part

#ifdef DEBUG_CODE
    std::cout << "Joint Port Name: " << jointPortName << std::endl;
#endif

    angleRead = rf.check("angleRead",Value(""),"Angle Read Port").asString();
    angleRead = "/" + angleRead;
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);                  // attach to port
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }





    /* create the thread and pass pointers to the module parameters */
    rThread = new simplePMPThread(robotName);
    rThread->init();
    //rThread->setName(getName().c_str());
    //rThread->setRobotName(robotName);
    //rThread->setVerbose(verboseFile,verboseTerm);


    //=======================================================================

    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool simplePMPModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool simplePMPModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    printf("PMPModule::close:end \n");
    return true;
}

bool simplePMPModule::respond(const Bottle& command, Bottle& reply)
{
    string helpMessage =  string(getName().c_str()) +
                " commands are: \n" +
                "help \n" +
                "quit \n";
    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool simplePMPModule::updateModule()
{
    return true;
}

double simplePMPModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

