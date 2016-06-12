#include<simplePMPModule.h>

bool simplePMPModule::configure(yarp::os::ResourceFinder &rf){

    moduleName = rf.find("name").asString();
    robotName = rf.find("robot").asString();
    partName = rf.find("part").asString();

    rThread = new simplePMPThread(partName);
    rThread->start();
    return true;

}

bool simplePMPModule::close(){

    std::cout << "Stopping SimplePMP Thread!" << std::endl;
    rThread->stop();
    return true;
}

double simplePMPModule::getPeriod(){
    return 1;
}

bool simplePMPModule::updateModule(){
    return true;
}

bool simplePMPModule::interruptModule(){
    std::cout << "Interrupting SimplePMP Module for Port Cleanup..." << std::endl;
    return true;
}



