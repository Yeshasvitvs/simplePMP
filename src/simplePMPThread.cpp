#include<simplePMPThread.h>
#include<utils.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

//Instance for getting the clock configured with the ports
typedef struct InternalData InternalData;

//Creating instances of synchrinizer data structure
InternalData* syncObject = new InternalData();

simplePMPThread::simplePMPThread(){ //Default Constructor
}
simplePMPThread::simplePMPThread(std::string partName){
    armName = partName;
    std::cout << "Running PMPModule on iCub Robot " << armName.c_str() << " arm " << std::endl;
}

simplePMPThread::~simplePMPThread(){

}

bool simplePMPThread::start(){
    threadInit();
    return true;
}

void simplePMPThread::gzSync(){

    period = 0.001; //This is 10ms Default

    syncObject->configuration.clientPortName = "/sync/clock:o"; //This should be the name of the clock port opened by the block
    syncObject->configuration.serverPortName = "/clock/rpc"; //This is the name of the clock port opened by gazebo yarp clock plugin


    syncObject->clientPort.open(syncObject->configuration.clientPortName);
    if(!yarp::os::Network::connect(syncObject->configuration.clientPortName, syncObject->configuration.serverPortName)){ //Why are this ports connected?
         std::cout << "Problem connecting /clock/rpc and " << (syncObject->configuration.clientPortName).c_str() << " Ports!!!" << std::endl;
    }else{
         std::cout << "Port " << (syncObject->configuration.clientPortName).c_str() << " opened..." << std::endl;
    }

    //The client port is attached to this object to stream its YARP state (the state of ClientPort)
    syncObject->clockServer.yarp().attachAsClient(syncObject->clientPort);

    stepSize = syncObject->clockServer.getStepSize(); //This is 1 ms Default
    std::cout << "Gazebo Simulation Step Size : " << stepSize << std::endl;
    syncObject->configuration.numberOfSteps = period/stepSize;

}

void simplePMPThread::setJAnglesMean(){

    //TODO Use iKin Library to Set the Mean Values

    if(armName == "left" || armName == "lr"){

        //Mean Joint Angles Torso + Left Arm
        jAnglesMeanLeft[0] = 0.0; //Torso
        jAnglesMeanLeft[1] = 0.0;
        jAnglesMeanLeft[2] = 0.0;

        jAnglesMeanLeft[3] = -0.7854;//Left Arm
        jAnglesMeanLeft[4] = 0.0;
        jAnglesMeanLeft[5] = 0.7098;
        jAnglesMeanLeft[6] = 0.9730;
        jAnglesMeanLeft[7] = 1.6;
        jAnglesMeanLeft[8] = 0.0;
        jAnglesMeanLeft[9] = 0.0;


#if DEBUG_CODE>0
    std::cout << "Mean Joint Angles for Torso + Left Arm Configuration : " << " " << jAnglesMeanLeft[0] << " " << jAnglesMeanLeft[1] //
              << " " << jAnglesMeanLeft[2] << " " << jAnglesMeanLeft[3] << " " << jAnglesMeanLeft[4] << " " << jAnglesMeanLeft[5] //
              << " " << jAnglesMeanLeft[6] << " " << jAnglesMeanLeft[7] << " " << jAnglesMeanLeft[8] << " " << jAnglesMeanLeft[9] << std::endl;
#endif

    }else if(armName == "right" || armName == "lr"){

        //Mean Joint Angles Torso + Right Arm
        jAnglesMeanRight[0] = 0.0; //Torso
        jAnglesMeanRight[1] = 0.0;
        jAnglesMeanRight[2] = 0.0;

        jAnglesMeanRight[3] = -0.7854;//Right Arm
        jAnglesMeanRight[4] = 0.0;
        jAnglesMeanRight[5] = 0.7098;
        jAnglesMeanRight[6] = 0.9730;
        jAnglesMeanRight[7] = 1.6;
        jAnglesMeanRight[8] = 0.0;
        jAnglesMeanRight[9] = 0.0;

#if DEBUG_CODE>0
    std::cout << "Mean Joint Angles for Torso + Right Arm Configuration : " << " " << jAnglesMeanRight[0] << " " << jAnglesMeanRight[1] //
              << " " << jAnglesMeanRight[2] << " " << jAnglesMeanRight[3] << " " << jAnglesMeanRight[4] << " " << jAnglesMeanRight[5] //
              << " " << jAnglesMeanRight[6] << " " << jAnglesMeanRight[7] << " " << jAnglesMeanRight[8] << " " << jAnglesMeanRight[9] << std::endl;
#endif

    }
}

void simplePMPThread::setJAdmit(){

    if(armName == "left" || armName == "lr"){

        //TODO: Change the Admittance Values for each joint later
        admitTorso.assign(3,0);
        admitLeft.assign(7,0);//TODO: Have to change it when using more than 10 joints

        //Torso Admittance Values
        admitTorso.at(0) = 0.6;//KOMP_WAISZT;
        admitTorso.at(1) = 0;
        admitTorso.at(2) = 0.5;//KOMP_WAISZT2;

        //Left Arm Admittance Values
        admitLeft.at(0) = 2.8;//0.09
        admitLeft.at(1) = 0.05;
        admitLeft.at(2) = 2;
        admitLeft.at(3) = 2.5;
        admitLeft.at(4) = 1;
        admitLeft.at(5) = 1;
        admitLeft.at(6) = 1;

    }else if (armName == "right" || armName == "lr"){

        //TODO: Change the Admittance Values for each joint later
        admitTorso.assign(3,0);
        admitRight.assign(7,0);

        //Torso Admittance Values
        admitTorso.at(0) = 0.6;//KOMP_WAISZT;
        admitTorso.at(1) = 0;
        admitTorso.at(2) = 0.5;//KOMP_WAISZT2;

        //Right Arm Admittance Values
        admitRight.at(0) = 2.8;//0.09
        admitRight.at(1) = 0.05;
        admitRight.at(2) = 2;
        admitRight.at(3) = 2.5;
        admitRight.at(4) = 1;
        admitRight.at(5) = 1;
        admitRight.at(6) = 1;

    }


}

bool simplePMPThread::threadInit(){
    init();
    return true;
}
void simplePMPThread::threadRelease(){
    std::cout << "Releasing SimplePMP Thread..." << std::endl;
}
void simplePMPThread::onStop(){

    inputPort.interrupt();
    inputPort.close();

    outputPort.interrupt();
    outputPort.close();

    cmdGraspPort.interrupt();
    cmdGraspPort.close();

    if(armName == "left" || armName == "lr"){

        leftArmAnglesPort.interrupt();
        leftArmAnglesPort.close();

        leftEEPort.interrupt();
        leftEEPort.close();

        cmdTorsoPort.interrupt();
        cmdTorsoPort.close();

        cmdLeftArmPort.interrupt();
        cmdLeftArmPort.close();

    }else if(armName == "right" || armName == "lr"){

        rightArmAnglesPort.interrupt();
        rightArmAnglesPort.close();

        rightEEPort.interrupt();
        rightEEPort.close();

        cmdTorsoPort.interrupt();
        cmdTorsoPort.close();

        cmdRightArmPort.interrupt();
        cmdRightArmPort.close();
    }
}

void simplePMPThread::init(){

#if DEBUG_CODE>0
    std::cout << "Initializing PMP Thread..." << std::endl;
#endif

    //Gazebo Synchronization
    //gzSync();

    //armName = "right"; //TODO: Get this form the config file to decide which arm to move
    armLeft = new armUtils("left");
    armRight = new armUtils("right");

    RAMP_KONSTANT = 0.005;
    t_dur = 5;

    grasped=false;//This for now can be user input

    posErr = 0.00001;//Position Error Threshold

    if(armName == "left" || armName == "lr"){

        //X Y Z
        memset(Gam_ArrxLeft,0,ITERATION*sizeof(double));
        memset(Gam_ArryLeft,0,ITERATION*sizeof(double));
        memset(Gam_ArrzLeft,0,ITERATION*sizeof(double));

        //Joint Angle Variables Torso + Left Arm
        memset(q1L,0,ITERATION*sizeof(double));
        memset(q2L,0,ITERATION*sizeof(double));
        memset(q3L,0,ITERATION*sizeof(double));
        memset(q4L,0,ITERATION*sizeof(double));
        memset(q5L,0,ITERATION*sizeof(double));
        memset(q6L,0,ITERATION*sizeof(double));
        memset(q7L,0,ITERATION*sizeof(double));
        memset(q8L,0,ITERATION*sizeof(double));
        memset(q9L,0,ITERATION*sizeof(double));
        memset(q10L,0,ITERATION*sizeof(double));

    }else if( armName == "right" || armName == "lr" ){


        //X Y Z
        memset(Gam_ArrxRight,0,ITERATION*sizeof(double));
        memset(Gam_ArryRight,0,ITERATION*sizeof(double));
        memset(Gam_ArrzRight,0,ITERATION*sizeof(double));


        //Joint Angle Variables Torso + Right Arm
        memset(q1R,0,ITERATION*sizeof(double));
        memset(q2R,0,ITERATION*sizeof(double));
        memset(q3R,0,ITERATION*sizeof(double));
        memset(q4R,0,ITERATION*sizeof(double));
        memset(q5R,0,ITERATION*sizeof(double));
        memset(q6R,0,ITERATION*sizeof(double));
        memset(q7R,0,ITERATION*sizeof(double));
        memset(q8R,0,ITERATION*sizeof(double));
        memset(q9R,0,ITERATION*sizeof(double));
        memset(q10R,0,ITERATION*sizeof(double));

    }

    //Setting Joint Angles Mean Values
    setJAnglesMean();

    //Compliances Values from PMP Analytic Module
    KFORCE = 0.01;
    KOMP_JANG = 0.02; //0.02
    KOMP_WAISZT = 0.0001;
    KOMP_WAISZT2 = 0.0001;//0.0001

    //TODO Eperiment with the Stiffness Values - Match with findings from human experiments
    kX=0.01; kY=0.01; kZ=0.01;

    if(armName == "left" || armName == "lr"){

        //TODO Not sure what name to be for these
        J0HL = 0.041; J1HL = 0.52; J2HL = 50; J3HL = 0.2; J4HL = 0.041;
        J5HL = 0.041; J6HL = 0.041; J7HL = 0.041; J8HL = 0.041; J9HL = 0.041;

        //TODO Not sure what name to be for these
        JHdL[0] = J0HL; JHdL[1] = 0.000041; JHdL[2] = J2HL; JHdL[3] = 0.041; JHdL[4] = 0.041;
        JHdL[5] = 5; JHdL[6] = 0.041; JHdL[7] = 75; JHdL[8] = J8HL; JHdL[9] = J9HL;

    }else if(armName == "right" || armName == "lr"){

        //TODO Not sure what name to be for these
        J0HR = 0.041; J1HR = 0.52; J2HR = 50; J3HR = 0.2; J4HR = 0.041;
        J5HR = 0.041; J6HR = 0.041; J7HR = 0.041; J8HR = 0.041; J9HR = 0.041;

        //TODO Not sure what name to be for these
        JHdR[0] = J0HR; JHdR[1] = 0.000041; JHdR[2] = J2HR; JHdR[3] = 0.041; JHdR[4] = 0.041;
        JHdR[5] = 5; JHdR[6] = 0.041; JHdR[7] = 75; JHdR[8] = J8HR; JHdR[9] = J9HR;
    }

    //Setting Joint Admittance Values
    setJAdmit();

#if DEBUG_CODE>0
    //Output Stream Files
    virTarget.open("target.txt");
    curPosition.open("position.txt");
    jointsLoc.open("joints.txt");
#endif

    if(!inputPort.open("/targetPos")){
#if DEBUG_CODE>0
        std::cout << "Input Target Position Port Created : " << inputPort.getName().c_str() << std::endl;
#endif
    }

    if(!outputPort.open("/commandedPos")){
#if DEBUG_CODE>0
        std::cout << "Output Commanded Position Port Created : " << outputPort.getName().c_str() << std::endl;
#endif
    }

    if(armName == "left" || armName == "lr"){

        //Ports to Receive Joint Angles
        if(!torsoAnglesPort.open("/torsoAnglePort:i")){//Receives Torso Joint Angle Values
    #if DEBUG_CODE>0
            std::cout << "Torso Angle Port Created : " << torsoAnglesPort.getName().c_str() << std::endl;
    #endif
        }

        if(!leftArmAnglesPort.open("/leftArmAnglePort:i")){//Receives Left Arm Joint Angle Values
    #if DEBUG_CODE>0
            std::cout << "Left Arm Angle Port Created : " << leftArmAnglesPort.getName().c_str() << std::endl;
    #endif
        }


#if DEBUG_CODE>0
    if(!leftEEPort.open("/leftEEPort:i")){//Receives Left Arm End Effector Value Using Cartesian Controller
#if DEBUG_CODE>0
        std::cout << "Left End Effector Port Created : " << leftEEPort.getName().c_str() << std::endl;
#endif
    }
#endif

        //Ports to Command Joints
        if(!cmdTorsoPort.open("/cmdTorso:o")){//Sends Joint Motor Commands to Torso
    #if DEBUG_CODE>0
            std::cout << "Torso Output Commanded Position Port Created : " << cmdTorsoPort.getName().c_str() << std::endl;
    #endif
        }

        if(!cmdLeftArmPort.open("/cmdLeftArm:o")){//Sends Joint Motor Commands to Left Arm
    #if DEBUG_CODE>0
            std::cout << "Left Arm Output Commanded Position Port Created : " << cmdLeftArmPort.getName().c_str() << std::endl;
    #endif
        }

    }else if(armName == "right" || armName == "lr"){

        //Ports to Receive Joint Angles
        if(!torsoAnglesPort.open("/torsoAnglePort:i")){//Receives Torso Joint Angle Values
    #if DEBUG_CODE>0
            std::cout << "Torso Angle Port Created : " << torsoAnglesPort.getName().c_str() << std::endl;
    #endif
        }

        if(!rightArmAnglesPort.open("/rightArmAnglePort:i")){//Receives Right Arm Joint Angle Values
    #if DEBUG_CODE>0
            std::cout << "Right Arm Angle Port Created : " << rightArmAnglesPort.getName().c_str() << std::endl;
    #endif
        }

        //Ports to Command Joints
        if(!cmdTorsoPort.open("/cmdTorso:o")){//Sends Joint Motor Commands to Torso
    #if DEBUG_CODE>0
            std::cout << "Torso Output Commanded Position Port Created : " << cmdTorsoPort.getName().c_str() << std::endl;
    #endif
        }

        if(!cmdRightArmPort.open("/cmdRightArm:o")){//Sends Joint Motor Commands to Right Arm
    #if DEBUG_CODE>0
            std::cout << "Right Arm Output Commanded Position Port Created : " << cmdRightArmPort.getName().c_str() << std::endl;
    #endif
        }

#if DEBUG_CODE>0
    if(!rightEEPort.open("/rightEEPort:i")){//Receives Right Arm End Effector Value Using Cartesian Controller
#if DEBUG_CODE>0
        std::cout << "Right End Effector Port Created : " << rightEEPort.getName().c_str() << std::endl;
#endif
    }
#endif

    }

    //This is for Commanding Joints for Grasping
    if(!cmdGraspPort.open("/cmdGrasp:o")){//This Sends The Joint Commands for Grasping
#if DEBUG_CODE>0
        std::cout << "Grasp Commanded Port Created : " << cmdGraspPort.getName().c_str() << std::endl;
#endif
    }

    //grasp();//Calling the Grap Routine
    //yarp::os::Time::delay(2); //This is to show difference between Grasping and PMP Module

    //Target/tar for the End Effector
    //getTarget();
    setTarget();

#if DEBUG_CODE>0
    //Read the Joint Values from Gazebo in Degrees
    //readJoints();

    //Joint Value Initialization in Radians
    //initializeJoints();

    //Read the End Effector Position Values Using Cartesian Controller
    //readEE(); //This method uses cartesian controller for getting the end effector positions

    //Computing the End Effector Position Value Using Forward Kinematics from iKin Library
    //computeEEPos();
#endif

    //Calling Simple Virtual Trajectory Generation System
    simpleVTGS(); //TODO: Put This in the Thread run() routine

}

void simplePMPThread::initializeJoints(){
    if(armName == "left" || armName == "lr"){

        std::cout << "Initializing Torso + Left Arm Joint Configuration in Radians : " << endl;
        jAnglesLT.assign(10,0); //Initialization to Zero

        jAnglesLT.at(0) = 0; //Torso
        jAnglesLT.at(1) = 0;
        jAnglesLT.at(2) = 0;

        jAnglesLT.at(3) = -1.74; //Left Arm
        jAnglesLT.at(4) = 0.78;
        jAnglesLT.at(5) = 0;
        jAnglesLT.at(6) = 1.3;
        jAnglesLT.at(7) = 0;
        jAnglesLT.at(8) = 0;
        jAnglesLT.at(9) = 0;

        for(int i=0; i < jAnglesLT.size() ; i++){
            std::cout << jAnglesLT.at(i) << " ";
        }std::cout << std::endl;
    }else if(armName == "left" || armName == "lr"){

        std::cout << "Initializing Torso + Right Arm Joint Configuration in Radians : " << endl;
        jAnglesRT.assign(10,0); //Initialization to Zero

        jAnglesRT.at(0) = 0; //Torso
        jAnglesRT.at(1) = 0;
        jAnglesRT.at(2) = 0;

        jAnglesRT.at(3) = -1.74; //Left Arm
        jAnglesRT.at(4) = 0.78;
        jAnglesRT.at(5) = 0;
        jAnglesRT.at(6) = 1.3;
        jAnglesRT.at(7) = 0;
        jAnglesRT.at(8) = 0;
        jAnglesRT.at(9) = 0;

#if DEBUG_CODE>0
        for(int i=0; i < jAnglesRT.size() ; i++){
            std::cout << jAnglesRT.at(i) << " ";
        }std::cout << std::endl;
#endif
    }

}

void simplePMPThread::getTarget(){ //TODO Make this reply with OK command using rpc

    //TODO Have to make it like a Blocking Call - Like wait until a target is given
    //TODO for "lr"  condition need to modify the target assignment
    yarp::os::Bottle *tar = inputPort.read();

    if(tar!=NULL && tar->size() ==3){
#if DEBUG_CODE>0
        std::cout << "Acquired Target Position Correctly..." << std::endl;
#endif
        if(armName == "left" || armName == "lr"){

            tarLEE[0] = tar->get(0).asDouble();
            tarLEE[1] = tar->get(1).asDouble();
            tarLEE[2] = tar->get(2).asDouble();

            target = true;
            std::cout << "Acquired Left Arm Target (x,y,z) : " << "(" << tarLEE[0]
                      << "," << tarLEE[1] << "," << tarLEE[2] << ")" << std::endl;

        }else if(armName == "right" || armName == "lr"){

            tarREE[0] = tar->get(0).asDouble();
            tarREE[1] = tar->get(1).asDouble();
            tarREE[2] = tar->get(2).asDouble();

            target = true;
            std::cout << "Acquired Right Arm Target (x,y,z) : " << "(" << tarREE[0]
                      << "," << tarREE[1] << "," << tarREE[2] << ")" << std::endl;
        }

    }else{
        target = false;
#if DEBUG_CODE>0
        std::cout << "Error in Target Position!!!" << std::endl;
#endif
    }

}

void simplePMPThread::setTarget(){

#if DEBUG_CODE>0
    std::cout << "Using hard coded Targets..." << std::endl;
#endif

    if(armName == "left" || armName == "lr"){
        //Left Arm
        tarLEE[0] = -0.25; //y direction positive towards left of icub 1000//-1000
        tarLEE[1] = -0.1;// x direction  negative front wards -1000
        tarLEE[2] = 0.3;//z upwards postive - This is Changing X Position? -1000

        target = true;
#if DEBUG_CODE>0
std::cout << "Acquired Left Arm Target (x,y,z) : " << "(" << tarLEE[0] << "," << tarLEE[1] << "," << tarLEE[2] << ")" << std::endl;
#endif
    }else if(armName == "right" || armName == "lr"){

        //Right Arm
        tarREE[0] = -0.25; //y direction positive towards left of icub 1000//-1000
        tarREE[1] = 0.1;// x direction  negative front wards -1000
        tarREE[2] = 0.3;//z upwards postive - This is Changing X Position? -1000

        target = true;
#if DEBUG_CODE>0
std::cout << "Acquired Right Arm Target (x,y,z) : " << "(" << tarREE[0] << "," << tarREE[1] << "," << tarREE[2] << ")" << std::endl;
#endif
    }
}

void simplePMPThread::readJoints(){

    //Reading Torso Joint Angles
    if(!yarp::os::Network::isConnected("/icubGazeboSim/torso/state:o",torsoAnglesPort.getName().c_str())){
        yarp::os::Network::connect("/icubGazeboSim/torso/state:o",torsoAnglesPort.getName().c_str());
        yarp::os::Bottle *jAngle;
        jAngle = torsoAnglesPort.read();
        if(jAngle!= NULL){
#if DEBUG_CODE>0
            std::cout << "Joint Angles Read from Torso..." << std::endl;
            std::cout << "Size of the Joint Angles Bottle: " << jAngle->size() << std::endl;
            std::cout << "The Angles read to the Bottle are: " << jAngle->toString().c_str() << std::endl;
#endif

            jAnglesTorso.resize(jAngle->size());

#if DEBUG_CODE>0
            std::cout << "Joint Angles size of " << jAnglesTorso.size() << " for Torso Configuration Set..." << std::endl ;
#endif
            for(int i=0; i < jAngle->size() ; i++){
                jAnglesTorso.at(i) = (jAngle->get(i).asDouble())*CTRL_DEG2RAD;
                //jAnglesTorso.at(i) = (jAngle->get(i).asDouble());
            }
        }
    }else{
        yarp::os::Network::disconnect("/icubGazeboSim/torso/state:o",torsoAnglesPort.getName().c_str());
    }


    if(armName == "left" || armName == "lr"){

        //Reading Left Arm Joint Angles
        if(!yarp::os::Network::isConnected("/icubGazeboSim/left_arm_no_hand/state:o",leftArmAnglesPort.getName().c_str())){
            yarp::os::Network::connect("/icubGazeboSim/left_arm_no_hand/state:o",leftArmAnglesPort.getName().c_str());
            yarp::os::Bottle *jAngle;
            jAngle = leftArmAnglesPort.read();
            if(jAngle!= NULL){
    #if DEBUG_CODE>0
                std::cout << "Joint Angles Read from Left Arm..." << std::endl;
                std::cout << "Size of the Joint Angles Bottle: " << jAngle->size() << std::endl;
                std::cout << "The Angles read to the Bottle are: " << jAngle->toString().c_str() << std::endl;
    #endif

                jAnglesLeftArm.resize(jAngle->size());

    #if DEBUG_CODE>0
                std::cout << "Joint Angles size of " << jAnglesLeftArm.size() << " for Left Arm Configuration Set..." << std::endl ;
    #endif
                for(int i=0; i < jAngle->size() ; i++){
                    jAnglesLeftArm.at(i) = (jAngle->get(i).asDouble())*CTRL_DEG2RAD;
                    //jAnglesLeftArm.at(i) = (jAngle->get(i).asDouble());
                }
            }
        }else{
            yarp::os::Network::disconnect("/icubGazeboSim/left_arm_no_hand/state:o",leftArmAnglesPort.getName().c_str());
        }

        //Setting the Joint Angles for Torso + Left Arm Configurations
        jAnglesLT.resize(jAnglesTorso.size() + jAnglesLeftArm.size());
        jAnglesLT = jAnglesTorso;
        jAnglesLT.insert(jAnglesLT.end(),jAnglesLeftArm.begin(),jAnglesLeftArm.end());


#if DEBUG_CODE>0
    std::cout << "Joint Angles size of " << jAnglesLT.size() << " for Torso + Left Arm Configuration is :";
    for(int a=0 ; a < jAnglesLT.size(); a++){
        std::cout << " " << jAnglesLT.at(a);
    }
    std::cout << std::endl;
#endif


    }else if(armName == "right" || armName == "lr"){

        //Reading Right Arm Joint Angles
        if(!yarp::os::Network::isConnected("/icubGazeboSim/right_arm_no_hand/state:o",rightArmAnglesPort.getName().c_str())){
            yarp::os::Network::connect("/icubGazeboSim/right_arm_no_hand/state:o",rightArmAnglesPort.getName().c_str());
            yarp::os::Bottle *jAngle;
            jAngle = rightArmAnglesPort.read();
            if(jAngle!= NULL){
    #if DEBUG_CODE>0
                std::cout << "Joint Angles Read from Right Arm..." << std::endl;
                std::cout << "Size of the Joint Angles Bottle: " << jAngle->size() << std::endl;
                std::cout << "The Angles read to the Bottle are: " << jAngle->toString().c_str() << std::endl;
    #endif

                jAnglesRightArm.resize(jAngle->size());

    #if DEBUG_CODE>0
                std::cout << "Joint Angles size of " << jAnglesRightArm.size() << " for Right Arm Configuration Set..." << std::endl ;
    #endif
                for(int i=0; i < jAngle->size() ; i++){
                    jAnglesRightArm.at(i) = (jAngle->get(i).asDouble())*CTRL_DEG2RAD;
                    //jAnglesRightArm.at(i) = (jAngle->get(i).asDouble());
                }
            }
        }else{
            yarp::os::Network::disconnect("/icubGazeboSim/right_arm_no_hand/state:o",rightArmAnglesPort.getName().c_str());
        }


        //Setting the Joint Angles for Torso + Right Arm Configurations
        jAnglesRT.resize(jAnglesTorso.size() + jAnglesRightArm.size());
        jAnglesRT = jAnglesTorso;
        jAnglesRT.insert(jAnglesRT.end(),jAnglesRightArm.begin(),jAnglesRightArm.end());


#if DEBUG_CODE>0
    std::cout << "Joint Angles size of " << jAnglesLT.size() << " for Torso + Right Arm Configuration is :";
    for(int a=0 ; a < jAnglesRT.size(); a++){
        std::cout << " " << jAnglesRT.at(a);
    }
    std::cout << std::endl;
#endif

    }
}

void simplePMPThread::readEE(){

#if DEBUG_CODE>0
    std::cout << "Reading End Effector Position and Orientation using Cartesian Control Interface" << std::endl;
#endif

    if(armName == "left" || armName == "lr"){

        //Left Arm End Effector Coordinates
        if(!yarp::os::Network::isConnected("/icubGazeboSim/cartesianController/left_arm_no_hand/state:o",leftEEPort.getName().c_str())){
            yarp::os::Network::connect("/icubGazeboSim/cartesianController/left_arm_no_hand/state:o",leftEEPort.getName().c_str());
            yarp::os::Bottle *eePos;
            eePos = leftEEPort.read();
            if(eePos!=NULL){
    #if DEBUG_CODE>0
                std::cout << "End Effector Position and Orientation from Left Arm read..." << std::endl;
                std::cout << "Size of the EE Position and Orientation Bottle: " << eePos->size() << std::endl;
                std::cout << "The Position and Orientation values read to the Bottle are: " << eePos->toString().c_str() << std::endl;
    #endif
                //Noting only the position values of the End Effector
                initPosLeftEE[0] = eePos->get(0).asDouble(); // X Value
                initPosLeftEE[1] = eePos->get(1).asDouble(); // Y Value
                initPosLeftEE[2] = eePos->get(2).asDouble(); // Z Value

                std::cout << "Initial Position of Left End Effector (x,y,z) : " << "(" << initPosLeftEE[0] << "," << initPosLeftEE[1] << "," << initPosLeftEE[2] << ")" << std::endl;

            }
        }else{
            yarp::os::Network::disconnect("/icubGazeboSim/cartesianController/left_arm_no_hand/state:o",leftEEPort.getName().c_str());
    #if DEBUG_CODE>0
            std::cout << " Disconnected /icubGazeboSim/cartesianController/left_arm_no_hand/state:o and " << leftEEPort.getName().c_str() << std::endl;
    #endif
        }

    }else if(armName== "right" || armName == "lr"){

        //Right Arm End Effector Coordinates
        if(!yarp::os::Network::isConnected("/icubGazeboSim/cartesianController/right_arm_no_hand/state:o",rightEEPort.getName().c_str())){
            yarp::os::Network::connect("/icubGazeboSim/cartesianController/right_arm_no_hand/state:o",rightEEPort.getName().c_str());
            yarp::os::Bottle *eePos;
            eePos = rightEEPort.read();
            if(eePos!=NULL){
    #if DEBUG_CODE>0
                std::cout << "End Effector Position and Orientation from Right Hand read..." << std::endl;
                std::cout << "Size of the EE Position and Orientation Bottle: " << eePos->size() << std::endl;
                std::cout << "The Position and Orientation values read to the Bottle are: " << eePos->toString().c_str() << std::endl;
    #endif
                //Noting only the position values of the End Effector
                initPosRightEE[0] = eePos->get(0).asDouble(); // X Value
                initPosRightEE[1] = eePos->get(1).asDouble(); // Y Value
                initPosRightEE[2] = eePos->get(2).asDouble(); // Z Value

                std::cout << "Initial Position of Right End Effector (x,y,z) : " << "(" << initPosRightEE[0] << "," << initPosRightEE[1] << "," << initPosRightEE[2] << ")" << std::endl;


            }
        }else{
            yarp::os::Network::disconnect("/icubGazeboSim/cartesianController/right_arm_no_hand/state:o",rightEEPort.getName().c_str());
    #if DEBUG_CODE>0
            std::cout << " Disconnected /icubGazeboSim/cartesianController/right_arm_no_hand/state:o and " << rightEEPort.getName().c_str() << std::endl;
    #endif
        }
    }
}

void simplePMPThread::checkEEPos(){

    if(armName == "left" || armName == "lr"){

#if DEBUG_CODE>0
    std::cout << "Commanded Joint Angles in Degrees :               " << " " <<  jAnglesLT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(9)*CTRL_RAD2DEG << std::endl;
#endif

    //Left Arm End Effector Position Computation
    //std::cout << "Size of Computed Joint Angle Commands : " << jAnglesLT.size() << std::endl;
    for(int a=0; a < jAnglesLT.size() ; a++){
        leftAngles[a] = jAnglesLT.at(a)*CTRL_RAD2DEG;
    }

    //computeFKLeft(leftPos,leftAngles); //This Uses Utils.h  File

    yarp::sig::Vector lpos; //Temp Variable
    lpos = armLeft->getEEPos(jAnglesLT);
    leftPos[0] = lpos[0];
    leftPos[1] = lpos[1];
    leftPos[2] = lpos[2];

//#if DEBUG_CODE>0
    std::cout << "Values of the Left Arm EE Position to be attained (x,y,z) : " << "(" << leftPos[0] << ","
              << leftPos[1] << "," << leftPos[2] << ")" << std::endl;
//#endif

    }else if(armName == "right" || armName == "lr"){

#if DEBUG_CODE>0
    std::cout << "Commanded Joint Angles in Degrees :               " << " " <<  jAnglesRT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(9)*CTRL_RAD2DEG << std::endl;
#endif

        //Right Arm End Effector Position Computation
        //std::cout << "Size of Computed Joint Angle Commands : " << jAnglesRT.size() << std::endl;
        for(int a=0; a < jAnglesRT.size() ; a++){
            rightAngles[a] = jAnglesRT.at(a)*CTRL_RAD2DEG;
        }

        //computeFKRight(rightPos,rightAngles); //This Uses Utils.h File

        yarp::sig::Vector rpos; //Temp Variable
        rpos = armRight->getEEPos(jAnglesRT);
        rightPos[0] = rpos[0];
        rightPos[1] = rpos[1];
        rightPos[2] = rpos[2];

//#if DEBUG_CODE>0
    std::cout << "Values of the Right Arm EE Position to be attained (x,y,z) : " << "(" << rightPos[0] << ","
              << rightPos[1] << "," << rightPos[2] << ")" << std::endl;
//#endif
    }
}

void simplePMPThread::computeEEPos(){

#if DEBUG_CODE>0
    std::cout << "Computing End Effector Position and Orientation using Forward Kinematics of iCub Robot from iKin Library" << std::endl;
#endif

    if(armName == "left" || armName == "lr"){

        //Left End Effector Position Computation
#if DEBUG_CODE>0
        std::cout << "Computing Forward Kinematics of Left Arm + Torso Configuration..." << std::endl;
#endif
        for(int a=0; a < jAnglesLT.size() ; a++){
            leftAngles[a] = jAnglesLT.at(a);
        }

#if DEBUG_CODE>0
        std::cout << "Size of Joint Angles Received : " << sizeof(leftAngles)/sizeof(double) << std::endl;
#endif
        //computeFKLeft(leftPos,leftAngles); //This Uses Utils.h File

        yarp::sig::Vector lpos; //Temp Variable
        lpos = armLeft->getEEPos(jAnglesLT);
        leftPos[0] = lpos[0];
        leftPos[1] = lpos[1];
        leftPos[2] = lpos[2];

#if DEBUG_CODE>0
        std::cout << "New Values of the Left EE Position (x,y,z) : " << "(" << leftPos[0] << ","
                  << leftPos[1] << "," << leftPos[2] << ")" << std::endl;
#endif

        curPosLeftEE[0] = leftPos[0];//X Value
        curPosLeftEE[1] = leftPos[1];//Y Value
        curPosLeftEE[2] = leftPos[2];//Z Value

    }else if(armName == "right" || armName == "lr"){

        //Right End Effector Position Computation
#if DEBUG_CODE>0
        std::cout << "Computing Forward Kinematics of Right Arm + Torso Configuration..." << std::endl;
#endif

        for(int a=0; a < jAnglesRT.size() ; a++){
            rightAngles[a] = jAnglesRT.at(a);
        }

#if DEBUG_CODE>0
        std::cout << "Size of Joint Angles Received : " << sizeof(rightAngles)/sizeof(double) << std::endl;
#endif
        //computeFKRight(rightPos,rightAngles); //This Uses Utils.h File

        yarp::sig::Vector rpos; //Temp Variable
        rpos = armRight->getEEPos(jAnglesRT);
        rightPos[0] = rpos[0];
        rightPos[1] = rpos[1];
        rightPos[2] = rpos[2];

#if DEBUG_CODE>0
        std::cout << "New Values of the Right EE Position (x,y,z) : " << "(" << rightPos[0] << ","
                  << rightPos[1] << "," << rightPos[2] << ")" << std::endl;
#endif

        curPosRightEE[0] = rightPos[0];//X Value
        curPosRightEE[1] = rightPos[1];//Y Value
        curPosRightEE[2] = rightPos[2];//Z Value

    }
}


void simplePMPThread::simpleVTGS(){//This depends on the number of intermediate points we need

//#if DEBUG_CODE>0
    std::cout << "Running Virtual Trajectory Generation System for the Acquired Target Position..." << std::endl;
//#endif

    readJoints(); //Reading Joint Values

#if DEBUG_CODE>0
    //initializeJoints();
#endif

    computeEEPos(); //Computing End Effector Position Corresponding to Initially Read Joint Values

    //Local Variables
    int time;
    double Gam;
    double xoff=1, yoff=1 , zoff=1; //Offsets

    //Storing the Initial Values of End Effector

    if(armName == "left" || armName == "lr"){

        initPosLeftEE[0] = curPosLeftEE[0];
        initPosLeftEE[1] = curPosLeftEE[1];
        initPosLeftEE[2] = curPosLeftEE[2];

#if DEBUG_CODE>0
    std::cout << "Initial Left Arm EE Position (x,y,z) : " << "(" << initPosLeftEE[0] << ","
              << initPosLeftEE[1] << "," << initPosLeftEE[2] << ")" << std::endl;
    std::cout << "Final Left Arm tar (x,y,z) : " << "("
              << tarLEE[0] << "," << tarLEE[1] << "," << tarLEE[2] << ")" << std::endl;
#endif

        initPosLeftEEIC[0] = initPosLeftEE[0];
        initPosLeftEEIC[1] = initPosLeftEE[1];
        initPosLeftEEIC[2] = initPosLeftEE[2];

        //Storing the Initial Values of Joint Angles
        initJAnglesLT.assign(jAnglesLT.size(),0);
        for(int j = 0 ; j < jAnglesLT.size() ; j++){
            initJAnglesLT.at(j) = jAnglesLT.at(j);
        }


    }else if(armName == "right" || armName == "lr"){


        initPosRightEE[0] = curPosRightEE[0];
        initPosRightEE[1] = curPosRightEE[1];
        initPosRightEE[2] = curPosRightEE[2];

#if DEBUG_CODE>0
    std::cout << "Initial Right Arm EE Position (x,y,z) : " << "(" << initPosRightEE[0] << ","
              << initPosRightEE[1] << "," << initPosRightEE[2] << ")" << std::endl;
    std::cout << "Final Right Arm tar (x,y,z) : " << "("
              << tarREE[0] << "," << tarREE[1] << "," << tarREE[2] << ")" << std::endl;
#endif

        initPosRightEEIC[0] = initPosRightEE[0];
        initPosRightEEIC[1] = initPosRightEE[1];
        initPosRightEEIC[2] = initPosRightEE[2];

        //Storing the Initial Values of Joint Angles
        initJAnglesRT.assign(jAnglesRT.size(),0);

        for(int j = 0 ; j < jAnglesRT.size() ; j++){
            initJAnglesRT.at(j) = jAnglesRT.at(j);
        }
    }


    for(time=0; time<ITERATION ; time++){// 2000 Incremental Steps of Size 1

        Gam = GammaDisc(time);

        if(armName == "left" || armName == "lr"){

            //Left Arm Virtual Trajectory
            //X Value
            virPosLeftEE[0] = (tarLEE[0]-initPosLeftEE[0])*Gam;
            Gam_ArrxLeft[time] = virPosLeftEE[0];
            double *GarxLeft = Gam_ArrxLeft;
            virPosLeftEE[0] = Gamma_Int(GarxLeft,time) + initPosLeftEEIC[0];
            initPosLeftEE[0] = virPosLeftEE[0];

            //Y Value
            virPosLeftEE[1] = (tarLEE[1]-initPosLeftEE[1])*Gam;
            Gam_ArryLeft[time] = virPosLeftEE[1];
            double *GaryLeft = Gam_ArryLeft;
            virPosLeftEE[1] = Gamma_Int(GaryLeft,time) + initPosLeftEEIC[1];
            initPosLeftEE[1] = virPosLeftEE[1];

            //Z Value
            virPosLeftEE[2] = (tarLEE[2]-initPosLeftEE[2])*Gam;
            Gam_ArrzLeft[time] = virPosLeftEE[2];
            double *GarzLeft = Gam_ArrzLeft;
            virPosLeftEE[2] = Gamma_Int(GarzLeft,time) + initPosLeftEEIC[2];
            initPosLeftEE[2] = virPosLeftEE[2];

#if DEBUG_CODE>0
        std::cout << "Left EE Virtual Target Position (x,y,z) #" << time << " :              "
                  << "(" << virPosLeftEE[0] << "," << virPosLeftEE[1] << "," << virPosLeftEE[2] << ")" << std::endl;
#endif

        forceFieldLeft = computeForceFieldLeft(curPosLeftEE,initPosLeftEE);//here initPosLeftEE is actually the virtual target
        computeTorqueLeft(forceFieldLeft);
        computeJointVelLeft();
        jVel2AngleLeft(time);

#if DEBUG_CODE>0
        checkEEPos(); // Checking the End Effector Value That Will Be Reached With Each New Joint Configuration
#endif
        //Condition To Break the VTGS After Position Threshold Error is Reached
        if( ( fabs((virPosLeftEE[0]-tarLEE[0])) < posErr ) &&
            ( fabs((virPosLeftEE[1]-tarLEE[1])) < posErr ) &&
            ( fabs((virPosLeftEE[2]-tarLEE[2])) < posErr ) ){

 #if DEBUG_CODE>0
     std::cout << "Virtual Target Position reached Left EE tar Position approximately  (x,y,z) : ";
     std::cout << "(" << virPosLeftEE[0] << "," << virPosLeftEE[1] << "," << virPosLeftEE[2] << ")" << std::endl;
 #endif
             break;
         }else{
             vTarget = true; //Virtual Target is Valid
         }

        }else if(armName == "right" || armName == "lr"){

            //Right Arm Virtual Trajectory
            //X Value
            virPosRightEE[0] = (tarREE[0]-initPosRightEE[0])*Gam;
            Gam_ArrxRight[time] = virPosRightEE[0];
            double *GarxRight = Gam_ArrxRight;
            virPosRightEE[0] = Gamma_Int(GarxRight,time) + initPosRightEEIC[0];
            initPosRightEE[0] = virPosRightEE[0];

            //Y Value
            virPosRightEE[1] = (tarREE[1]-initPosRightEE[1])*Gam;
            Gam_ArryRight[time] = virPosRightEE[1];
            double *GaryRight = Gam_ArryRight;
            virPosRightEE[1] = Gamma_Int(GaryRight,time) + initPosRightEEIC[1];
            initPosRightEE[1] = virPosRightEE[1];

            //Z Value
            virPosRightEE[2] = (tarREE[2]-initPosRightEE[2])*Gam;
            Gam_ArrzRight[time] = virPosRightEE[2];
            double *GarzRight = Gam_ArrzRight;
            virPosRightEE[2] = Gamma_Int(GarzRight,time) + initPosRightEEIC[2];
            initPosRightEE[2] = virPosRightEE[2];

#if DEBUG_CODE>0
        std::cout << "Right EE Virtual Target Position (x,y,z) #" << time << " : "
                  << "(" << virPosRightEE[0] << "," << virPosRightEE[1] << "," << virPosRightEE[2] << ")" << std::endl;
#endif

        forceFieldRight = computeForceFieldRight(curPosRightEE,initPosRightEE);//here initPosRightEE is actually the virtual target
        computeTorqueRight(forceFieldRight);
        computeJointVelRight();
        jVel2AngleRight(time);

#if DEBUG_CODE>0
        checkEEPos();
#endif


        //Condition To Break the VTGS After Position Threshold Error is Reached
        if( ( fabs((virPosRightEE[0]-tarREE[0])) < posErr ) &&
            ( fabs((virPosRightEE[1]-tarREE[1])) < posErr ) &&
            ( fabs((virPosRightEE[2]-tarREE[2])) < posErr ) ){

#if DEBUG_CODE>0
    std::cout << "Virtual Target(x,y,z) Position reached Right EE tar Position approximately : ";
    std::cout << "(" << virPosRightEE[0] << "," << virPosRightEE[1] << "," << virPosRightEE[2] << ")" << std::endl;
#endif
            break;
        }else{
            vTarget = true; //Virtual Target is Valid
        }
        }
    }//End of Time For Loop


    //TODO : Not Sure if it is Better to Have one cmdJoint or two separate threads
    if(armName == "left" || armName == "lr"){
        //Commanding Joint Motors
        cmdJointLeft();

    }else if(armName == "right" || armName == "lr"){
        //Commanding Joint Motors
        cmdJointRight();
    }


    //Checking Final End Effector Position Reached With Commanded Joint Values
    checkEEPos();

}

void simplePMPThread::cmdJointLeft(){

    std::cout << "Commanding the Motors..." << std::endl;
    std::cout << "Received Joint Angles in Degrees : ";
    double cmdJLeft[jAnglesLT.size()];
    for(int j = 0; j < jAnglesLT.size() ; j++){
        cmdJLeft[j] = jAnglesLT.at(j)*CTRL_RAD2DEG; //This is Conversion From Radians to Degrees
        std::cout << " " << cmdJLeft[j];
    }std::cout<<std::endl;

#if DEBUG_CODE>0
    //correctJAngles(cmdJLeft);
#endif

#if DEBUG_CODE>0
    std::cout << "Received Joint Angles, Commanding the Motors..." << std::endl;
#endif

    //This is Torso Motor Control
    if(!yarp::os::Network::isConnected(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i")){
        yarp::os::Network::connect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
        for(int j = 0 ; j < 3 ; j++){//TODO Here only 3 joints are controlled
            yarp::os::Bottle& cmdT = cmdTorsoPort.prepare();
            cmdT.clear();
            cmdT.addString("set");
            cmdT.addString("pos");
            cmdT.addInt(j);
            cmdT.addDouble(cmdJLeft[j]);
            cmdTorsoPort.writeStrict();
            yarp::os::Time::delay(1);

#if DEBUG_CODE>0
            //syncObject->clockServer.stepSimulation(syncObject->configuration.numberOfSteps);
            //syncObject->clockServer.continueSimulation();
#endif
        }


    }else{
        yarp::os::Network::disconnect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdTorsoPort.getName().c_str() << " and /icubGazeboSim/torso/rpc:i"<< std::endl;
#endif
    }

    //This is Left Arm Motor Control
    if(!yarp::os::Network::isConnected(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i")){
        yarp::os::Network::connect(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i");
        for(int j = 3 ; j < 10 ; j++){//TODO Here only 7 joints are controlled
            yarp::os::Bottle& cmdLA = cmdLeftArmPort.prepare();
            cmdLA.clear();
            cmdLA.addString("set");
            cmdLA.addString("pos");
            cmdLA.addInt(j-3);
            cmdLA.addDouble(cmdJLeft[j]);
            cmdLeftArmPort.writeStrict();
            yarp::os::Time::delay(1);

#if DEBUG_CODE>0
            //syncObject->clockServer.stepSimulation(syncObject->configuration.numberOfSteps);
            //syncObject->clockServer.continueSimulation();
#endif
        }


    }else{
        yarp::os::Network::disconnect(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdLeftArmPort.getName().c_str() << " and /icubGazeboSim/left_arm_no_hand/rpc:i" << std::endl;
#endif
    }


}

void simplePMPThread::cmdJointRight(){

    std::cout << "Commanding the Motors..." << std::endl;
    std::cout << "Received Joint Angles in Degrees : ";
    double cmdJRight[jAnglesRT.size()];
    for(int j = 0; j < jAnglesRT.size() ; j++){
        cmdJRight[j] = jAnglesRT.at(j)*CTRL_RAD2DEG;
        std::cout << " " << cmdJRight[j];
    }std::cout<<std::endl;

#if DEBUG_CODE>0
    //correctJAngles(cmdJRight);
#endif

#if DEBUG_CODE>0
    std::cout << "Received Joint Angles, Commanding the Motors..." << std::endl;
#endif


    //This is Torso Motor Control
    if(!yarp::os::Network::isConnected(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i")){
        yarp::os::Network::connect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
        for(int j = 0 ; j < 3 ; j++){//TODO Here only 3 joints are controlled
            yarp::os::Bottle& cmdT = cmdTorsoPort.prepare();
            cmdT.clear();
            cmdT.addString("set");
            cmdT.addString("pos");
            cmdT.addInt(j);
            cmdT.addDouble(cmdJRight[j]);
            cmdTorsoPort.writeStrict();
            yarp::os::Time::delay(1);

#if DEBUG_CODE>0
            //syncObject->clockServer.stepSimulation(syncObject->configuration.numberOfSteps);
            //syncObject->clockServer.continueSimulation();
#endif
        }


    }else{
        yarp::os::Network::disconnect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdTorsoPort.getName().c_str() << " and /icubGazeboSim/torso/rpc:i"<< std::endl;
#endif
    }

    //This is Right Arm Motor Control
    if(!yarp::os::Network::isConnected(cmdRightArmPort.getName().c_str(),"/icubGazeboSim/right_arm_no_hand/rpc:i")){
        yarp::os::Network::connect(cmdRightArmPort.getName().c_str(),"/icubGazeboSim/right_arm_no_hand/rpc:i");
        for(int j = 3 ; j < 10 ; j++){//TODO Here only 7 joints are controlled
            yarp::os::Bottle& cmdRA = cmdRightArmPort.prepare();
            cmdRA.clear();
            cmdRA.addString("set");
            cmdRA.addString("pos");
            cmdRA.addInt(j-3);
            cmdRA.addDouble(cmdJRight[j]);
            cmdRightArmPort.writeStrict();
            yarp::os::Time::delay(1);

#if DEBUG_CODE>0
            //syncObject->clockServer.stepSimulation(syncObject->configuration.numberOfSteps);
            //syncObject->clockServer.continueSimulation();
#endif
        }


    }else{
        yarp::os::Network::disconnect(cmdRightArmPort.getName().c_str(),"/icubGazeboSim/right_arm_no_hand/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdRightArmPort.getName().c_str() << " and /icubGazeboSim/right_arm_no_hand/rpc:i" << std::endl;
#endif
    }

}

void simplePMPThread::jVel2AngleLeft(int _time){

#if DEBUG_CODE>0
    std::cout << "Initial Left Arm Joint Angles in Degrees :    " << " " <<  jAnglesLT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(9)*CTRL_RAD2DEG << std::endl;
#endif

    q1L[_time] = jointVelLeft.at(0);
    double *j1L=q1L;
    double joi1L = Gamma_Int(j1L,_time);
    jAnglesLT.at(0) = joi1L + initJAnglesLT.at(0);

    q2L[_time] = jointVelLeft.at(1);
    double *j2L=q2L;
    double joi2L = Gamma_Int(j2L,_time);
    jAnglesLT.at(1) = joi2L + initJAnglesLT.at(1);

    q3L[_time] = jointVelLeft.at(2);
    double *j3L=q3L;
    double joi3L = Gamma_Int(j3L,_time);
    jAnglesLT.at(2) = joi3L + initJAnglesLT.at(2);

    q4L[_time] = jointVelLeft.at(3);
    double *j4L=q4L;
    double joi4L = Gamma_Int(j4L,_time);
    jAnglesLT.at(3) = joi4L + initJAnglesLT.at(3);

    q5L[_time] = jointVelLeft.at(4);
    double *j5L=q5L;
    double joi5L = Gamma_Int(j5L,_time);
    jAnglesLT.at(4) = joi5L + initJAnglesLT.at(4);

    q6L[_time] = jointVelLeft.at(5);
    double *j6L=q6L;
    double joi6L = Gamma_Int(j6L,_time);
    jAnglesLT.at(5) = joi6L + initJAnglesLT.at(5);

    q7L[_time] = jointVelLeft.at(6);
    double *j7L=q7L;
    double joi7L = Gamma_Int(j7L,_time);
    jAnglesLT.at(6) = joi7L + initJAnglesLT.at(6);

    q8L[_time] = jointVelLeft.at(7);
    double *j8L=q8L;
    double joi8L = Gamma_Int(j8L,_time);
    jAnglesLT.at(7) = joi8L + initJAnglesLT.at(7);

    q9L[_time] = jointVelLeft.at(8);
    double *j9L=q9L;
    double joi9L = Gamma_Int(j9L,_time);
    jAnglesLT.at(8) = joi9L + initJAnglesLT.at(8);

    q10L[_time] = jointVelLeft.at(9);
    double *j10L=q10L;
    double joi10L = Gamma_Int(j10L,_time);
    jAnglesLT.at(9) = joi10L + initJAnglesLT.at(9);

#if DEBUG_CODE>0
    std::cout << "Computed New Left Arm Joint Angles in Degrees :   " << " " <<  jAnglesLT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesLT.at(9)*CTRL_RAD2DEG << std::endl;
#endif


}

void simplePMPThread::jVel2AngleRight(int _time){


#if DEBUG_CODE>0
    std::cout << "Initial Right Arm Joint Angles in Degrees : " << " " <<  jAnglesRT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(9)*CTRL_RAD2DEG << std::endl;
#endif

    q1R[_time] = jointVelRight.at(0);
    double *j1R=q1R;
    double joi1R = Gamma_Int(j1R,_time);
    jAnglesRT.at(0) = joi1R + initJAnglesRT.at(0);

    q2R[_time] = jointVelRight.at(1);
    double *j2R=q2R;
    double joi2R = Gamma_Int(j2R,_time);
    jAnglesRT.at(1) = joi2R + initJAnglesRT.at(1);

    q3R[_time] = jointVelRight.at(2);
    double *j3R=q3R;
    double joi3R = Gamma_Int(j3R,_time);
    jAnglesRT.at(2) = joi3R + initJAnglesRT.at(2);

    q4R[_time] = jointVelRight.at(3);
    double *j4R=q4R;
    double joi4R = Gamma_Int(j4R,_time);
    jAnglesRT.at(3) = joi4R + initJAnglesRT.at(3);

    q5R[_time] = jointVelRight.at(4);
    double *j5R=q5R;
    double joi5R = Gamma_Int(j5R,_time);
    jAnglesRT.at(4) = joi5R + initJAnglesRT.at(4);

    q6R[_time] = jointVelRight.at(5);
    double *j6R=q6R;
    double joi6R = Gamma_Int(j6R,_time);
    jAnglesRT.at(5) = joi6R + initJAnglesRT.at(5);

    q7R[_time] = jointVelRight.at(6);
    double *j7R=q7R;
    double joi7R = Gamma_Int(j7R,_time);
    jAnglesRT.at(6) = joi7R + initJAnglesRT.at(6);

    q8R[_time] = jointVelRight.at(7);
    double *j8R=q8R;
    double joi8R = Gamma_Int(j8R,_time);
    jAnglesRT.at(7) = joi8R + initJAnglesRT.at(7);

    q9R[_time] = jointVelRight.at(8);
    double *j9R=q9R;
    double joi9R = Gamma_Int(j9R,_time);
    jAnglesRT.at(8) = joi9R + initJAnglesRT.at(8);

    q10R[_time] = jointVelRight.at(9);
    double *j10R=q10R;
    double joi10R = Gamma_Int(j10R,_time);
    jAnglesRT.at(9) = joi10R + initJAnglesRT.at(9);

#if DEBUG_CODE>0
    std::cout << "Computed New Right Arm Joint Angles in Degrees : " << " " <<  jAnglesRT.at(0)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(1)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(2)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(3)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(4)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(5)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(6)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(7)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(8)*CTRL_RAD2DEG
              << " " << jAnglesRT.at(9)*CTRL_RAD2DEG << std::endl;
#endif

}

double* simplePMPThread::computeForceFieldLeft(double *curPos, double *tarPos){

#if DEBUG_CODE>0
    std::cout << "Computed Force Field for Left End Effector..." << std::endl;
#endif

    static double res[3]; //Local Variable to Store External Force Field Values

#if DEBUG_CODE>0
    double posDiff[3];
    posDiff[0] = ((*(tarPos+0)- *(curPos+0)));
    posDiff[1] = ((*(tarPos+1)- *(curPos+1)));
    posDiff[2] = ((*(tarPos+2)- *(curPos+2)));
    std::cout << "Left Arm VT and tar Position Differece :                   " << "(" << posDiff[0]
              << "," << posDiff[1] << "," << posDiff[2] << std::endl;
#endif

    //Computing External Force Field Values
    res[0] = 3.4*((*(tarPos+0)- *(curPos+0))); //0.09
    res[1] = 1.95*((*(tarPos+1)- *(curPos+1))); //0.09
    res[2] = 1.22*((*(tarPos+2)- *(curPos+2))); //0.07

#if DEBUG_CODE>0
    std::cout << "Computed Left EE Force (Fx,Fy,Fz) :                         " << "(" << res[0]
              << "," << res[1] << "," << res[2] << ")" << std::endl;
#endif
    return res;
}

double* simplePMPThread::computeForceFieldRight(double *curPos, double *tarPos){

#if DEBUG_CODE>0
    std::cout << "Computed Force Field for Right End Effector..." << std::endl;
#endif

    static double res[3]; //Local Variable to Store External Force Field Values

#if DEBUG_CODE>0
    double posDiff[3];
    posDiff[0] = ((*(tarPos+0)- *(curPos+0)));
    posDiff[1] = ((*(tarPos+1)- *(curPos+1)));
    posDiff[2] = ((*(tarPos+2)- *(curPos+2)));
    std::cout << "Right Arm VT and tar Position Differece :                  " << "(" << posDiff[0]
              << "," << posDiff[1] << "," << posDiff[2] << std::endl;
#endif

    //Computing External Force Field Values
    res[0] = 0.09*(*(tarPos+0)- *(curPos+0)); //0.09
    res[1] = 0.09*((*(tarPos+1)- *(curPos+1))); //0.09
    res[2] = 0.07*((*(tarPos+2)- *(curPos+2))); //0.07

#if DEBUG_CODE>0
    std::cout << "Computed Right EE Force (Fx,Fy,Fz) :                         " << "(" << res[0]
              << "," << res[1] << "," << res[2] << ")" << std::endl;
#endif
    return res;
}

void simplePMPThread::computeTorqueLeft(double *leftForce){

    //Local Variables
    double ffL[3];
    torqueLeft.assign(10,0);

    //Assigning Force Field Values to Local Variables
    ffL[0] = *(leftForce+0);
    ffL[1] = *(leftForce+1);
    ffL[2] = *(leftForce+2);

    //Computing the Joint Limit Force Field from Mean Joint Values
    for(int i = 0; i < 10 ; i++){
        jLimitLeft[i] = (jAnglesMeanLeft[i] - jAnglesLT.at(i)) * JHdL[i];
        //jLimitLeft[i] = 0;
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Limit Force Field : " << jLimitLeft[0] << " " << jLimitLeft[1] << " " //
              << jLimitLeft[2] << " " << jLimitLeft[3] << " " << jLimitLeft[4] << " " << jLimitLeft[5] //
              << " " << jLimitLeft[6] << " " << jLimitLeft[7] << " " << jLimitLeft[8] << " " << jLimitLeft[9] << std::endl;
#endif

    //jacobianLeft = computeJacobianLeft(jAnglesLT);
    jacobianLeft = armLeft->getJacobian(jAnglesLT);


#if DEBUG_CODE>0
    std::cout << "Computed Jacobian of size " << jacobianLeft.size() <<  " for Left Arm + Torso Configuration : ";
    for(int j=0; j < jacobianLeft.size() ; j++){
        std::cout << " " << jacobianLeft.at(j);
    }
    std::cout << std::endl;
#endif


    //Computing Torque from Force Field Using Jacobian Transpose
    for(int p=0 ; p < (jacobianLeft.size()/3) ; p++){
        torqueLeft.at(p) = jacobianLeft.at(p)*ffL[0] +
                           jacobianLeft.at(p+10)*ffL[1] +
                           jacobianLeft.at(p+20)*ffL[2] +
                           jLimitLeft[p];
    }

#if DEBUG_CODE>0
    std::cout << "Computed Torques of size " << torqueLeft.size() <<  " for Left Arm + Torso Joints : ";
    for(int t=0; t < torqueLeft.size() ; t++){
        std::cout << " " << torqueLeft.at(t);
    }
    std::cout << std::endl;
#endif

}

void simplePMPThread::computeTorqueRight(double *rightForce){

    //Local Variables
    double ffR[3];
    torqueRight.assign(10,0);

    //Assigning force to local variables
    ffR[0] = *(rightForce+0);
    ffR[1] = *(rightForce+1);
    ffR[2] = *(rightForce+2);

    //Computing the Joint Limit Force Field from Mean Joint Values
    for(int i = 0; i < 10 ; i++){
        jLimitRight[i] = (jAnglesMeanRight[i] - jAnglesRT.at(i)) * JHdR[i];
        //jLimitRight[i] = 0;
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Limit Force Field : " << jLimitRight[0] << " " << jLimitRight[1] << " " //
              << jLimitRight[2] << " " << jLimitRight[3] << " " << jLimitRight[4] << " " << jLimitRight[5] //
              << " " << jLimitRight[6] << " " << jLimitRight[7] << " " << jLimitRight[8] << " " << jLimitRight[9] << std::endl;
#endif

    //jacobianRight = computeJacobianRight(jAnglesRT);
    jacobianRight = armRight->getJacobian(jAnglesRT);

#if DEBUG_CODE>0
    std::cout << "Computed Jacobian of size " << jacobianRight.size() <<  " for Right Arm + Torso Configuration : ";
    for(int j=0; j < jacobianRight.size() ; j++){
        std::cout << " " << jacobianRight.at(j);
    }
    std::cout << std::endl;
#endif

    //Computing Torque from Force Field Using Jacobian Transpose
    for(int p=0 ; p < (jacobianRight.size()/3) ; p++){
        torqueRight.at(p) = jacobianRight.at(p)*ffR[0] +
                            jacobianRight.at(p+10)*ffR[1] +
                            jacobianRight.at(p+20)*ffR[2] +
                            jLimitRight[p];
    }

#if DEBUG_CODE>0
    std::cout << "Computed Torques of size " << torqueRight.size() <<  " for Right Arm + Torso Joints : ";
    for(int t=0; t < torqueRight.size() ; t++){
        std::cout << " " << torqueRight.at(t);
    }
    std::cout << std::endl;
#endif
}

void simplePMPThread::computeJointVelLeft(){

    jointVelLeft.assign(10,0); //This is Initialization to 0
    for(int t=0; t < 3 ; t++){
        jointVelLeft.at(t) = admitTorso.at(t)*torqueLeft.at(t);
    }

    for(int t=3; t < 10 ; t++){
        //jointVelLeft.at(t) = KOMP_JANG*torqueLeft.at(t);
        jointVelLeft.at(t) = admitLeft.at(t-3)*torqueLeft.at(t);
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Veclocity for the joints of Torso + Left Arm Configuration : " << std::endl;
    for(int v=0; v < jointVelLeft.size() ; v++){
        std::cout << " " << jointVelLeft.at(v);
    }
    std::cout << std::endl;
#endif
}

void simplePMPThread::computeJointVelRight(){

    jointVelRight.assign(10,0); //This is Initialization to 0
    for(int t=0; t < 3 ; t++){
        jointVelRight.at(t) = admitTorso.at(t)*torqueRight.at(t);
    }

    for(int t=3; t < 10 ; t++){
        //jointVelRight.at(t) = KOMP_JANG*torqueRight.at(t);
        jointVelRight.at(t) = admitRight.at(t-3)*torqueRight.at(t);
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Veclocity for the joints of Torso + Right Arm Configuration : " << std::endl;
    for(int v=0; v < jointVelRight.size() ; v++){
        std::cout << " " << jointVelRight.at(v);
    }
    std::cout << std::endl;
#endif
}

double simplePMPThread::GammaDisc(int _Time)	{
    double t_ramp=(_Time)*RAMP_KONSTANT; //0.0025 to
    double t_init=0.1,z,t_win,t_window,csi,csi_dot,prod1,prod2,Gamma;

    z=(t_ramp-t_init)/t_dur;
    t_win=(t_init+t_dur)-t_ramp;
    if(t_win>0)	{
        t_window=1;
    }
    else	{
        t_window=0;
    }
    csi=(6*pow(z,5))-(15*pow(z,4))+(10*pow(z,3));  //6z^5-15z^4+10z^3
    csi_dot=(30*pow(z,4))-(60*pow(z,3))+(30*pow(z,2)); //csi_dot=30z^4-60z^3+30z^2
    //fprintf(wrL,"\n  %f     %f \t  %f",csi,csi_dot);
    prod1=(1/(1.0001-(csi*t_window)));
    prod2=(csi_dot*0.3333*t_window);
    Gamma=prod1*prod2;
    return Gamma;

}

double simplePMPThread::Gamma(int _Time)	{
    double t_ramp=(_Time)*0.0015; //0.0025 to
    double t_init=0.1,t_dur=3,z,t_win,t_window,csi,csi_dot,prod1,prod2,Gamma;//3

    z=(t_ramp-t_init)/t_dur;
    t_win=(t_init+t_dur)-t_ramp;
    if (t_win>0) {
        t_window=1;
    }
    else  {
        t_window=0;
    }
    csi=(6*pow(z,5))-(15*pow(z,4))+(10*pow(z,3));  //6z^5-15z^4+10z^3
    csi_dot=(30*pow(z,4))-(60*pow(z,3))+(30*pow(z,2)); //csi_dot=30z^4-60z^3+30z^2
    //fprintf(wrL,"\n  %f     %f \t  %f",csi,csi_dot);
    prod1=(1/(1.0001-(csi*t_window)));
    prod2=(csi_dot*0.3333*t_window);
    Gamma=prod1*prod2;
    return Gamma;
}


double simplePMPThread::Gamma1(int _Time1)	{

    double t_ramp1=(_Time1)*0.001;
    double t_init1=0,t_dur1=2,z1,t_win1,t_window1,csi1,csi_dot1,prod11,prod21,Gamma1;

    z1=(t_ramp1-t_init1)/t_dur1;
    t_win1=(t_init1+t_dur1)-t_ramp1;
    if(t_win1>0)	{
        t_window1=1;
    }
    else	{
        t_window1=0;
    }
    csi1=(6*pow(z1,5))-(15*pow(z1,4))+(10*pow(z1,3));  //6z^5-15z^4+10z^3
    csi_dot1=(30*pow(z1,4))-(60*pow(z1,3))+(30*pow(z1,2)); //csi_dot=30z^4-60z^3+30z^2
    prod11=(1/(1.0001-(csi1*t_window1)));
    prod21=(csi_dot1*0.3333*t_window1);
    Gamma1=(prod11*prod21);
    return Gamma1;
}


double simplePMPThread::Gamma_Int(double *ptr,int n)	{

    int k=1;        /* Counters in the algorithm */
    double a=0;
    double h,sum,fk;

    // Simpsons 1/3 rule for Integration

    sum=*(ptr);             /* Initial function value */
    int c=2;
    h=1;                	/*Step Size*/
    while (k <= n-1)  { 	/* Steps through the iteration */
        fk=*(ptr+k);
        c=6-c;       		/* gives the 4,2,4,2,... */
        sum = (sum + c*fk);  /* Adds on the next area */
        k++;         		/* Increases k value by +1 */
    }
    sum=RAMP_KONSTANT*sum/3; 		// changed 0.0025 to

    return sum;

}

double simplePMPThread::Gamma_IntDisc(double *Gar,int n)	{

    int k=1;        /* Counters in the algorithm */
    double a=0;
    double h,sum,fk;

    // Simpsons 1/3 rule for Integration

    sum=*(Gar);             /* Initial function value */
    int c=2;
    h=1;                	/*Step Size*/
    while (k <= n-1)  { 	/* Steps through the iteration */

        fk=*(Gar+k);
        c=6-c;       		/* gives the 4,2,4,2,... */
        sum = (sum + c*fk); /* Adds on the next area */
        k++;         		/* Increases k value by +1 */
    }
    sum=RAMP_KONSTANT*sum/3; // changed 0.0025 to

    return sum;
}

//This is a Single Routine for Correcting Angles of Left/Right Arm
void simplePMPThread::correctJAngles(double *angles){

    for(int j = 0 ; j < 10 ; j++){
        int dummy = fabs(*(angles+j)/360);
        int sign = *(angles+j)/fabs(*(angles+j));
        if(dummy > 0){
            if(sign > 0){//Positive Angle
                *(angles+j) = *(angles+j)-(dummy*360);
            }else{//Negative Angle
                *(angles+j) = *(angles+j)+(dummy*360);
            }

        }
    }
}


void simplePMPThread::grasp(){

    std::cout << "Performing Grasping..." << std::endl;
    if(!yarp::os::Network::isConnected("/icubGazeboSim/left_arm/state:o",leftArmAnglesPort.getName().c_str())){
        yarp::os::Network::connect("/icubGazeboSim/left_arm/state:o",leftArmAnglesPort.getName().c_str());
        yarp::os::Bottle *jAngle;
        jAngle = leftArmAnglesPort.read();
        if(jAngle!= NULL){
#if DEBUG_CODE>0
            std::cout << "Joint Angles Read from Left Arm..." << std::endl;
            std::cout << "Size of the Joint Angles Bottle: " << jAngle->size() << std::endl;
            std::cout << "The Angles read to the Bottle are: " << jAngle->toString().c_str() << std::endl;
#endif

            jAnglesLeftArm.resize(jAngle->size());

#if DEBUG_CODE>0
            std::cout << "Joint Angles size of " << jAnglesLeftArm.size() << " for Left Arm Configuration Set..." << std::endl ;
#endif
            for(int i=0; i < jAngle->size() ; i++){
                jAnglesLeftArm.at(i) = (jAngle->get(i).asDouble())*CTRL_DEG2RAD;
            }
        }
    }else{
        yarp::os::Network::disconnect("/icubGazeboSim/left_arm_no_hand/state:o",leftArmAnglesPort.getName().c_str());
    }


    std::vector<double> cmdGrasp;
    cmdGrasp.assign(16,0); //Commanding only the Left Arm

    /*
    //Left Arm
    cmdGrasp.at(0) = -50.4;
    cmdGrasp.at(1) = 22.4;
    cmdGrasp.at(2) = 59.67;
    cmdGrasp.at(3) = 53.69;

    //Left Wrist
    cmdGrasp.at(4) = 8.4;
    cmdGrasp.at(5) = -7.35;
    cmdGrasp.at(6) = 24.75;

    //Left Hand Fingers
    cmdGrasp.at(7) = jAnglesLeftArm.at(7); //Haven't fixed yet - Hand Finger
    cmdGrasp.at(8) = jAnglesLeftArm.at(8); //Haven't fixed yet -Thumb Oppose
    cmdGrasp.at(9) = 43.2; // Thumb Proximal
    cmdGrasp.at(10) = 39.6; // Thumb Distal
    cmdGrasp.at(11) = jAnglesLeftArm.at(11); // Index Proximal
    cmdGrasp.at(12) = 50.4; // Index Distal
    cmdGrasp.at(13) = jAnglesLeftArm.at(13);//Haven't fixed yet - Middle Proximal
    cmdGrasp.at(14) = jAnglesLeftArm.at(14);//Haven't fixed yet - Middle Distal
    cmdGrasp.at(15) = jAnglesLeftArm.at(15);//Haven't fixed yet - Pinky*/

    //Left Arm
    cmdGrasp.at(0) = -85;
    cmdGrasp.at(1) = 17.6;
    cmdGrasp.at(2) = 79.56;
    cmdGrasp.at(3) = 45.5;

    //Left Wrist
    cmdGrasp.at(4) = 0;
    cmdGrasp.at(5) = 0;
    cmdGrasp.at(6) = 24.75;

    //Left Hand Fingers
    cmdGrasp.at(7) = jAnglesLeftArm.at(7); //Haven't fixed yet - Hand Finger
    cmdGrasp.at(8) = 30.4; //Haven't fixed yet -Thumb Oppose
    cmdGrasp.at(9) = 45.0; // Thumb Proximal
    cmdGrasp.at(10) = 45.0; // Thumb Distal
    cmdGrasp.at(11) = 45.0; // Index Proximal
    cmdGrasp.at(12) = 45.0; // Index Distal
    cmdGrasp.at(13) = 45.0;//Haven't fixed yet - Middle Proximal
    cmdGrasp.at(14) = 45.0;//Haven't fixed yet - Middle Distal
    cmdGrasp.at(15) = 90;//Haven't fixed yet - Pinky

    //This is Left Arm Control
    if(!yarp::os::Network::isConnected(cmdGraspPort.getName().c_str(),"/icubGazeboSim/left_arm/rpc:i")){
        yarp::os::Network::connect(cmdGraspPort.getName().c_str(),"/icubGazeboSim/left_arm/rpc:i");

        for(int j = 0 ; j < cmdGrasp.size() ; j++){

            yarp::os::Bottle& cmdG = cmdGraspPort.prepare();
            cmdG.clear();
            cmdG.addString("set");
            cmdG.addString("pos");
            cmdG.addInt(j);
            cmdG.addDouble(cmdGrasp.at(j));
            cmdGraspPort.writeStrict();
            yarp::os::Time::delay(40);

        }

        yarp::os::Bottle& cmdG = cmdGraspPort.prepare();
        cmdG.clear();
        cmdG.addString("set");
        cmdG.addString("pos");
        cmdG.addInt(0);
        cmdG.addDouble(-77);
        cmdGraspPort.writeStrict();
        yarp::os::Time::delay(10);


    }else{
        yarp::os::Network::disconnect(cmdGraspPort.getName().c_str(),"/icubGazeboSim/left_arm/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdGraspPort.getName().c_str() << " and /icubGazeboSim/left_arm/rpc:i" << std::endl;
#endif
    }

    std::cout << "Grasping Done!" << std::endl;

}


void simplePMPThread::computeEEVelLeft(){


    //Computing End Effector Velocity
    eeVelLeft.assign(3,0);
    for(int v=0 ; v < 3 ; v++){//This loop runs 3 times - X,Y,Z
        for(int i=0; i<10 ; i++){
            eeVelLeft.at(v) = eeVelLeft.at(v) + jacobianLeft.at(i+(v*10)) * jointVelLeft.at(i) ;
        }
    }
#if DEBUG_CODE>0
    std::cout << "Computed End Effector Veclocity (x',y',z') : " << "(" << eeVelLeft.at(0) << "," << eeVelLeft.at(1) << "," << eeVelLeft.at(2) << ")" << std::endl;
#endif
}

void simplePMPThread::run(){

}
