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

simplePMPThread::simplePMPThread(std::string _robot){
    robot = _robot;
    //configFile = _configFile;
}

simplePMPThread::~simplePMPThread(){

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
                jAnglesLeftArm.at(i) = (jAngle->get(i).asDouble())/rad2degree;
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

void simplePMPThread::init(){

#if DEBUG_CODE>0
    std::cout << "Initializing PMP Thread..." << std::endl;
#endif

    //Initializing Joint Angle Variables
    //memset(jAnglesLeftArm,0,7*sizeof(double));
    //memset(jAnglesRightArm,0,7*sizeof(double));
    //memset(jAnglesTorso,0,3*sizeof(double));

    /*period = 0.001; //This is 10ms Default

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
    syncObject->configuration.numberOfSteps = period/stepSize;*/


    RAMP_KONSTANT = 0.005;
    t_dur = 5;

    grasped=false;//This for now can be user input

    posErr = 1;
    memset(Gam_Arrx,0,ITERATION*sizeof(double));
    memset(Gam_Arry,0,ITERATION*sizeof(double));
    memset(Gam_Arrz,0,ITERATION*sizeof(double));

    //Joint Angle Variables
    memset(q1,0,ITERATION*sizeof(double));
    memset(q2,0,ITERATION*sizeof(double));
    memset(q3,0,ITERATION*sizeof(double));
    memset(q4,0,ITERATION*sizeof(double));
    memset(q5,0,ITERATION*sizeof(double));
    memset(q6,0,ITERATION*sizeof(double));
    memset(q7,0,ITERATION*sizeof(double));
    memset(q8,0,ITERATION*sizeof(double));
    memset(q9,0,ITERATION*sizeof(double));
    memset(q10,0,ITERATION*sizeof(double));

    //Mean Joint Angles
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

    //Compliances Values from PMP Analytic Module
    KFORCE = 0.01;
    KOMP_JANG = 0.02; //0.02
    KOMP_WAISZT = 0.0001;
    KOMP_WAISZT2 = 0.0001;//0.0001

    //TODO Eperiment with the Stiffness Values - Match with findings from human experiments
    kX=0.01; kY=0.01; kZ=0.01;

    //TODO Not sure what name to be for these
    J0H = 0.041; J1H = 0.52; J2H = 50; J3H = 0.2; J4H = 0.041;
    J5H = 0.041; J6H = 0.041; J7H = 0.041; J8H = 0.041; J9H = 0.041;

    //TODO Not sure what name to be for these
    JHdL[0] = J0H; JHdL[1] = 0.000041; JHdL[2] = J2H; JHdL[3] = 0.041; JHdL[4] = 0.041;
    JHdL[5] = 5; JHdL[6] = 0.041; JHdL[7] = 75; JHdL[8] = J8H; JHdL[9] = J9H;

    //TODO: Change the Admittance Values for each joint later
    admitTorso.assign(3,0);
    admitLeft.assign(7,0);//TODO: Have to change it when using more than 10 joints
    admitRight.assign(7,0);

    //Torso Admittance Values
    admitTorso.at(0) = KOMP_WAISZT;
    admitTorso.at(1) = 0;
    admitTorso.at(2) = KOMP_WAISZT2;

    //Left Arm Admittance Values
    admitLeft.at(0) = 0.09;//0.09
    admitLeft.at(1) = 0.09;
    admitLeft.at(2) = 0.09;
    admitLeft.at(3) = 0.09;
    admitLeft.at(4) = 0.09;
    admitLeft.at(5) = 0.09;
    admitLeft.at(6) = 0.09;

    virTarget.open("target.txt");
    curPosition.open("position.txt");
    jointsLoc.open("joints.txt");

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

    if(!leftArmAnglesPort.open("/leftArmAnglePort:i")){
#if DEBUG_CODE>0
        std::cout << "Left Arm Angle Port Created : " << leftArmAnglesPort.getName().c_str() << std::endl;
#endif
    }

    if(!rightArmAnglesPort.open("/rightArmAnglePort:i")){
#if DEBUG_CODE>0
        std::cout << "Right Arm Angle Port Created : " << rightArmAnglesPort.getName().c_str() << std::endl;
#endif
    }

    if(!torsoAnglesPort.open("/torsoAnglePort:i")){
#if DEBUG_CODE>0
        std::cout << "Torso Angle Port Created : " << torsoAnglesPort.getName().c_str() << std::endl;
#endif
    }

    if(!leftEEPort.open("/leftEEPort:i")){
#if DEBUG_CODE>0
        std::cout << "Left End Effector Port Created : " << leftEEPort.getName().c_str() << std::endl;
#endif
    }

    if(!rightEEPort.open("/rightEEPort:i")){
#if DEBUG_CODE>0
        std::cout << "Right End Effector Port Created : " << rightEEPort.getName().c_str() << std::endl;
#endif
    }

    if(!cmdTorsoPort.open("/cmdTorso:o")){
#if DEBUG_CODE>0
        std::cout << "Torso Output Commanded Position Port Created : " << cmdTorsoPort.getName().c_str() << std::endl;
#endif
    }

    if(!cmdLeftArmPort.open("/cmdLeftArm:o")){
#if DEBUG_CODE>0
        std::cout << "Left Arm Output Commanded Position Port Created : " << cmdLeftArmPort.getName().c_str() << std::endl;
#endif
    }

    if(!cmdRightArmPort.open("/cmdRightArm:o")){
#if DEBUG_CODE>0
        std::cout << "Right Arm Output Commanded Position Port Created : " << cmdRightArmPort.getName().c_str() << std::endl;
#endif
    }

    if(!cmdGraspPort.open("/cmdGrap:o")){
#if DEBUG_CODE>0
        std::cout << "Grasp Commanded Port Created : " << cmdGraspPort.getName().c_str() << std::endl;
#endif
    }



    grasp();//Calling the Grap Routine
    yarp::os::Time::delay(2); //This is to show difference between Grasping and PMP Module
    //getTarget(); //Get the target, TODO: Probably can also specify which arm to use
    //setTarget();
    //readJoints();//TODO Decide which arm to move
    //initializeJointsLeft();
    //initializeJointRight();
    //readEE(); //This method uses cartesian controller for getting the end effector positions
    //computeEEPos(); //Computing EE values using Forwards Kinematics
    //simpleVTGS();

}

void simplePMPThread::initializeJointsLeft(){

    std::cout << "Initializing Torso + Left Arm Joint Configuration : " << endl;
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


}

void simplePMPThread::initializeJointRight(){

    //std::cout << "Initializing Torso + Right Arm Joint Configuration : " << endl;
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

    //for(int i=0; i < jAnglesRT.size() ; i++){
        //std::cout << jAnglesRT.at(i) << " ";
    //}std::cout << std::endl;


}


void simplePMPThread::getTarget(){ //TODO Make this reply with OK command

    yarp::os::Bottle *tar = inputPort.read();//TODO Have to make it like a Blocking Call - Like wait until a target is given
    if(tar!=NULL && tar->size() ==3){
#if DEBUG_CODE>0
        std::cout << "Acquired Target Position Correctly..." << std::endl;
#endif
        goalLEE[0] = tar->get(0).asDouble();
        goalLEE[1] = tar->get(1).asDouble();
        goalLEE[2] = tar->get(2).asDouble();
        target = true;
        std::cout << "Acquired Target (x,y,z) : " << "(" << goalLEE[0] << "," << goalLEE[1] << "," << goalLEE[2] << ")" << std::endl;
    }else{
        target = false;
#if DEBUG_CODE>0
        std::cout << "Error in Target Position!!!" << std::endl;
#endif
    }

}

void simplePMPThread::setTarget(){
#if DEBUG_CODE>0
        std::cout << "Using hard coded Goal Targets..." << std::endl;
#endif

    goalLEE[0] = 300; //y direction positive towards left of icub 1000//-1000
    goalLEE[1] = -300;// x direction  negative front wards -1000
    goalLEE[2] = 300;//z upwards postive - This is Changing X Position? -1000
    target = true;

#if DEBUG_CODE>0
    std::cout << "Acquired Target (x,y,z) : " << "(" << goalLEE[0] << "," << goalLEE[1] << "," << goalLEE[2] << ")" << std::endl;
#endif
}

void simplePMPThread::readJoints(){

    //NOTE: Updated Gazebo Model gives 16 joints values
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
                jAnglesLeftArm.at(i) = (jAngle->get(i).asDouble())/rad2degree;
            }
        }
    }else{
        yarp::os::Network::disconnect("/icubGazeboSim/left_arm_no_hand/state:o",leftArmAnglesPort.getName().c_str());
    }

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
                jAnglesRightArm.at(i) = (jAngle->get(i).asDouble())/rad2degree;
            }
        }
    }else{
        yarp::os::Network::disconnect("/icubGazeboSim/right_arm_no_hand/state:o",rightArmAnglesPort.getName().c_str());
    }

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
                jAnglesTorso.at(i) = (jAngle->get(i).asDouble())/rad2degree;
            }
        }
    }else{
        yarp::os::Network::disconnect("/icubGazeboSim/torso/state:o",torsoAnglesPort.getName().c_str());
    }

    //Setting the Joint Angles for Torso + Left Arm Configurations
    jAnglesLT.resize(jAnglesTorso.size() + jAnglesLeftArm.size());
    jAnglesLT = jAnglesTorso;
    jAnglesLT.insert(jAnglesLT.end(),jAnglesLeftArm.begin(),jAnglesLeftArm.end()); //NOTE: This now has 16 arm DOF Values + 3 Torso Values

    //Setting the Joint Angles for Torso + Right Arm Configurations
    jAnglesRT.resize(jAnglesTorso.size() + jAnglesRightArm.size());
    jAnglesRT = jAnglesTorso;
    jAnglesRT.insert(jAnglesRT.end(),jAnglesRightArm.begin(),jAnglesRightArm.end());

#if DEBUG_CODE>0
    std::cout << "Joint Angles size of " << jAnglesLT.size() << " for Torso + Left Arm Configuration is :";
    for(int a=0 ; a < jAnglesLT.size(); a++){
        std::cout << " " << jAnglesLT.at(a);
    }
    std::cout << std::endl;

    std::cout << "Joint Angles size of " << jAnglesLT.size() << " for Torso + Right Arm Configuration is :";
    for(int a=0 ; a < jAnglesRT.size(); a++){
        std::cout << " " << jAnglesRT.at(a);
    }
    std::cout << std::endl;
#endif

}

/*void simplePMPThread::readEE(){

#if DEBUG_CODE>0
            std::cout << "Reading End Effector Position and Orientation using Cartesian Control Interface" << std::endl;
#endif

    //Getting the EE Position in the Cartesian Space
    //left Hand EE Coordinates
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

    //Right Hand EE Coordinates
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

}*/

void simplePMPThread::checkEEPos(){

    //std::cout << "Corrected Joint Angles : " << " " <<  jAnglesLT.at(0) << " " << jAnglesLT.at(1) << " " << jAnglesLT.at(2) //
      //        << " " << jAnglesLT.at(3) << " " << jAnglesLT.at(4) << " " << jAnglesLT.at(5) << " " << jAnglesLT.at(6) //
        //      << " " << jAnglesLT.at(7) << " " << jAnglesLT.at(8) << " " <<  jAnglesLT.at(9) << std::endl;

    //Left EE
    std::cout << "Size of Computed Joint Angle Commands : " << jAnglesLT.size() << std::endl;
    for(int a=0; a < jAnglesLT.size() ; a++){
        leftAngles[a] = jAnglesLT.at(a);
    }
    computeFKLeft(leftPos,leftAngles);

    std::cout << "Values of the Left EE Position to be attained (x,y,z) : " << "(" << leftPos[0] << "," << leftPos[1] << "," << leftPos[2] << ")" << std::endl;

}

void simplePMPThread::computeEEPos(){

#if DEBUG_CODE>0
            std::cout << "Computing End Effector Position and Orientation using Forward Kinematics of iCub Robot" << std::endl;
#endif

            //Left EE
#if DEBUG_CODE>1
            std::cout << "Computing Forward Kinematics of Left Arm + Torso Configuration..." << std::endl;
#endif
            for(int a=0; a < jAnglesLT.size() ; a++){
                leftAngles[a] = jAnglesLT.at(a);
            }

#if DEBUG_CODE>1
            std::cout << "Size of Joint Angles Received : " << sizeof(leftAngles)/sizeof(double) << std::endl;
#endif
            computeFKLeft(leftPos,leftAngles);

#if DEBUG_CODE>0
            std::cout << "Values of the Left EE Position (x,y,z) : " << "(" << leftPos[0] << "," << leftPos[1] << "," << leftPos[2] << ")" << std::endl;
#endif
            //Right EE

#if DEBUG_CODE>0
            std::cout << "Computing Forward Kinematics of Right Arm + Torso Configuration..." << std::endl;
#endif

            for(int a=0; a < jAnglesRT.size() ; a++){
                rightAngles[a] = jAnglesRT.at(a);
            }

#if DEBUG_CODE>0
            std::cout << "Size of Joint Angles Received : " << sizeof(rightAngles)/sizeof(double) << std::endl;
#endif
            computeFKRight(rightPos,rightAngles);
#if DEBUG_CODE>0
            std::cout << "Values of the Right EE Position (x,y,z) : " << "(" << rightPos[0] << "," << rightPos[1] << "," << rightPos[2] << ")" << std::endl;
#endif
            //if(target==true && vTarget==true){//If a target is not set and a virtual target is not generated use this for initialization

                //std::cout << "Setting Initial End Effector Position..." << std::endl;
                curPosLeftEE[0] = leftPos[0];//X Value
                curPosLeftEE[1] = leftPos[1];//Y Value
                curPosLeftEE[2] = leftPos[2];//Z Value
                //std::cout << "New Left End Effector Position (x,y,z) : " << "(" << leftPos[0] << "," << leftPos[1] << "," << leftPos[2] << ")" << std::endl;

                curPosRightEE[0] = rightPos[0];//X Value
                curPosRightEE[1] = rightPos[1];//Y Value
                curPosRightEE[2] = rightPos[2];//Z Value


            /*}else{//This is while running for virtual targets

                //std::cout << "Setting Current End Effector Position..." << std::endl;
                curPosLeftEE[0] = leftPos[0];//X Value
                curPosLeftEE[1] = leftPos[1];//Y Value
                curPosLeftEE[2] = leftPos[2];//Z Value

                curPosRightEE[0] = rightPos[0];//X Value
                curPosRightEE[1] = rightPos[1];//Y Value
                curPosRightEE[2] = rightPos[2];//Z Value

            }*/

}


void simplePMPThread::simpleVTGS(){//This depends on the number of intermediate points we need

//#if DEBUG_CODE>0
    std::cout << "Running Virtual Trajectory Generation System for the Acquired Target Position..." << std::endl;
//#endif

    readJoints();
    //initializeJointsLeft();
    //initializeJointRight();
    computeEEPos();

    initPosLeftEE[0] = curPosLeftEE[0]; //-48;
    initPosLeftEE[1] = curPosLeftEE[1];//227;
    initPosLeftEE[2] = curPosLeftEE[2]; //411;

    initPosRightEE[0] = curPosRightEE[0]; //-48;
    initPosRightEE[1] = curPosRightEE[1];//227;
    initPosRightEE[2] = curPosRightEE[2]; //411;

//#if DEBUG_CODE>0
    std::cout << "Initial Left EE Position (x,y,z) : " << "(" << initPosLeftEE[0] << "," << initPosLeftEE[1] << "," << initPosLeftEE[2] << ")" << std::endl;
    std::cout << "Final Left Goal (x,y,z) : " << "(" << goalLEE[0] << "," << goalLEE[1] << "," << goalLEE[2] << ")" << std::endl;
//#endif

    int time;
    double Gam; //What is this variable ? Value of Gamma after a time step
    double xoff=1, yoff=1 , zoff=1; //Offsets

    //Storing the Initial Values of End Effector
    initPosLeftEEIC[0] = initPosLeftEE[0];
    initPosLeftEEIC[1] = initPosLeftEE[1];
    initPosLeftEEIC[2] = initPosLeftEE[2];

    //Storing the Initial Values of Joint Angles
    initJAnglesLT.assign(jAnglesLT.size(),0);
    initJAnglesRT.assign(jAnglesRT.size(),0);

    for(int j = 0 ; j < jAnglesLT.size() ; j++){
        initJAnglesLT.at(j) = jAnglesLT.at(j);
    }

    for(int j = 0 ; j < jAnglesRT.size() ; j++){
        initJAnglesRT.at(j) = jAnglesRT.at(j);
    }

    //TODO Use offset values for the goal

    for(time=0; time<ITERATION ; time++){// 2000 incremental steps of delta(?) 0.005

        Gam = GammaDisc(time);

        //std::cout << "Initial Target Position (x,y,z) : " << "(" << initPosLeftEE[0] << "," << initPosLeftEE[1] << "," << initPosLeftEE[2] << ")" << std::endl;

        //X Value
        virPosLeftEE[0] = (goalLEE[0]-initPosLeftEE[0])*Gam;
        Gam_Arrx[time] = virPosLeftEE[0];
        double *Garx = Gam_Arrx;//This is just a pointer
        virPosLeftEE[0] = Gamma_Int(Garx,time) + initPosLeftEEIC[0];
        initPosLeftEE[0] = virPosLeftEE[0];

        //Y Value
        virPosLeftEE[1] = (goalLEE[1]-initPosLeftEE[1])*Gam;
        Gam_Arry[time] = virPosLeftEE[1];
        double *Gary = Gam_Arry;
        virPosLeftEE[1] = Gamma_Int(Gary,time) + initPosLeftEEIC[1];
        initPosLeftEE[1] = virPosLeftEE[1];

        //Z Value
        virPosLeftEE[2] = (goalLEE[2]-initPosLeftEE[2])*Gam;
        Gam_Arrz[time] = virPosLeftEE[2];
        double *Garz = Gam_Arrz;
        virPosLeftEE[2] = Gamma_Int(Garz,time) + initPosLeftEEIC[2];
        initPosLeftEE[2] = virPosLeftEE[2];

        virTarget  << initPosLeftEE[0] << "		" << initPosLeftEE[1] << "		" << initPosLeftEE[2] <<endl;
        computeEEPos(); //Computing the End Effector Positions
        curPosition  << curPosLeftEE[0] << "		" << curPosLeftEE[1] << "		" << curPosLeftEE[2] <<endl;

        //std::cout << "Virtual Target Position (x,y,z) : " << "(" << virPosLeftEE[0] << "," << virPosLeftEE[1] << "," << virPosLeftEE[2] << ")" << std::endl;
        virTarget  << initPosLeftEE[0] << "		" << initPosLeftEE[1] << "		" << initPosLeftEE[2] <<endl;

        //TODO: This is where the motor command should go - which is computing the rest of PMP
        //TODO Get the current EE Position

        forceField = computeForceFieldLeft(curPosLeftEE,initPosLeftEE);//here initPosLeftEE is actually the virtual target
        computeTorqueLeft(forceField);
        computeJointVelLeft();
        jVelAngleLeft(time);

        //Storing The Joing Angles to the File
        for(int j = 0 ; j < jAnglesLT.size() ; j++){
                    jointsLoc << jAnglesLT.at(j) <<"		";}
                jointsLoc<<endl;

        //computeEEVelLeft();
        //std::cout << "Computed End Effector Veclocity (x',y',z') : " << "(" << eeVelLeft.at(0) << "," << eeVelLeft.at(1) << "," << eeVelLeft.at(2) << ")" << std::endl;
        //TODO Put this condition after the execution of the motor command and after reading the EE positions

        if( ( fabs((virPosLeftEE[0]-goalLEE[0])) < posErr ) && ( fabs((virPosLeftEE[1]-goalLEE[1])) < posErr ) && ( fabs((virPosLeftEE[2]-goalLEE[2])) < posErr ) ){
//#if DEBUG_CODE>0
    std::cout << "Virtual Target(x,y,z) Position reached Goal Position approximately : ";
    std::cout << "(" << virPosLeftEE[0] << "," << virPosLeftEE[1] << "," << virPosLeftEE[2] << ")" << std::endl;

//#endif
            break;
        }else{
            vTarget = true; //Virtual Target is Valid
        }

    }

    virTarget.close();
    curPosition.close();
    cmdJointLeft();
    checkEEPos();

}

void simplePMPThread::cmdJointLeft(){

    std::cout << "Received Joint Angles, Commanding the Motors..." << std::endl;
    std::cout << "Corrected Joint Angles in Degrees : ";
    double cmdJLeft[jAnglesLT.size()];
    for(int j = 0; j < jAnglesLT.size() ; j++){
        cmdJLeft[j] = jAnglesLT.at(j)*rad2degree;
        std::cout << " " << cmdJLeft[j];
    }std::cout<<std::endl;

    correctAnglesLeft(cmdJLeft);

#if DEBUG_CODE>1
    std::cout << "Received Joint Angles, Commanding the Motors..." << std::endl;
#endif


    //This is Torso Control
    if(!yarp::os::Network::isConnected(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i")){
        yarp::os::Network::connect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
        for(int j = 0 ; j < 3 ; j++){//TODO Here only 3 joints are controlled
            yarp::os::Bottle& cmdT = cmdTorsoPort.prepare();
            cmdT.clear();
            cmdT.addString("set");
            cmdT.addString("pos");
            cmdT.addInt(j);
            cmdT.addDouble(cmdJLeft[j]);
            //std::cout << "j value : " << j << " and Angle value : " << jAnglesLT.at(j) << std::endl;
            cmdTorsoPort.writeStrict();
            //syncObject->clockServer.stepSimulation(syncObject->configuration.numberOfSteps);
            //syncObject->clockServer.continueSimulation();
        }


    }else{
        yarp::os::Network::disconnect(cmdTorsoPort.getName().c_str(),"/icubGazeboSim/torso/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdTorsoPort.getName().c_str() << " and /icubGazeboSim/torso/rpc:i"<< std::endl;
#endif
    }

    //This is Left Arm Control
    if(!yarp::os::Network::isConnected(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i")){
        yarp::os::Network::connect(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i");
        for(int j = 3 ; j < 10 ; j++){//TODO Here only 7 joints are controlled
            yarp::os::Bottle& cmdLA = cmdLeftArmPort.prepare();
            cmdLA.clear();
            cmdLA.addString("set");
            cmdLA.addString("pos");
            cmdLA.addInt(j-3);
            cmdLA.addDouble(jAnglesLT.at(j));
            //std::cout << "j value : " << j-3 << " and Angle value : " << jAnglesLT.at(j) << std::endl;
            cmdLeftArmPort.writeStrict();
            yarp::os::Time::delay(1);
        }


    }else{
        yarp::os::Network::disconnect(cmdLeftArmPort.getName().c_str(),"/icubGazeboSim/left_arm_no_hand/rpc:i");
#if DEBUG_CODE>0
        std::cout << " Disconnected " << cmdLeftArmPort.getName().c_str() << " and /icubGazeboSim/left_arm_no_hand/rpc:i" << std::endl;
#endif
    }

}

void simplePMPThread::jVelAngleLeft(int _time){


    q1[_time] = jointVelLeft.at(0);
    double *j1=q1;
    double joi1 = Gamma_Int(j1,_time);
    jAnglesLT.at(0) = joi1 + initJAnglesLT.at(0);

    q2[_time] = jointVelLeft.at(1);
    double *j2=q2;
    double joi2 = Gamma_Int(j2,_time);
    jAnglesLT.at(1) = joi2 + initJAnglesLT.at(1);

    q3[_time] = jointVelLeft.at(2);
    double *j3=q3;
    double joi3 = Gamma_Int(j3,_time);
    jAnglesLT.at(2) = joi3 + initJAnglesLT.at(2);

    q4[_time] = jointVelLeft.at(3);
    double *j4=q4;
    double joi4 = Gamma_Int(j4,_time);
    jAnglesLT.at(3) = joi4 + initJAnglesLT.at(3);

    q5[_time] = jointVelLeft.at(4);
    double *j5=q5;
    double joi5 = Gamma_Int(j5,_time);
    jAnglesLT.at(4) = joi5 + initJAnglesLT.at(4);

    q6[_time] = jointVelLeft.at(5);
    double *j6=q6;
    double joi6 = Gamma_Int(j6,_time);
    jAnglesLT.at(5) = joi6 + initJAnglesLT.at(5);

    q7[_time] = jointVelLeft.at(6);
    double *j7=q7;
    double joi7 = Gamma_Int(j7,_time);
    jAnglesLT.at(6) = joi7 + initJAnglesLT.at(6);

    q8[_time] = jointVelLeft.at(7);
    double *j8=q8;
    double joi8 = Gamma_Int(j8,_time);
    jAnglesLT.at(7) = joi8 + initJAnglesLT.at(7);

    q9[_time] = jointVelLeft.at(8);
    double *j9=q9;
    double joi9 = Gamma_Int(j9,_time);
    jAnglesLT.at(8) = joi9 + initJAnglesLT.at(8);

    q10[_time] = jointVelLeft.at(9);
    double *j10=q10;
    double joi10 = Gamma_Int(j10,_time);
    jAnglesLT.at(9) = joi10 + initJAnglesLT.at(9);

#if DEBUG_CODE>0
    std::cout << "Computed New Joint Angles : " << " " <<  jAnglesLT.at(0) << " " << jAnglesLT.at(1) << " " << jAnglesLT.at(2) //
              << " " << jAnglesLT.at(3) << " " << jAnglesLT.at(4) << " " << jAnglesLT.at(5) << " " << jAnglesLT.at(6) //
              << " " << jAnglesLT.at(7) << " " << jAnglesLT.at(8) << " " <<  jAnglesLT.at(9) << std::endl;
#endif



#if DEBUG_CODE>0
    //std::cout << "Corrected New Joint Angles : " << " " <<  jAnglesLT.at(0) << " " << jAnglesLT.at(1) << " " << jAnglesLT.at(2) //
    //          << " " << jAnglesLT.at(3) << " " << jAnglesLT.at(4) << " " << jAnglesLT.at(5) << " " << jAnglesLT.at(6) //
    //          << " " << jAnglesLT.at(7) << " " << jAnglesLT.at(8) << " " <<  jAnglesLT.at(9) << std::endl;
#endif
}

void simplePMPThread::correctAnglesLeft(double *angles){

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

void simplePMPThread::computeJointVelLeft(){

    jointVelLeft.assign(10,0);//TODO: Have to change it when using more than 10 joints
    for(int t=0; t < 3 ; t++){
        jointVelLeft.at(t) = admitTorso.at(t)*torqueLeft.at(t);
    }

    for(int t=0; t < 7 ; t++){
        jointVelLeft.at(t) = KOMP_JANG*torqueLeft.at(t);
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Veclocity for the joints of Torso + Left Arm Configuration : " << std::endl;
    for(int v=0; v < jointVelLeft.size() ; v++){
        std::cout << " " << jointVelLeft.at(v);
    }
    std::cout << std::endl;
#endif


}

double* simplePMPThread::computeForceFieldLeft(double *curPos, double *tarPos){

#if DEBUG_CODE>0
    std::cout << "Computed Force Field for Left End Effector..." << std::endl;
#endif

    //std::cout << "Present Position (x,y,z) : "  << "(" << *(curPos+0) << "," << *(curPos+1) << "," << *(curPos+2) << ")" ;
    //std::cout << "Target Position (x,y,x) : " << "(" << *(tarPos+0) << "," << *(tarPos+1) << "," << *(tarPos+2) << ")" << std::endl;
    static double res[3];

    res[0] = 0.09*(*(tarPos+0)- *(curPos+0)); //0.09
    res[1] = 0.09*((*(tarPos+1)- *(curPos+1))); //0.09
    res[2] = 0.07*((*(tarPos+2)- *(curPos+2))); //0.07
#if DEBUG_CODE>0
    std::cout << "Computed Force (Fx,Fy,Fz) : " << "(" << res[0] << "," << res[1] << "," << res[2] << ")" << std::endl;
#endif
    return res;
}

void simplePMPThread::computeTorqueLeft(double *leftForce){

    //Local Variables
    double ff[3];
    static double jac[30];//TODO Change it to vectors better
    torqueLeft.assign(10,0); //TODO: Have to change it when using more than 10 joints
    //Assigning force to local variables
    ff[0] = *(leftForce+0);
    ff[1] = *(leftForce+1);
    ff[2] = *(leftForce+2);

    jacobianLeft = computeJacobianLeft(jAnglesLT);//Note Sending Joint Angles of Torso(3)+Left Arm(16)
#if DEBUG_CODE>0
    std::cout << "Computed Jacobian of size " << jacobianLeft.size() <<  " for Left Arm + Torso Configuration : ";
    for(int j=0; j < jacobianLeft.size() ; j++){
        std::cout << " " << jacobianLeft.at(j);
    }
    std::cout << std::endl;
#endif

    //Computing the Joint Limit Force Field
    for(int i = 0; i < 10 ; i++){
        jLimiteFFLeft[i] = (jAnglesMeanLeft[i] - jAnglesLT.at(i)) * JHdL[i];
    }

#if DEBUG_CODE>0
    std::cout << "Computed Joint Limit Force Field : " << jLimiteFFLeft[0] << " " << jLimiteFFLeft[1] << " " //
              << jLimiteFFLeft[2] << " " << jLimiteFFLeft[3] << " " << jLimiteFFLeft[4] << " " << jLimiteFFLeft[5] //
              << " " << jLimiteFFLeft[6] << " " << jLimiteFFLeft[7] << " " << jLimiteFFLeft[8] << " " << jLimiteFFLeft[9] << std::endl;
#endif

    //Computing Torque from Force Field Using Jacobian Transpose
    for(int p=0 ; p < (jacobianLeft.size()/3) ; p++){
        torqueLeft.at(p) = jacobianLeft.at(p)*ff[0] + jacobianLeft.at(p+10)*ff[1] //
                + jacobianLeft.at(p+20)*ff[2] + jLimiteFFLeft[p];
    }

#if DEBUG_CODE>0
    std::cout << "Computed Torques of size " << torqueLeft.size() <<  " for Left Arm + Torso Joints : ";
    for(int t=0; t < torqueLeft.size() ; t++){
        std::cout << " " << torqueLeft.at(t);
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

};

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

};


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
};


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
};

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
};



void simplePMPThread::run(){

}
