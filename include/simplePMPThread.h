#ifndef SIMPLEPMPTHREAD_H
#define SIMPLEPMPTHREAD_H

#define DEBUG_CODE 0 //1 Debug Mode, 0 Debug Off
#define pi M_PI
#define ITERATION 2000
#define rad2degree          180/3.14159

#include<iostream>
//#include<string>
#include<vector>
#include <fstream>
#include <cmath>

#include<yarp/math/Math.h>
#include<yarp/os/all.h>
#include<yarp/sig/all.h>
#include<yarp/dev/all.h>

#include<synchronizer.hpp>
#include<iCub/iKin/iKinFwd.h>
#include<iCub/ctrl/math.h>


#include<armUtils.h>

using namespace std;
using namespace yarp;


class simplePMPThread:public yarp::os::Thread{
private:
    //iCub Arm Objects for Left and Right Arms using iKin Library
    armUtils* armLeft;
    armUtils* armRight;

    std::string armName;//This should be form the congiuration file
    std::string robot; //name of the robot
    std::string inputPortName; //input port

    double RAMP_KONSTANT, t_dur;
    bool target; //Variable for indicating if the Actual Target is set
    bool vTarget; //Variable for indication Virtual Target as from VTGS is set
    double posErr;

    bool grasped;

    std::ofstream virTarget, curPosition,jointsLoc; //Text Files


    //Gazebo Simulation Sync
    double period; //This is 10ms Default
    double stepSize;

    double kX,kY,kZ; //Stiffness Constants for X,Y,Z

    //Output Ports to send commands to robot ports
    yarp::os::BufferedPort<yarp::os::Bottle > cmdLeftArmPort,cmdRightArmPort,cmdTorsoPort,cmdInterfacePort,cmdEndEffectorPort;
    yarp::os::BufferedPort<yarp::os::Bottle > activationsPort;
    yarp::os::BufferedPort<yarp::os::Bottle> leftArmAnglesPort,rightArmAnglesPort,torsoAnglesPort;
    yarp::os::BufferedPort<yarp::os::Bottle> leftEEPort, rightEEPort; //Port to Connect to the Cartesian Controller
    yarp::os::BufferedPort<yarp::os::Bottle> inputPort, outputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> cmdGraspPort;

    //Joint Angles Variables
    std::vector<double> jAnglesLeftArm; //Left Arm Joint Angles
    std::vector<double> jAnglesRightArm; //Right Arm Joint Angles
    std::vector<double> jAnglesTorso; //Torso Joint Angles

    std::vector<double> jAnglesLT; //Left Arm + Torso Joint Angles
    std::vector<double> jAnglesRT; //Right Arm + Torso Joint Angles

    //Vectors to store the initial joint angles before VTGS
    std::vector<double> initJAnglesLeftArm;
    std::vector<double> initJAnglesRightArm;
    std::vector<double> initJAnglesTorso;

    std::vector<double> initJAnglesLT;
    std::vector<double> initJAnglesRT;

    //Array for Angles
    double leftAngles[10], rightAngles[10]; //These are along with the torso

    //Array for New Angles
    double newAnglesLeft[10];
    double newAnglesRight[10];

    //TODO Change in to more efficient code
    double q1L[ITERATION], q2L[ITERATION], q3L[ITERATION], q4L[ITERATION], q5L[ITERATION];
    double q6L[ITERATION], q7L[ITERATION], q8L[ITERATION], q9L[ITERATION], q10L[ITERATION];

    double q1R[ITERATION], q2R[ITERATION], q3R[ITERATION], q4R[ITERATION], q5R[ITERATION];
    double q6R[ITERATION], q7R[ITERATION], q8R[ITERATION], q9R[ITERATION], q10R[ITERATION];

    //Mean Joint Angles
    double jAnglesMeanLeft[10];
    double jAnglesMeanRight[10];

    //Array EE Position
    double leftPos[3], rightPos[3];

    //Initial End Effector 3D positions - x,y,z
    double initPosLeftEE[3];
    double initPosRightEE[3];

    //Current End Effector 3D positions - x,y,z
    double curPosLeftEE[3];
    double curPosRightEE[3];

    //Virtual Positions 3D - x,y,z
    double virPosLeftEE[3];
    double virPosRightEE[3];

    //tar Position 3D coordinates - x,y,z
    double tarLEE[3];
    double tarREE[3];

    //TODO Modify these variables to be more efficient
    double Gam_ArrxLeft[ITERATION], Gam_ArryLeft[ITERATION], Gam_ArrzLeft[ITERATION];
    double Gam_ArrxRight[ITERATION], Gam_ArryRight[ITERATION], Gam_ArrzRight[ITERATION];
    double initPosLeftEEIC[3], initPosRightEEIC[3];

    //Force Field in Extrinsic Space
    double *forceFieldLeft; //This is a pointer
    double *forceFieldRight;
    double KFORCE;//TODO: try various values
    double KOMP_JANG;
    double KOMP_WAISZT;
    double KOMP_WAISZT2;

    //Joint Limit Admitances
    double J0HL, J1HL, J2HL, J3HL, J4HL, J5HL, J6HL, J7HL, J8HL, J9HL;
    double JHdL[10];

    double J0HR, J1HR, J2HR, J3HR, J4HR, J5HR, J6HR, J7HR, J8HR, J9HR;
    double JHdR[10];


    double jLimitLeft[10];
    double jLimitRight[10];

    //Jacobian Vectors
    std::vector<double> jacobianLeft;
    std::vector<double> jacobianRight;

    //Torque Vectors
    std::vector<double> torqueLeft;
    std::vector<double> torqueRight;

    //Admittance Vectors
    std::vector<double> admitLeft;
    std::vector<double> admitRight;
    std::vector<double> admitTorso;

    //Joint Velocity Vectors
    std::vector<double> jointVelLeft;
    std::vector<double> jointVelRight;

    //End Effector Velocity
    std::vector<double> eeVelLeft;
    std::vector<double> eeVelRight;

public:

    /**
      * constructor default
      */
      simplePMPThread();

      /**
          * constructor
          * @param robotname name of the robot
          */
      simplePMPThread(std::string robotname);

      /**
       * destructor
       */
      ~simplePMPThread();

//      bool threadInit();
//      void threadRelease();
     void run();
//      void onStop();
      void init();
      void readJoints();
      void readEE();//Uses Cartesian Controller for EE Position and Orientation
      void computeEEPos();//Computes the EE Position and Orientation from the joint angles using the forwards kinematics
      void checkEEPos();
      void simpleVTGS();
      void getTarget(); //3D Target Position
      void setTarget();

      //Joint Manual Initialization
      void initializeJoints();

      //Setting Joint Mean Angles
      void setJAnglesMean();

      //Set Joint Admittance Values
      void setJAdmit();

      //These Methods are from Utils.h File
      double forwardKinematicsLeft(double*,int);
      double forwardKinematicsRight(double*,int);

      //External Force Field Computation
      double* computeForceFieldLeft(double *curPos,double *tarPos);
      double* computeForceFieldRight(double *curPos,double *tarPos);//TODO Finish this module

      //Joint Torque Computation
      void computeTorqueLeft(double *leftForce);
      void computeTorqueRight(double *rightForce);

      //Joint Velocity Computation
      void computeJointVelLeft();
      void computeJointVelRight();

      //Joint Velocity to Joint Angle Conversion
      void jVel2AngleLeft(int _time);
      void jVel2AngleRight(int _time);

      //End Effector Velocity Computation
      void computeEEVelLeft();
      void computeEEVelRight();

      //Motor Command Routines
      void cmdJointLeft();
      void cmdJointRight();

      //Correct Joint Angles
      void correctJAngles(double *angles);

      //Grasp Routine for 2R Manipulator
      void grasp();

      //Gazebo Synchronization
      void gzSync();

      double Gamma_Int(double *Gar,int n);
      double Gamma(int _Time); //returns gamma after the time duration
      double Gamma1(int _Time1);
      double GammaDisc(int _Time);
      double Gamma_IntDisc(double *Gar,int n);


};



#endif // SIMPLEPMPTHREAD_H

