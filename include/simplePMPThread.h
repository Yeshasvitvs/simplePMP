#ifndef SIMPLEPMPTHREAD_H
#define SIMPLEPMPTHREAD_H

#define DEBUG_CODE 0 //1 Debug Mode, 0 Debug Off
#define pi M_PI
#define ITERATION 2000
#define rad2degree          180/3.14159

#include<yarp/os/all.h>
#include<yarp/sig/all.h>
#include<yarp/dev/all.h>
#include<iostream>
#include<string>
#include<vector>
#include <fstream>
#include <cmath>

#include<synchronizer.hpp>



class simplePMPThread:public yarp::os::Thread{
private:


    std::string robot; //name of the robot
    std::string inputPortName; //input port

    double RAMP_KONSTANT, t_dur;
    bool target; //Variable for indicating if the Actual Target is set
    bool vTarget; //Variable for indication Virtual Target as from VTGS is set
    double posErr;

    bool grasped;

    std::ofstream virTarget, curPosition,jointsLoc;


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
    double q1[ITERATION], q2[ITERATION], q3[ITERATION], q4[ITERATION], q5[ITERATION];
    double q6[ITERATION], q7[ITERATION], q8[ITERATION], q9[ITERATION], q10[ITERATION];

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

    //Goal Position 3D coordinates - x,y,z
    double goalLEE[3];
    double goalREE[3];

    //TODO Modify these variables to be more efficient
    double Gam_Arrx[ITERATION], Gam_Arry[ITERATION], Gam_Arrz[ITERATION];
    double initPosLeftEEIC[3], initPosRightEEIC[3];

    //Force Field in Extrinsic Space
    double *forceField; //This is a pointer
    double KFORCE;//TODO: try various values
    double KOMP_JANG;
    double KOMP_WAISZT;
    double KOMP_WAISZT2;

    //Joint Limit Admitances
    double J0H, J1H, J2H, J3H, J4H, J5H, J6H, J7H, J8H, J9H;
    double JHdL[10];
    double jLimiteFFLeft[10];

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

      void initializeJointsLeft();
      void initializeJointRight();

      double forwardKinematicsLeft(double*,int);
      double forwardKinematicsRight(double*,int);
      double* computeForceFieldLeft(double *curPos,double *tarPos);
      double* computeForceFieldRight(double *,double*);//TODO Finish this module

      void computeTorqueLeft(double *leftForce);
      void computeTorqueRight(double *rightForce);

      void computeJointVelLeft();
      void computeJointVelRight();

      //Convert Joint Velocity to Joint Angle
      void jVelAngleLeft(int _time);
      void jVelAngleRight();

      void computeEEVelLeft();
      void computeEEVelRight();

      //Motor Command Routines
      void cmdJointLeft();
      void cmdJointRight();

      //Correct Joint Angles
      void correctAnglesLeft(double *angles);

      //Grasp Routine for 2R Manipulator
      void grasp();

      double Gamma_Int(double *Gar,int n);
      double Gamma(int _Time); //returns gamma after the time duration
      double Gamma1(int _Time1);
      double GammaDisc(int _Time);
      double Gamma_IntDisc(double *Gar,int n);


};



#endif // SIMPLEPMPTHREAD_H

