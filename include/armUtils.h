#ifndef ARMUTILS_H
#define ARMUTILS_H

#include<cmath>
#include<vector>
#include<yarp/sig/Matrix.h>
#include<yarp/sig/Vector.h>
#include<yarp/math/Math.h>

#include<iCub/iKin/iKinFwd.h>
#include<iCub/ctrl/math.h>

#define version 2.0

using namespace std;
using namespace iCub;
using namespace iCub::iKin;
using namespace iCub::ctrl;

class armiCub   :   public  iCub::iKin::iKinLimb{

public:
    armiCub(const std::string _arm):iKinLimb(){
        allocate(_arm);
    }

protected:
    virtual void allocate(const string &_type){

    yarp::sig::Matrix H0(4,4);
    H0.zero();
    H0(0,1)=-1.0;
    H0(1,2)=-1.0;
    H0(2,0)=1.0;
    H0(3,3)=1.0;
    setH0(H0);

    if(_type=="right"){

        std::cout << "Allocating Links for the chain of Torso + " << _type.c_str() << " Arm Configuration..."<<std::endl;
        //Torso Links
        pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));

        //Right Arm Links
        pushLink(new iKinLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
        pushLink(new iKinLink(    -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));

        if (version<1.7)//This is for V1
            pushLink(new iKinLink(       0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
        else//This is for V1.7 & V2
            pushLink(new iKinLink(       0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));

        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));

        if (version<2.0)//This is for V1 & V1.7
            pushLink(new iKinLink(    0.0625,    0.016,       0.0,                M_PI, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        else//This is for V2
            pushLink(new iKinLink(    0.0625,  0.02598,       0.0,                M_PI, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));

    }else if(_type=="left"){

        std::cout << "Allocating Links for the chain of Torso + " << _type.c_str() << " Arm Configuration..."<<std::endl;
        //Torso Links
        pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));

        //Left Arm Links
        pushLink(new iKinLink(       0.0,  0.10774, -M_PI/2.0,            M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    -0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));

        if (version<1.7)//This is for V1
            pushLink(new iKinLink(       0.0,   0.1373,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
        else//This is for V1.7 & V2
            pushLink(new iKinLink(       0.0,   0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));

        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));

        if (version<2.0)//Thi sis for V1 & V1.7
            pushLink(new iKinLink(    0.0625,   -0.016,       0.0,                 0.0, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        else//This is for V2
        pushLink(new iKinLink(    0.0625, -0.02598,       0.0,                 0.0, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));

    }else{//For both Left and Right Arm Use

    }
    }


};

class armUtils{

private:
    std::string armType;
    iKinChain *chain;
    armiCub *arm;
    yarp::sig::Vector q;
    int nDof;

public:

    armUtils(const std::string _arm);
    ~armUtils();
    yarp::sig::Vector getEEPos(std::vector<double> qJoints);
    std::vector<double> getJacobian();

};

#endif
