#include<armUtils.h>

armUtils::armUtils(const std::string _arm){

    //std::cout << "iCub Arm Utils Constructor..." << std::endl;
    arm = new armiCub(_arm);
    chain = arm->asChain();
    armType = arm->getType() ;

    //Releasing Torso Links
    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    nDof = chain->getDOF();

    //std::cout << "DoFs Available for the " << armType.c_str() << " Arm + Torso Configuration : " << nDof << std::endl;

    q.resize(nDof);

}

armUtils::~armUtils(){

}

yarp::sig::Vector armUtils::getEEPos(std::vector<double> qJoints){

    //std::cout << "Computing EE Position from Forward Kinematics of iKin Library..."<<std::endl;

    //TODO Condition to check if the DOFs and the joint angles received are the same
    for(int i=0 ; i<qJoints.size(); i++){
        q[i] = qJoints.at(i);//This is Double
        //std::cout << q[i] << " " ;
    }
    //std::cout << "Size of the Joint Angles Received : " << q.size() << std::endl;
    std::cout << "The Current Joint Configuration is : " << (q).toString().c_str() << std::endl;
    chain->setAng(q);
    yarp::sig::Vector eePos = chain->EndEffPosition();
    //std::cout << armType.c_str() << " Arm End Effector Position (x,y,z) : " << "(" << eePos[0] << "," << eePos[1] << "," << eePos[2] << ")" << std::endl;
    return eePos;
}
std::vector<double> armUtils::getJacobian(){

    std::vector<double> jacobian;
    jacobian.assign(30,0);
    std::cout << "Computing Jacobian with the Current Joint Configuration as : " << (q).toString().c_str() << std::endl;
    yarp::sig::Matrix armJacobian = chain->AnaJacobian();
    int rows = armJacobian.rows();
    int cols = armJacobian.cols();
    for(int r = 0 ; r < rows ; r++){
        for(int c = 0 ; c < cols ; c++){
            jacobian.at((r*10)+c) = armJacobian(r,c);
        }
    }
    return jacobian;
}
