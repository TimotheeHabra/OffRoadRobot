/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <RobotranMotionControlBoard.h>


using namespace yarp::dev;


/**
 * robotran stuff
 */
bool RobotranYarpMotionControl::robotran_init()
{
    std::cout << "robotran_init " << std::endl;
    return true;
}

void RobotranYarpMotionControl::onUpdate(const MBSdataStruct * MBSdata)
{

}

/////////////////////////////////////
// DEVICE DRIVER
/////////////////////////////////////

RobotranYarpMotionControl::RobotranYarpMotionControl()
{

}

RobotranYarpMotionControl::~RobotranYarpMotionControl()
{

}

bool RobotranYarpMotionControl::open(yarp::os::Searchable& config)
{
    std::cout << "robotran motionControl parameters are " << config.toString() << std::endl;
    config.check("joint");

    config.find("joint").isInt();

    int joint = config.find("joint").asInt();

    std::cout << "joint is " << joint << std::endl;
    return true;
}

bool RobotranYarpMotionControl::close()
{
    std::cout << "closing robotran motionControl " << std::endl;
    return true;
}


/////////////////////////////////////
// POSITION CONTROL
/////////////////////////////////////

bool RobotranYarpMotionControl::positionMove(int j, double ref) //WORKS
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::stop(int j) //WORKS
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::stop() //WORKS
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::positionMove(const double *refs) //WORKS
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getAxes(int *ax) // WORKS
{
    std::cout << "robotran motionControl: getAxes " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setRefSpeed(int j, double sp) //WORKS
{
    std::cout << "robotran motionControl: setRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeed(int j, double *ref) //WORKS
{
    std::cout << "robotran motionControl: getRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeeds(double *spds) //WORKS
{
    std::cout << "robotran motionControl: getRefSpeeds " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(int j, double delta) //NOT TESTED
{
    std::cout << "robotran motionControl: relativeMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(const double *deltas) //NOT TESTED
{
    std::cout << "robotran motionControl: relativeMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    std::cout << "robotran motionControl: checkMotionDone " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::checkMotionDone(bool *flag) //NOT TESTED
{
    std::cout << "robotran motionControl: checkMotionDone " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setPositionMode() //NOT TESTED
{
    std::cout << "robotran motionControl: setPositionMode " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setRefSpeeds(const double *spds) //NOT TESTED
{
    std::cout << "robotran motionControl: setRefSpeeds " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: setRefAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: setRefAccelerations " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getRefAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getRefAccelerations " << std::endl;
    return false;
}

// IPositionControl2

bool RobotranYarpMotionControl::positionMove(const int n_joint, const int *joints, const double *refs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(const int n_joint, const int *joints, const double *deltas) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::checkMotionDone(const int n_joint, const int *joints, bool *flags) //NOT IMPLEMENTED
{
    return false;
}

bool RobotranYarpMotionControl::setRefSpeeds(const int n_joint, const int *joints, const double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::setRefAccelerations(const int n_joint, const int *joints, const double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::getRefSpeeds(const int n_joint, const int *joints, double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::getRefAccelerations(const int n_joint, const int *joints, double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::stop(const int n_joint, const int *joints) //NOT IMPLEMENTED
{
    return false;
}

/////////////////////////////////////
// POSITION DIRECT
/////////////////////////////////////

bool RobotranYarpMotionControl::setPositionDirectMode() //NOT IMPLEMENTED -> Is it the same as setPositionMode?
{
    return false;
}

bool RobotranYarpMotionControl::setPosition(int j, double ref)
{
    std::cout << "robotran motionControl: setPosition " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setPositions(const int n_joint, const int *joints, double *refs)
{
    std::cout << "robotran motionControl: setPositions " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setPositions(const double *refs)
{
    std::cout << "robotran motionControl: setPositions " << std::endl;
    return false;
}

/////////////////////////////////////
// VELOCITY CONTROL
/////////////////////////////////////


bool RobotranYarpMotionControl::setVelocityMode() //NOT TESTED
{
    std::cout << "robotran motionControl: setVelocityMode " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::velocityMove(int j, double sp) //NOT TESTED
{
    std::cout << "robotran motionControl: velocityMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::velocityMove(const double *sp) //NOT TESTED
{
    std::cout << "robotran motionControl: velocityMove " << std::endl;
    return false;
}


/////////////////////////////////////
// ENCODER
/////////////////////////////////////

bool RobotranYarpMotionControl::getEncoder(int j, double *v) //WORKS
{
    std::cout << "robotran motionControl: getEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoders(double *encs) //WORKS
{
    std::cout << "robotran motionControl: getEncoders " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncodersTimed(double *encs, double *time)
{
    std::cout << "robotran motionControl: getEncodersTimed " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::getEncoderTimed(int j, double *encs, double *time)
{
    std::cout << "robotran motionControl: getEncoderTimed " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::resetEncoder(int j) //WORKS
{
    std::cout << "robotran motionControl: resetEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::resetEncoders() //WORKS
{
    std::cout << "robotran motionControl: resetEncoders " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoder(int j, double val) //WORKS
{
    std::cout << "robotran motionControl: setEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoders(const double *vals) //WORKS
{
    std::cout << "robotran motionControl: setEncoders " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
    std::cout << "robotran motionControl: getEncoderSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderSpeeds(double *spds) //NOT TESTED
{
    std::cout << "robotran motionControl: getEncoderSpeeds " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getEncoderAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getEncoderAccelerations " << std::endl;
    return false;
}
