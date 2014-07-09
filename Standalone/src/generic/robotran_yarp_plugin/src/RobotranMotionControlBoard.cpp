/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifdef YARP

#include <RobotranMotionControlBoard.h>


using namespace yarp::dev;


/**
 * robotran stuff
 */
bool RobotranYarpMotionControl::robotran_init()
{
    std::cout << "robotran_init : is this function really necessary?" << std::endl;
    return true;
}

void RobotranYarpMotionControl::updateToYarp(const MBSdataStruct * MBSdata)
{
    //update the vector pos
    for(int i=0; i<pos.size();i++)
    {
        pos[i] = MBSdata->q[jointID_map[i]];
    }

    //update time
    simu_time = MBSdata->tsim;

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

    // Get joints names
    if(!config.check("jointNames"))
    {
        std::cout << "joints names not specified in config file " << std::endl;
        return false;
    }

    yarp::os::Bottle & jointNames = config.findGroup("jointNames");
    numberOfJoints = jointNames.size()-1;

    std::cout << "nbr joints = " << numberOfJoints << std::endl;

    pos.resize(numberOfJoints);
    pos.zero();

    // Get joints id
    if(!config.check("robotran_joint_id"))
    {
        std::cout << "robotran joints id not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & jointID = config.findGroup("robotran_joint_id");

    jointID_map.resize(numberOfJoints);
    for(int i=0; i< jointID.size()-1; i++)
    {
        jointID_map[i] = jointID.get(i+1).asInt();
        printf("jointID_map[%d] = %d \n", i, jointID_map[i]);
    }

    return true;
}

bool RobotranYarpMotionControl::close()
{
    std::cout << "RobotranYarpMotionControl::close" << std::endl;
    return true;
}


/////////////////////////////////////
// POSITION CONTROL
/////////////////////////////////////

bool RobotranYarpMotionControl::positionMove(int j, double ref) //TESTING
{

   // std::cout << "robotran motionControl: positionMove " << std::endl;
    if (j >= 0 && j < (int) numberOfJoints) {
        desiredPosition[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask Robotran to set PIDs ref_pos to this value
        return true;
    }
    return false;
}

bool RobotranYarpMotionControl::stop(int j) //TO BE DONE
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::stop() //TO BE DONE
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::positionMove(const double *refs) //TO BE DONE
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getAxes(int *ax) //WORKS
{
    std::cout << "robotran motionControl: getAxes " << std::endl;
    *ax = numberOfJoints;
    return true;
}

bool RobotranYarpMotionControl::setRefSpeed(int j, double sp) //TO BE DONE
{
    std::cout << "robotran motionControl: setRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeed(int j, double *ref) //TO BE DONE
{
    std::cout << "robotran motionControl: getRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeeds(double *spds) //TO BE DONE
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

bool RobotranYarpMotionControl::getEncoder(int j, double *v) //TO BE TESTED
{
    std::cout << "robotran motionControl: getEncoder " << std::endl;
    if (v && j >= 0 && j < (int)numberOfJoints) {
        *v = pos[j];
        return true;
    }
    return false;
}

bool RobotranYarpMotionControl::getEncoders(double *encs) //TO BE TESTED
{
    std::cout << "robotran motionControl: getEncoders " << std::endl;
    if (!encs) return false;
    for (unsigned int i = 0; i < numberOfJoints; ++i) {
        encs[i] = pos[i];  //should we just use memcopy here?
    }
    return true;
}

bool RobotranYarpMotionControl::getEncodersTimed(double *encs, double *time) //TO BE TESTED
{
    std::cout << "robotran motionControl: getEncodersTimed " << std::endl;
    if (!encs) return false;
    for (unsigned int i = 0; i <numberOfJoints; ++i) {
        encs[i] = pos[i];  //should we just use memcopy here?
        time[i] = simu_time;
    }    
    return true;
}


bool RobotranYarpMotionControl::getEncoderTimed(int j, double *enc, double *time) //TO BE TESTED
{
    std::cout << "robotran motionControl: getEncoderTimed " << std::endl;
    if (time && enc && j >= 0 && j < (int)numberOfJoints) {
        *enc = pos[j];
        *time = simu_time;
        return true;
    }
    return false;
}


bool RobotranYarpMotionControl::resetEncoder(int j) //TO BE DONE
{
    std::cout << "robotran motionControl: resetEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::resetEncoders() //TO BE DONE
{
    std::cout << "robotran motionControl: resetEncoders " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoder(int j, double val) //TO BE DONE
{
    std::cout << "robotran motionControl: setEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoders(const double *vals) //TO BE DONE
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

#endif
