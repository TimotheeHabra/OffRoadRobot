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
    for(unsigned int i=0; i<pos.size();i++)
    {
        pos[i] = MBSdata->q[jointID_map[i]];
    }

    //update time
    simu_time = MBSdata->tsim;

}

void RobotranYarpMotionControl::updateFromYarp(MBSdataStruct *MBSdata)
{
    for(unsigned int i=0; i<numberOfJoints; i++)
    {
//        std::cout << "index " << i << " ref " << desiredPosition[i] << std::endl;
        MBSdata->user_IO->refs[motorID_map[i]]  = desiredPosition[i];
        MBSdata->user_IO->servo_type[motorID_map[i]] = controlMode[i];
    }

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

    std::cout << "\n*********\nrobotran motionControl parameters are " << config.toString() << "\n***********\n" << std::endl;

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

    desiredPosition.resize(numberOfJoints);
    desiredPosition.zero();

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

    yarp::os::Bottle & motorID = config.findGroup("robotran_motor_id");
    motorID_map.resize(numberOfJoints);

    for(int i=0; i< motorID.size()-1; i++)
    {
        motorID_map[i] = motorID.get(i+1).asInt();
        printf("motorID_map[%d] = %d \n", i, motorID_map[i]);
    }

    // Get max/min joints limits
    if(!config.check("max") || !config.check("min"))
    {
        std::cout << "robotran max /min joints positions not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & max = config.findGroup("max");
    yarp::os::Bottle & min = config.findGroup("min");
    max_pos.resize(numberOfJoints);
    min_pos.resize(numberOfJoints);
    for(int i=0; i< max.size()-1; i++)  // we suppose that max and min have same size (it should be check!)
    {
        max_pos[i] = max.get(i+1).asDouble();
        printf("max pos[%d] = %f \n", i, max_pos[i]);

        min_pos[i] = min.get(i+1).asDouble();
        printf("min pos[%d] = %f \n", i, min_pos[i]);
    }

    controlMode.resize(numberOfJoints);
    for(int i=0; i< numberOfJoints; i++)
    {
        controlMode[i] = POSITION_CTRL;  //by default init to pos control
    }



//    yarp::os::Property wrapProp;
////    yarp::os::Property &mmm =wrapProp.addGroup();

//    wrapProp.put("device","controlboardwrapper2");
//    yarp::os::Property  &net = wrapProp.addGroup(networks);
//    net.
//    wrapProp.put("networks","myself");
//    wrapProp.put("joints", numberOfJoints);
//    char str[100];
//    sprintf(str, "0 %d 0 %d", numberOfJoints, numberOfJoints);
//    wrapProp.put("myself", str);

//    std::cout << "$$$$$$$$$$$$$$$$$ " << wrapProp.toString() << std::endl;

    if(!config.check("useWrapper"))
    {
        std::cout << "\nNOT USING WRAPPERS!\n" << std::endl;
        return true;
    }

    std::cout << "\nUSING WRAPPERS!\n" << std::endl;

    yarp::dev::IMultipleWrapper* iWrap;
    yarp::dev::PolyDriver* wrap = new yarp::dev::PolyDriver();
    yarp::os::Property wrapProp;
    wrapProp.fromString(config.toString());
    yarp::os::ConstString partName, robotName, wholeName;
    partName = config.find("name").asString();
    robotName = config.find("robot").asString();
    wrapProp.unput("device");
    wrapProp.put("device", "controlboardwrapper2");
    wrapProp.unput("name");

    wholeName = robotName + "/" + partName;
    wrapProp.put("name", wholeName);


    std::cout << "robotName is " << robotName << "; partName is " << partName << "; wholeName is " << wholeName << std::endl;

    std::cout << "\n*********\n before wrapper " << wrapProp.toString() << "\n***********\n" << std::endl;

    wrap->open(wrapProp);
    if (!wrap->isValid())
        fprintf(stderr, "RobotranYarpMotionControl: wrapper did not open\n");
    else
        fprintf(stderr, "RobotranYarpMotionControl: wrapper opened correctly\n");

    if (!wrap->view(iWrap)) {
        printf("RobotranYarpMotionControl Wrapper interface not found\n");
    }

   yarp::dev::PolyDriverList polyList;
   yarp::os::Bottle *netList = config.find("networks").asList();
   if (netList->isNull()) {
       printf("RobotranYarpMotionControl ERROR, net list to attach to was not found, exiting\n");
       wrap->close();
       // m_controlBoard.close();
       return false;
   }
   std::cout << "NNNNNNNNNNNNNNNNNNNNetwork list found !!" << netList->toString() << std::endl;

   polyList.push((yarp::dev::PolyDriver*) this, netList->get(0).asString().c_str() );
   if(!iWrap->attachAll(polyList) )
   {
       std::cout << "\n\n\nERROR while attching\n\n\n" << std::endl;
       return false;
    }
   else
   {
       std::cout << "\n ATTACH WAS OK\n" << std::endl;
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
// CONTROL MODE
/////////////////////////////////////

bool RobotranYarpMotionControl::setPositionMode(int j)  //TO BE TESTED
{
    controlMode[j] = POSITION_CTRL;
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
    static int getEncodersTimedCounter = 0;
    if((getEncodersTimedCounter % 100) == 0)
    {
        std::cout << "robotran motionControl: getEncodersTimed " << std::endl;
    }
    getEncodersTimedCounter++;
return true;
    if (!encs) return false;
    for (unsigned int i = 0; i <numberOfJoints; ++i) {
        encs[i] = pos[i];  //should we just use memcopy here?
        time[i] = simu_time;
    }    
    return true;
}


bool RobotranYarpMotionControl::getEncoderTimed(int j, double *enc, double *time) //TO BE TESTED
{
    static int getEncodersTimedCounter2 = 0;
    if((getEncodersTimedCounter2 % 100) == 0)
    {
        std::cout << "robotran motionControl: getEncodersTimed " << std::endl;
    }
    getEncodersTimedCounter2++;    if (time && enc && j >= 0 && j < (int)numberOfJoints) {
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

bool RobotranYarpMotionControl::getEncoderSpeeds(double *spds) //NOT IMPLEMENTED
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

// ICONTROLLIMITS2

bool  RobotranYarpMotionControl::getLimits (int axis, double *min, double *max)  //TO BE TESTED
{
    if (!min || !max) return false;
    *min = min_pos[axis];
    *max = max_pos[axis];
    return true;
}

#endif
