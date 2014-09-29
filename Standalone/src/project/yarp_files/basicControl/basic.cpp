#ifdef YARP

#include "basic.h"

#include <stdexcept>      // std::out_of_range


using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

basicControl::basicControl() : RateThread(500), _verbose(true)
{
    std::string part = std::string("front");
    partNameList.push_back(part);

    part = std::string("rear");
    partNameList.push_back(part);
}

bool basicControl::open(yarp::os::Searchable &rf)
{
    cout << "\nbasicControl::open; params are " << rf.toString() << std::endl ;
    return true;
}

//bool basicControl::updateModule()
void basicControl::run()
{
    std::cout << "\nbasicControl::updateModule() " << std::endl;

    double *enc = new double[frontMC->_axes];
    double *time = new double[frontMC->_axes];
    frontMC->encoder_p->getEncodersTimed(enc, time);
}


// IMultipleWrapper interface
bool basicControl::attachAll(const yarp::dev::PolyDriverList &device2attach)
{
    cout << "basicControl::attachAll" << endl;
    PolyDriver * device;

    frontMC = new MotorControlHelper;
    rearMC  = new MotorControlHelper;

    for(partNameListIterator = partNameList.begin(); partNameListIterator != partNameList.end(); partNameListIterator++)
    {
        for(int i=0; i<device2attach.size(); i++)
        {
            device = (yarp::dev::PolyDriver*) device2attach[i]->poly;
            if(*partNameListIterator ==  device2attach[i]->key.c_str())
            {
                std::cout << "attaching device " << *partNameListIterator << std::endl;
                if(!frontMC->attach(device))
                {
                    std::cout << "ERROR while attaching device " << *partNameListIterator << std::endl;
                    return false;
                }
            }
        }
    }
    return true;
}

bool basicControl::detachAll()
{
    cout << "basicControl::detachAll" << endl;
    return true;
}

#endif
