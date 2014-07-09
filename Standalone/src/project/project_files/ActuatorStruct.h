//---------------------------
// Houman Dallali
//
// Creation : 9-7-2014
// Last update : 9-7-2014
//---------------------------


// Series Elastic Actuator Structure
typedef struct SEActuatorStruct
{
    char   type[50];
    double Resistance;
    double Inductance;
    double TrqConst;
    double Kbemf;
    double Inertia;
    double Damping;
    double GearRatio;
    double SeriesSpring;
    double SeriesDamping;
    double Isaturation;
    double Vsaturation;

} SEActuatorStruct;

// Parallel Elastic Actuator Structure
typedef struct PActuatorStruct
{
    char   type[50];
    double Resistance;
    double Inductance;
    double TrqConst;
    double Kbemf;
    double Inertia;
    double Damping;
    double BallScrewRatio;
    double PSpring;
    double PDamping;
    double saturation;

} PActuatorStruct;
