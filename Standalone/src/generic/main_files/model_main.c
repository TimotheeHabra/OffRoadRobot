
/*
 * Top-level function of the model.
 * 1. initialization
 * 2. loop
 * 3. end of the simulation
 *
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"

/*
 * Main function
 */
int main(int argc, char const *argv[])
{
    main_simulation();
    
    return 0;
}

/*
 * Main loop
 */

void main_simulation()
{
    Loop_inputs *loop_inputs;

    // initialization
	loop_inputs = init_simulation();
	
	// simulation
    // updateDataFromYarp(yarpDevice, controlDataStructure);
	loop_simulation(loop_inputs);

    // updateDataToYarp(controlDataStructure, yarpDevice);
    // end of the simulation
    finish_simulation(loop_inputs);
}
