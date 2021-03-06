function[simulation_headers, controller_headers, simulation_files, controller_files] = simu_ctrl_files()

simulation_headers = {...
    'simu_def.h'...
    };

controller_headers = {...
    'controller_def.h'...
    'ControllersStruct.h'...
    };

simulation_files = {...
    'simu_outputs.c'...
    'simu_controller_loop.c'...
    'controller_inputs.c'...
    'controller_outputs.c'...
    };

controller_files = {...
    'ControllersStruct.c'...
    'controller_init.c'...
    'controller_loop.c'...
    'control.c'...
    'controller_functions.c'...
    };

end