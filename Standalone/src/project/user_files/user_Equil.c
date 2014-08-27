#include "equil.h"
#include "user_all_id.h"


int EQUIL_extra_fct(MBSdataStruct *MBSdata, double *f)
{
	// f[x] must be zero after the equilibrium, (x = 0 1 2 3... n_extra-1)

	//f[0] = MBSdata->user_IO->equil_wheel_rr_x + 1.55;//
	//printf("run : f0 = %f",f[0]); 
	//f[1] = MBSdata->user_IO->equil_wheel_ft_lt_x - 0.445;
	//f[2] = MBSdata->user_IO->equil_wheel_ft_lt_y - 0.350;

	return EXIT_SUCCESS;
}

void EQUIL_get_options_from_user(EQUIL_option_strct *options, MBSdataStruct *MBSdata)
{
	//options->solvemethod = 1;
	//options->relax = 1.0;
	//options->smooth = 0;
	//options->senstol = 1e-6;
	//options->equitol = 1e-6;
	//options->devjac = 1e-6;
	//options->itermax = 30;
	//options->static_v = 1;
	options->verbose = 1;
	//options->visualise = 0; 
	//options->clearmbsglobal = 1;

	////////////////////////////////////////////////////////////////////////////////
	options->n_ignored_qu = 0;

	options->ignored_qu = get_int_vec(options->n_ignored_qu);
	options->n_changed_qu = 0;

	options->changed_qu = get_int_vec(options->n_changed_qu);


	options->x_subst_ptr = (double**) malloc(options->n_changed_qu * sizeof(double*));
	//options->x_subst_ptr[0] = &(MBSdata->user_model->damper_ft.Z_0);

	options->n_x_extra = 0;

	options->EQUIL_extra_fct_ptr = EQUIL_extra_fct;
	options->x_extra_ptr = (double**) malloc(options->n_x_extra * sizeof(double*));
	//options->x_extra_ptr[0] = &(MBSdata->dpt[1][body_to_fork_id+1]);
	
	
	////////////////////////////////////////////////////////////////////////////////
}