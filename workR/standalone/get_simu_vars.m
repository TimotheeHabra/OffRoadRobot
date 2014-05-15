function [simu_vars] = get_simu_vars()

    [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables();

    [nb_none, ~] = size(simu_vars_none);
    if nb_none > 0
        simu_vars_none = [simu_vars_none , repmat({'none'},nb_none,1)];
    end

    [nb_in, ~] = size(simu_vars_in);
    if nb_in > 0
        simu_vars_in = [simu_vars_in , repmat({'in'},nb_in,1)];
    end

    [nb_out, ~] = size(simu_vars_out);
    if nb_out > 0
        simu_vars_out = [simu_vars_out , repmat({'out'},nb_out,1)];
    end

    [nb_struct, ~] = size(simu_vars_struct);
    if nb_struct > 0
        simu_vars_struct = [simu_vars_struct , repmat({'structure'},nb_struct,1)];
    end

    simu_vars = cat(1,simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct);
    
end