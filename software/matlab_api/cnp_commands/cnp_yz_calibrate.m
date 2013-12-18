function [y_range_mm, z_range_mm] = cnp_yz_calibrate()
global scnp;

cnp_cmd('axis_yz', 'autopos');
y_range_mm = [scnp.y.cmd_min_mm scnp.y.cmd_max_mm];
z_range_mm = [scnp.z.cmd_min_mm scnp.z.cmd_max_mm];

end
