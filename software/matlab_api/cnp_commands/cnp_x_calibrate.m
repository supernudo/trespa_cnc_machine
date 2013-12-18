function range_mm = cnp_x_calibrate()
global scnp;

cnp_cmd('axis_x', 'autopos');
range_mm = [scnp.x.cmd_min_mm scnp.x.cmd_max_mm];

end