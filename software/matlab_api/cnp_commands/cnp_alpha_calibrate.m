function range_deg = cnp_alpha_calibrate()
global scnp;

cnp_cmd('alpha', 'autopos');
range_deg = [scnp.alpha.cmd_min_deg scnp.alpha.cmd_max_deg];

end