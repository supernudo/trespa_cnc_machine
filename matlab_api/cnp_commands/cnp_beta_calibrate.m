function range_deg = cnp_beta_calibrate()
global scnp;

cnp_cmd('beta', 'autopos');
range_deg = [scnp.beta.cmd_min_deg scnp.beta.cmd_max_deg];

end