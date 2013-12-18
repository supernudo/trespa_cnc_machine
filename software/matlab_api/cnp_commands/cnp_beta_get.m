function pos_deg = cnp_beta_get()
global scnp;

cnp_cmd('beta', 'get');
pos_deg = scnp.beta.pos_deg;
end