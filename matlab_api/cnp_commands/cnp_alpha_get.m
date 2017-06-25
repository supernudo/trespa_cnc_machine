function pos_deg = cnp_alpha_get()
global scnp;

cnp_cmd('alpha', 'get');
pos_deg = scnp.alpha.pos_deg;
end