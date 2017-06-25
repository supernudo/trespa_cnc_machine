function pos_mm = cnp_x_get()
global scnp;

cnp_cmd('axis_x', 'get');
pos_mm = scnp.x.pos_mm;
end