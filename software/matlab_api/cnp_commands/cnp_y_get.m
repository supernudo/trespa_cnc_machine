function pos_mm = cnp_y_get()
global scnp;

cnp_cmd('axis_y', 'get');
pos_mm = scnp.y.pos_mm;
end