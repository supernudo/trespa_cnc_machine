function pos_mm = cnp_z_get()
global scnp;

cnp_cmd('axis_z', 'get');
pos_mm = scnp.z.pos_mm;
end