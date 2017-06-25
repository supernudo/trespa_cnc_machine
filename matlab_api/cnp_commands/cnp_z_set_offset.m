function range_mm = cnp_z_set_offset(pos_mm)
global scnp;

% check offset range
if pos_mm < 0
    display('>> cnp: offset value must be positive');
    return;
end

cnp_cmd('axis_z', 'offset', pos_mm);
range_mm = [scnp.z.cmd_min_mm scnp.z.cmd_max_mm];

end