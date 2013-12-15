function range_mm = cnp_y_set_offset(pos_mm)
global scnp;

% check offset range
if pos_mm < 0
    display('>> cnp: offset value must be positive');
    return;
end

cnp_cmd('axis_y', 'offset', pos_mm);
range_mm = [scnp.y.cmd_min_mm scnp.y.cmd_max_mm];

end