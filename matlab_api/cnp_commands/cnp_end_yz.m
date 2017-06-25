function cnp_end_yz ()

global scnp;

%% Close serial conection
fclose(scnp.serial_yz);

end

