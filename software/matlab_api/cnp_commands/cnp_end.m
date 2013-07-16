function cnp_end ()

global scnp;

%% Close serial conection
fclose(scnp.serial);

end

