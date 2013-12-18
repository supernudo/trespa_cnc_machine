function scnp = cnp_init_yz (port)
% Initialize Control Numeric Platform (CNP) Axis Y and Z
%
% scnp = cnp_init_yz(port)
%
% scnp: structure returned.
% port: serial port conection (COM2 by default).
%

global scnp;
scnp_temp = scnp;

%% Create serial conection
if nargin == 1
    scnp_temp.serial_yz = serial(port, 'BaudRate', 115200);
else
    scnp_temp.serial_yz = serial('COM2', 'BaudRate', 115200);
end

%% Init structure

scnp_temp.y.cmd_mm = 0;
scnp_temp.y.cmd_min_mm = 0;
scnp_temp.y.cmd_max_mm = 0;
scnp_temp.y.pos_mm = 0.0;
scnp_temp.y.offset = 0;

scnp_temp.z.cmd_mm = 0;
scnp_temp.z.cmd_min_mm = 0;
scnp_temp.z.cmd_max_mm = 0;
scnp_temp.z.pos_mm = 0.0;
scnp_temp.z.offset = 0;

scnp = scnp_temp

%% Open serial conection
fopen(scnp.serial_yz);

end

