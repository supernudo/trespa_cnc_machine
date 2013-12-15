function scnp = cnp_init_yz (port)
% Initialize Control Numeric Platform (CNP) Axis Y and Z
%
% scnp = cnp_init_yz(port)
%
% scnp: structure returned.
% port: serial port conection (COM2 by default).
%

%% Create serial conection
if nargin == 1
    scnp.serial_yz = serial(port, 'BaudRate', 115200);
else
    scnp.serial_yz = serial('COM2', 'BaudRate', 115200);
end

%% Init structure
scnp.y.cmd_mm = 0;
scnp.y.cmd_min_mm = 0;
scnp.y.cmd_max_mm = 0;
scnp.y.pos_mm = 0.0;
scnp.y.offset = 0;

scnp.z.cmd_mm = 0;
scnp.z.cmd_min_mm = 0;
scnp.z.cmd_max_mm = 0;
scnp.z.pos_mm = 0.0;
scnp.z.offset = 0;

%% Open serial conection
fopen(scnp.serial_yz);

end

