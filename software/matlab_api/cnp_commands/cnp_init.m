function scnp = cnp_init (port, port_yz)
% Initialize Control Numeric Platform (CNP)
%
% scnp = cnp_init(port, port_yz)
%
% scnp: 		structure returned.
% port: 		serial port conection of axis "X", and angles "alpha" and "beta" (COM1 by default).
% port_yz: 	serial port conection of axis "Y" and "Z" (COM2 by default).
%

%% Create serial conection
if nargin == 2
    scnp.serial = serial(port, 'BaudRate', 115200);
		scnp.serial_yz = serial(port_yz, 'BaudRate', 115200);
elseif narging == 1
		display('>> cnp: No enought arguments')
		return
elseif narging > 2
		display('>> cnp: Too much arguments')
		return
else
    scnp.serial = serial('COM1', 'BaudRate', 115200);
		scnp.serial_yz = serial('COM2', 'BaudRate', 115200);
end

%% Init structure
scnp.x.cmd_mm = 0;
scnp.x.cmd_min_mm = 0;
scnp.x.cmd_max_mm = 0;
scnp.x.pos_mm = 0.0;
scnp.x.offset = 0;

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

scnp.alpha.cmd_deg = 0;
scnp.alpha.cmd_min_deg = 0;
scnp.alpha.cmd_max_deg = 0;
scnp.alpha.pos_deg = 0.0;

scnp.beta.cmd_deg = 0;
scnp.beta.cmd_min_deg = 0;
scnp.beta.cmd_max_deg = 0;
scnp.beta.pos_deg = 0.0;

%% Open serial conections
fopen(scnp.serial);
fopen(scnp.serial_yz);

end

