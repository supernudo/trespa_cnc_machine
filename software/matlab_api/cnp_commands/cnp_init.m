function scnp = cnp_init (port)
% Initialize Control Numeric Platform (CNP)
%
% scnp = cnp_init(port)
%
% scnp: structure returned.
% port: serial port conection (COM1 by default).
%

%% Create serial conection
% if nargin == 1
%     scnp.serial = serial(port, 'BaudRate', 115200);
% else
%     scnp.serial = serial('COM1', 'BaudRate', 115200);
% end

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

%% Open serial conection
%fopen(scnp.serial);

end

