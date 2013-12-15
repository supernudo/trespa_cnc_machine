% Scritp for test the Control Numeric Platform (CNP)

% add commands path
path(path,'../cnp_commands/');

% init
close all
global scnp;
scnp = cnp_init('COM1');
scnp = cnp_init_yz('COM2');

% calibrate
%cnp_x_calibrate();
%cnp_yz_calibrate();
%cnp_alpha_calibrate();
%cnp_beta_calibrate();


% x measurements
x_cmd = 500:250:3000;
x_pos = zeros(1,length(x_cmd));
for i=1:length(x_cmd)
   cnp_x_set(x_cmd(i));
   pause(1);
   x_pos(i) = cnp_x_get();
end

% results
figure
x_error = x_cmd-x_pos;
hist(x_error);
figure
plot(x_cmd, x_pos,'+', x_cmd,x_cmd,'o');


% y measurements
y_cmd = -700:100:700;
y_pos = zeros(1,length(y_cmd));
for i=1:length(y_cmd)
   cnp_y_set(y_cmd(i));
   pause(1);
   y_pos(i) = cnp_y_get();
end

% results
figure
y_error = y_cmd-y_pos;
hist(y_error);
figure
plot(y_cmd, y_pos,'+', y_cmd,y_cmd,'o');

% z measurements
z_cmd = -700:100:700;
z_pos = zeros(1,length(z_cmd));
for i=1:length(z_cmd)
   cnp_z_set(z_cmd(i));
   pause(1);
   z_pos(i) = cnp_z_get();
end

% results
figure
z_error = z_cmd-z_pos;
hist(z_error);
figure
plot(z_cmd, z_pos,'+', z_cmd,z_cmd,'o');

% alpha measurements
alpha_cmd = -45:5:45;
alpha_pos = zeros(1,length(alpha_cmd));
for i=1:length(alpha_cmd)
   cnp_alpha_set(alpha_cmd(i));
   pause(1);
   alpha_pos(i) = cnp_alpha_get();
end

% results
figure
alpha_error = alpha_cmd-alpha_pos;
hist(alpha_error);
figure
plot(alpha_cmd,alpha_pos,'+', alpha_cmd,alpha_cmd,'o');


% beta measurements
%beta_cmd = scnp.x.cmd_min_mm:beta_step_deg:beta_step_deg*floor(scnp.x.cmd_mabeta_deg/beta_step_deg);
% beta_cmd = -25:5:25;
% beta_pos = zeros(1,length(beta_cmd));
% for i=1:length(beta_cmd)
%    cnp_beta_set(beta_cmd(i));
%    pause(1);
%    beta_pos(i) = cnp_beta_get();
% end
% 
% % results
% figure
% beta_error = beta_cmd-beta_pos;
% hist(beta_error);
% figure
% plot(beta_cmd, beta_pos,'x', beta_cmd,beta_cmd,'o');

% end cnp
cnp_end();
cnp_end_yz();




