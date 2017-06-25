function cnp_cmd(cmd, arg1, arg2)
% Send command to Control Numeric Platform
%
% cnp_cmd(cmd, arg1, arg2)
%
% IMPORTANT: 'cnp_cmd' need a GLOBAL structure named 'scnp' previously declarated.
%
% cmd: command to send (string).
% arg1: first argument of command (string or number)
% arg2: second argument of command, if requiered (string or number)
% 

global scnp

%% Check input args
if nargin < 2
    display('>> cnp: No enought arguments')
    return
end

%% Flush input
while(scnp.serial.BytesAvailable > length('slavedspic > '))
    fgetl(scnp.serial);
end

while(scnp.serial_yz.BytesAvailable > length('slavedspic > '))
    fgetl(scnp.serial_yz);
end


%% Send command

% slavedspic bypass in
if strcmp(cmd, 'alpha') || strcmp(cmd, 'beta')
    fprintf(scnp.serial, 'slavedspic raw_mode')
    pause(0.1)
    skip_line = 2;
else
    skip_line = 1;
end

if strcmp(cmd, 'axis_x') || strcmp(cmd, 'alpha') || strcmp(cmd, 'beta')
    if nargin == 3
        fprintf(scnp.serial,[cmd ' ' arg1 ' ' num2str(arg2)])
    else
        fprintf(scnp.serial,[cmd ' ' arg1])
    end    
elseif strcmp(cmd, 'axis_y') || strcmp(cmd, 'axis_z') || strcmp(cmd, 'axis_yz')
    if nargin == 3
        fprintf(scnp.serial_yz,[cmd ' ' arg1 ' ' num2str(arg2)])
    else
        fprintf(scnp.serial_yz,[cmd ' ' arg1])
    end   
end

pause(0.1);

%% Receive response

% init flags
done = 0;
global timeout;
timeout = 0;

% set serial timeout
scnp.serial.Timeout = 60;
% scnp.serial.ErrorFcn = {'timeout_flag_on'};

% read lines until done or timeout
while(done == 0 && timeout == 0)
    
    % read line
    if strcmp(cmd, 'axis_x') || strcmp(cmd, 'alpha') || strcmp(cmd, 'beta')
        line = fgetl(scnp.serial);
    elseif strcmp(cmd, 'axis_y') || strcmp(cmd, 'axis_z')|| strcmp(cmd, 'axis_yz')
        line = fgetl(scnp.serial_yz);
    end
    
    % echo, skip lines depens on command
    if skip_line == 0
        display(['>> ' line]);
    else
       skip_line = skip_line - 1;
    end

%     parse sintaxis errors 
    if strcmp(line, 'Ambiguous command')
       display('>> cnp: Ambiguous command');
    end

    if strcmp(line, 'Command not found')
        display('>> cnp: Command not found')
    end

    if strcmp(line, 'Bad arguments')
       display('>> cnp: Bad arguments') 
    end

    % parse 'Done'
    if strcmp(line, 'Done') || strcmp(line(2:end), 'Done')
                
        % update cmd
        if strcmp(cmd, 'axis_x') && strcmp(arg1, 'set')
            scnp.x.cmd_mm = arg2;
        elseif strcmp(cmd, 'axis_y') && strcmp(arg1, 'set')
            scnp.y.cmd_mm = arg2;
        elseif strcmp(cmd, 'axis_z') && strcmp(arg1, 'set')
            scnp.z.cmd_mm = arg2;
        elseif strcmp(cmd, 'alpha') && strcmp(arg1, 'set')
            scnp.alpha.cmd_deg = arg2;
        elseif strcmp(cmd, 'beta') && strcmp(arg1, 'set')
            scnp.beta.cmd_deg = arg2;
        end

        % slavedspic bypass out
        if strcmp(cmd, 'alpha') || strcmp(cmd, 'beta')
            fprintf(scnp.serial, sprintf('%c',3))
            pause(0.1)
        end

        % flag on
        done = 1;
    end
    
    % parse 'autopos' answers
    [ret count] = sscanf(line, 'Axis X range is [%d %d] mm');
    if count == 2
        scnp.x.cmd_min_mm = ret(1);
        scnp.x.cmd_max_mm = ret(2);
    end
    [ret count] = sscanf(line, 'Axis Y range is [%d %d] mm');
    if count == 2
        scnp.y.cmd_min_mm = ret(1);
        scnp.y.cmd_max_mm = ret(2);
    end
    [ret count] = sscanf(line, 'Axis Z range is [%d %d] mm');
    if count == 2
        scnp.z.cmd_min_mm = ret(1);
        scnp.z.cmd_max_mm = ret(2);
    end
    [ret count] = sscanf(line, 'Alpha range is [%d %d] deg');
    if count == 2
        scnp.alpha.cmd_min_deg = ret(1);
        scnp.alpha.cmd_max_deg = ret(2);
    end
    [ret count] = sscanf(line, 'Beta range is [%d %d] deg');
    if count == 2
        scnp.beta.cmd_min_deg = ret(1);
        scnp.beta.cmd_max_deg = ret(2);
    end
    
    % parse 'get' answers
    [ret count] = sscanf(line, 'axis_x pos = %f mm');
    if count == 1
        scnp.x.pos_mm = ret;
    end
    [ret count] = sscanf(line, 'axis_y pos = %f mm');
    if count == 1
        scnp.y.pos_mm = ret;
    end
    [ret count] = sscanf(line, 'axis_z pos = %f mm');
    if count == 1
        scnp.z.pos_mm = ret;
    end
    [ret count] = sscanf(line, 'alpha angle = %f deg');
    if count == 1
        scnp.alpha.pos_deg = ret;
    end
    [ret count] = sscanf(line, 'beta angle = %f deg');
    if count == 1
        scnp.beta.pos_deg = ret;
    end
    
    % out of range
    if strcmp(line, 'Consign out of range');
        display('Consign out of range')
    end
 
    % axis not calibrated
    if strcmp(line, 'Axis is not calibrated yet');
        display('Axis is not calibrated yet')
    end

    % angle not calibrated
    if strcmp(line, 'Angle is not calibrated yet');
        display('Angle is not calibrated yet')
    end
 
    
end

% check if sucess OK
if done == 0
    display('>> cnp: ERROR serial conection fails')
end

end
    
function timeout_flag_on()
    global timeout;
    timeout = 1;
end
