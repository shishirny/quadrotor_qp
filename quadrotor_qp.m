%% serial communication with arduino
%% aaaand, of course, serial communication of qp output to arduino
%%
fclose(instrfindall);
% delete(s);
clear s;
s = serial('COM3','BaudRate',9600);
fopen(s);

% fprintf(s,'*IDN?');
while(1)
    tic;
out = fscanf(s);
n = numel(out);
q_act = str2double(out(1:n-1));
% disp(q_act)

if ~isnan(q_act)
tic
u_in = qp_gen(q_act);
toc
    u = u_in(2);
else
    u = 0;
end
% disp(u);
%     u = - 5*q_act;

fprintf(s,'%c','a');
if abs(u) < 1000
    fprintf(s,'%c','0');
    if abs(u) < 100
        fprintf(s,'%c','0');
        if abs(u) < 10
            fprintf(s,'%c','0');
        end
    end
end
            
fprintf(s,'%s',num2str(u));
toc;
end

