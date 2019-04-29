function vref = setLeadVelocity(tvec,dt)

vref = zeros(1,length(tvec));

highwaySpeed = 100/3.6;      % [m/s]
trafficSpeed = 15/3.6;       % [m/s]
Zero2Hundred = 10;            % [s]
Hundred2Zero = 5;            % [s]

% acceleration to highway speed (1-11 seconds)
t = tvec(101:1101);
m_fAccel = highwaySpeed/Zero2Hundred;
vref(101:1101) = m_fAccel*t - m_fAccel;

% constant highway speed (11-31 sec)
vref(1101:3100) = highwaySpeed;

% basic decelerateion then acceleration back to highway speed
% deceleration (31-41 sec)
m = (0.75-1)*highwaySpeed/(41-31);
t = tvec(3101:4100);
vref(3101:4100) = m*t + (highwaySpeed-31*m);
% acceleration (41-51 sec)
b = (0.75*highwaySpeed) - 41*m_fAccel;
t_end = (highwaySpeed - b)/m_fAccel;
t = tvec(4101:t_end/dt);
vref(4101:t_end/dt) = m_fAccel*t + b;

% constant highway speed (38.5-60 sec)
vref(t_end/dt:6000) = highwaySpeed;


% Oscilatory velocity change (60-120  sec) [TODO]
vref(6001:7000) = highwaySpeed;

tend = 368;
t = 0:dt:tend;
rad2Hz = 0.1591549;
method = 'linear';
phi = -90;
fs1 = 0.06*rad2Hz; fe1 = 1.13*rad2Hz;
fs2 = 1.13*rad2Hz; fe2 = 2.26*rad2Hz;
fs3 = 2.26*rad2Hz; fe3 = 3.14*rad2Hz;
fs4 = 2.26*rad2Hz; fe4 = 3.14*rad2Hz;

y{1} = chirp(t,fs1,tend,fe1,method,phi);
y{2} = chirp(t,fs2,tend,fe2,method,phi);
y{3} = chirp(t,fs3,tend,fe3,method,phi);
% y{4} = chirp(t,fs4,tend,fe4,method,phi);

a = y{1}+y{2}+y{3};
v = cumtrapz(t,a);

offset = highwaySpeed - v(12501);
vref(6001:10362) = v(12501:16862)+offset;

% figure;
% for i = 1:3
%     subplot(3,1,i);
%     plot(t,y{i});
%     xlim([125 175]);
% end
% figure;
% hold on; grid on; box on;
% plot(t,v);
% ylim([9 19]);
% xlim([125 175]);

vref(10363:12000) = highwaySpeed;

% %%
% % traffic scenario speed back and forth (x3)
% % sudden braking
% m_fDecel = -highwaySpeed/Hundred2Zero;
% m_hDecel = -highwaySpeed/(2*Hundred2Zero);
% m = -0.5*highwaySpeed/(125-120);
% t = tvec(12001:12500);
% vref(12001:12500) = m*t + (highwaySpeed-m*120);
% 
% vref(12501:13500) = highwaySpeed/2;
% 
% m = -0.5*highwaySpeed/(140-135);
% t = tvec(13501:14000);
% vref(13501:14000) = m*t - 140*m;
% 
% % constant zero speed (94-104 sec)
% vref(14001:15500) = zeros(1,length(14001:15500));
% 
% % Phase 1
% m_tAccel = trafficSpeed/(1.5*Zero2Hundred);
% b_accel = -m_tAccel*155;
% t_end = (trafficSpeed-b_accel)/m_tAccel;
% t = tvec(15501:t_end/dt);
% vref(15501:t_end/dt) = m_tAccel*t + b_accel;
% 
% vref(t_end/dt:t_end/dt+200) = trafficSpeed;
% 
% t_curr = t_end/dt+200;
% 
% b_decel = trafficSpeed - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+500) = 0;
% 
% t_curr = t_end/dt+500;
% 
% % Phase 2
% b_accel = -m_tAccel*t_curr*dt;
% t_end = ((trafficSpeed/2)-b_accel)/m_tAccel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_tAccel*t + b_accel;
% 
% t_curr = t_end/dt;
% 
% b_decel = (trafficSpeed/2) - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+500) = 0;
% 
% t_curr = t_end/dt+500;
% 
% % Phase 3
% b_accel = -m_tAccel*t_curr*dt;
% t_end = (trafficSpeed-b_accel)/m_tAccel;
% t = tvec(t_curr+1:t_end/dt+1);
% vref(t_curr+1:t_end/dt+1) = m_tAccel*t + b_accel;
% 
% t_curr = t_end/dt;
% 
% b_decel = trafficSpeed - t_curr*dt*m_hDecel;
% t_end = ((trafficSpeed/2)-b_decel)/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% vref(t_end/dt:t_end/dt+200) = trafficSpeed/2;
% 
% t_curr = t_end/dt+200;
% 
% b_decel = trafficSpeed/2 - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+1000) = 0;
% 
% t_curr = t_end/dt+1000;
% 
% % Phase 2
% b_accel = -m_tAccel*t_curr*dt;
% t_end = ((trafficSpeed/2)-b_accel)/m_tAccel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_tAccel*t + b_accel;
% 
% t_curr = t_end/dt;
% 
% b_decel = (trafficSpeed/2) - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+500) = 0;
% 
% t_curr = t_end/dt+500;
% 
% % Phase 3
% b_accel = -m_tAccel*t_curr*dt;
% t_end = (trafficSpeed-b_accel)/m_tAccel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_tAccel*t + b_accel;
% 
% t_curr = t_end/dt;
% 
% b_decel = trafficSpeed - t_curr*dt*m_hDecel;
% t_end = ((trafficSpeed/2)-b_decel)/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% vref(t_end/dt:t_end/dt+300) = trafficSpeed/2;
% 
% t_curr = t_end/dt+300;
% 
% b_decel = trafficSpeed/2 - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+200) = 0;
% 
% t_curr = t_end/dt+200;
% 
% % Phase 1
% b_accel = -m_tAccel*t_curr*dt;
% t_end = (trafficSpeed-b_accel)/m_tAccel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_tAccel*t + b_accel;
% 
% vref(t_end/dt:t_end/dt+300) = trafficSpeed;
% 
% t_curr = t_end/dt+300;
% 
% b_decel = trafficSpeed - t_curr*dt*m_hDecel;
% t_end = -b_decel/m_hDecel;
% t = tvec(t_curr+1:t_end/dt);
% vref(t_curr+1:t_end/dt) = m_hDecel*t + b_decel;
% 
% vref(t_end/dt:t_end/dt+2000) = 0;
% 
% t_curr = t_end/dt+2000;
% 
% 
% % Stop & Go
% t = tvec(t_curr+1:t_curr+Zero2Hundred/dt);
% vref(t_curr+1:t_curr+Zero2Hundred/dt) = m_fAccel*t - m_fAccel*t_curr*dt;
% 
% t_curr = t_curr+Zero2Hundred/dt;
% 
% vref(t_curr+1:t_curr+2000) = highwaySpeed;
% 
% t_curr = t_curr+2000;
% 
% t = tvec(t_curr+1:t_curr+Hundred2Zero/dt);
% vref(t_curr+1:t_curr+Hundred2Zero/dt) = m_fDecel*t - (t_curr+(Hundred2Zero/dt))*dt*m_fDecel;
% % filler just because of unknown error
% % vref(16385) = m_fDecel*163.85 - (t_curr+(Hundred2Zero/dt))*dt*m_fDecel;
% t_curr = t_curr+Hundred2Zero/dt;
% 
% vref(t_curr+1:t_curr+2000) = 0;
% 
% t_curr = t_curr+2000;
% 
% t = tvec(t_curr+1:t_curr+Zero2Hundred/dt);
% vref(t_curr+1:t_curr+Zero2Hundred/dt) = m_fAccel*t - m_fAccel*t_curr*dt;
% 
% t_curr = t_curr+Zero2Hundred/dt;
% 
% vref(t_curr+1:t_curr+2000) = highwaySpeed;
% 
% t_curr = t_curr+2000;
% 
% m_Decel = -highwaySpeed/(5*Hundred2Zero);
% t = tvec(t_curr+1:t_curr+5*Hundred2Zero/dt);
% vref(t_curr+1:t_curr+5*Hundred2Zero/dt) = m_Decel*t - (t_curr+(5*Hundred2Zero/dt))*dt*m_Decel;
% 
% t_curr = t_curr+5*Hundred2Zero/dt;
% 
% vref(t_curr+1:t_curr+2000) = 0;

% plot(tvec,vref);

