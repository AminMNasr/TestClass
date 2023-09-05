clear
clc

% PID model for standing balance.
% Writer: Amin Nasr - SMPLab - amin.nasr@ubc.ca

% This code uses the PID controller to maintain the standing balance of an 
% AP inverted pendulum model. PID and Environment Classes accompanies 
% this code. They should be in the same folder as this code.

mainAddress = pwd;
mainAddress = horzcat(mainAddress,'\');

% The slider idea: The base for delay and noise condition 
% base_setting = [52 0 0 0] ;
base_setting = [52 0.02 0.003 0.001];

Kp_options = [1000 1250 1500 1750 2000];
Ki_options = [0 25 50 75 100];
Kd_options = [0 50 100 150 200];

setting_options = [52 
                   0.02
                   0.003
                   0.001];

% setting_options = [12 32 52 72 92
%                    0 0.002 0.02 0.2 2
%                    0 0.0003 0.003 0.03 0.3
%                    0 0.0001 0.001 0.01 0.1
%                    0 0.001 0.01 0.1 1];


dtms = 4; %ms
dt = dtms/1000;
decisionLoopTiming = 8; %ms
decisionLoop = decisionLoopTiming/dtms;
   

startIterations = 1;
endIterations = 10;

for iItr = startIterations:endIterations
    rng(iItr);

    counterForTests = zeros(1,size(base_setting,1));

    for iSetting = 1:size(base_setting,1)
        for iOption = 1:size(setting_options,1)
            for iValueOpt = 1:size(setting_options,2)
                

                clear env
                clear pid

                setting = base_setting(iSetting,:);
                setting(iOption) = setting_options(iOption, iValueOpt);

                if setting(:)' == base_setting(iSetting,:) 
                    counterForTests(iSetting) = ...
                        counterForTests(iSetting) + 1;
                end

                
                if all(counterForTests(iSetting) > 1) && ...
                        all(setting == base_setting(iSetting,:))
                    continue;
                end
                
                env = Environment;
                n = env.num_samples;
                
                timing_delay = setting(1); %ms
                env.mn_coeff = setting(2);
                env.thn_coeff = setting(3);
                env.thdotn_coeff = setting(4);
                for iP = 1: numel(Kp_options)
                    for iI = 1:numel(Ki_options)
                        for iD = 1:numel(Kd_options)

                            pid = PID;
                            pid.Kp = Kp_options(iP);
                            pid.Ki = Ki_options(iI);
                            pid.Kd = Kd_options(iD);                                
            
                            Description = horzcat('PID','ITR',num2str(iItr), ...
                                'D',num2str(timing_delay),'MN',num2str(env.mn_coeff), ...
                                'TN',num2str(env.thn_coeff),'TDN',num2str(env.thdotn_coeff), ...
                                'Kp',num2str(pid.Kp),'Ki',num2str(pid.Ki),'Kd',num2str(pid.Kd));
            
                            mkdir(Description);
                           
                            delayTimeAction = timing_delay;
                            delayTimeSensory = timing_delay;
                            delaybuffAction = delayTimeAction/dtms;
                            delaybuffSensory = delayTimeSensory/dtms;
                                           
                            
                            % constants
                            passiveTHistory = zeros(500,2);
                            counter = 0;
                            % Run the simulation
                            
            %                 for j=1:n_epochs
            % 
            %     
            %                     if mod(j,500) == 0
            %                         j
            %                     end
            %                 
            %                     % Initialize the state
            %                     now_state = [env.n_thdot(randsample(1:env.n_thetadot,1)); ...
            %                         env.n_th(randsample(1:env.n_theta,1))];
            %                 %     now_state = randn(2,1) * pi/180;
            %                     true_state = now_state;
            %                     [delayed_disc_th, delayed_disc_thdot] = env.find_state(now_state);
            %                     %now_state = randn(2,1) * pi/180;
            %                     
            %                     env.reset_pt(true_state);
            %                 
            %                     action_buff = ones(delaybuffAction)*floor(env.n_action/2);
            %                 %     now_act = floor(env.n_action/2);
            %                     now_act = env.action_zero;
            %                 
            %                     
            %                     for iDelay=1:delaybuffSensory
            %                         state_buff(:,iDelay) = now_state;
            %                     end               
            %                     
            %                     % Simulate
            %                     for i=1:n
            %                 
            %                         action_buff(delaybuffAction) = now_act;
            %                         delayed_act = action_buff(1);
            %                         action_buff(1) = [];
            %                         
            %                         state_buff(:,delaybuffSensory) = now_state;
            %                         delayed_state = state_buff(:,1);
            %                         state_buff(:,1) = [];
            %                         
            %                         counter = counter + 1;
            %                 
            %                         % Choose the action
            %                         if mod(i,decisionLoop) == 1
            %                              
            %                             
            %                         end
            %                 
            %                         if sign(true_torque) == -1
            %                             MVC = 100;
            %                         else
            %                             MVC = 40;
            %                         end
            %                 
            %                         true_torque = env.muscle_dyn (true_torque,MVC, 0.004);
            %                 
            %                         ps_torque = env.passive_torque(true_state);
            %                         
            %                         [now_state, true_state] = env.dynamics(true_state, true_torque, ...
            %                             ps_torque, i, j, dt);
            %                 
            %                         failure_flag = env.failure_check(delayed_state);
            %                 
            %                         if counter<=10000
            %                             passiveTHistory(counter,1) = true_state(2);
            %                             passiveTHistory(counter,2) = ps_torque;
            %                         end
            %                         
            %                         if failure_flag == 1
            %                 
            %                             qagent.qUpdate_failure(delayed_q_ind, now_act);
            %                             break;
            %                         end
            %                 
            %                         % Q-update
            %                         if mod(i,decisionLoop) == 1
            %                 
            %                             [disc_th_next, disc_thdot_next] = env.find_state(delayed_state);
            %                             if isempty( disc_thdot_next )
            %                                 disc_thdot_next = 20;
            %                             end
            %                             q_ind_next = qagent.qind(disc_thdot_next, disc_th_next);
            %                             qagent.qUpdate(q_ind_prev, q_ind_next, act_prev, now_act);
            %                             
            %                         end          
            %                 
            %                     end
            %                    
            %                     q_history(j) = mean(mean(qagent.Q_vals));
            % 
            %                     if j>window
            % 
            %                         q_changes(j-window)= (q_history(j)-q_history(j-window))/window;
            % 
            %                     end
            % 
            %                 end
                            
            
                            for iTest = 1:10
                                result_char = 's';
                                % Run 10 test
                                % Initialize the state
                                % now_state = [n_thdot(randsample(1:20,1)); n_th(randsample(1:20,1))];
                                % now_state = randn(2,1) * pi/180;
                                totalDelay = delayTimeSensory + delayTimeAction;
                                now_state = env.initiate_test(totalDelay)*pi/180;
                                % now_act = floor(env.n_action/2);
                                now_act = 0;
                                
                                true_state = now_state;
                                action_buff = zeros(delaybuffAction);
                                
                                for iDelay=1:delaybuffSensory
                                        state_buff(:,iDelay) = now_state;
                                end
                                    
                                % State history for this run
                                state_history = zeros( 6, n );
                                
                                env.reset_pt(true_state)
                                pid.reset_pid()
                                
                                for i=1:n
                                
                                    action_buff(delaybuffAction) = now_act;
                                    delayed_act = action_buff(1);
                                    action_buff(1) = [];
                                    
                                    state_buff(:,delaybuffSensory) = now_state;
                                    delayed_state = state_buff(:,1);
                                    state_buff(:,1) = [];
                                        
                                
                                    % Choose the action
                                    if mod(i,decisionLoop) == 1
                                
            %                             % Save the discretized state that the current decision is being
            %                             % made for
            %                             act_prev = now_act;
            %                             disc_prev = delayed_disc_th;
            %                             disc_prevdot = delayed_disc_thdot;
            %                             q_ind_prev = qagent.qind(disc_prevdot, disc_prev);
            %                     
            %                             % Get the current Q index - this will get saved
            %                     
            %                             [delayed_disc_th, delayed_disc_thdot] = env.find_state(delayed_state);
            %                       
            %                     
            %                             if isempty( delayed_disc_thdot )
            %                                 delayed_disc_thdot = 20;
            %                             end
            %                             delayed_q_ind = qagent.qind(delayed_disc_thdot, delayed_disc_th);
            %                             
            %                             now_act = qagent.policy(delayed_q_ind);  
            %                     
            %                             true_torque = env.n_acts( delayed_act );
                                        c_torque = pid.ControlOut(now_state(2));
                                        
            
                                    end
                                
                                    if sign(c_torque) == -1
                                        MVC = 100;
                                    else
                                        MVC = 40;
                                    end
                                
                                    true_torque = env.muscle_dyn (c_torque,MVC, 0.004);
                                
                                    ps_torque = env.passive_torque(true_state);
                                
                                    [now_state, true_state] = env.dynamics_test ...
                                        ( true_state, true_torque, ...
                                        ps_torque, i, iTest, dt );
                                
                                    failure_flag = env.failure_check(delayed_state);
                                
                                    state_history( 1:2, i ) = true_state;
                                    state_history( 3, i ) = true_torque;
                                    state_history( 4:5, i ) = delayed_state;
                                    state_history( 6, i ) = c_torque;
                                
                                    if failure_flag == 1
                                        disp( "Failed" );
                                        result_char = 'f';
                                        break;
                                    end
                                end
            
                               
                                if result_char == 's'
                                    disp("Successful")
                                end
            
                                figure('visible','off');
                                x0=300;
                                y0=300;
                                width=800;
                                height=350;  
                                set(gcf,'position',[x0,y0,width,height])   
                                t = 0:dt:(n-1)*dt;
                                plot(t, state_history(2,:)*180/pi )
                                nameFigure = horzcat('PendulumBehavior',num2str(iTest),...
                                    result_char);
                                title(nameFigure,'fontsize',15)
                                xlabel('Time(s)','fontsize',12)
                                ylabel('Pendulum Angle (deg)','fontsize',12) 
                                ylim(env.th_range)
                                formatFile = horzcat(mainAddress, Description, '\', ...
                                    nameFigure);
                                saveas(gcf,horzcat(formatFile,'.png'))
                                save(formatFile,'state_history')
                                
            
                            end
                        end
                    end
                end
    %                 % Policy Map
    %                 
    %                 for ith = 1:env.n_theta
    %                     for ithdot = 1:env.n_thetadot
    %                 
    %                         delayed_q_ind = qagent.qind(ithdot, ith);
    %                         
    %                         actMap(ith,ithdot) = env.n_acts(qagent.policy(delayed_q_ind));
    %                 
    %                     end
    %                 end
    %                 
    % 
    %                 
    %                 figure('visible','off');
    %                 cDataa = actMap;
    %                 x0=100;
    %                 y0=100;
    %                 width=400;
    %                 height=200;        
    %                 set(gcf,'position',[x0,y0,width,height])
    %                 x = 1:env.n_theta;
    %                 y = 1:env.n_thetadot;
    %                 imagesc(x,y,cDataa'); colormap jet; axis xy;
    %                 hcb = colorbar;
    %                 maxLim = max(max(cDataa));
    %                 minLim = min(min(cDataa));
    %                 caxis([minLim, maxLim])
    %                 title(hcb,'action','fontsize',12)
    %                 nameFigure = 'policyMap';
    %                 title(nameFigure,'fontsize',15)
    %                 xlabel('Theta','fontsize',13)
    %                 ylabel('ThetaDot','fontsize',13)      
    %                 formatFile = horzcat(mainAddress, Description, '\', ...
    %                     nameFigure);
    %                 saveas(gcf,horzcat(formatFile,'.png'))
    % 
    %                 save(formatFile,'actMap')
    %                 save(formatFile, 'q_changes')
    % 
    %                 q_changes(end)
                    
                
            end
        end
    end
end