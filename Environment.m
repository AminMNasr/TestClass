% Q-learning AP RL model for standing balance.
% Writer: Amin Nasr - SMPLab - amin.nasr@ubc.ca

% To see the comments please check the environment class for the RL
% algorithm

classdef Environment < handle
    
    properties (Constant)


        th_range = [-3, 6];
        thdot_range = [-15, 15];
        act_range = [-100, 50];

        m = 78;
        h = 1.79*0.52;
        I = Environment.m*Environment.h*Environment.h;
        g = 9.81;
        c = 5.73;
        
        num_samples = 250*60*1;

        m_noise = pinknoise(Environment.num_samples,1000);
        t_noise = pinknoise(Environment.num_samples,1000);
        td_noise = pinknoise(Environment.num_samples,1000);
    end

    properties (Access = public)

        prevVelocitySign = 1;
        prevPStorque = 0;
        prevReversal = 0;
        prevPStorqueHistory = 0;
        prev_act = 0;

        mn_coeff = 0.02;
        thn_coeff = 0.003;
        thdotn_coeff = 0.001;
         
        motor_noise  = Environment.m_noise ./ rms(Environment.m_noise);
        th_noise  = Environment.t_noise ./ rms(Environment.t_noise);
        thdot_noise  = Environment.td_noise ./ rms(Environment.td_noise);

        % Loading the noises for the tests. 
        motor_noiseTest  = readmatrix("Noise1.txt");
        th_noiseTest  = readmatrix("Noise2.txt");
        thdot_noiseTest  = readmatrix("Noise3.txt");

    end

    methods

        function psTorque = passive_torque(obj, now_state )
            
            th = now_state(2);
            thdot = now_state(1);

            rotationSign = sign(thdot);
            if rotationSign == 0
                rotationSign = 1;
            end
            if rotationSign ~= obj.prevVelocitySign 
                obj.prevReversal = th;
                obj.prevVelocitySign = rotationSign;
                obj.prevPStorque = obj.prevPStorqueHistory;
            end
            
            if abs(th - obj.prevReversal) <= 0.03 / 180 * pi              
                rotation = rotationSign * 0.03 / 180 * pi;
            else
                rotation = th - obj.prevReversal;
            end
            
            coeffT = 0.467 * abs(rotation)^(-0.334) * 2 * obj.m * 180 / pi * ...
                obj.g * obj.h / (11 * 180/pi);

            psTorque = coeffT * rotation + obj.prevPStorque;

            obj.prevPStorqueHistory = psTorque;
            
            

        end

        function reset_pt(obj, now_state)

            obj.prevVelocitySign = sign(now_state(1));
            obj.prevPStorque = 0;
            obj.prevReversal = 0;
            obj.prevPStorqueHistory = 0;
            obj.prev_act = 0;
            
        end

                function [perceived_state, next_state, exerted_torque] = ...
                dynamics_test(obj, ...
                true_state, true_torque, ps_torque, t, num_episode, dt)
            
            if true_torque < obj.act_range(1)
                true_torque = obj.act_range(1);
            end

            if true_torque > obj.act_range(2)
                true_torque = obj.act_range(2);
            end

            if true_torque < 0 
                mn_added = obj.mn_coeff * abs(true_torque) * 0.77 * ...
                    obj.motor_noiseTest(t,(mod(num_episode,1000)));
            else
                mn_added = obj.mn_coeff * abs(true_torque) * ...
                    obj.motor_noiseTest(t,(mod(num_episode,1000)));
            end
            
            exerted_torque = true_torque + mn_added;

            A = [-obj.c/obj.I, obj.m*obj.g*obj.h/obj.I; 1, 0];
            B = [1/obj.I; 0];
            C = [-1/obj.I; 0];
            
            diff_state = A * true_state + B * exerted_torque + C * ps_torque;
        
            next_state = diff_state * dt + true_state;
            
            tdn_added = obj.thdotn_coeff * ...
                obj.thdot_noiseTest(t,(mod(num_episode,1000)));

            perceived_state(1,1) = next_state(1) + tdn_added;

            tn_added = obj.thn_coeff * ...
                obj.th_noiseTest(t,(mod(num_episode,1000)));

            perceived_state(2,1) = next_state(2) + tn_added;

        end

        function [perceived_state, next_state] = dynamics(obj, ...
                true_state, true_torque, ps_torque, t, num_episode, dt)
            
            if true_torque < obj.act_range(1)
                true_torque = obj.act_range(1);
            end

            if true_torque > obj.act_range(2)
                true_torque = obj.act_range(2);
            end

            if true_torque < 0 
                mn_added = obj.mn_coeff * abs(true_torque) * 0.77 * ...
                    obj.motor_noise(t,(mod(num_episode,1000)+1));
            else
                mn_added = obj.mn_coeff * abs(true_torque) * ...
                    obj.motor_noise(t,(mod(num_episode,1000)+1));
            end
            
            exerted_torque = true_torque + mn_added;

            A = [-obj.c/obj.I, obj.m*obj.g*obj.h/obj.I; 1, 0];
            B = [1/obj.I; 0];
            C = [-1/obj.I; 0];
            
            diff_state = A * true_state + B * exerted_torque + C * ps_torque;
        
            next_state = diff_state * dt + true_state;
            
            tdn_added = obj.thdotn_coeff * ...
                obj.thdot_noise(t,(mod(num_episode,1000)+1));

            perceived_state(1,1) = next_state(1) + tdn_added;

            tn_added = obj.thn_coeff * ...
                obj.th_noise(t,(mod(num_episode,1000)+1));

            perceived_state(2,1) = next_state(2) + tn_added;
%             perceived_state = perceived_state';

        end

        function failure_flag = failure_check(obj, now_state)
            failure_flag = 0;
            if now_state(2) > obj.th_range(2) /180*pi || now_state(2) < ...
                    obj.th_range(1)/180*pi
                failure_flag = 1;
            end

        end

        function torque_final = muscle_dyn(obj, true_torque, MVC, dt)

            activation = true_torque/MVC;
            if sign(true_torque) == -1
                prevAct = 0;
            else
                prevAct = obj.prev_act;
            end

            if abs(activation) >= abs(prevAct)
                tau = 0.01 * (0.5 + 1.5 * abs(prevAct));
            else
                tau = 0.04 / (0.5 + 1.5 * abs(prevAct));
            end

            output = (abs(activation) - abs(prevAct)) / tau * dt ...
                + abs(prevAct);
            
            torque_final = sign(true_torque) * output * MVC;

            obj.prev_act = activation;
        end

        % This function will initiate the tests and randomly assign a theta
        % and theta dot to the inverted pendulum. This will assure that the
        % initiation dynamically will not result in the failure. 
        function state = initiate_test(obj, totalDelay) 
            
            % In this part, the offset angle because of delay will be
            % computed in the worst case scenario. 
            gTmax = obj.m * obj.g * obj.h * obj.th_range(2) * pi / 180;            
            galpha = gTmax/obj.I;
            Vmax = galpha * totalDelay / 1000;
            deltaThDelay = Vmax ^ 2 / 2 / galpha;
            TBackward = abs(obj.act_range(1))-gTmax;
            balpha = TBackward/obj.I;
            deltaThTorque = Vmax ^ 2 / 2 / balpha;
            dThTot1 = (deltaThTorque + deltaThDelay)* 180/pi;            
            thetaRange = [obj.th_range(1) + dThTot1, obj.th_range(2) - dThTot1];
            
            % In this part, the offset for the lunching the pendulum
            % through the edge (velocity to fall) will be computed. 
            gTmin = obj.m * obj.g * obj.h * (obj.th_range(2)-1) * pi / 180;
            TBackwardMax = abs(obj.act_range(1))-gTmin;
            balphaMax = TBackwardMax/obj.I;
            dThMaxDot = (obj.thdot_range(2)*pi/180) ^ 2 / 2 / balphaMax;
            dThTot2 = (deltaThDelay + dThMaxDot) * 180/pi;
            thInnerLim = [obj.th_range(1) + dThTot2, ...
                obj.th_range(2) - dThTot2];
            
            % Now I will randomize the theta. The theta will be in the
            % range in the thetaRange variable. 
            th_rand = mean(thetaRange) + ...
                (max(thetaRange) - min(thetaRange)) / 8 * randn(1,1);

            % Based on the theta, the theta dot will be initiated. Theta
            % dot will not be initiated in the edge direction at the end of
            % the range theta. The ability to have velocities in the
            % direction of falling will come back linearly when the theta is far
            % from the edge. In the thInnerLim, the theta that the theta
            % dot could be initiated in the full range is shown. 
            if th_rand < thInnerLim(2)
                vel_range(2) = obj.thdot_range(2);
            else
                mLine = obj.thdot_range(2)/(thInnerLim(2)-thetaRange(2));
                bLine = - mLine * thetaRange(2);
                vel_range(2) = mLine * th_rand + bLine;
            end

            if th_rand > thInnerLim(1)
                vel_range(1) = obj.thdot_range(1);
            else
                mLine = obj.thdot_range(1)/(thInnerLim(1)-thetaRange(1));
                bLine = - mLine * thetaRange(1);
                vel_range(1) = mLine * th_rand + bLine;
            end
            
            % Randomized theta dot based on the range that is possible
            % according to the theta. 
            thDot_rand = mean(vel_range) + ...
                (max(vel_range) - min(vel_range)) / 6 * randn(1,1);

            state = [thDot_rand; th_rand];

        end

    end

end
