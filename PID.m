classdef PID < handle
    properties (Constant)
        
        % Timing constant
        dt = 0.004

    end

    properties (Access = public)
        
        % constants for the controller
        SetPoint = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        error_prev = 0;
        integral_error = 0;
        
    end

    methods
        
        function reset_pid(obj)
            
            % This will reset the PID
            obj.error_prev = 0;
            obj.integral_error = 0;

        end
        
        function true_torque = ControlOut (obj, perceived_state)
            
            % Proportional part of the PID controller.
            error = obj.SetPoint - perceived_state;
            
            % Derivative part of the PID controller.
            dervivative = (error - obj.error_prev)/obj.dt;
            
            % Integral part of the PID controller.
            integral = obj.integral_error + error * obj.dt;
            
            % The raw exerted torque based on the controller
            true_torque = obj.Kp * error + obj.Ki * integral + obj.Kd * dervivative;

            % Storing variables for the next step.
            obj.integral_error = integral;
            obj.error_prev = error;

        end

    end

end
