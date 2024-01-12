classdef user_pid_continuous
    %continuous_PID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=private)
        error=0;
        p_error=0;
        d_error=0;
        I_error=0;
        kp=0;
        ki=0;
        kd=0;
        output=0;
    end
    methods  
        function obj = user_pid_continuous(kp_init,ki_init,kd_init)
            obj.kp=kp_init;
            obj.ki=ki_init;
            obj.kd=kd_init;
            
        end
        function out = compute_speed(obj,setpoint,input)
                obj.error=setpoint-input;
                obj.d_error=obj.error-obj.p_error;
                obj.I_error=obj.I_error + obj.error;
                obj.output=obj.kp*obj.error + obj.ki* obj.I_error + obj.kd* obj.d_error;
                obj.p_error=obj.error;
                out=obj.output;
        end
    end
end


