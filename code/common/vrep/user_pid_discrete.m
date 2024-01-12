classdef user_pid_discrete
    %DISCRETE_PID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=private)
        error=0;
        I_error=0;
        II_error=0;
        kp=0;
        ki=0;
        kd=0;
        output=0;
    end
    methods  
        function obj = user_pid_discrete(kp_init,ki_init,kd_init)
            obj.kp=kp_init;
            obj.ki=ki_init;
            obj.kd=kd_init;
            
        end
        function out = compute_speed(obj,setpoint,input)
                obj.error=setpoint-input;
                obj.I_error=obj.I_error + obj.error;
                %%algorithm taken from the paper or can be obtained
                %%mathematically
                A=obj.kp+obj.ki+obj.kd;
                B=obj.ki-obj.kp-2*obj.kd;
                obj.output=obj.output+A*obj.error+B*obj.I_error+obj.kd*obj.II_error;
                obj.II_error=obj.I_error;
                obj.I_error=obj.error;
                out=obj.output;
        end
    end
end


