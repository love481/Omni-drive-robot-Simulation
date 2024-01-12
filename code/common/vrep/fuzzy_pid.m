classdef fuzzy_pid
    %FUZZY_PID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=private)
        error=0;
        d_error=0;
        I_error=0;
        II_error=0;
        fis=readfis('fuzzy_controller2.fis');
        kp=0;
        ki=0;
        kd=0;
        output=0;
    end
    methods  
        function obj = fuzzy_pid(kp_init,ki_init,kd_init)
            obj.kp=kp_init;
            obj.ki=ki_init;
            obj.kd=kd_init;
            
        end
        function out = compute_speed(obj,setpoint,input)
                obj.error=setpoint-input;
                obj.d_error=(obj.error-obj.I_error);
                %fprintf('%f %f\n',obj.error, obj.d_error);
                del=evalfis(obj.fis,[obj.error,obj.d_error]);
                obj.kp=obj.kp+del(1);
                obj.ki=obj.ki+del(2);
                obj.kd=obj.kd+del(3);
                 if obj.kp<0
                      obj.kp=0;
                  end
                  if obj.ki<0
                      obj.ki=0;
                  end
                  if obj.kd<0
                      obj.kd=0;
                  end
                A=obj.kp+obj.ki+obj.kd;
                B=obj.ki-obj.kp-2*obj.kd;
                obj.output=obj.output+A*obj.error+B*obj.I_error+obj.kd*obj.II_error;
                obj.II_error=obj.I_error;
                obj.I_error=obj.error;
                out=obj.output;
        end
    end
end

