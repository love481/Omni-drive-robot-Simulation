classdef fuzzy_pid2
    %FUZZY_PID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=private)
        error=0;
        d_error=0;
        I_error=0;
        II_error=0;
        fis = convertToType2(readfis('IT1FUZZY.fis'));
        kp=0;
        ki=0;
        kd=0;
        output=0;
    end
    methods  
        function obj = fuzzy_pid2(kp_init,ki_init,kd_init)
            obj.kp=kp_init;
            obj.ki=ki_init;
            obj.kd=kd_init;
            for i = 1:length(obj.fis.Inputs)
                for j = 1:length(obj.fis.Inputs(i).MembershipFunctions)
                    obj.fis.Inputs(i).MembershipFunctions(j).LowerScale = 0.7;
                    obj.fis.Inputs(i).MembershipFunctions(j).LowerLag = 0.2;
                end
            end
            % 
            % obj.fis.AndMethod = "prod";
            % obj.fis.ImplicationMethod = "prod";
            %obj.fis.TypeReductionMethod="eiasc";
            
        end
        function out = compute_speed(obj,setpoint,input,error_gain,d_error_gain,out_gain)
                obj.error=setpoint-input;
                obj.d_error=obj.error-obj.I_error;
                %fprintf('%f %f\n',obj.error, obj.d_error);
                del=evalfis(obj.fis,[obj.error*error_gain,obj.d_error*d_error_gain]);
                del = del * out_gain;
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

