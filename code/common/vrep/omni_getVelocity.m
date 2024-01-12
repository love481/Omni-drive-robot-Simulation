function [x_dot,y_dot,omega] = omni_getVelocity(connection)
%OMNI_GETPOSE Summary of this function goes here
        [result,data]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_velocity',num2str(connection.robotNb)),connection.vrep.simx_opmode_buffer);
        if (result~=connection.vrep.simx_error_noerror)
            err = MException('VREP:RemoteApiError', ...
                            'simxGetStringSignal failed');
            throw(err);
        end
        if(isempty(data))
            err = MException('VREP:RemoteApiError', ...
                'Empty data returned');
            throw(err);
        end
        v=connection.vrep.simxUnpackFloats(data);
	x_dot=v(1);
	y_dot=v(2);
	omega=v(3); % rad/sec
end

