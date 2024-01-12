function [x_s,y_s,theta_s] = omni_getPose(connection)
%OMNI_GETPOSE Summary of this function goes here
        [result,data]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_pose',num2str(connection.robotNb)),connection.vrep.simx_opmode_buffer);
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
        pose=connection.vrep.simxUnpackFloats(data);
	x_s=pose(1);
	y_s=pose(2);
	theta_s=pose(3); % rad
end

