function [bodyDiameter ,wheelDiameter ,interWheelDist] = omni_init(connection)
%OMNI_INIT Summary of this function goes here
% start some data streaming from V-REP to Matlab: 
connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_encoders',num2str(connection.robotNb)),connection.vrep.simx_opmode_streaming);
connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_velocities',num2str(connection.robotNb)),connection.vrep.simx_opmode_streaming);
connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_pose',num2str(connection.robotNb)),connection.vrep.simx_opmode_streaming);
connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_laserData',num2str(connection.robotNb)),connection.vrep.simx_opmode_streaming);
if nargout > 0
            [result,bodyDiameter]=connection.vrep.simxGetFloatSignal(connection.clientID,strcat('omni_bodyDiameter',num2str(connection.robotNb)),connection.vrep.simx_opmode_oneshot_wait);
            if (result~=connection.vrep.simx_error_noerror)
                err = MException('VREP:RemoteApiError', ...
                                'simxGetFloatSignal failed');
                throw(err);
            end
end
        
        if nargout > 1
            [result,wheelDiameter]=connection.vrep.simxGetFloatSignal(connection.clientID,strcat('omni_wheelDiameter',num2str(connection.robotNb)),connection.vrep.simx_opmode_oneshot_wait);
            if (result~=connection.vrep.simx_error_noerror)
                err = MException('VREP:RemoteApiError', ...
                                'simxGetFloatSignal failed');
                throw(err);
            end
        end
        
        if nargout > 2
            interWheelDist=sqrt(2)*bodyDiameter;
            if (result~=connection.vrep.simx_error_noerror)
                err = MException('VREP:RemoteApiError', ...
                                'simxGetFloatSignal failed');
                throw(err);
            end
        end
        
end

