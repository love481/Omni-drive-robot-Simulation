function [motor1Velocity,motor2Velocity,motor3Velocity,motor4Velocity] = omni_getWheelSpeeds(connection)
%omni_getWheelSpeeds Summary of this function goes here
        [ result,data ]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_velocities',num2str(connection.robotNb)),connection.vrep.simx_opmode_buffer);
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
        vel=connection.vrep.simxUnpackFloats(data);
    motor1Velocity=vel(1);
    motor2Velocity=vel(2);
    motor3Velocity=vel(3);
    motor4Velocity=vel(4);
end

