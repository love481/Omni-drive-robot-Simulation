function [motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder ] = omni_getEncoders(connection)
%OMNI_GETENCODERS Summary of this function goes here
        [ result,data]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_encoders',num2str(connection.robotNb)),connection.vrep.simx_opmode_buffer);
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
        enc=connection.vrep.simxUnpackFloats(data);
	motor1_encoder=enc(1);  %rad
	motor2_encoder=enc(2); %rad
    motor3_encoder=enc(3); %rad
    motor4_encoder=enc(4); %rad
end

