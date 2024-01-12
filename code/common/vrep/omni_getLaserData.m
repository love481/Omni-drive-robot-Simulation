function [laserDataX, laserDataY] = omni_getLaserData(connection)
        [result,data]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_laserData',num2str(connection.robotNb)),connection.vrep.simx_opmode_buffer);
        if (result~=connection.vrep.simx_error_noerror)
            err = MException('VREP:RemoteApiError', ...
                            'simxGetStringSignal failed');
            throw(err);
        end
        laserData=connection.vrep.simxUnpackFloats(data);
	laserDataX=laserData(1:2:end-1);
    laserDataY=laserData(2:2:end);
end