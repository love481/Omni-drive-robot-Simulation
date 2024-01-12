function [scannerPose] = omni_getScannerPose(connection)
        [result,data]=connection.vrep.simxGetStringSignal(connection.clientID,strcat('omni_laserScannerPose',num2str(connection.robotNb)),connection.vrep.simx_opmode_oneshot_wait);
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
        scannerPose=connection.vrep.simxUnpackFloats(data);
end