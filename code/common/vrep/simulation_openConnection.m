function [connection] = simulation_openConnection(connection, robotNb)
%SIMULATION_OPENCONNECTION Opens a remote connection to V-REP. Note that
%the V-REP simulation has to be running at this point.

    connection.robotNb = robotNb;
    
     % to properly close the last opened connection if it wasn't closed:
    global lastConnectionId;
    if ~isempty(lastConnectionId)
            connection.vrep.simxFinish(lastConnectionId);
    end

    % 19997 is the port specified in remoteApiConnections.txt in the V-Rep
    % folder.
    % Additionally V-Rep opens port 19997 on simulation start in the omni_robot
    % script. You could also use that port here, but then simulation has to
    % be started manually before connecting through MATLAB.

    connection.clientID=connection.vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    if (connection.clientID<=-1)
        err = MException('VREP:RemoteApiError', ...
                        'Could not open connection');
        throw(err);
    else
        disp('Connection is successfull!');
    end
    lastConnectionId = connection.clientID;
end