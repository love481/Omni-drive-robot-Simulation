function [] = simulation_closeConnection(connection)
        connection.vrep.simxFinish(connection.clientID);
end