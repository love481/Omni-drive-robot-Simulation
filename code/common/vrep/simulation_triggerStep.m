function simulation_triggerStep(connection)
        connection.vrep.simxSynchronousTrigger(connection.clientID);
end