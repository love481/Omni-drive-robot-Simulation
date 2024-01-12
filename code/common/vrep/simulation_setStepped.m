function simulation_setStepped(connection,steppedSimulation)
        connection.vrep.simxSynchronous(connection.clientID,steppedSimulation);
end