function simulation_pause(connection)
        connection.vrep.simxPauseSimulation(connection.clientID,connection.vrep.simx_opmode_oneshot_wait);

end