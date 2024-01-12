function simulation_stop(connection)
        connection.vrep.simxStopSimulation(connection.clientID,connection.vrep.simx_opmode_oneshot_wait);
        connection.vrep.simxStopSimulation(connection.clientID,connection.vrep.simx_opmode_oneshot_wait);
        connection.vrep.simxStopSimulation(connection.clientID,connection.vrep.simx_opmode_oneshot_wait);
end