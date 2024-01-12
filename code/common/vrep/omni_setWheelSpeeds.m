function omni_setWheelSpeeds(connection,motor1Velocity,motor2Velocity,motor3Velocity,motor4Velocity)
signalValue=connection.vrep.simxPackFloats([motor1Velocity,motor2Velocity,motor3Velocity,motor4Velocity]);
connection.vrep.simxSetStringSignal(connection.clientID,strcat('omni_reqVelocities',num2str(connection.robotNb)),signalValue,connection.vrep.simx_opmode_oneshot);
end
