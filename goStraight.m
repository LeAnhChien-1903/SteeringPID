function goStraight(sim, clientID, left_Motor, right_Motor, speed, direction, time)
% direction 1 is forward 2 is backward
if direction == 1
sim.simxGetFloatSignal(clientID,'SimulationTime',sim.simx_opmode_buffer);
sim.simxSetJointTargetVelocity( clientID,left_Motor,speed,sim.simx_opmode_blocking);
sim.simxSetJointTargetVelocity( clientID,right_Motor,speed,sim.simx_opmode_blocking);
end
if direction == 2
sim.simxGetFloatSignal(clientID,'SimulationTime',sim.simx_opmode_buffer);
sim.simxSetJointTargetVelocity( clientID,left_Motor,-speed,sim.simx_opmode_blocking);
sim.simxSetJointTargetVelocity( clientID,right_Motor,-speed,sim.simx_opmode_blocking);
end
pause(time)
end