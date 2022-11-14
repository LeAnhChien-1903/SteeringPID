function turn(sim, clientID, left_Motor, right_Motor, velocity, orientation)
% direction 1 is turn left 0 is turn right
if velocity > 5
    velocity = 5;
end
if orientation == 0
    sim.simxGetFloatSignal(clientID,'SimulationTime',sim.simx_opmode_buffer);
    sim.simxSetJointTargetVelocity( clientID,left_Motor,velocity,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity( clientID,right_Motor,0,sim.simx_opmode_blocking);
else
    sim.simxGetFloatSignal(clientID,'SimulationTime',sim.simx_opmode_buffer);
    sim.simxSetJointTargetVelocity( clientID,left_Motor,0,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity( clientID,right_Motor,velocity,sim.simx_opmode_blocking);
end

end