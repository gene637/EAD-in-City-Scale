from EAD_regressor_new import *

import numpy as np
import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import datetime


def calculate_pass_time(veh_id, tlsid):
    """
    Calculate the time interval during which a vehicle can pass through a traffic light.

    :param veh_id: The ID of the vehicle.
    :param tlsid: The ID of the traffic light system.
    :return: A tuple (Tstart, Tend) representing the start and end time intervals during which the vehicle can pass.
    """
    
    # Get the index of the traffic light and the current state of the vehicle's lane
    tls_index = traci.vehicle.getNextTLS(veh_id)[0][1]  # Traffic light index for the vehicle
    vehicle_state = traci.vehicle.getNextTLS(veh_id)[0][3]  # Current state for the vehicle's lane

    # Get all the traffic light logic
    tl_log = traci.trafficlight.getAllProgramLogics(tlsid)
    program = traci.trafficlight.getProgram(tlsid)
    for i in range(len(tl_log)):
        if tl_log[i].programID == program:
            phases = tl_log[i].phases

    # Get the current phase and state of the traffic light
    current_phase = traci.trafficlight.getPhase(tlsid)
    current_state = traci.trafficlight.getRedYellowGreenState(tlsid)
    next_switch = traci.trafficlight.getNextSwitch(tlsid)
    
    # Retrieve phase information from tl_log
    current_duration = phases[current_phase].duration
    phase_states = [phase.state for phase in phases]
    
    # Calculate the current time and time to the next signal change
    current_time = traci.simulation.getTime()
    Cycle= sum(phase.duration for phase in phases)
    
    # Initialize Tstart and Tend
    Tstart = None
    Tend = None
    TGy = None

    if vehicle_state in 'GgYy':
        # If the current state is green or yellow
        Tstart = 0
        time_to_next_red = 0
        time_to_last_red = 0
        remaining_time_in_current_phase = next_switch - current_time
        
        # Find the time to the next red light
        for i in range(current_phase + 1, len(phases) + current_phase + 1):
            next_phase_index = i % len(phases)
            next_phase_state = phases[next_phase_index].state
            
            if 'r' in next_phase_state[tls_index]:
                break
            else:
                time_to_next_red += phases[next_phase_index].duration
        
        Tend = time_to_next_red + remaining_time_in_current_phase

        # Find the time of Green and yellow duration
        for i in range(current_phase - 1, current_phase - 1 - len(phases)):
            last_phase_index = i % len(phases)
            last_phase_state = phases[last_phase_index].state
            
            if 'r' in last_phase_state[tls_index]:
                break
            else:
                time_to_last_red += phases[last_phase_index].duration
        TGy = time_to_last_red + current_duration + time_to_next_red
    
    elif vehicle_state == 'r':
        # If the current state is red
        time_to_next_green = 0
        remaining_time_in_current_phase = next_switch - current_time
        
        # Find the time to the next green light
        for i in range(current_phase + 1, len(phases) + current_phase + 1):
            next_phase_index = i % len(phases)
            next_phase_state = phases[next_phase_index].state
            
            if 'G' in next_phase_state[tls_index] or 'g' in next_phase_state[tls_index]:
                Tstart = time_to_next_green + remaining_time_in_current_phase
                time_to_next_red2 = phases[next_phase_index].duration
                # Find the time to the next red light after green light
                for j in range(i + 1, len(phases) + current_phase + 1):
                    next_phase_index2 = j % len(phases)
                    next_phase_state2 = phases[next_phase_index2].state                    
                    if 'r' in next_phase_state2[tls_index]:
                        Tend = Tstart + time_to_next_red2
                        break
                    else:
                        time_to_next_red2 += phases[next_phase_index2].duration
            else:
                time_to_next_green += phases[next_phase_index].duration

        TGy = Tend - Tstart
    
    return Tstart, Tend, Cycle, TGy


def EAD_acceleration(veh_id, speed, last_speed, vehType, vehQs, EV):
    #  ffv for free flow velocity, clg for car-length + gap.

    ffv = 18
    clg = 7.5
    laneid = traci.vehicle.getLaneID(veh_id)
    tlsid = traci.vehicle.getNextTLS(veh_id)[0][0]
    dist = traci.vehicle.getNextTLS(veh_id)[0][2]

    if 20 <= dist <= 750:

        Tstart, Tend, Cycle, TGy = calculate_pass_time(veh_id, tlsid)
        # print(f"Tstart: {Tstart}, Tend: {Tend}")
        passing_interval = [Tstart, Tend]
        # Velocity Prediction
        minimumTimeToPass = passing_interval[0]
        maximumTimeToPass = passing_interval[1]
        initialVelocity = speed
        distanceToTravel = dist

        for i in range(len(vehQs)):
            if vehQs[i][0] == tlsid and vehQs[i][1] == laneid:
                tmp = (vehQs[i][2] * clg) / (ffv / 2)
                minimumTimeToPass = minimumTimeToPass + tmp

        roundVelocity = round(initialVelocity * 2) / 2
        if maximumTimeToPass > 0:
            # predicted_velocity = initialVelocity
            # if veh_id == 'veh51':
            #     print(distanceToTravel / maximumTimeToPass)
            #     print(minimumTimeToPass)
            #     print(maximumTimeToPass)
            if (distanceToTravel / maximumTimeToPass) <= ffv:
                predicted_velocity = predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, roundVelocity, distanceToTravel, EV) - roundVelocity + initialVelocity
                # print("predicted 1: " + str(predicted_velocity))
                predict = 1
            else:
                predicted_velocity = predict_velocity_v1(maximumTimeToPass + (Cycle - TGy), maximumTimeToPass + Cycle, roundVelocity, distanceToTravel, EV) - roundVelocity + initialVelocity
                predict = 2

        else:
            predicted_velocity = initialVelocity
            predict = 3
        traci.vehicle.setSpeed(veh_id, predicted_velocity)

    else:
        traci.vehicle.setSpeedMode(veh_id, 29)
        traci.vehicle.setSpeed(veh_id, -1)

    speed = traci.vehicle.getSpeed(veh_id)
    acc = speed - last_speed

    last_speed = speed
    return acc, last_speed