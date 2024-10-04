from EAD_regressor import *

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

# PEN_RATE = 1
# RandSeed = 42
# CACCstatus = 'Off'
# Lanelvl = 'Off'
#
# save_path = 'C:/Users/rdosw/Documents/SUMO/Eco-TOps/Data2/OutputData/EcoTops/Normal'
# EAD_filename = 'EAD_' + str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '_') + '_PenRate_' + str(PEN_RATE*100) + '_RandSeed_' + str(RandSeed) + '_CACC_' + CACCstatus + '_lanelvl_' + Lanelvl
# complete_EADfile = os.path.join(save_path, EAD_filename + ".txt")
# f2 = open(complete_EADfile, "w")
# f2.close()

# need these variables in global scope:
pK = 0
rK = 0
lK = 0
lT = 0

#  09/29/2021 David Oswald: changed function to have angles and new phase times (nPT).
#  Also added in all four directions. Previously function only calculated EAD for East to West
#  Also changed state from traci.trafficlight.getRedYellowGreenState("0") to traci.trafficlight.getPhase("0"), which
#  gives the index of the red, yellow, green state rather than actual states like rrrrgGGrrrrrgGGr
def EAD_acceleration(veh_position, state, next_switch, simulation_time, speed, ID, last_speed, angle, nPT, vehType, vehQs, prl, lID, rID):
    # Distance to the intersection
    # veh_position = traci.vehicle.getPosition('newVeh')
    # veh_position1 = traci.vehicle.getPosition('newVeh1')
    # print('Position:', veh_position)
    #  Position of stop bar for each direction.
    # global f2
    # f2 = open(complete_EADfile, "a")
    #  f2.write(str(ID) + ',' + str(simulation_time) + ',' + str(veh_position[0]) + ',' + str(veh_position[1]) + ',' + str(speed))

    #  Cy for cycle, AvgG for Average Green time, ffv for free flow velocity, clg for car-length + gap.
    Cy = 66
    AvgG = 15
    ffv = 18
    clg = 7.5
    signal_positionEW = (523.94, 514.8)
    signal_positionWE = (495.97, 505.08)
    signal_positionNS = (504.98, 524.2)
    signal_positionSN = (514.7, 496)
    #  tl_log = traffic light logic
    tl_log = traci.trafficlight.getAllProgramLogics("0")
    #  ylt = yellow light times
    ylt = [tl_log[0].phases[1].duration,
           tl_log[0].phases[3].duration,
           tl_log[0].phases[4].duration,
           tl_log[0].phases[6].duration,
           tl_log[0].phases[8].duration,
           tl_log[0].phases[9].duration]
    global pK
    global rK
    global lK
    global lT
    pK = 0
    rK = 0
    lK = 0
    lT = 0

    # dist = np.sqrt((veh_position[0] - signal_position[0]) ** 2 + (
    #         veh_position[1] - signal_position[1]) ** 2)  # Distance between EV and traffic light
    # print('Distance:', dist)

    # Acceleration and speed
    # acceleration = traci.vehicle.getAcceleration('newVeh')
    #speed = traci.vehicle.getSpeed('newVeh')
    # print(speed)


    if 260 < angle < 280:  # traveling east to west
        if veh_position[0] - signal_positionEW[0] >= 0:  # Before Intersection
            dist = np.sqrt((veh_position[0] - signal_positionEW[0]) ** 2)
            if 20 <= dist <= 750:
                # print(dist)
                #  11/17/2021 David Oswald: added 'or vehType == "typeCAV_R"' to if statement
                if vehType == "typeCAV" or vehType == "typeCAV_R":
                    # Passing Interval
                    if state == 0:
                        remaining_signal_time = next_switch + ylt[0] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 1:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:]) + nPT[2] + nPT[3]
                                                 - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 3:
                        remaining_signal_time = (next_switch + sum(ylt[2:]) + nPT[2] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 4:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + nPT[2] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 5:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 6:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 8:
                        remaining_signal_time = (next_switch + ylt[5] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 9:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                elif vehType == "typeCAV_L":
                    if state == 2:
                        remaining_signal_time = next_switch + ylt[1] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 3:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 4:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + ylt[0] + nPT[2] + nPT[3] + nPT[
                            0] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 5:
                        remaining_signal_time = (
                                    next_switch + sum(ylt[3:]) + ylt[0] + nPT[3] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 6:
                        remaining_signal_time = (
                                    next_switch + sum(ylt[4:]) + ylt[0] + nPT[0] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 8:
                        remaining_signal_time = (next_switch + ylt[5] + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 9:
                        remaining_signal_time = (next_switch + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 0:
                        remaining_signal_time = next_switch + ylt[0] - simulation_time
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 1:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    # print(timeUntilStartOfPassingInterval)
                    # print(timeUntilEndOfPassingInterval)
                passing_interval = [timeUntilStartOfPassingInterval, timeUntilEndOfPassingInterval]
                # print(passing_interval)
                # print(passing_interval[0])

                # Velocity Prediction
                minimumTimeToPass = passing_interval[0]
                maximumTimeToPass = passing_interval[1]
                initialVelocity = speed
                distanceToTravel = dist

                #  David Oswald, 11/02/2021:
                #  Checks the queue of the lane vehicle is in and the lane next to it
                #  if vehicle is not in lane with less vehicles queued, then switch to that lane.
                #  David Oswald, 11/12/2021:
                #  Peng's version of the change lane code:
                ################## Start lane change code #####################################################
                for pNum in range(len(vehQs)):
                    for rNum in range(len(vehQs[pNum])):
                        for lNum in range(len(vehQs[pNum][rNum])):
                            if lID in prl[pNum][rNum][lNum]:
                                pK = pNum
                                rK = rNum
                                lK = lNum + 1

                                lT = len(prl[pNum][rNum])

                if vehType == "typeCAV":
                    if lT > 1:
                        if 50 < dist < 200:
                            if 0 < (lK - 1) < (lT - 1):
                                if vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK] and vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK - 2]:
                                    traci.vehicle.changeLane(ID, lK, 50)
                                else:
                                    if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 2]:
                                        traci.vehicle.changeLane(ID, lK + 1, 50)
                                    else:
                                        traci.vehicle.changeLane(ID, lK - 1, 50)
                            elif (lK - 1) == (lT - 1):
                                if vehQs[pK][rK][lK - 2] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK - 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                            elif (lK - 1) == lT:
                                traci.vehicle.changeLane(ID, lK - 1, 50)
                            else:
                                if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK + 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                        elif dist <= 50:
                            # print("ID and lK and lT:")
                            # print(ID)
                            # print(lK)
                            # print(lT)
                            traci.vehicle.changeLane(ID, lK, 50)
                        elif dist > 200:
                            traci.vehicle.changeLane(ID, lK, 50)
                # if vehType == "typeCAV_L" or vehType == "typeCAV_R":
                #     traci.vehicle.setLaneChangeMode(ID, 256)
                ##############   end lane change code ##########################################################
                tmp = (vehQs[pK][rK][lK - 1] * clg) / (ffv / 2)
                # if rID == "2i" or rID == "2_2i":
                #     tmp = (vehQs[pK][rK][lK-1] * clg) / (ffv/2)
                # else:
                #     tmp = (vehQs[pK][rK][lK] * clg) / (ffv / 2)
                minimumTimeToPass = minimumTimeToPass + tmp

                                # print(vehQs[pNum][rNum])
                                # if max(vehQs[pNum][rNum]) != min(vehQs[pNum][rNum]):
                                #     if min(vehQs[pNum][rNum]) == vehQs[pNum][rNum][lNum]:
                                #         if traci.vehicle.getLaneIndex(ID) != lNum:
                                #             print("changed lanes")
                                #             traci.vehicle.changeLane(ID, lnum, minimumTimeToPass)
                                #         else:
                                #             tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                                #             minimumTimeToPass = minimumTimeToPass + tmp
                                #             print("Temp: " + str(tmp))
                                #             print("Queue: " + str(vehQs[pNum][rNum][lNum]))
                                # else:
                                #     tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                                #     minimumTimeToPass = minimumTimeToPass + tmp
                                #     print("Temp: " + str(tmp))
                                #     print("Queue: " + str(vehQs[pNum][rNum][lNum]))


                roundVelocity = round(initialVelocity * 2) / 2
                if maximumTimeToPass > 0:
                    if (distanceToTravel / maximumTimeToPass) <= ffv:
                        predicted_velocity = predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        # print("predicted 1: " + str(predicted_velocity))
                        predict = 1
                    else:
                        predicted_velocity = predict_velocity_v1(maximumTimeToPass + (Cy - AvgG), maximumTimeToPass + Cy, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        predict = 2
                        # print("predicted 2: " + str(predicted_velocity))
                        # print("cy: " + str(Cy))
                        # print("AvgG: " + str(AvgG))
                else:
                    predicted_velocity = initialVelocity
                    predict = 3
                # print('predicted speed:', predicted_velocity)
                # if vehType == "typeCAV":
                #     print(str(ID) + "," + str(predicted_velocity) + "," + str(speed) + "," + str(dist) + "," + str(minimumTimeToPass) + "," + str(maximumTimeToPass))
                traci.vehicle.setSpeed(ID, predicted_velocity)
                # f2.write(str(ID) + ',' + str(simulation_time) + ',' + str(veh_position[0]) + ',' + str(
                #     veh_position[1]) + ',' + str(speed) + ',' + traci.trafficlight.getRedYellowGreenState("0") + ',' + str(remaining_signal_time) + ',' + str(predicted_velocity) + '\n')
                # print('speed:', traci.vehicle.getSpeed('newVeh'))
                # print('acceleration:', traci.vehicle.getAcceleration('newVeh'))
                # Calculated_Acceleration = predicted_velocity - speed
                # print('calculated acceleration:', Calculated_Acceleration)

                # before intersection #
                # a = Calculated_Acceleration
            elif dist < 20:
                if vehType == "typeCAV_R":
                    traci.vehicle.setSpeedMode(ID, 29)
                    traci.vehicle.setSpeed(ID, -1)
                 # a = traci.vehicle.getAccel(ID)
            # print('before intersection acceleration:', a)

        # after intersection
        else:
            traci.vehicle.setSpeedMode(ID, 29)
            traci.vehicle.setSpeed(ID, -1)
            # traci.vehicle.setMaxSpeed('newVeh', 11)        # Set maximum speed after passing the intersection
            # a = traci.vehicle.getAccel(ID)
            # f2.close()
            # print('after intersection acceleration:', a)
            # if traci.vehicle.getDistance(ID) >= 200:
            #     traci.vehicle.slowDown(ID, 15.0, 0.4)
            #     a = -(traci.vehicle.getSpeed(ID) - 15) / 4

    if 80 < angle < 100:  # traveling west to east
        if veh_position[0] - signal_positionWE[0] <= 0:  # Before Intersection
            dist = np.sqrt((veh_position[0] - signal_positionWE[0]) ** 2)
            if 20 <= dist <= 750:
                if vehType == "typeCAV" or vehType == "typeCAV_R":
                    # Passing Interval
                    if state == 0:
                        remaining_signal_time = next_switch + ylt[0] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                        # print("next switch: " + str(next_switch))
                    elif state == 1:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:]) + nPT[2] + nPT[3]
                                                 - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 3:
                        remaining_signal_time = (next_switch + sum(ylt[2:]) + nPT[2] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 4:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + nPT[2] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 5:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 6:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 8:
                        remaining_signal_time = (next_switch + ylt[5] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                    elif state == 9:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[0]
                elif vehType == "typeCAV_L":
                    if state == 2:
                        remaining_signal_time = next_switch + ylt[1] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 3:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 4:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + ylt[0] + nPT[2] + nPT[3] + nPT[0] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 5:
                        remaining_signal_time = (next_switch + sum(ylt[3:]) + ylt[0] + nPT[3] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 6:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + ylt[0] + nPT[0] + nPT[3] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 8:
                        remaining_signal_time = (next_switch + ylt[5] + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 9:
                        remaining_signal_time = (next_switch + ylt[0] + nPT[0] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 0:
                        remaining_signal_time = next_switch + ylt[0] - simulation_time
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                    elif state == 1:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[1]
                # print(timeUntilStartOfPassingInterval)
                # print(timeUntilEndOfPassingInterval)
                passing_interval = [timeUntilStartOfPassingInterval, timeUntilEndOfPassingInterval]
                # print(passing_interval)
                # print(passing_interval[0])

                # Velocity Prediction
                minimumTimeToPass = passing_interval[0]
                maximumTimeToPass = passing_interval[1]
                initialVelocity = speed
                distanceToTravel = dist

                ################## Start lane change code #####################################################
                for pNum in range(len(vehQs)):
                    for rNum in range(len(vehQs[pNum])):
                        for lNum in range(len(vehQs[pNum][rNum])):
                            if lID in prl[pNum][rNum][lNum]:
                                pK = pNum
                                rK = rNum
                                lK = lNum + 1
                                lT = len(prl[pNum][rNum])
                if vehType == "typeCAV":
                    if lT > 1:
                        if 50 < dist < 200:
                            if 0 < (lK - 1) < (lT - 1):
                                if vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK] and vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK - 2]:
                                    traci.vehicle.changeLane(ID, lK, 50)
                                else:
                                    if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 2]:
                                        traci.vehicle.changeLane(ID, lK + 1, 50)
                                    else:
                                        traci.vehicle.changeLane(ID, lK - 1, 50)
                            elif (lK - 1) == (lT - 1):
                                if vehQs[pK][rK][lK - 2] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK - 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                            elif (lK - 1) == lT:
                                traci.vehicle.changeLane(ID, lK - 1, 50)
                            else:
                                if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK + 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                        elif dist <= 50:
                            # print("ID and lK and lT:")
                            # print(ID)
                            # print(lK)
                            # print(lT)
                            traci.vehicle.changeLane(ID, lK, 50)
                        elif dist > 200:
                            traci.vehicle.changeLane(ID, lK, 50)
                # if vehType == "typeCAV_L" or vehType == "typeCAV_R":
                #     traci.vehicle.setLaneChangeMode(ID, 0b000000000000)
                ##############   end lane change code ##########################################################

                tmp = (vehQs[pK][rK][lK - 1] * clg) / (ffv / 2)
                # if rID == "1i":
                #     tmp = (vehQs[pK][rK][lK-1] * clg) / (ffv/2)
                # else:
                #     tmp = (vehQs[pK][rK][lK] * clg) / (ffv / 2)
                minimumTimeToPass = minimumTimeToPass + tmp


                # for pNum in range(len(vehQs)):
                #     for rNum in range(len(vehQs[pNum])):
                #         for lNum in range(len(vehQs[pNum][rNum])):
                #             if prl[pNum][rNum][lNum] == traci.vehicle.getLaneID(ID):
                #                 # print(vehQs[pNum][rNum])
                #                 if max(vehQs[pNum][rNum]) != min(vehQs[pNum][rNum]):
                #                     if min(vehQs[pNum][rNum]) == vehQs[pNum][rNum][lNum]:
                #                         if traci.vehicle.getLaneIndex(ID) != lNum:
                #                             print("changed lanes")
                #                             traci.vehicle.changeLane(ID, lnum, minimumTimeToPass)
                #                         else:
                #                             tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                             minimumTimeToPass = minimumTimeToPass + tmp
                #                 else:
                #                     tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                     minimumTimeToPass = minimumTimeToPass + tmp

                # print("vehicle: " + str(ID))
                # print("time: " + str(simulation_time))
                # print("temp: " + str(tmp))
                # print("minimum time: " + str(minimumTimeToPass))
                # print("dist: " + str(dist))
                # print("initial vel: " + str(initialVelocity))
                # print("maximum time: " + str(maximumTimeToPass))
                roundVelocity = round(initialVelocity * 2) / 2
                if maximumTimeToPass > 0:
                    if (distanceToTravel / maximumTimeToPass) <= ffv:
                        predicted_velocity = predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        # print("predicted 1: " + str(predicted_velocity))
                        predict = 1
                    else:
                        predicted_velocity = predict_velocity_v1(maximumTimeToPass + (Cy - AvgG), maximumTimeToPass + Cy, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        predict = 2
                        # print("predicted 2: " + str(predicted_velocity))
                        # print("cy: " + str(Cy))
                        # print("AvgG: " + str(AvgG))
                else:
                    predicted_velocity = initialVelocity
                    predict = 3
                    # print("predicted 3: " + str(predicted_velocity))

                # print('predicted speed:', predicted_velocity)
                traci.vehicle.setSpeed(ID, predicted_velocity)
                # f2.write(str(ID) + ',' + str(simulation_time) + ',' + str(dist) + ',' + str(speed) + ',' + str(state) + ',' + str(remaining_signal_time) + ',' + str(minimumTimeToPass) + "," + str(maximumTimeToPass) + "," + str(predict) + "," + str(predicted_velocity) + '\n')
                # print('speed:', traci.vehicle.getSpeed('newVeh'))
                # print('acceleration:', traci.vehicle.getAcceleration('newVeh'))
                # Calculated_Acceleration = predicted_velocity - speed
                # print('calculated acceleration:', Calculated_Acceleration)

                # before intersection #
                # a = Calculated_Acceleration
            elif dist < 20:
                if vehType == "typeCAV_R":
                    traci.vehicle.setSpeedMode(ID, 29)
                    traci.vehicle.setSpeed(ID, -1)
                # a = traci.vehicle.getAccel(ID)
            # print('before intersection acceleration:', a)

        # after intersection
        else:
            traci.vehicle.setSpeedMode(ID, 29)
            traci.vehicle.setSpeed(ID, -1)
            # traci.vehicle.setMaxSpeed('newVeh', 11)        # Set maximum speed after passing the intersection
            # a = traci.vehicle.getAccel(ID)
            # f2.close()
            # print('after intersection acceleration:', a)
            # if traci.vehicle.getDistance(ID) >= 200:
            #     traci.vehicle.slowDown(ID, 15.0, 0.4)
            #     a = -(traci.vehicle.getSpeed(ID) - 15) / 4

    if 170 < angle < 190:  # traveling north to south
        if veh_position[1] - signal_positionNS[1] >= 0:  # Before Intersection
            dist = np.sqrt((veh_position[1] - signal_positionNS[1]) ** 2)
            if 20 <= dist <= 750:
                if vehType == "typeCAV" or vehType == "typeCAV_R":
                    # Passing Interval
                    if state == 5:
                        remaining_signal_time = next_switch + ylt[3] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 6:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + sum(ylt[0:3]) + nPT[0] + nPT[1] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 8:
                        remaining_signal_time = (next_switch + sum(ylt[5:]) + sum(ylt[0:3]) + nPT[0] + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 9:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[0] + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 0:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 1:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:3]) - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 3:
                        remaining_signal_time = (next_switch + ylt[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 4:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                elif vehType == "typeCAV_L":
                    if state == 7:
                        remaining_signal_time = next_switch + ylt[4] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 8:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 9:
                        remaining_signal_time = (next_switch + sum(ylt[0:4]) + nPT[0] + nPT[1] + nPT[2] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 0:
                        remaining_signal_time = (next_switch + sum(ylt[0:4]) + nPT[1] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 1:
                        remaining_signal_time = (next_switch + sum(ylt[1:4]) + nPT[1] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:4]) + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 3:
                        remaining_signal_time = (next_switch + sum(ylt[2:4]) + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 4:
                        remaining_signal_time = (next_switch + ylt[3] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 5:
                        remaining_signal_time = next_switch + ylt[3] - simulation_time
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 6:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                # print(timeUntilStartOfPassingInterval)
                # print(timeUntilEndOfPassingInterval)
                passing_interval = [timeUntilStartOfPassingInterval, timeUntilEndOfPassingInterval]
                # print(passing_interval)
                # print(passing_interval[0])

                # Velocity Prediction
                minimumTimeToPass = passing_interval[0]
                maximumTimeToPass = passing_interval[1]
                initialVelocity = speed
                distanceToTravel = dist

                ################## Start lane change code #####################################################
                for pNum in range(len(vehQs)):
                    for rNum in range(len(vehQs[pNum])):
                        for lNum in range(len(vehQs[pNum][rNum])):
                            if lID in prl[pNum][rNum][lNum]:
                                pK = pNum
                                rK = rNum
                                lK = lNum + 1
                                lT = len(prl[pNum][rNum])
                if vehType == "typeCAV":
                    if lT > 1:
                        if 50 < dist < 200:
                            if 0 < (lK - 1) < (lT - 1):
                                if vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK] and vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK - 2]:
                                    traci.vehicle.changeLane(ID, lK, 50)
                                else:
                                    if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 2]:
                                        traci.vehicle.changeLane(ID, lK + 1, 50)
                                    else:
                                        traci.vehicle.changeLane(ID, lK - 1, 50)
                            elif (lK - 1) == (lT - 1):
                                if vehQs[pK][rK][lK - 2] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK - 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                            elif (lK - 1) == lT:
                                traci.vehicle.changeLane(ID, lK - 1, 50)
                            else:
                                if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK + 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                        elif dist <= 50:
                            # print("ID and lK and lT:")
                            # print(ID)
                            # print(lK)
                            # print(lT)
                            traci.vehicle.changeLane(ID, lK, 50)
                        elif dist > 200:
                            traci.vehicle.changeLane(ID, lK, 50)
                # if vehType == "typeCAV_L" or vehType == "typeCAV_R":
                #     traci.vehicle.setLaneChangeMode(ID, 256)
                ##############   end lane change code ##########################################################

                tmp = (vehQs[pK][rK][lK - 1] * clg) / (ffv / 2)
                # if rID == "4i":
                #     tmp = (vehQs[pK][rK][lK-1] * clg) / (ffv/2)
                # else:
                #     tmp = (vehQs[pK][rK][lK] * clg) / (ffv / 2)
                minimumTimeToPass = minimumTimeToPass + tmp

                # for pNum in range(len(vehQs)):
                #     for rNum in range(len(vehQs[pNum])):
                #         for lNum in range(len(vehQs[pNum][rNum])):
                #             if prl[pNum][rNum][lNum] == traci.vehicle.getLaneID(ID):
                #                 # print(vehQs[pNum][rNum])
                #                 if max(vehQs[pNum][rNum]) != min(vehQs[pNum][rNum]):
                #                     if min(vehQs[pNum][rNum]) == vehQs[pNum][rNum][lNum]:
                #                         if traci.vehicle.getLaneIndex(ID) != lNum:
                #                             print("changed lanes")
                #                             traci.vehicle.changeLane(ID, lnum, minimumTimeToPass)
                #                         else:
                #                             tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                             minimumTimeToPass = minimumTimeToPass + tmp
                #                 else:
                #                     tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                     minimumTimeToPass = minimumTimeToPass + tmp

                roundVelocity = round(initialVelocity * 2) / 2
                if maximumTimeToPass > 0:
                    if (distanceToTravel / maximumTimeToPass) <= ffv:
                        predicted_velocity = predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        # print("predicted 1: " + str(predicted_velocity))
                        predict = 1
                    else:
                        predicted_velocity = predict_velocity_v1(maximumTimeToPass + (Cy - AvgG), maximumTimeToPass + Cy, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        predict = 2
                        # print("predicted 2: " + str(predicted_velocity))
                        # print("cy: " + str(Cy))
                        # print("AvgG: " + str(AvgG))
                else:
                    predicted_velocity = initialVelocity
                    predict = 3
                # print('predicted speed:', predicted_velocity)
                traci.vehicle.setSpeed(ID, predicted_velocity)
                # f2.write(str(ID) + ',' + str(simulation_time) + ',' + str(veh_position[0]) + ',' + str(
                #     veh_position[1]) + ',' + str(speed) + ',' + traci.trafficlight.getRedYellowGreenState("0") + ',' +
                #     str(remaining_signal_time) + ',' + str(predicted_velocity) + '\n')
                # print('speed:', traci.vehicle.getSpeed('newVeh'))
                # print('acceleration:', traci.vehicle.getAcceleration('newVeh'))
                # Calculated_Acceleration = predicted_velocity - speed
                # print('calculated acceleration:', Calculated_Acceleration)

                # before intersection #
                # a = Calculated_Acceleration
            elif dist < 20:
                if vehType == "typeCAV_R":
                    traci.vehicle.setSpeedMode(ID, 29)
                    traci.vehicle.setSpeed(ID, -1)
                # a = traci.vehicle.getAccel(ID)
            # print('before intersection acceleration:', a)

        # after intersection
        else:
            traci.vehicle.setSpeedMode(ID, 29)
            traci.vehicle.setSpeed(ID, -1)
            # traci.vehicle.setMaxSpeed('newVeh', 11)        # Set maximum speed after passing the intersection
            # a = traci.vehicle.getAccel(ID)
            # f2.close()
            # print('after intersection acceleration:', a)
            # if traci.vehicle.getDistance(ID) >= 200:
            #     traci.vehicle.slowDown(ID, 15.0, 0.4)
            #     a = -(traci.vehicle.getSpeed(ID) - 15) / 4

    if 0 <= angle < 10 or 350 < angle <= 360:  # traveling south to north
        if veh_position[1] - signal_positionSN[1] <= 0:  # Before Intersection
            dist = np.sqrt((veh_position[1] - signal_positionSN[1]) ** 2)
            if 20 <= dist <= 750:
                if vehType == "typeCAV" or vehType == "typeCAV_R":
                    # Passing Interval
                    if state == 5:
                        remaining_signal_time = next_switch + ylt[3] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 6:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 7:
                        remaining_signal_time = (next_switch + sum(ylt[4:]) + sum(ylt[0:3]) + nPT[0] + nPT[
                            1] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 8:
                        remaining_signal_time = (
                                    next_switch + sum(ylt[5:]) + sum(ylt[0:3]) + nPT[0] + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 9:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[0] + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 0:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 1:
                        remaining_signal_time = (next_switch + sum(ylt[0:3]) + nPT[1] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:3]) - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 3:
                        remaining_signal_time = (next_switch + ylt[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                    elif state == 4:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[2] + ylt[3]
                elif vehType == "typeCAV_L":
                    if state == 7:
                        remaining_signal_time = next_switch + ylt[4] - simulation_time
                        # print(remaining_signal_time)
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 8:
                        remaining_signal_time = next_switch - simulation_time
                        timeUntilStartOfPassingInterval = 0
                        timeUntilEndOfPassingInterval = remaining_signal_time
                    elif state == 9:
                        remaining_signal_time = (next_switch + sum(ylt[0:4]) + nPT[0] + nPT[1] + nPT[
                            2] - simulation_time)  # Time remaining of this red light
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 0:
                        remaining_signal_time = (next_switch + sum(ylt[0:4]) + nPT[1] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 1:
                        remaining_signal_time = (next_switch + sum(ylt[1:4]) + nPT[1] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 2:
                        remaining_signal_time = (next_switch + sum(ylt[1:4]) + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 3:
                        remaining_signal_time = (next_switch + sum(ylt[2:4]) + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 4:
                        remaining_signal_time = (next_switch + ylt[3] + nPT[2] - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 5:
                        remaining_signal_time = next_switch + ylt[3] - simulation_time
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                    elif state == 6:
                        remaining_signal_time = (next_switch - simulation_time)
                        timeUntilStartOfPassingInterval = remaining_signal_time
                        timeUntilEndOfPassingInterval = remaining_signal_time + nPT[3] + ylt[4]
                # print(timeUntilStartOfPassingInterval)
                # print(timeUntilEndOfPassingInterval)
                passing_interval = [timeUntilStartOfPassingInterval, timeUntilEndOfPassingInterval]
                # print(passing_interval)
                # print(passing_interval[0])

                # Velocity Prediction
                minimumTimeToPass = passing_interval[0]
                maximumTimeToPass = passing_interval[1]
                initialVelocity = speed
                distanceToTravel = dist

                ################## Start lane change code #####################################################

                for pNum in range(len(vehQs)):
                    for rNum in range(len(vehQs[pNum])):
                        for lNum in range(len(vehQs[pNum][rNum])):
                            if lID in prl[pNum][rNum][lNum]:
                                pK = pNum
                                rK = rNum
                                lK = lNum + 1
                                lT = len(prl[pNum][rNum])
                if vehType == "typeCAV":
                    if lT > 1:
                        if 50 < dist < 200:
                            if 0 < (lK - 1) < (lT - 1):
                                if vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK] and vehQs[pK][rK][lK - 1] <= vehQs[pK][rK][lK - 2]:
                                    traci.vehicle.changeLane(ID, lK, 50)
                                else:
                                    if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 2]:
                                        traci.vehicle.changeLane(ID, lK + 1, 50)
                                    else:
                                        traci.vehicle.changeLane(ID, lK - 1, 50)
                            elif (lK - 1) == (lT - 1):
                                if vehQs[pK][rK][lK - 2] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK - 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                            elif (lK - 1) == lT:
                                traci.vehicle.changeLane(ID, lK - 1, 50)
                            else:
                                if vehQs[pK][rK][lK] <= vehQs[pK][rK][lK - 1] - 1:
                                    traci.vehicle.changeLane(ID, lK + 1, 50)
                                else:
                                    traci.vehicle.changeLane(ID, lK, 50)
                        elif dist <= 50:
                            # print("ID and lK and lT:")
                            # print(ID)
                            # print(lK)
                            # print(lT)
                            traci.vehicle.changeLane(ID, lK, 50)
                        elif dist > 200:
                            traci.vehicle.changeLane(ID, lK, 50)

                # if vehType == "typeCAV_L" or vehType == "typeCAV_R":
                #     traci.vehicle.setLaneChangeMode(ID, 256)
                ##############   end lane change code ##########################################################

                tmp = (vehQs[pK][rK][lK - 1] * clg) / (ffv / 2)
                # if rID == "3i":
                #     tmp = (vehQs[pK][rK][lK-1] * clg) / (ffv/2)
                # else:
                #     tmp = (vehQs[pK][rK][lK] * clg) / (ffv / 2)
                minimumTimeToPass = minimumTimeToPass + tmp

                # for pNum in range(len(vehQs)):
                #     for rNum in range(len(vehQs[pNum])):
                #         for lNum in range(len(vehQs[pNum][rNum])):
                #             if prl[pNum][rNum][lNum] == traci.vehicle.getLaneID(ID):
                #                 # print(vehQs[pNum][rNum])
                #                 if max(vehQs[pNum][rNum]) != min(vehQs[pNum][rNum]):
                #                     if min(vehQs[pNum][rNum]) == vehQs[pNum][rNum][lNum]:
                #                         if traci.vehicle.getLaneIndex(ID) != lNum:
                #                             print("changed lanes")
                #                             traci.vehicle.changeLane(ID, lnum, minimumTimeToPass)
                #                         else:
                #                             tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                             minimumTimeToPass = minimumTimeToPass + tmp
                #                 else:
                #                     tmp = (vehQs[pNum][rNum][lNum] * clg) / (ffv/2)
                #                     minimumTimeToPass = minimumTimeToPass + tmp

                roundVelocity = round(initialVelocity * 2) / 2
                if maximumTimeToPass > 0:
                    if (distanceToTravel / maximumTimeToPass) <= ffv:
                        predicted_velocity = predict_velocity_v1(minimumTimeToPass, maximumTimeToPass, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        # print("predicted 1: " + str(predicted_velocity))
                        predict = 1
                    else:
                        predicted_velocity = predict_velocity_v1(maximumTimeToPass + (Cy - AvgG), maximumTimeToPass + Cy, roundVelocity, distanceToTravel) - roundVelocity + initialVelocity
                        predict = 2
                        # print("predicted 2: " + str(predicted_velocity))
                        # print("cy: " + str(Cy))
                        # print("AvgG: " + str(AvgG))
                else:
                    predicted_velocity = initialVelocity
                    predict = 3
                # if ID == "CAV_up_68":
                #     print("CAV 68: " + str(veh_position[0]) + ", " + str(veh_position[1]) + ", " +
                #           str(maximumTimeToPass) + ", " + str(minimumTimeToPass) + ", " +
                #           str(distanceToTravel/maximumTimeToPass) + ", " + str(initialVelocity) + ", " +
                #           str(predicted_velocity))
                # print('predicted speed:', predicted_velocity)
                traci.vehicle.setSpeed(ID, predicted_velocity)
                # f2.write(str(ID) + ',' + str(simulation_time) + ',' + str(veh_position[0]) + ',' + str(
                #     veh_position[1]) + ',' + str(speed) + ',' + traci.trafficlight.getRedYellowGreenState("0") + ',' +
                #     str(remaining_signal_time) + ',' + str(predicted_velocity) + '\n')
                # print('speed:', traci.vehicle.getSpeed('newVeh'))
                # print('acceleration:', traci.vehicle.getAcceleration('newVeh'))
                # Calculated_Acceleration = predicted_velocity - speed
                # print('calculated acceleration:', Calculated_Acceleration)

                # before intersection #
                # a = Calculated_Acceleration
            elif dist < 20:
                if vehType == "typeCAV_R":
                    traci.vehicle.setSpeedMode(ID, 29)
                    traci.vehicle.setSpeed(ID, -1)
                # a = traci.vehicle.getAccel(ID)
            # print('before intersection acceleration:', a)

        # after intersection
        else:
            traci.vehicle.setSpeedMode(ID, 29)
            traci.vehicle.setSpeed(ID, -1)
            # traci.vehicle.setMaxSpeed('newVeh', 11)        # Set maximum speed after passing the intersection
            # a = traci.vehicle.getAccel(ID)
            # f2.close()
            # print('after intersection acceleration:', a)
            # if traci.vehicle.getDistance(ID) >= 200:
            #     traci.vehicle.slowDown(ID, 15.0, 0.4)
            #     a = -(traci.vehicle.getSpeed(ID) - 15) / 4

    acc = speed - last_speed
    # print('acc:', acc)

    # Arrival_List = traci.simulation.getArrivedIDList()
    # if ID in Arrival_List:
    #     acc = 0

    last_speed = speed
    # f2.close()
    return acc, last_speed