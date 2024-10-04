#!/usr/bin/env python

# @file    I-TSC.py
# @author  David Oswald
# @date    August 2021

# Code is to recreate I-TSC from the paper:
# A Coupled Vehicle-Signal Control Method at Signalized Intersections in Mixed Traffic Environment
# written by Yu Du, Wei ShangGuan, and Linguo Chai

# function: generate_routefile is taken from runner.py from the traci_tls example code.
# function generate a route file that randomly routes vehicles.

from __future__ import absolute_import
from __future__ import print_function

from sklearn.ensemble import RandomForestRegressor
import os
import sys
import optparse
import random
import math
import datetime
import time
import pickle
# from EAD_regressor import *
from EAD_algorithm_new import *
from timeit import default_timer as timer

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
from scipy.optimize import Bounds
from scipy.optimize import minimize
import numpy as np


def generate_routefile():
    global RandSeed
    random.seed(RandSeed)  # make tests reproducible
    N = 500  # number of time steps; usually is 3600
    Pr = PEN_RATE   # CAV penetration rate



def run():
    """execute the TraCI control loop"""

    step = 0
    # CAV penetration rate
    pen_rate = PEN_RATE
    Qab = []
    EP_ID_list = []
    veh_vel = []
    last_speed = []
    total_co2_emission = 0.0
    total_fuel_consumption = 0.0

    # we start with phase 0 where EW has green
    # traci.trafficlight.setPhase("0", 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # 7 a.m. scenario
        if step >= 25200 and step <= 27000:
        # 2 a.m. scenario
        # if step >= 7200 and step <= 9000:
            vehicle_ids = traci.vehicle.getIDList()
            for veh_id in vehicle_ids:
                # Use TRACI to get CO2 emissions and fuel consumption
                co2_emission = traci.vehicle.getCO2Emission(veh_id)/1000000
                fuel_consumption = traci.vehicle.getFuelConsumption(veh_id)/1000000
                total_co2_emission += co2_emission
                total_fuel_consumption += fuel_consumption

            if EP == 'on':
                # EAD Code Section of while loop:
                # add EAD vehicle to list
                vehicle_ID_list = traci.vehicle.getIDList()
                for veh in vehicle_ID_list:
                    if traci.vehicle.getTypeID(veh) == "EP":
                    # if traci.vehicle.getTypeID(veh) == "typeCAV" or traci.vehicle.getTypeID(veh) == "typeCAV_L" or traci.vehicle.getTypeID(veh) == "typeCAV_R":
                        if veh not in EP_ID_list:
                            EP_ID_list.append(veh)
                            last_speed.append(-1)

                ### Exist EAD Vehicle
                Intersection = [val for val in vehicle_ID_list if val in EP_ID_list]
                # print('Exist EAD ID:', Intersection)

                ### Find the place of existing EAD vehicle
                place = [None] * len(Intersection)
                for p in range(len(Intersection)):
                    place[p] = EP_ID_list.index(Intersection[p])

                for q in range(len(Intersection)):
                    ### Eco-Pedaling strategy1
                    # if traci.vehicle.getLeader(Intersection[q]) is not None and traci.vehicle.getLeader(Intersection[q])[1] < 20:
                    #     #  print("Back to CACC")
                    #     traci.vehicle.setSpeed(Intersection[q], -1)
                    #     speed_q = traci.vehicle.getSpeed(Intersection[q])
                    #     last_speed[place[q]] = speed_q
                    # else:
                    #     speed_q = traci.vehicle.getFollowSpeed(Intersection[q])
                    #     print('Follow: ', speed_q)
                    #     speed_q_wo = traci.vehicle.getSpeedWithoutTraCI(Intersection[q])
                    #     print('without traci: ', speed_q_wo)
                    #     last_speed_q = traci.vehicle.getSpeed(Intersection[q])
                    #     print('last: ', last_speed_q)
                    #     speed_q = (speed_q-last_speed_q)*0.8+last_speed_q
                    #     print('new: ', speed_q)
                    #     traci.vehicle.setSpeed(EP_ID_list[place[q]],speed_q)
                    #     speed_q = traci.vehicle.getSpeed(EP_ID_list[place[q]])
                    #     print('new_real: ', speed_q)

                    ### Eco-Pedaling strategy2
                    if traci.vehicle.getLeader(Intersection[q]) is not None and traci.vehicle.getLeader(Intersection[q])[1] < 20:
                        #  print("Back to CACC")
                        traci.vehicle.setSpeed(Intersection[q], -1)
                        speed_q = traci.vehicle.getSpeed(Intersection[q])
                        last_speed[place[q]] = speed_q
                    elif last_speed[place[q]] == -1:
                        traci.vehicle.setSpeed(Intersection[q], -1)
                        speed_q = traci.vehicle.getSpeed(Intersection[q])
                        last_speed[place[q]] = speed_q
                    else: 
                        speed_q = traci.vehicle.getSpeed(EP_ID_list[place[q]])
                        print(speed_q)
                        last_speed_q = last_speed[place[q]]
                        speed_q_new = (speed_q-last_speed_q)*0.8+last_speed_q
                        print(speed_q_new)
                        traci.vehicle.setSpeed(EP_ID_list[place[q]],speed_q_new)
                        last_speed[place[q]] = speed_q



                    veh_vel.append([step, Intersection[q], speed_q])

        if step > 27000:
        # if step > 9000:
            break


        step += 1
    
    print(f"Total CO2 emission: {total_co2_emission} kg")
    print(f"Total fuel consumption: {total_fuel_consumption} kg")

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    optParser.add_option('--PR', type='int', default=100, help='CAV penetration rate')
    optParser.add_option('--evPR', type='int', default=100, help='EV penetration rate')
    optParser.add_option('--RS', type='int', default=42, help='Random Seed number')
    optParser.add_option('--vcRat', type='int', default=10, help='VC ratio: 10, 67, 52, ...')
    optParser.add_option('--Hour', type='int', default=0, help='traffic hour: 0, 7, 10, 14, ...')
    options, args = optParser.parse_args()
    return options


###########################################
# Global Constants
options = get_options()
PEN_RATE = (options.PR/100)
RandSeed = options.RS
EP = 'off'
###########################################


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    print("options:")
    print(options)
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    # generate_routefile()

    save_folder = '/home/gene/Downloads/EAD code test/peak_hour_0.1eb/'

    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    traci.start([sumoBinary, "-c", "/home/gene/numo/nagoya_ep.sumocfg", "--step-length", "1", "--device.rerouting.threads",
                     "64", "--device.rerouting.synchronize", "true", "--time-to-teleport", "100","--tripinfo-output",
                     save_folder+"tripinfo.xml", "--summary", save_folder+"summary_100.xml", "--amitran-output", save_folder+"TrajectoryOutput.xml",
                     "--queue-output", save_folder+"QueueOutput.xml", "--statistic-output", save_folder+"StatOutput.xml", "--fcd-output", save_folder+"FCDOutput.xml", "--emission-output", save_folder+"EmissionOutput.xml"])
    run()