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
import xml.etree.ElementTree as ET

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

def generate_routefile_cav(path, newpath):
    global RandSeed, CAEPVPr
    print(CAEPVPr)
    random.seed(RandSeed)

    veh_Nr = {}

    sub_items = os.listdir(path)
    for item in sub_items:

        # read XML file
        tree = ET.parse(path + item)
        root = tree.getroot()

        # read vehicle id range
        vehicles = root.findall('vehicle')
        if vehicles:
            first_vehicle_id = vehicles[0].get('id')
            last_vehicle_id = vehicles[-1].get('id')
        else:
            print("No vehicle elements found.")

        # calculate the number of vehicles generated in one route
        time = item[7:12].replace('_','.')
        veh_Nr.update({time: int(last_vehicle_id)-int(first_vehicle_id)+1})

        # modify vtype randomly
        for i in range(0, int(last_vehicle_id)-int(first_vehicle_id)+1):
            if random.uniform(0, 1) < CAEPVPr:
                vehicles[i].set('type', 'EP')
            else:
                vehicles[i].set('type', 'EV')

        tree.write(newpath + item, encoding='utf-8', xml_declaration=True)

def edit_acc_ratio(path, ratio):
    print('EPRatio: ', ratio)
    # read XML file
    tree = ET.parse(path)
    root = tree.getroot()

    # read vehicle id range
    vehicles = root.findall('vType')
    for i in range(len(vehicles)):
        if vehicles[i].get('id') == 'EP':
            accel = 2.6*ratio
            decel = 4.5*ratio
            vehicles[i].set('accel', str(accel))
            vehicles[i].set('decel', str(decel))
        
    tree.write(path, encoding='utf-8', xml_declaration=True)

def run(save_folder):
    """execute the TraCI control loop"""

    step = 0
    Qab = []
    EAD_ID_list = []
    veh_vel = []
    last_speed = []
    total_co2_emission = 0.0
    total_fuel_consumption = 0.0
    total_co2_emission_cav = 0.0
    total_fuel_consumption_cav = 0.0
    total_electricity = 0.0
    total_second = 0.0
    total_meter = 0.0
    total_electricity_caev = 0.0
    total_second_caev = 0.0
    total_meter_caev = 0.0


    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        time = traci.simulation.getTime()

        # 7 a.m. scenario
        # if step >= 25200 and step <= 27000:
        # 2 a.m. scenario
        if step >= 7200 and step <= 9000:

            vehicle_ids = traci.vehicle.getIDList()
            for veh_id in vehicle_ids:
                if EV == 'off':
                    # Use TRACI to get CO2 emissions and fuel consumption
                    co2_emission = traci.vehicle.getCO2Emission(veh_id)/1000000
                    fuel_consumption = traci.vehicle.getFuelConsumption(veh_id)/1000000
                    total_co2_emission += co2_emission
                    total_fuel_consumption += fuel_consumption
                    if traci.vehicle.getTypeID(veh_id) == "CAV":
                        co2_emission = traci.vehicle.getCO2Emission(veh_id)/1000000
                        fuel_consumption = traci.vehicle.getFuelConsumption(veh_id)/1000000
                        total_co2_emission_cav += co2_emission
                        total_fuel_consumption_cav += fuel_consumption
                        if EAD == 'on':
                            if veh_id not in EAD_ID_list:
                                EAD_ID_list.append(veh_id)
                                last_speed.append(13)
                else:
                    if not np.isnan(traci.vehicle.getElectricityConsumption(veh_id)):
                        # Use TraCi to get electricity in Wh
                        electricity = traci.vehicle.getElectricityConsumption(veh_id)
                        second = 1
                        meter = traci.vehicle.getSpeed(veh_id)
                        total_electricity += electricity
                        total_second += second
                        total_meter += meter
                        if traci.vehicle.getTypeID(veh_id) == "EP":
                            electricity = traci.vehicle.getElectricityConsumption(veh_id)
                            second = 1
                            meter = traci.vehicle.getSpeed(veh_id)
                            total_electricity_caev += electricity
                            total_second_caev += second
                            total_meter_caev += meter
                            if EAD == 'on':
                                if veh_id not in EAD_ID_list:
                                    EAD_ID_list.append(veh_id)
                                    last_speed.append(13)

            if EAD == 'on':                   

                ### Exist EAD Vehicle
                Intersection = [val for val in vehicle_ids if val in EAD_ID_list and traci.vehicle.getNextTLS(val)]
                # print('Exist EAD ID:', Intersection)

                ### Find the place of existing EAD vehicle
                place = [None] * len(Intersection)
                for p in range(len(Intersection)):
                    place[p] = EAD_ID_list.index(Intersection[p])
                    # print('place:', EAD_ID_list.index(Intersection[p]))
                # print('EAD place list:', place)

                ### Calculate the number of vehicles in Queue
                Qab = []
                for tlsid in traci.trafficlight.getIDList():
                    lanes = traci.trafficlight.getControlledLanes(tlsid)
                    for l in lanes:
                        Qab.append([tlsid,l,0])
                        lane_veh = traci.lane.getLastStepVehicleIDs(l)
                        for v in lane_veh:
                            if traci.vehicle.getSpeed(v) < 0.1:
                                # print(v)
                                # print(traci.vehicle.getNextTLS(v))
                                if traci.vehicle.getNextTLS(v):
                                    if traci.vehicle.getNextTLS(v)[0][2] < 150:
                                        Qab[-1][-1] = Qab[-1][-1] + 1

                ### Implement EAD to vehicles
                # 09/30/2021 David: Changed EAD_acceleration to include vehicle angle and new phase times
                #  Also changed state from traci.trafficlight.getRedYellowGreenState("0") to traci.trafficlight.getPhase("0"), which
                #  gives the index of the red, yellow, green state rather than actual states like rrrrgGGrrrrrgGGr
                #  10/18/2021 David: Testing getLeader function
                for q in range(len(Intersection)):
                    _, last_speed[place[q]] = EAD_acceleration(Intersection[q], traci.vehicle.getSpeed(Intersection[q]), last_speed[place[q]], traci.vehicle.getTypeID(Intersection[q]), Qab, EV)

                    if traci.vehicle.getLeader(Intersection[q]) is not None:
                        if traci.vehicle.getLeader(Intersection[q])[1] < 20:
                            #  print("Back to CACC")
                            traci.vehicle.setSpeed(Intersection[q], -1)
                    veh_vel.append([step, Intersection[q], traci.vehicle.getSpeed(Intersection[q])])

        # if step > 27000:
        if step > 9000:
            break


        step += 1
    
    if EV == 'off':
        print(f"Total CO2 emission: {total_co2_emission} kg")
        print(f"Total fuel consumption: {total_fuel_consumption} kg")
    else:
        print(f"Total elecctricity consumption: {total_electricity} Wh")
        print(f"cav elecctricity consumption: {total_electricity_caev} Wh")
        print(f"other ev elecctricity consumption: {total_electricity - total_electricity_caev} Wh")
        print(f"Total travel time in hour (VHT): {total_second/3600} h")
        print(f"cav travel time in hour (VHT): {total_second_caev/3600} h")
        print(f"other ev travel time in hour (VHT): {(total_second - total_second_caev)/3600} h")
        print(f"Total travel miles: {total_meter/1609.34} miles")
        print(f"cav travel miles: {total_meter_caev/1609.34} miles")
        print(f"other travel miles: {(total_meter - total_meter_caev)/1609.34} miles")

    filename = save_folder+'Electricity.txt'
    with open(filename, 'w') as file:
        # Write directly to the file
        file.write(f"Total elecctricity consumption: {total_electricity} Wh\n")
        file.write(f"caepv elecctricity consumption: {total_electricity_caev} Wh\n")
        file.write(f"other ev elecctricity consumption: {total_electricity - total_electricity_caev} Wh\n")
        file.write(f"Total travel time in hour (VHT): {total_second/3600} h")
        file.write(f"cav travel time in hour (VHT): {total_second_caev/3600} h")
        file.write(f"other ev travel time in hour (VHT): {(total_second - total_second_caev)/3600} h")
        file.write(f"Total travel miles: {total_meter/1609.34} miles")
        file.write(f"cav travel miles: {total_meter_caev/1609.34} miles")
        file.write(f"other travel miles: {(total_meter - total_meter_caev)/1609.34} miles")

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    optParser.add_option('--PR', type='int', default=10, help='CAEPV penetration rate')
    optParser.add_option('--evPR', type='int', default=100, help='EV penetration rate')
    optParser.add_option('--RS', type='int', default=42, help='Random Seed number')
    optParser.add_option('--vcRat', type='int', default=10, help='VC ratio: 10, 67, 52, ...')
    optParser.add_option('--Hour', type='int', default=0, help='traffic hour: 0, 7, 10, 14, ...')
    options, args = optParser.parse_args()
    return options


###########################################
# Global Constants
options = get_options()
RandSeed = options.RS
EAD = 'on'
EV = 'on'
CAEPVPr = options.PR/100
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

    # for i in np.linspace(10, 100, 10):
    CAEPVPr = 15/100

    path = '/home/gene/numo/routes/'
    new_path = '/home/gene/numo/new_routes_ep/'
    generate_routefile_cav(path, new_path)

    for j in np.linspace(50, 90, 5):
        
        EPRatio = j/100
        path_ep = '/home/gene/numo/new_routes_ep/vehicle_type.rou.xml'
        edit_acc_ratio(path_ep, EPRatio)

        save_folder = '/home/gene/Downloads/EAD code test/trough_hour_'+str(CAEPVPr)+'caep_'+str(EPRatio)+'ep_static/'
        if not os.path.exists(save_folder):
            os.makedirs(save_folder)

        traci.start([sumoBinary, "-c", "/home/gene/numo/nagoya_ep_static_tls.sumocfg", "--step-length", "1", "--device.rerouting.threads",
                        "64", "--device.rerouting.synchronize", "true", "--time-to-teleport", "100", "--tripinfo-output", save_folder+"tripinfo.xml"])
                        #  "--tripinfo-output", save_folder+"tripinfo.xml", "--summary", save_folder+"summary_100.xml", "--amitran-output", save_folder+"TrajectoryOutput.xml",
                        #  "--queue-output", save_folder+"QueueOutput.xml", "--statistic-output", save_folder+"StatOutput.xml", "--fcd-output", save_folder+"FCDOutput.xml", "--emission-output", save_folder+"EmissionOutput.xml"])
        run(save_folder)