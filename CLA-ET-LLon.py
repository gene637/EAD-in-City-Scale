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
from EAD_regressor import *
from EAD_algorithm_Data2 import *
# from acc import *
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


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option('--PR', type='int', default=100, help='CAV penetration rate')
    optParser.add_option('--evPR', type='int', default=100, help='EV penetration rate')
    optParser.add_option('--RS', type='int', default=42, help='Random Seed number')
    optParser.add_option('--vcRat', type='int', default=10, help='VC ratio: 10, 67, 52, ...')
    optParser.add_option('--Hour', type='int', default=0, help='traffic hour: 0, 7, 10, 14, ...')
    options, args = optParser.parse_args()
    return options


options = get_options()
print("options:")
print(options)

###########################################
# Global Constants
TGMAX = 30
TGMIN = 4
PEN_RATE = (options.PR/100)
CYCLE_LENGTH = 66   # if changed, change cycle length in EAD_algorithm.py file too.
RandSeed = options.RS
CACCstatus = 'On'
Lanelvl = 'On'
TechType = 'EcoTops'
VC = options.vcRat
Hr = options.Hour
###########################################


# vehNr is to keep track of total number of vehicles generated in generate_routefile
vehNr = 0
# cavNr is to keep track of the number of CAV vehicles generated
cavNr = 0
# counts for each direction and route.
pWENr = 0
pWE_LNr = 0
pWE_RNr = 0
pEWNr = 0
pEW_LNr = 0
pEW_RNr = 0
pNSNr = 0
pNS_LNr = 0
pNS_RNr = 0
pSNNr = 0
pSN_LNr = 0
pSN_RNr = 0


def generate_routefile():
    global RandSeed
    random.seed(RandSeed)  # make tests reproducible
    N = 500  # number of time steps; usually is 3600
    Pr = PEN_RATE   # CAV penetration rate
    # demand per second from different directions
    if Hr == 7:
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 232 total vehicles is what Gridsmart has
        # V/C = 0.67
        pWE = 1. / 16
        pWE_L = 1. / 25
        pWE_R = 1. / 55
        pEW = 1. / 30
        pEW_L = 1. / 55
        pEW_R = 1. / 50
        pNS = 1. / 10
        pNS_L = 1. / 25
        pNS_R = 1. / 42
        pSN = 1. / 9
        pSN_L = 1. / 50
        pSN_R = 1. / 55
        print("traffic hour 7")
    elif Hr == 10:
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 213 total vehicles is what Gridsmart has
        # V/C = 0.52
        pWE = 1. / 12
        pWE_L = 1. / 20
        pWE_R = 1. / 80
        pEW = 1. / 17
        pEW_L = 1. / 45
        pEW_R = 1. / 60
        pNS = 1. / 20
        pNS_L = 1. / 25
        pNS_R = 1. / 25
        pSN = 1. / 23
        pSN_L = 1. / 50
        pSN_R = 1. / 60
        print("traffic hour 10")
    elif Hr == 14:
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 315 total vehicles is what Gridsmart has
        # v/C = 0.82
        pWE = 1. / 9
        pWE_L = 1. / 15
        pWE_R = 1. / 50
        pEW = 1. / 17
        pEW_L = 1. / 30
        pEW_R = 1. / 30
        pNS = 1. / 10
        pNS_L = 1. / 16
        pNS_R = 1. / 19
        pSN = 1. / 12
        pSN_L = 1. / 45
        pSN_R = 1. / 55
        print("traffic hour 14")
    elif Hr == 17:
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 379 total vehicles is what Gridsmart has
        # v/C = 1.09
        pWE = 1. / 5.8
        pWE_L = 1. / 14.2
        pWE_R = 1. / 45
        pEW = 1. / 15.4
        pEW_L = 1. / 36
        pEW_R = 1. / 42
        pNS = 1. / 6.5
        pNS_L = 1. / 13
        pNS_R = 1. / 18
        pSN = 1. / 14
        pSN_L = 1. / 40
        pSN_R = 1. / 45
        print("traffic hour 17")
    elif Hr == 21:
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 151 total vehicles is what Gridsmart has
        # v/C = 0.356
        pWE = 1. / 16
        pWE_L = 1. / 30
        pWE_R = 1. / 500
        pEW = 1. / 22
        pEW_L = 1. / 60
        pEW_R = 1. / 170
        pNS = 1. / 30
        pNS_L = 1. / 30
        pNS_R = 1. / 25
        pSN = 1. / 30
        pSN_L = 1. / 100
        pSN_R = 1. / 100
        print("traffic hour 21")
    else:
        # hour 0 traffic
        # 1. / 71 would be 1 vehicle every 71 seconds
        # about 40 total vehicles is what Gridsmart has
        # V/C = 0.1
        pWE = 1. / 70
        pWE_L = 1. / 125
        pWE_R = 0
        pEW = 1. / 80
        pEW_L = 1. / 170
        pEW_R = 0
        pNS = 1. / 100
        pNS_L = 1. / 170
        pNS_R = 1. / 70
        pSN = 1. / 125
        pSN_L = 1. / 500
        pSN_R = 1. / 150
        print("traffic hour 0")

    with open("Data2/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" guiShape="passenger" carFollow-Model="IDM"/>
        <vType id="typeWE_L" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" vClass="vip" guiShape="passenger" carFollow-Model="IDM"/>
        <vType id="typeWE_R" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" vClass="private" guiShape="passenger" carFollow-Model="IDM"/>
        <vType id="typeCAV" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" guiShape="passenger" carFollow-Model="CACC"/>
        <vType id="typeCAV_L" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" vClass="vip" guiShape="passenger" carFollow-Model="CACC"/>
        <vType id="typeCAV_R" accel="0.73" decel="4.5" sigma="0.5" length="5" minGap="2.5" tau="1.5" maxSpeed="16" vClass="private" guiShape="passenger" carFollow-Model="CACC"/>

        <route id="right" edges="1_4i 1_3i 1_2i 1i 2o" />
        <route id="left" edges="2_3i 2_2i 2i 1o" />
        <route id="down" edges="4_3i 4_2i 4i 3o" />
        <route id="up" edges="3_3i 3_2i 3i 4o" />
        <route id="right_L" edges="1_4i 1_3i 1_2i 1i 4o" />
        <route id="left_L" edges="2_3i 2_2i 2i 3o" />
        <route id="down_L" edges="4_3i 4_2i 4i 2o" />
        <route id="up_L" edges="3_3i 3_2i 3i 1o" />
        <route id="right_R" edges="1_4i 1_3i 1_2i 1i 3o" />
        <route id="left_R" edges="2_3i 2_2i 2i 4o" />
        <route id="down_R" edges="4_3i 4_2i 4i 1o" />
        <route id="up_R" edges="3_3i 3_2i 3i 2o" />""", file=routes)
        # vehNr = 0
        global vehNr
        global cavNr
        global pWENr
        global pWE_LNr
        global pWE_RNr
        global pEWNr
        global pEW_LNr
        global pEW_RNr
        global pNSNr
        global pNS_LNr
        global pNS_RNr
        global pSNNr
        global pSN_LNr
        global pSN_RNr
        for i in range(N):
            if random.uniform(0, 1) < pSN:
                if random.uniform(0, 1) < 0.80:
                    dLane = 2
                else:
                    dLane = 1
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_up_%i" type="typeCAV" route="up" depart="%i" departLane = "%i" color="1,0,0" />' % (
                        vehNr, i, dLane), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="up_%i" type="typeWE" route="up" depart="%i" departLane = "%i"  />' % (
                        vehNr, i, dLane), file=routes)
                vehNr += 1
                pSNNr += 1
            if random.uniform(0, 1) < pWE:
                if random.uniform(0, 1) < 0.65:
                    dLane = 2
                else:
                    dLane = 1
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_right_%i" type="typeCAV" route="right" depart="%i" departLane = "%i" color="1,0,0" />' % (
                        vehNr, i, dLane), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" departLane = "%i" />' % (
                        vehNr, i, dLane), file=routes)
                vehNr += 1
                pWENr += 1
            if random.uniform(0, 1) < pEW:
                if random.uniform(0, 1) < 0.85:
                    dLane = 2
                else:
                    dLane = 1
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_left_%i" type="typeCAV" route="left" depart="%i" departLane = "%i" color="1,0,0" />' % (
                        vehNr, i, dLane), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" departLane = "%i" />' % (
                        vehNr, i, dLane), file=routes)
                vehNr += 1
                pEWNr += 1
            if random.uniform(0, 1) < pNS:
                if random.uniform(0, 1) < 0.8:
                    dLane = 2
                else:
                    dLane = 1
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_down_%i" type="typeCAV" route="down" depart="%i" departLane = "%i" color="1,0,0" />' % (
                        vehNr, i, dLane), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="down_%i" type="typeWE" route="down" depart="%i" departLane = "%i" />' % (
                        vehNr, i, dLane), file=routes)
                vehNr += 1
                pNSNr += 1
            if random.uniform(0, 1) < pWE_L:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_right_L_%i" type="typeCAV_L" route="right_L" depart="%i" departLane = "2" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="right_L_%i" type="typeWE_L" route="right_L" depart="%i" departLane = "2" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pWE_LNr += 1
            if random.uniform(0, 1) < pEW_L:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_left_L_%i" type="typeCAV_L" route="left_L" depart="%i" departLane = "2" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="left_L_%i" type="typeWE_L" route="left_L" depart="%i" departLane = "2" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pEW_LNr += 1
            if random.uniform(0, 1) < pNS_L:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_down_L_%i" type="typeCAV_L" route="down_L" depart="%i" departLane = "2" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="down_L_%i" type="typeWE_L" route="down_L" depart="%i" departLane = "2" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pNS_LNr += 1
            if random.uniform(0, 1) < pWE_R:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_right_R_%i" type="typeCAV_R" route="right_R" depart="%i" departLane = "1" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="right_R_%i" type="typeWE_R" route="right_R" depart="%i" departLane = "1" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pWE_RNr += 1
            if random.uniform(0, 1) < pEW_R:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_left_R_%i" type="typeCAV_R" route="left_R" depart="%i" departLane = "1" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="left_R_%i" type="typeWE_R" route="left_R" depart="%i" departLane = "1" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pEW_RNr += 1
            if random.uniform(0, 1) < pNS_R:
                if random.uniform(0, 1) < Pr:
                    print(
                        '    <vehicle id="CAV_down_R_%i" type="typeCAV_R" route="down_R" depart="%i" departLane = "1" color="1,0,0" />' % (
                            vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="down_R_%i" type="typeWE_R" route="down_R" depart="%i" departLane = "1" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pNS_RNr += 1
            if random.uniform(0, 1) < pSN_L:
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_up_L_%i" type="typeCAV_L" route="up_L" depart="%i" departLane = "2" color="1,0,0" />' % (
                        vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="up_L_%i" type="typeWE_L" route="up_L" depart="%i" departLane = "2" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pSN_LNr += 1
            if random.uniform(0, 1) < pSN_R:
                if random.uniform(0, 1) < Pr:
                    print('    <vehicle id="CAV_up_R_%i" type="typeCAV_R" route="up_R" depart="%i" departLane = "1" color="1,0,0" />' % (
                        vehNr, i), file=routes)
                    cavNr += 1
                else:
                    print('    <vehicle id="up_R_%i" type="typeWE_R" route="up_R" depart="%i" departLane = "1" />' % (
                        vehNr, i), file=routes)
                vehNr += 1
                pSN_RNr += 1
        print("</routes>", file=routes)

# The initial program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
# 		<phase duration="15" state="rrrrgGGrrrrrgGGr"/>
#         <phase duration="4"  state="rrrryyyrrrrryyyr"/>
# 		<phase duration="8" state="rrrrrrrGrrrrrrrG"/>
#         <phase duration="4"  state="rrrrrrryrrrrrrry"/>
#         <phase duration="2"  state="rrrrrrrrrrrrrrrr"/>
#         <phase duration="15" state="gGGrrrrrgGGrrrrr"/>
#         <phase duration="4"  state="yyyrrrrryyyrrrrr"/>
# 		<phase duration="8" state="rrrGrrrrrrrGrrrr"/>
#         <phase duration="4"  state="rrryrrrrrrryrrrr"/>
# 		<phase duration="2"  state="rrrrrrrrrrrrrrrr"/>
#    </tlLogic>


def count_veh(lane, a, b):
    # count function from paper by Yu Du, Beijing Jiaotong University
    # counts the number of vehicles whose distance to the intersection is between a and b

    # route_num is the route number. 12 routes in total R23, R25, R27, R41, R45, R47, R61, R63, R67, R81, R83, and R85
    # we only care about 8 routes,  1=R25, 2=R27, 3=R41, 4=R47, 5=R61, 6=R63, 7=R83, 8=R85 (refer to paper fig. 4)
    idlist = traci.vehicle.getIDList()
    cnt = 0

    # for veh in idlist:
    #     for la in lane:
    #         if traci.vehicle.getLaneID(veh) == la:
    #             dist = traci.lane.getLength(la) - traci.vehicle.getLanePosition(veh)
    #             if b > dist > a:
    #                 cnt = cnt + 1

    for veh in idlist:
        for la in range(len(lane)):
            for la2 in lane[la]:
                if traci.vehicle.getLaneID(veh) == la2:
                    dist = traci.vehicle.getNextTLS(veh)[0][2]  # gives the distance to the next traffic signal
                    if b > dist > a:
                        cnt = cnt + 1
    return cnt


def delayed_vehicles(routes, Tg, Tr):
    # saturation (or critical) velocity (v_c) (meters/second), density (k_c) (veh/meter),
    # and volume (q_c) (veh/second)
    # constants depending on penetration rate
    # values calculated in MATLAB based on penetration rate
    pen_rate = PEN_RATE
    # free flow velocity
    v_f = 18
    if routes[0][0] == '1i_3' or routes[0][0] == '2i_3' or routes[0][0] == '3i_3' or routes[0][0] == '4i_3':
        tau = 3.2
    else:
        tau = 1.8

    Xmax = (Tr+Tg)*v_f
    Nmax = Tg/tau - 3
    Nab = count_veh(routes, 0, Xmax)

    Pab = min(count_veh(routes, 0, Xmax), Nmax)

    if Nab - Pab < 0:
        Sab = 0
    else:
        Sab = Nab - Pab

    return Sab, tau, Pab


def model(x, u):
    # x is Tg, u is phases, y is Tr
    Sab = [[0, 0], [0, 0], [0, 0], [0, 0]]
    Pab = [[0, 0], [0, 0], [0, 0], [0, 0]]
    tau_ab = [[0, 0], [0, 0], [0, 0], [0, 0]]
    tau = [0, 0, 0, 0]
    S = [0, 0, 0, 0]
    P = [0, 0, 0, 0]
    Z = [0, 0, 0, 0]
    PZ = [0, 0, 0, 0]
    # OBJ = 0

    y = [0, 4 + x[0], 10 + x[0] + x[1], 14 + x[0] + x[1] + x[2]]

    for i in range(len(u)):
        for j in range(len(u[i])):
            Sab[i][j], tau_ab[i][j], Pab[i][j] = delayed_vehicles(u[i][j], x[i], y[i])

    S[0] = max(Sab[0][0], Sab[0][1])
    S[1] = max(Sab[1][0], Sab[1][1])
    S[2] = max(Sab[2][0], Sab[2][1])
    S[3] = max(Sab[3][0], Sab[3][1])
    P[0] = max(Pab[0][0], Pab[0][1])
    P[1] = max(Pab[1][0], Pab[1][1])
    P[2] = max(Pab[2][0], Pab[2][1])
    P[3] = max(Pab[3][0], Pab[3][1])

    tau[0] = tau_ab[0][Sab[0].index(S[0])]
    tau[1] = tau_ab[1][Sab[1].index(S[1])]
    tau[2] = tau_ab[2][Sab[2].index(S[2])]
    tau[3] = tau_ab[3][Sab[3].index(S[3])]

    Z[0] = tau[0] * S[0]
    Z[1] = tau[1] * S[1]
    Z[2] = tau[2] * S[2]
    Z[3] = tau[3] * S[3]

    PZ[0] = tau[0] * P[0]
    PZ[1] = tau[1] * P[1]
    PZ[2] = tau[2] * P[2]
    PZ[3] = tau[3] * P[3]
    P_z = np.array(PZ)
    Z_ = np.array(Z)

    w1 = 0.7
    w2 = 0.3
    OBJ = -w1 * (1 / 4) * sum(P_z[0:]) + w2 * sum((1 / 4) * (Z_[0:] - sum(Z[0:])) ** 2)
    return OBJ


# def obj(x, u, y, v, k):
#     """The objective function"""
#     OBJ = np.array(model(x, u, y, v, k))
#     return OBJ


def run():
    """execute the TraCI control loop"""

    # Create a tuple of tuples with phases (1-4) and routes (1-8)
    # phase 1: R47 and R83
    # phase 2: R41 and R85
    # phase 3: R25 and R61
    # phase 4: R27 and R63
    # 8 routes,  1=R25, 2=R27, 3=R41, 4=R47, 5=R61, 6=R63, 7=R83, 8=R85 (refer to paper fig. 4)
    # routes 1, 4, 5, and 7 each consist of two lanes
    # -> 1: ["3i_0", "3i_1"], 2: ["3i_2"], 3: ["2i_2"], 4: ["2i_0", "2i_1"],
    # 5: ["4i_0", "4i_1"], 6: ["4i_2"], 7: ["1i_0", "1i_1"], 8: ["1i_2"]

    ########### old phases list ############################
    # phases[phase number][route number][name of each lane in route]
    # # only done in this "Data2" version: make "2i_0" a list of corresponding edges and lanes in new Data2 network.
    # phases = [[["2i_0", "2i_1"], ["1i_0", "1i_1"]], [["2i_2"], ["1i_2"]],
    #           [["3i_0", "3i_1"], ["4i_0", "4i_1"]], [["3i_2"], ["4i_2"]]]
    ############## end of old phases list ####################

    # so from ^ phases, 2i_0 = 2i_1 + 2_2i_1 + 2_3i_0.
    #              &    2i_1 = 2i_2 + 2_2i_2 + 2_3i_1.
    # new list: phases[phase number][route number][lane number][name of each lane in lane number]
    # phases = [[[["2i_1", "2_2i_1", "2_3i_0"], ["2i_2", "2_2i_2", "2_3i_1"]],  # phase 1
    #            [["1i_1", "1_2i_0", "1_3i_0", "1_4i_0"], ["1i_2", "1_2i_1", "1_3i_1", "1_4i_1"]]],  # phase 1
    #           [[["2i_3"]], [["1i_3", "1_2i_2", "1_3i_2"], ["1i_4", "1_2i_3"]]],  # phase 2
    #           [[["3i_0", "3_2i_0", "3_3i_0"], ["3i_1", "3_2i_1", "3_3i_1"]],  # phase 3
    #            [["4i_1", "4_2i_0", "4_3i_0"], ["4i_2", "4_2i_1", "4_3i_1"]]],  # phase 3
    #           [[["3i_3", "3_2i_2"], ["3i_4"]], [["4i_3"]]]]  # phase 4
    phases = [[[["2i_1", "2_2i_1", "2_3i_1"], ["2i_2", "2_2i_2", "2_3i_2"]],  # phase 1
               [["1i_1", "1_2i_1", "1_3i_1", "1_4i_1"], ["1i_2", "1_2i_2", "1_3i_2", "1_4i_2"]]],  # phase 1
              [[["2i_3"]], [["1i_3", "1_2i_3", "1_3i_3"], ["1i_4", "1_2i_4"]]],  # phase 2
              [[["3i_1", "3_2i_1", "3_3i_1"], ["3i_2", "3_2i_2", "3_3i_2"]],  # phase 3
               [["4i_1", "4_2i_1", "4_3i_1"], ["4i_2", "4_2i_2", "4_3i_2"]]],  # phase 3
              [[["3i_3", "3_2i_3"], ["3i_4"]], [["4i_3"]]]]  # phase 4
    step = 0
    # CAV penetration rate
    pen_rate = PEN_RATE
    # total traffic signal cycle length
    cycle_len = CYCLE_LENGTH - 20
    Tg = [15, 8, 15, 8]
    Tr = [0, 19, 33, 52]
    #newPhaseTimes = [18, 18, 18, 18]
    Qab = [[[0, 0], [0, 0]], [[0], [0, 0]], [[0, 0], [0, 0]], [[0, 0], [0]]]
    print("Number of vehicles in network: " + str(vehNr))
    print("Number of CAVs in network: " + str(cavNr))
    print("Number of total vehicles for each direction: " + str(pWENr) + "," + str(pWE_LNr) + "," + str(pWE_RNr) + "," + str(pEWNr) + "," + str(pEW_LNr) + "," + str(pEW_RNr) + "," + str(pNSNr) + "," + str(pNS_LNr) + "," + str(pNS_RNr) + "," + str(pSNNr) + "," + str(pSN_LNr) + "," + str(pSN_RNr))
    EAD_ID_list = []
    veh_vel = [[0 for col in range(cavNr)] for row in range(20000)]
    a_matrix = [[0 for col in range(cavNr)] for row in range(20000)]
    print(len(a_matrix))
    print(len(a_matrix[0]))
    a1 = np.ones((1, cavNr))
    last_speed = a1 * 13

    place = []
    # for disallow, private are right turning vehicles, vip are left turning, passenger are through vehicles.
    traci.lane.setDisallowed("1i_0", "passenger")
    traci.lane.setDisallowed("2i_0", "passenger")
    traci.lane.setDisallowed("3i_0", "passenger")
    traci.lane.setDisallowed("4i_0", "passenger")
    # West-to-East lane disallow.
    traci.lane.setDisallowed("1_4i_1", "vip")
    traci.lane.setDisallowed("1_4i_2", "private")
    traci.lane.setDisallowed("1_3i_1", "vip")
    traci.lane.setDisallowed("1_3i_2", "vip")
    traci.lane.setDisallowed("1_3i_2", "private")
    traci.lane.setDisallowed("1_3i_3", "passenger")
    traci.lane.setDisallowed("1_2i_1", "vip")
    traci.lane.setDisallowed("1_2i_2", "vip")
    traci.lane.setDisallowed("1_2i_2", "private")
    traci.lane.setDisallowed("1_2i_3", "passenger")
    traci.lane.setDisallowed("1i_1", "private")
    traci.lane.setDisallowed("1i_2", "vip")
    traci.lane.setDisallowed("1i_3", "passenger")

    # East-to-West lane disallow.
    traci.lane.setDisallowed("2_3i_1", "vip")
    traci.lane.setDisallowed("2_3i_2", "private")
    traci.lane.setDisallowed("2_2i_0", "vip")
    traci.lane.setDisallowed("2_2i_0", "passenger")
    traci.lane.setDisallowed("2_2i_1", "vip")
    traci.lane.setDisallowed("2_2i_1", "private")
    traci.lane.setDisallowed("2_2i_2", "private")
    traci.lane.setDisallowed("2i_1", "private")
    traci.lane.setDisallowed("2i_1", "vip")
    traci.lane.setDisallowed("2i_2", "vip")
    traci.lane.setDisallowed("2i_3", "passenger")

    # South-to-North lane disallow.
    traci.lane.setDisallowed("3_3i_1", "vip")
    traci.lane.setDisallowed("3_3i_2", "private")
    traci.lane.setDisallowed("3_2i_1", "vip")
    traci.lane.setDisallowed("3_2i_2", "vip")
    traci.lane.setDisallowed("3_2i_3", "private")
    traci.lane.setDisallowed("3_2i_3", "passenger")
    traci.lane.setDisallowed("3i_1", "private")
    traci.lane.setDisallowed("3i_2", "vip")
    traci.lane.setDisallowed("3i_3", "passenger")

    # North-to-South lane disallow.
    traci.lane.setDisallowed("4_3i_1", "vip")
    traci.lane.setDisallowed("4_3i_2", "private")
    traci.lane.setDisallowed("4_2i_1", "vip")
    traci.lane.setDisallowed("4_2i_2", "vip")
    traci.lane.setDisallowed("4_2i_2", "private")
    traci.lane.setDisallowed("4_2i_3", "private")
    traci.lane.setDisallowed("4i_1", "private")
    traci.lane.setDisallowed("4i_1", "vip")
    traci.lane.setDisallowed("4i_2", "vip")
    traci.lane.setDisallowed("4i_3", "passenger")

    # we start with phase 0 where EW has green
    traci.trafficlight.setPhase("0", 0)

    total_co2_emission = 0.0
    total_fuel_consumption = 0.0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        time = traci.simulation.getTime()

        vehicle_ids = traci.vehicle.getIDList()
        for veh_id in vehicle_ids:
            # Use TRACI to get CO2 emissions and fuel consumption
            co2_emission = traci.vehicle.getCO2Emission(veh_id)
            fuel_consumption = traci.vehicle.getFuelConsumption(veh_id)
            
            total_co2_emission += co2_emission
            total_fuel_consumption += fuel_consumption
        
        # if traci.trafficlight.getNextSwitch("0") - traci.simulation.getTime() < 0.1:

        #     if traci.trafficlight.getPhase("0") > 8:
        #         # this phase is where optimization for next cycle will happen.
        #         # for obj function:
        #         # x is Tg, u is phases, y is Tr

        #         bounds = Bounds([TGMIN, TGMIN, TGMIN, TGMIN], [TGMAX, TGMAX, TGMAX, TGMAX])

        #         eq_cons = {'type': 'eq', 'fun': lambda x: np.array([x[0] + x[1] + x[2] + x[3] - cycle_len]),
        #                    'jac': lambda x: np.array([1, 1, 1, 1])}

        #         Tg0 = np.array([15, 8, 15, 8])

        #         u = np.array(phases, dtype=object)

        #         res = minimize(model, Tg0, args=u, method='SLSQP', jac=None, bounds=bounds,
        #                        constraints=eq_cons, options={'ftol': 1e-3, 'iprint': -1})

        #         print(res.x)
        #         newPhases = [traci.trafficlight.Phase(res.x[0], 'rrrrgGGrrrrrrgGGrr', next=()),
        #                      traci.trafficlight.Phase(4.0, 'rrrryyyrrrrrryyyrr', next=()),
        #                      traci.trafficlight.Phase(res.x[1], 'grrrrrrGgrrrrrrrGG', next=()),
        #                      traci.trafficlight.Phase(4.0, 'yrrrrrryyrrrrrrryy', next=()),
        #                      traci.trafficlight.Phase(2.0, 'rrrrrrrrrrrrrrrrrr', next=()),
        #                      traci.trafficlight.Phase(res.x[2], 'gGGrrrrrgGGrrrrrrr', next=()),
        #                      traci.trafficlight.Phase(4.0, 'yyyrrrrryyyrrrrrrr', next=()),
        #                      traci.trafficlight.Phase(res.x[3], 'rrrGgrrrrrrGGgrrrr', next=()),
        #                      traci.trafficlight.Phase(4.0, 'rrryyrrrrrryyyrrrr', next=()),
        #                      traci.trafficlight.Phase(2.0, 'rrrrrrrrrrrrrrrrrr', next=())]
        #         newLogic = traci.trafficlight.Logic("0", 0, 9, newPhases)
        #         traci.trafficlight.setProgramLogic("0", newLogic)
        #         #  Set new Tg and Tr values based on optimization
        #         Tg.clear()
        #         Tg = [res.x[0], res.x[1], res.x[2], res.x[3]]
        #         # Tr.clear()
        #         # Tr = [0, 4+Tg[0], 10 + Tg[0] + Tg[1], 14 + Tg[0] + Tg[1] + Tg[2]]

        # EAD Code Section of while loop:
        # add EAD vehicle to list
        vehicle_ID_list = traci.vehicle.getIDList()
        for veh in vehicle_ID_list:
            if traci.vehicle.getTypeID(veh) == "typeCAV" or traci.vehicle.getTypeID(veh) == "typeCAV_L" or traci.vehicle.getTypeID(veh) == "typeCAV_R":
                if veh not in EAD_ID_list:
                    EAD_ID_list.append(veh)

        ### Exist EAD Vehicle
        Intersection = [val for val in vehicle_ID_list if val in EAD_ID_list]
        # print('Exist EAD ID:', Intersection)
        ### Find the place of existing EAD vehicle
        place = [None] * len(Intersection)
        for p in range(len(Intersection)):
            place[p] = EAD_ID_list.index(Intersection[p])
            # print('place:', EAD_ID_list.index(Intersection[p]))
        # print('EAD place list:', place)

        for phaseNum in range(len(phases)):
            for routeNum in range(len(phases[phaseNum])):
                for l in range(len(phases[phaseNum][routeNum])):
                    for e in range(len(phases[phaseNum][routeNum][l])):
                        lane_veh = traci.lane.getLastStepVehicleIDs(phases[phaseNum][routeNum][l][e])
                        for v in lane_veh:
                            if traci.vehicle.getSpeed(v) < 0.1:
                                if traci.vehicle.getNextTLS(v)[0][2] < 150:
                                    Qab[phaseNum][routeNum][l] = Qab[phaseNum][routeNum][l] + 1
        # print("Qab: ")
        # print(Qab)
        ### Implement EAD to vehicles
        # 09/30/2021 David: Changed EAD_acceleration to include vehicle angle and new phase times
        #  Also changed state from traci.trafficlight.getRedYellowGreenState("0") to traci.trafficlight.getPhase("0"), which
        #  gives the index of the red, yellow, green state rather than actual states like rrrrgGGrrrrrgGGr
        #  10/18/2021 David: Testing getLeader function
        for q in range(len(Intersection)):
            a_matrix[step][place[q]], last_speed[:, place[q]] = EAD_acceleration(traci.vehicle.getPosition(Intersection[q]), traci.trafficlight.getPhase("0"), traci.trafficlight.getNextSwitch("0"), time, traci.vehicle.getSpeed(Intersection[q]), Intersection[q], last_speed[:, place[q]], traci.vehicle.getAngle(Intersection[q]), Tg, traci.vehicle.getTypeID(Intersection[q]), Qab, phases, traci.vehicle.getLaneID(Intersection[q]), traci.vehicle.getRoadID(Intersection[q]))

            #  print('a matrix:', a_matrix)
            if traci.vehicle.getLeader(Intersection[q]) is not None:
                if traci.vehicle.getLeader(Intersection[q])[1] < 20:
                    #  print("Back to CACC")
                    traci.vehicle.setSpeed(Intersection[q], -1)
                    # traci.vehicle.setColor(Intersection[q], (0, 0, 255))
                    # if Intersection[q] in WaitCAVs:
                        # traci.vehicle.setColor(Intersection[q], (255, 255, 255))
                #else:
                    # traci.vehicle.setColor(Intersection[q], (255, 0, 0))
                    #if Intersection[q] in WaitCAVs:
                        # traci.vehicle.setColor(Intersection[q], (255, 255, 255))
            veh_vel[step][place[q]] = traci.vehicle.getSpeed(Intersection[q])

        Qab = [[[0, 0], [0, 0]], [[0], [0, 0]], [[0, 0], [0, 0]], [[0, 0], [0]]]
        step += 1

    print(f"Total CO2 emission: {total_co2_emission} kg")
    print(f"Total fuel consumption: {total_fuel_consumption} liters")

    traci.close()
    sys.stdout.flush()


# def get_options():
#     optParser = optparse.OptionParser()
#     optParser.add_option("--nogui", action="store_true",
#                          default=False, help="run the commandline version of sumo")
#     options, args = optParser.parse_args()
#     return options


# this is the main entry point of this script
if __name__ == "__main__":
    # options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "Data2/cross.sumocfg", "--step-length", "0.1", "--device.rerouting.threads",
    #              "64", "--device.rerouting.synchronize", "true", "--time-to-teleport", "100", "--tripinfo-output",
    #              complete_tripInfo, "--summary", complete_Summary, "--amitran-output", complete_trajectory,
    #              "--queue-output", complete_queue, "--statistic-output", complete_stat, "--fcd-output", complete_fcd])

    traci.start([checkBinary('sumo'), "-c", "Data2/cross.sumocfg", "--step-length", "0.1", "--device.rerouting.threads",
                     "64", "--device.rerouting.synchronize", "true", "--time-to-teleport", "100", "--tripinfo-output",
                     "tripinfo.xml", "--summary", "summary_100.xml", "--amitran-output", "TrajectoryOutput.xml",
                     "--queue-output", "QueueOutput.xml", "--statistic-output", "StatOutput.xml", "--fcd-output", "FCDOutput.xml"])
    run()
