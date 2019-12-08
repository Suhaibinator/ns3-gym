#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env
from tcp_base import TcpTimeBased
from tcp_newreno import TcpNewReno


__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=1,
                    help='Number of iterations, Default: 1')
args = parser.parse_args()
startSim = bool(args.start)
iterationNum = int(args.iterations)

port = 5555
simTime = 10 # seconds
stepTime = 0.5  # seconds
seed = 12
simArgs = {"--duration": simTime, "--transport_prot": "TcpRl"}
debug = False



env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
# simpler:
#env = ns3env.Ns3Env()
env.reset()

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

stepIdx = 0
currIt = 0

def get_agent(obs):
    socketUuid = obs[0]
    tcpEnvType = obs[1]
    tcpAgent = get_agent.tcpAgents.get(socketUuid, None)
    if tcpAgent is None:
        if tcpEnvType == 0:
            # event-based = 0
            tcpAgent = TcpNewReno()
        else:
            # time-based = 1
            tcpAgent = TcpTimeBased()
        tcpAgent.set_spaces(get_agent.ob_space, get_agent.ac_space)
        get_agent.tcpAgents[socketUuid] = tcpAgent

    return tcpAgent

# initialize variable
get_agent.tcpAgents = {}
get_agent.ob_space = ob_space
get_agent.ac_space = ac_space

observations = []

try:
    while True:
        print("Start iteration: ", currIt)
        obs = env.reset()
        reward = 0
        done = False
        info = None
        print("Step: ", stepIdx)
        print("---obs: ", obs)

        # get existing agent of create new TCP agent if needed
        tcpAgent = get_agent(obs)

        while True:
            stepIdx += 1
            action = tcpAgent.get_action(obs, reward, done, info)
            #print("---action: ", action)

            #print("Step: ", stepIdx)
            obs, reward, done, info = env.step(action)
            #print("---obs, reward, done, info: ", obs, reward, done, info)
            observations.append(obs)
            # get existing agent of create new TCP agent if needed
            tcpAgent = get_agent(obs)

            if done:
                stepIdx = 0
                if currIt + 1 < iterationNum:
                    env.reset()
                break

        currIt += 1
        if currIt == iterationNum:
            break

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    env.close()
    print("Done")



"""
print(nodeIds)
#print(congWindows)
print(segmentsAcked)
print(bytesInFlight)
print(calledFunctions)
print(congStates)


#plt.bar(range(len(congWindows)), list(congWindows.values()), align='center')
#plt.xticks(range(len(congWindows)), list(congWindows.keys()))
print(wins.keys())
"""

"""
k = 0
for i in wins.values():
    k+=1
    plt.subplot(len(wins), 1, k)
    plt.plot(i)
plt.show()
"""
import os
print(os.path.dirname(os.path.realpath(__file__)))

fileName = "data.csv"

f = open(fileName, "w")


for obs in observations:
    f.write(str(obs)[1:-1] + "\n")
f.close()
