#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gym
import argparse
from ns3gym import ns3env
from tcp_base import TcpTimeBased
from tcp_newreno import TcpNewReno

import tensorflow as tf
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.policies import FeedForwardPolicy
from stable_baselines import PPO1
from stable_baselines.common.base_class import BaseRLModel
import os
import sys
import inspect

arch_str = "32,16"
if arch_str == "":
    arch = []
else:
    arch = [int(layer_width) for layer_width in arch_str.split(",")]
print("Architecture is ", str(arch))


training_sess = None

iterationNum = 1

class MyMlpPolicy(FeedForwardPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **_kwargs):
        super(MyMlpPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse, net_arch=[{"pi":arch, "vf":arch}], feature_extraction="mlp", **_kwargs)
        global training_sess
        training_sess = sess


simArgs = {"--duration": 100000, "--transport_prot": "TcpRlTimeBased"}

env = ns3env.Ns3Env(port=5555, stepTime= 0.1, startSim=1,simSeed=12,simArgs=simArgs,debug=True)
ob = env.reset()
action = env.action_space.sample()
print(type(env.observation_space), type(env.action_space))


gamma = 0.99
print("gamma = ", gamma)
model = PPO1(MlpPolicy, env, verbose=2, schedule= 'constant', timesteps_per_actorbatch=8192, optim_batchsize=2048, gamma=gamma)
#model = PPO1(MyMlpPolicy, env, verbose=2, schedule= 'constant', timesteps_per_actorbatch=8192, optim_batchsize=2048, gamma=gamma)
#for i in range(0,6):
i=0
try:
    #model.learn(total_timesteps=(1600*410))
    #with model.graph.as_default():
    #    saver = tf.compat.v1.train.Saver()
    #    saver.save(training_sess, "./pcc_model_%d.ckpt" % i)
    model.learn(total_timesteps=(1*410))
    model.save("testmodel")

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    env.close()
    print("Done")
