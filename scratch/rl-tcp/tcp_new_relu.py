from tcp_base import TcpEventBased
import numpy as np
import keras

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"



def old_get_action(obs, reward, done, info):
    # unique socket ID
    socketUuid = obs[0]
    # TCP env type: event-based = 0 / time-based = 1
    envType = obs[1]
    # sim time in us
    simTime_us = obs[2]
    # unique node ID
    nodeId = obs[3]
    # current ssThreshold
    ssThresh = obs[4]
    # current contention window size
    cWnd = obs[5]
    # segment size
    segmentSize = obs[6]
    # number of acked segments
    segmentsAcked = obs[7]
    # estimated bytes in flight
    bytesInFlight  = obs[8]

    new_cWnd = 1
    new_ssThresh = 1

    # IncreaseWindow
    if (cWnd < ssThresh):
        # slow start
        if (segmentsAcked >= 1):
            new_cWnd = cWnd + segmentSize

    if (cWnd >= ssThresh):
        # congestion avoidance
        if (segmentsAcked > 0):
            adder = 1.0 * (segmentSize * segmentSize) / cWnd;
            adder = int(max (1.0, adder))
            new_cWnd = cWnd + adder

    # GetSsThresh
    new_ssThresh = int(max (2 * segmentSize, bytesInFlight / 2))

    # return actions
    actions = [new_ssThresh, int(new_cWnd)]

    return actions

def un_one_hot_encode(y):
    return np.amax(y)

class TcpNewReLu(TcpEventBased):
    """docstring for TcpNewReno"""
    def __init__(self, model, scaler):
        super(TcpNewReLu, self).__init__()
        self.model = model
        self.scaler = scaler
    def get_action(self, obs, reward, done, info):

        y = self.model.predict(self.scaler.transform(np.asarray(obs[15:115]+[obs[117]]).reshape(1, -1)))
        j = un_one_hot_encode(y)
        if j <= 0:
            return old_get_action(obs, reward, done, info)

        ssThresh = obs[4]
        # current contention window size
        cWnd = obs[5]
        # segment size
        segmentSize = obs[6]
        # number of acked segments
        # estimated bytes in flight
        bytesInFlight  = obs[8]

        if j <= 2:
            return [int(max (2 * segmentSize, bytesInFlight / 2)), cWnd + segmentSize]
        return [int(max (3 * segmentSize, bytesInFlight / 2)), cWnd + segmentSize*4]