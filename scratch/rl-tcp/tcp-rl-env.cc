/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 */

#include "tcp-rl-env.h"
#include "ns3/tcp-header.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/traffic-control-module.h"
#include <vector>
#include <queue>
#include <numeric>
#include <cmath>


namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("ns3::TcpGymEnv");
  NS_OBJECT_ENSURE_REGISTERED (TcpGymEnv);

  TcpGymEnv::TcpGymEnv ()
  {
    NS_LOG_FUNCTION (this);
    SetOpenGymInterface(OpenGymInterface::Get());
  }

  TcpGymEnv::~TcpGymEnv ()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  TcpGymEnv::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::TcpGymEnv")
    .SetParent<OpenGymEnv> ()
    .SetGroupName ("OpenGym")
    ;

    return tid;
  }

  void
  TcpGymEnv::DoDispose ()
  {
    NS_LOG_FUNCTION (this);
  }

  void
  TcpGymEnv::SetNodeId(uint32_t id)
  {
    NS_LOG_FUNCTION (this);
    m_nodeId = id;
  }

  void
  TcpGymEnv::SetSocketUuid(uint32_t id)
  {
    NS_LOG_FUNCTION (this);
    m_socketUuid = id;
  }

  std::string
  TcpGymEnv::GetTcpCongStateName(const TcpSocketState::TcpCongState_t state)
  {
    std::string stateName = "UNKNOWN";
    switch(state) {
      case TcpSocketState::CA_OPEN:
      stateName = "CA_OPEN";
      break;
      case TcpSocketState::CA_DISORDER:
      stateName = "CA_DISORDER";
      break;
      case TcpSocketState::CA_CWR:
      stateName = "CA_CWR";
      break;
      case TcpSocketState::CA_RECOVERY:
      stateName = "CA_RECOVERY";
      break;
      case TcpSocketState::CA_LOSS:
      stateName = "CA_LOSS";
      break;
      case TcpSocketState::CA_LAST_STATE:
      stateName = "CA_LAST_STATE";
      break;
      default:
      stateName = "UNKNOWN";
      break;
    }
    return stateName;
  }

  std::string
  TcpGymEnv::GetTcpCAEventName(const TcpSocketState::TcpCAEvent_t event)
  {
    std::string eventName = "UNKNOWN";
    switch(event) {
      case TcpSocketState::CA_EVENT_TX_START:
      eventName = "CA_EVENT_TX_START";
      break;
      case TcpSocketState::CA_EVENT_CWND_RESTART:
      eventName = "CA_EVENT_CWND_RESTART";
      break;
      case TcpSocketState::CA_EVENT_COMPLETE_CWR:
      eventName = "CA_EVENT_COMPLETE_CWR";
      break;
      case TcpSocketState::CA_EVENT_LOSS:
      eventName = "CA_EVENT_LOSS";
      break;
      case TcpSocketState::CA_EVENT_ECN_NO_CE:
      eventName = "CA_EVENT_ECN_NO_CE";
      break;
      case TcpSocketState::CA_EVENT_ECN_IS_CE:
      eventName = "CA_EVENT_ECN_IS_CE";
      break;
      case TcpSocketState::CA_EVENT_DELAYED_ACK:
      eventName = "CA_EVENT_DELAYED_ACK";
      break;
      case TcpSocketState::CA_EVENT_NON_DELAYED_ACK:
      eventName = "CA_EVENT_NON_DELAYED_ACK";
      break;
      default:
      eventName = "UNKNOWN";
      break;
    }
    return eventName;
  }


/*
Define game over condition
*/
  bool
  TcpGymEnv::GetGameOver()
  {
    m_isGameOver = false;
    bool test = false;
    static float stepCounter = 0.0;
    stepCounter += 1;
    if (stepCounter == 10 && test) {
      m_isGameOver = true;
    }
    NS_LOG_INFO ("MyGetGameOver: " << m_isGameOver);
    return m_isGameOver;
  }

/*
Define reward function
*/
  float
  TcpTimeStepGymEnv::GetReward()
  {
    NS_LOG_INFO("MyGetReward: " << m_envReward);
    return m_envReward;
  }


  float
  TcpEventGymEnv::GetReward()
  {
    NS_LOG_INFO("MyGetReward: " << m_envReward);
    return m_envReward;
  }
/*
Define extra info. Optional
*/
  std::string
  TcpGymEnv::GetExtraInfo()
  {
    NS_LOG_INFO("MyGetExtraInfo: " << m_info);
    return m_info;
  }

/*
Execute received actions
*/
  bool
  TcpGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
  {
    Ptr<OpenGymBoxContainer<float> > box = DynamicCast<OpenGymBoxContainer<float> >(action);
    m_new_ssThresh = ceil(box->GetValue(0));
    m_new_cWnd = ceil(box->GetValue(1));
    NS_LOG_INFO ("MyExecuteActions: " << action);
    return true;
  }


  NS_OBJECT_ENSURE_REGISTERED (TcpEventGymEnv);

  TcpEventGymEnv::TcpEventGymEnv () : TcpGymEnv()
  {
    NS_LOG_FUNCTION (this);

    latestPackets = Names::Find<MyReceived>("RxPacketsHolder");
    latestTimes = Names::Find<MyReceived>("RxTimesHolder");
    for (int i = 0; i < latestTimes->GetNumLeaf(); i++){
      lastPackets.push_back(0);
      lastTimes.push_back(0);
      lastThpts.push_back(0.0);
    }
    sendSideBottle = Names::Find<PointToPointNetDevice>("SendBottleND");
    recvSideBottle = Names::Find<PointToPointNetDevice>("RecvBottleND");
  }

  TcpEventGymEnv::~TcpEventGymEnv ()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  TcpEventGymEnv::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::TcpEventGymEnv")
    .SetParent<TcpGymEnv> ()
    .SetGroupName ("OpenGym")
    .AddConstructor<TcpEventGymEnv> ()
    ;

    return tid;
  }

  void
  TcpEventGymEnv::DoDispose ()
  {
    NS_LOG_FUNCTION (this);
  }

/*
Define action space
*/
  Ptr<OpenGymSpace>
  TcpEventGymEnv::GetActionSpace()
  {
  // new_ssThresh
  // new_cWnd
    uint32_t parameterNum = 2;
    float low = m_tcb->m_segmentSize;
    float high = 65535;
    std::vector<uint32_t> shape = {parameterNum,};
  //std::string dtype = TypeNameGet<uint32_t> ();
    std::string dtype = TypeNameGet<float> ();

    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
    NS_LOG_INFO ("MyGetActionSpace: " << box);
    return box;
  }

  void
  TcpEventGymEnv::SetReward(float value)
  {
    NS_LOG_FUNCTION (this);
    m_reward = value;
  }

  void
  TcpEventGymEnv::SetPenalty(float value)
  {
    NS_LOG_FUNCTION (this);
    m_penalty = value;
  }

/*
Define observation space
*/
  Ptr<OpenGymSpace>
  TcpEventGymEnv::GetObservationSpace()
  {
  // socket unique ID
  // tcp env type: event-based = 0 / time-based = 1
  // sim time in us
  // node ID
  // ssThresh
  // cWnd
  // segmentSize
  // segmentsAcked
  // bytesInFlight
  // rtt in us
  // min rtt in us
  // called func
  // congetsion algorithm (CA) state
  // CA event
  // ECN state
    uint32_t parameterNum = 120;
    float low = 0.0;
    float high = 1000000000.0;
    std::vector<uint32_t> shape = {parameterNum,};
    std::string dtype = TypeNameGet<uint64_t> ();

    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
    NS_LOG_INFO ("MyGetObservationSpace: " << box);
    return box;
  }

/*
Collect observations
*/
  Ptr<OpenGymDataContainer>
  TcpEventGymEnv::GetObservation()
  {
    uint32_t parameterNum = 120;
    std::vector<uint32_t> shape = {parameterNum,};

    Ptr<OpenGymBoxContainer<uint64_t> > box = CreateObject<OpenGymBoxContainer<uint64_t> >(shape);

  box->AddValue(m_socketUuid);  // 0
  box->AddValue(0); // 1
  box->AddValue(Simulator::Now().GetMicroSeconds ()); // 2
  box->AddValue(m_nodeId); // 3
  box->AddValue(m_tcb->m_ssThresh); // 4
  box->AddValue(m_tcb->m_cWnd); // 5
  box->AddValue(m_tcb->m_segmentSize);  // 6
  box->AddValue(m_segmentsAcked); // 7
  box->AddValue(m_bytesInFlight); // 8
  box->AddValue(m_rtt.GetMicroSeconds ());  // 9
  box->AddValue(m_tcb->m_minRtt.GetMicroSeconds ());  // 10
  box->AddValue(m_calledFunc);  // 11
  box->AddValue(m_tcb->m_congState);  // 12
  box->AddValue(m_event); // 13
  box->AddValue(m_tcb->m_ecnState); // 14
  std::queue<uint> temp;
  while(!recentRtts.empty()){
    uint j = recentRtts.front();
    recentRtts.pop();
    temp.push(j);
    box->AddValue(j);
  }
  while (!temp.empty()){
    recentRtts.push(temp.front());
    temp.pop();
  }

  for (uint i = recentRtts.size(); i < rttQueueSize; i++){
    box->AddValue(-1);
  }

  Ptr<QueueDisc> SenderSideBottleNeckQueue = Names::Find<QueueDisc> ("SSBNQ");
  Ptr<QueueDisc> ReceiverSideBottleNeckQueue = Names::Find<QueueDisc> ("RSBNQ");
  box->AddValue(SenderSideBottleNeckQueue-> GetCurrentSize().GetValue());
  box->AddValue(ReceiverSideBottleNeckQueue -> GetCurrentSize().GetValue());

  int nLeaf = latestPackets->GetNumLeaf();

  for (int i = 0; i < nLeaf; i++){
    if (latestTimes->GetPackets(i) - lastTimes[i] > 2e5){
      double currentThpt = ((double)(latestPackets->GetPackets(i)-lastPackets.at(i)))/((double)(latestTimes->GetPackets(i) - lastTimes.at(i))*1e-3);
      box->AddValue(currentThpt);
      lastPackets[i] = latestPackets->GetPackets(i);
      lastTimes[i] = latestTimes->GetPackets(i);
      lastThpts[i] = currentThpt;
    } else {
      box->AddValue(lastThpts[i]);
    }
  }


  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);
  return box;
}

void
TcpEventGymEnv::TxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>)
{
  NS_LOG_FUNCTION (this);
}

void
TcpEventGymEnv::RxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>)
{
  NS_LOG_FUNCTION (this);
}

uint32_t
TcpEventGymEnv::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this);
  // pkt was lost, so penalty
  m_envReward = m_penalty;

  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " GetSsThresh, BytesInFlight: " << bytesInFlight);
  m_calledFunc = CalledFunc_t::GET_SS_THRESH;
  m_info = "GetSsThresh";
  m_tcb = tcb;
  m_bytesInFlight = bytesInFlight;
  Notify();
  return m_new_ssThresh;
}

void
TcpEventGymEnv::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this);
  // pkt was acked, so reward
  // m_envReward = m_reward;

  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " IncreaseWindow, SegmentsAcked: " << segmentsAcked);
  m_calledFunc = CalledFunc_t::INCREASE_WINDOW;
  m_info = "IncreaseWindow";
  m_tcb = tcb;
  m_segmentsAcked = segmentsAcked;
  Notify();
  tcb->m_cWnd = m_new_cWnd;
}

void
TcpEventGymEnv::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " PktsAcked, SegmentsAcked: " << segmentsAcked << " Rtt: " << rtt);
  m_calledFunc = CalledFunc_t::PKTS_ACKED;
  m_info = "PktsAcked";
  m_tcb = tcb;
  m_segmentsAcked = segmentsAcked;
  m_rtt = rtt;
  recentRtts.push(m_rtt.GetMicroSeconds());
  if (recentRtts.size() > rttQueueSize){
    recentRtts.pop();
  }
}

void
TcpEventGymEnv::CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this);
  std::string stateName = GetTcpCongStateName(newState);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CongestionStateSet: " << newState << " " << stateName);

  m_calledFunc = CalledFunc_t::CONGESTION_STATE_SET;
  m_info = "CongestionStateSet";
  m_tcb = tcb;
  m_newState = newState;
}

void
TcpEventGymEnv::CwndEvent (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
  NS_LOG_FUNCTION (this);
  std::string eventName = GetTcpCAEventName(event);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CwndEvent: " << event << " " << eventName);

  m_calledFunc = CalledFunc_t::CWND_EVENT;
  m_info = "CwndEvent";
  m_tcb = tcb;
  m_event = event;
}


NS_OBJECT_ENSURE_REGISTERED (TcpTimeStepGymEnv);

TcpTimeStepGymEnv::TcpTimeStepGymEnv () : TcpGymEnv()
{
  NS_LOG_FUNCTION (this);
  m_envReward = 0.0;
}

TcpTimeStepGymEnv::TcpTimeStepGymEnv (Time timeStep) : TcpGymEnv()
{
  NS_LOG_FUNCTION (this);
  m_timeStep = timeStep;
  m_envReward = 0.0;

  latestPackets = Names::Find<MyReceived>("RxPacketsHolder");
  latestTimes = Names::Find<MyReceived>("RxTimesHolder");
  for (int i = 0; i < latestTimes->GetNumLeaf(); i++){
    lastPackets.push_back(0);
    lastTimes.push_back(0);
  }

  sendSideBottle = Names::Find<PointToPointNetDevice>("SendBottleND");
  recvSideBottle = Names::Find<PointToPointNetDevice>("RecvBottleND");
}

void
TcpTimeStepGymEnv::ScheduleNextStateRead ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule (m_timeStep, &TcpTimeStepGymEnv::ScheduleNextStateRead, this);
  Notify();
}

TcpTimeStepGymEnv::~TcpTimeStepGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
TcpTimeStepGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpTimeStepGymEnv")
  .SetParent<TcpGymEnv> ()
  .SetGroupName ("OpenGym")
  .AddConstructor<TcpTimeStepGymEnv> ()
  ;

  return tid;
}

void
TcpTimeStepGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
TcpTimeStepGymEnv::GetObservationSpace()
{
  // socket unique ID
  // tcp env type: event-based = 0 / time-based = 1
  // sim time in us
  // node ID
  // ssThresh
  // cWnd
  // segmentSize
  // bytesInFlightSum
  // bytesInFlightAvg
  // segmentsAckedSum
  // segmentsAckedAvg
  // avgRtt
  // minRtt
  // avgInterTx
  // avgInterRx
  // throughput
  uint32_t parameterNum = 121;
  float low = -2;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint64_t> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetObservationSpace: " << box);
  return box;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
TcpTimeStepGymEnv::GetObservation()
{
  uint32_t parameterNum = 121;
  std::vector<uint32_t> shape = {parameterNum,};

  Ptr<OpenGymBoxContainer<uint64_t> > box = CreateObject<OpenGymBoxContainer<uint64_t> >(shape);

  box->AddValue(m_socketUuid); // 0
  box->AddValue(1); // 1
  box->AddValue(Simulator::Now().GetMicroSeconds ()); // 2
  box->AddValue(m_nodeId); // 3
  box->AddValue(m_tcb->m_ssThresh); // 4
  box->AddValue(m_tcb->m_cWnd); // 5
  box->AddValue(m_tcb->m_segmentSize); // 6

  //bytesInFlightSum
  uint64_t bytesInFlightSum = std::accumulate(m_bytesInFlight.begin(), m_bytesInFlight.end(), 0);
  box->AddValue(bytesInFlightSum); // 7

  //bytesInFlightAvg
  uint64_t bytesInFlightAvg = 0;
  if (m_bytesInFlight.size()) {
    bytesInFlightAvg = bytesInFlightSum / m_bytesInFlight.size();
  }
  box->AddValue(bytesInFlightAvg); // 8

  //segmentsAckedSum
  uint64_t segmentsAckedSum = std::accumulate(m_segmentsAcked.begin(), m_segmentsAcked.end(), 0);
  box->AddValue(segmentsAckedSum); // 9

  //segmentsAckedAvg
  uint64_t segmentsAckedAvg = 0;
  if (m_segmentsAcked.size()) {
    segmentsAckedAvg = segmentsAckedSum / m_segmentsAcked.size();
  }
  box->AddValue(segmentsAckedAvg); // 10
  //avgRtt
  Time avgRtt = Seconds(0.0);
  if(m_rttSampleNum) {
    avgRtt = m_rttSum / m_rttSampleNum;
  }
  box->AddValue(avgRtt.GetMicroSeconds ()); // 11

  //m_minRtt
  box->AddValue(m_tcb->m_minRtt.GetMicroSeconds ()); // 12

  //avgInterTx
  Time avgInterTx = Seconds(0.0);
  if (m_interTxTimeNum) {
    avgInterTx = m_interTxTimeSum / m_interTxTimeNum;
  }
  box->AddValue(avgInterTx.GetMicroSeconds ()); // 13

  //avgInterRx
  Time avgInterRx = Seconds(0.0);
  if (m_interRxTimeNum) {
    avgInterRx = m_interRxTimeSum / m_interRxTimeNum;
  }
  box->AddValue(avgInterRx.GetMicroSeconds ()); // 14

  //throughput  bytes/s
  float throughput = (segmentsAckedSum * m_tcb->m_segmentSize) / m_timeStep.GetSeconds();
  box->AddValue(throughput); // 15

  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);
  float segmentNotAcked = bytesInFlightSum / float(m_tcb -> m_segmentSize);
  if (segmentsAckedSum + segmentNotAcked > 0)
    m_envReward = 10.0* throughput - 2000.0* segmentNotAcked / (segmentsAckedSum + segmentNotAcked) - 1000.0 *avgInterTx.GetSeconds();
  else
    m_envReward = 10.0* throughput - 1000.0 *avgInterTx.GetSeconds();
  

  int nLeaf = latestPackets->GetNumLeaf();

  // 16 - 18
  for (int i = 0; i < nLeaf; i++){
    //double currentBandwidth = (double)(latestPackets->GetPackets(i)-lastPackets.at(i))/(latestTimes->GetPackets(i) - lastTimes.at(i));//;(latestPackets->GetPackets(i)-lastPackets.at(i))/(1+latestTimes->GetPackets(i) - lastTimes.at(i));
    box->AddValue(((double)(latestPackets->GetPackets(i)-lastPackets.at(i)))/((double)(latestTimes->GetPackets(i) - lastTimes.at(i))*1e-3));
    lastPackets[i] = latestPackets->GetPackets(i);
    lastTimes[i] = latestTimes->GetPackets(i);
  }

  Ptr<QueueDisc> SenderSideBottleNeckQueue = Names::Find<QueueDisc> ("SSBNQ");
  Ptr<QueueDisc> ReceiverSideBottleNeckQueue = Names::Find<QueueDisc> ("RSBNQ");
  box->AddValue(SenderSideBottleNeckQueue-> GetCurrentSize().GetValue()); // 19
  box->AddValue(ReceiverSideBottleNeckQueue -> GetCurrentSize().GetValue()); // 20

  DataRateValue sdr;
  sendSideBottle->GetAttribute("DataRate", sdr);
  box->AddValue(sdr.Get().GetBitRate()/1000.0);
  DataRateValue rdr;
  recvSideBottle->GetAttribute("DataRate", rdr);
  box->AddValue(rdr.Get().GetBitRate()/1000.0);

  std::queue<uint> temp;
  while(!recentRtts.empty()){
    uint j = recentRtts.front();
    recentRtts.pop();
    temp.push(j);
    box->AddValue(j);
  }
  while (!temp.empty()){
    recentRtts.push(temp.front());
    temp.pop();
  }

  for (uint i = recentRtts.size(); i < rttQueueSize; i++){
    box->AddValue(-1);
  }


  m_bytesInFlight.clear();
  m_segmentsAcked.clear();

  m_rttSampleNum = 0;
  m_rttSum = MicroSeconds (0.0);

  m_interTxTimeNum = 0;
  m_interTxTimeSum = MicroSeconds (0.0);

  m_interRxTimeNum = 0;
  m_interRxTimeSum = MicroSeconds (0.0);

  return box;
}

void
TcpTimeStepGymEnv::TxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>)
{
  NS_LOG_FUNCTION (this);
  if ( m_lastPktTxTime > MicroSeconds(0.0) ) {
    Time interTxTime = Simulator::Now() - m_lastPktTxTime;
    m_interTxTimeSum += interTxTime;
    m_interTxTimeNum++;
  }

  m_lastPktTxTime = Simulator::Now();
}

void
TcpTimeStepGymEnv::RxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>)
{
  NS_LOG_FUNCTION (this);
  if ( m_lastPktRxTime > MicroSeconds(0.0) ) {
    Time interRxTime = Simulator::Now() - m_lastPktRxTime;
    m_interRxTimeSum +=  interRxTime;
    m_interRxTimeNum++;
  }

  m_lastPktRxTime = Simulator::Now();
}

uint32_t
TcpTimeStepGymEnv::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " GetSsThresh, BytesInFlight: " << bytesInFlight);
  m_tcb = tcb;
  m_bytesInFlight.push_back(bytesInFlight);

  if (!m_started) {
    m_started = true;
    Notify();
    ScheduleNextStateRead();
  }

  // action
  return m_new_ssThresh;
}

void
TcpTimeStepGymEnv::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " IncreaseWindow, SegmentsAcked: " << segmentsAcked);
  m_tcb = tcb;
  m_segmentsAcked.push_back(segmentsAcked);
  m_bytesInFlight.push_back(tcb->m_bytesInFlight);

  if (!m_started) {
    m_started = true;
    Notify();
    ScheduleNextStateRead();
  }
  // action
  tcb->m_cWnd = m_new_cWnd;
}

void
TcpTimeStepGymEnv::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " PktsAcked, SegmentsAcked: " << segmentsAcked << " Rtt: " << rtt);
  m_tcb = tcb;
  m_rttSum += rtt;
  m_rttSampleNum++;


  recentRtts.push(rtt.GetMicroSeconds());
  if (recentRtts.size() > rttQueueSize){
    recentRtts.pop();
  }
}

void
TcpTimeStepGymEnv::CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this);
  std::string stateName = GetTcpCongStateName(newState);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CongestionStateSet: " << newState << " " << stateName);
  m_tcb = tcb;
}

void
TcpTimeStepGymEnv::CwndEvent (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
  NS_LOG_FUNCTION (this);
  std::string eventName = GetTcpCAEventName(event);
  NS_LOG_INFO(Simulator::Now() << " Node: " << m_nodeId << " CwndEvent: " << event << " " << eventName);
  m_tcb = tcb;
}

/*
Define action space
*/
Ptr<OpenGymSpace>
TcpTimeStepGymEnv::GetActionSpace()
{
  // new_ssThresh
  // new_cWnd
  uint32_t parameterNum = 2;
  float low = m_tcb->m_segmentSize;
  float high = 65535;
  std::vector<uint32_t> shape = {parameterNum,};
  //std::string dtype = TypeNameGet<uint32_t> ();
  std::string dtype = TypeNameGet<float> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetActionSpace: " << box);
  return box;
}

} // namespace ns3
