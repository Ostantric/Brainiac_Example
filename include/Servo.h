#ifndef __SERVO_H__
#define __SERVO_H__

#include "pubSysCls.h"
#include <condition_variable>
#include <iostream>
#include <sstream> 
#include <mutex>
#include <thread>
using namespace sFnd;
using namespace std;

#define MAX_CONFIG_PATH_LENGTH 256
#define MAX_CONFIG_FILENAME_LENGTH 384

#define RAS_CODE 5
#define G_STOP 2
#define Attack 100
#define TRQ_PCT 85
#define ACC_LIM_RPM_PER_SEC 2500
#define VEL_LIM_RPM 2500
#define TRACKING_LIM (m_node->Info.PositioningResolution.Value() / 4)
#define TIME_TILL_TIMEOUT 30000
#define LEADSCREW_PITCH 5 // mm

// Servo class needs to have a reference to its Brainiac
class Brainiac;

// servo state machine interface
class Servo {
private:
  Brainiac *m_brainiac; // The Brainiac
  INode *m_node;        // The ClearPath-SC for this axis
  thread m_thread;      // Handle to this Servo thread
  bool kick;
  double lead_pitch = LEADSCREW_PITCH;
  int empty_buffer_space;

  // Filename for the config file stored at the start

  char m_configFile[MAX_CONFIG_FILENAME_LENGTH];

  // State machine information
  enum StateEnum {
    STATE_IDLE,
    STATE_WAIT_FOR_COMMAND,
    STATE_SEND_HOME,
    STATE_SEND_MOVE,
    STATE_SIGNAL_HOME_SENT,
    STATE_SIGNAL_MOVE_SENT,
    STATE_WAIT_FOR_MOVE_DONE,
    STATE_WAIT_FOR_HOME_DONE,
    STATE_EXITED
  };
  StateEnum m_state;
  StateEnum m_lastState;

  // Move information
  mgMoveProfiledInfo m_move;
  int m_moveVel;
  ::uint32_t m_moveCount;
  ::int32_t m_moveTimeoutMs;
  ::int32_t m_homingTimeoutMs;

  bool m_quitting; // Servo quitting

  // Synchronization
  condition_variable m_cond;
  mutex m_mutex;

  // Enable the node and get it ready to go
  void Enable();

  // Initialize accLim, velLim, etc
  void InitMotionParams();

  // Initiate a move

  // Wait for attention that move has completed
  bool WaitForHome(::int32_t timeoutMs);

  // Wait for attention that move has completed
  bool WaitForMove(::int32_t timeoutMs);

  // The state machine
  void ServoMain(Brainiac *theBrainiac);

public:
  // Constructor/Destructor
  Servo(INode *node);
  ~Servo();
  string UserID;
  Uint16 Addr;
  // Return a reference to our node
  INode *MyNode() { return (m_node); };

  void KickServo();

  // Print the current stats (number of moves performed and
  // current measured position)
  void PrintStats(bool onInit = false) {

    m_node->Motion.PosnMeasured.Refresh();
    double i = m_node->Motion.PosnMeasured.Value();
    if (onInit) {
      printf("  [%2d]:\t\t**at startup**\t\t%8.0f\n", m_node->Info.Ex.Addr(),
             m_node->Motion.PosnMeasured.Value());
    } else {
      printf("  [%2d]:\t\t%8d\t\t%8.0f\t\t%8.0d\n", m_node->Info.Ex.Addr(),
             m_moveCount, i, ((int)m_move.value - (int)i));
    }
  }
  int Move();
  // Set the move distance to the given number of revolutions
  void SetMove(double mm, double feedrate) {
    m_moveVel = feedrate / lead_pitch;
    m_move.value = (long)(mm * (6400 / lead_pitch));
  }

  void Home();

  // Create the thread and get it going
  void CreateThread(Brainiac *theBrainiac) {
    m_thread = thread(&Servo::ServoMain, this, theBrainiac);
  };

  // Reset sequencer to "idle" state
  void ResetToIdle() {
    if (m_state != STATE_IDLE)
      m_state = m_lastState = STATE_IDLE;
  };

  // Time to quit
  void Quit() { m_quitting = true; }

  // Park here to wait for the thread to exit
  void Join() { m_thread.join(); }

  void WaitForCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    while (!condition && !m_quitting)
      m_cond.wait(lock);
  }
  // Clear a condition that doesn't relay on all the nodes
  void ResetCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    condition = false;
  }
  // Set a condition that doesn't rely on all the nodes to do something
  void SetCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    condition = true;
    //printf("kick_notf\n");
    m_cond.notify_all();
  }

  
};
//																			   *
//******************************************************************************

#endif