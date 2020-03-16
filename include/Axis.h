/****************************************************************************
 * $Source: $
 * $Revision: 7 $ $State: $ $Date: 6/02/16 5:32p $
 *
 * !NAME!
 *		Axis
 *
 * DESCRIPTION:
 *		This class provides an interface to the ClearPath SC node
 *		in our example program
 *
 * PRINCIPLE AUTHORS:
 *		JS
 *
 * CREATION DATE:
 *		03/30/2016
 *
 * !end!
 **
 ****************************************************************************/

#ifndef __AXIS_H__
#define __AXIS_H__

/****************************************************************************
 * !NAME!
 ** Axis.h headers !0! */
#include "pubSysCls.h"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
using namespace sFnd;
using namespace std;
/* 																	   !end!*
 ****************************************************************************/

/****************************************************************************
 * !NAME!
 * * Axis.h constants !0!	*/
#define RAS_CODE 5
#define G_STOP 2
#define Attack 100
#define TRQ_PCT 85
#define ACC_LIM_RPM_PER_SEC 10000
#define VEL_LIM_RPM 2500
#define TRACKING_LIM (m_node->Info.PositioningResolution.Value() / 4)
#define TIME_TILL_TIMEOUT 10000
#define leadscrew_pitch 5 // mm
/* !end!
 **
 ****************************************************************************/

// This class needs to have a reference to its Supervisor
class Supervisor;

//******************************************************************************
// NAME
// * 	Axis class
//
// DESCRIPTION
//	This is the interface to the ClearPath SC. It includes a thread
//	which runs the state machine for the node.
//
class Axis {
private:
  Supervisor *m_super; // The supervisor
  INode *m_node;       // The ClearPath-SC for this axis
  thread m_thread;     // Handle to this Axis's thread

  double lead_pitch = leadscrew_pitch;
  int empty_buffer_space;

  // Filename for the config file stored at the start
#define MAX_CONFIG_PATH_LENGTH 256
#define MAX_CONFIG_FILENAME_LENGTH 384
  char m_configFile[MAX_CONFIG_FILENAME_LENGTH];

  // State machine information
  enum StateEnum {
    STATE_IDLE,
    STATE_WAIT_FOR_KICK,
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

  bool m_quitting; // Axis quitting

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
  void AxisMain(Supervisor *theSuper);

public:
  // Constructor/Destructor
  Axis(INode *node);
  ~Axis();

  // Return a reference to our node
  INode *MyNode() { return (m_node); };

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
  void CreateThread(Supervisor *theSuper) {
    m_thread = thread(&Axis::AxisMain, this, theSuper);
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
};
//																			   *
//******************************************************************************

#endif