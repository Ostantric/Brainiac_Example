#include "Servo.h"
#include "pubSysCls.h"
#include "stdarg.h"
#include <chrono>
#include <fstream>
//	synchronization of servos

class Brainiac {
private:
  thread m_thread;      // main's thread
  SysManager &m_sysMgr; // sFoundation manager

  vector<Servo *> m_servos; // the list of nodes
  Servo *X_1, *Y_1, *Y_2, *Z_1, *Z_2, *Z_3, *E_1;

  bool m_quitting; // are we quitting?

  // synchronization variables
  mutex m_mutex;

  condition_variable m_cond;

  // Flags to indicate that all nodes have reported back
  // for various conditions
  bool m_nodesIdle, m_nodesDone, m_nodesCommandSent, m_nodesReady, m_commandReady;
  // Flags to keep track of which nodes have reported back
  // for various conditions
  // vector<bool> m_servoReady;
  // vector<bool> m_servoIdle;
  // vector<bool> m_servoCommanded;
  // vector<bool> m_servoCommandDone;
  // bool m_servoReady[32]{true};
  // bool m_servoIdle[32]{true};
  // bool m_servoCommanded[32]{true};
  // bool m_servoCommandDone[32]{true};
  array<bool, 32> m_servoReady;
  array<bool, 32> m_servoIdle;
  array<bool, 32> m_servoCommandSent;
  array<bool, 32> m_servoCommandDone;

  // State machine information
  enum BrainiacStateEnum {
    BRAINIAC_IDLE,
    BRAINIAC_WAIT_FOR_ALL,
    BRAINIAC_WAIT_FOR_COMMAND,
    BRAINIAC_WAIT_FOR_X_MOVE_SENT,
    BRAINIAC_WAIT_FOR_Y_MOVE_SENT,
    BRAINIAC_WAIT_FOR_Z_MOVE_SENT,
    BRAINIAC_WAIT_FOR_E_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XY_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XZ_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_YZ_MOVE_SENT,
    BRAINIAC_WAIT_FOR_YE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_ZE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XYZ_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XYE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XZE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_YZE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_XYZE_MOVE_SENT,
    BRAINIAC_WAIT_FOR_HOMING_X,
    BRAINIAC_WAIT_FOR_HOMING_Y,
    BRAINIAC_WAIT_FOR_HOMING_Z,
    BRAINIAC_WAIT_FOR_HOMING_XY,
    BRAINIAC_WAIT_FOR_HOMING_XZ,
    BRAINIAC_WAIT_FOR_HOMING_YZ,
    BRAINIAC_WAIT_FOR_HOMING_XYZ,
    BRAINIAC_WAIT_FOR_X_MOVE_DONE,
    BRAINIAC_WAIT_FOR_Y_MOVE_DONE,

    BRAINIAC_EXITED
  };

  BrainiacStateEnum m_state;
  BrainiacStateEnum m_lastState;

public:
  // Constructor
  mutex *mutex_ptr = nullptr;
  condition_variable *main_cond = nullptr;
  bool command_done, servos_idle, command_requested;
  uint8_t XYZE_RQ, HOME_RQ;
  enum CommandType { Idle, Home, Move };

  CommandType command_type;

  Brainiac(vector<Servo *> servos, SysManager &myMgr);

  void CreateThread() { m_thread = thread(&Brainiac::BrainiacMain, this); }

  // Print stats for all the nodes
  void PrintStats() {
    printf("\nStats:\t\tnumMoves\t\tPosition\t\tPosError\n");
    for (Uint16 iAxis = 0; iAxis < m_servos.size(); iAxis++) {
      m_servos.at(iAxis)->PrintStats();
    }
  }

  // Signals and waits by the nodes
  void
  SignalIdle(Uint16 nodeNum); // Nodes tell the supervisor when they are idle
  void WaitForCommand(CommandType &command); // Nodes need to wait for the
                                             // supervisor to kick them
  void WaitForDone();
  void WaitForIdle();
  bool ReturnDoneCondition();
  void SignalReadyCommand();
  void SignalCommandSent(
      Uint16 nodeNum); // Nodes tell the supervisor when they've sent their move
  void SignalCommandDone(
      Uint16 nodeNum); // Nodes tell the supervisor whne their move is done
  void SignalReady(Uint16 nodeNum); // Nodes signal when they are initialized
  void SignalReset();

  void BrainiacMain(); // the work for this thread

  // sFoundation supervisor for access to foundation support
  SysManager &Mgr() { return m_sysMgr; }

  // Quit and tell the nodes to quit, too
  void Quit() {
    printf("  Supervisor quitting\n");
    m_quitting = true;
    for (Uint16 iAxis = 0; iAxis < m_servos.size(); iAxis++)
      m_servos.at(iAxis)->Quit();
  }

  // Wait for the thread to finish
  void Terminate() { m_thread.join(); }

private:
  // Mark that a particular node has signalled back for the given condition

  void SignalPerNode(array<bool, 32> &nodeFlags, bool &condition,
                     Uint16 nodeNum) {
    unique_lock<mutex> lock(m_mutex);
    nodeFlags.at(nodeNum) = true;
    // nodeFlags[nodeNum] = true;
    bool gotAll = true;
    for (Uint16 i = 0; i < nodeFlags.size(); i++)
      gotAll = gotAll && nodeFlags.at(i);
    if (gotAll) {
      condition = true;
      m_cond.notify_all();
    }
  }

  // Wait for a given condition
  void WaitForCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    while (!condition && !m_quitting)
      m_cond.wait(lock);
  }

  void WaitForFlag(array<bool, 32> &flag, int cnt, ...) {
    //using reference_to_bit = std::vector<bool>::reference;
    unique_lock<mutex> lock(m_mutex);
    /*bool *temp_bool_ptr = new bool(cnt);
    bool *final = new bool(true);
    Servo *temp_servo;
    va_list marker;
    va_start(marker, cnt);
    for (int index = 0; index < cnt; index++) {
      temp_servo = va_arg(marker, Servo *);
      temp_bool_ptr[index] = flag.at(index);
      *final = temp_bool_ptr[index] && &final;
    }
    va_end(marker);*/
    //printf("waiting_for_bit\n");
    m_cond.wait(lock, [&] { return flag.at(0); });
    //delete[] temp_bool_ptr;
    //delete final;
  }

  // Clear the indicators for a particular condition
  void ResetCondition(array<bool, 32> nodeFlags, bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    for (Uint16 i = 0; i < sizeof(nodeFlags); i++)
      nodeFlags.at(i) = false;
    condition = false;
  }

  // Set a condition that doesn't rely on all the nodes to do something
  void SetCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    condition = true;
    m_cond.notify_all();
  }

  // Clear a condition that doesn't relay on all the nodes
  void ResetCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    condition = false;
  }

  bool ReturnCondition(bool &condition) {
    unique_lock<mutex> lock(m_mutex);
    return condition;
  }

  void SetTriggerGroup(Servo *cnt, ...) {
    // servo->MyNode()->Motion.Adv.TriggerGroup(1);
    va_list marker;
    va_start(marker, cnt);
    // for (int index = 0; index < cnt; index++) {
    //  Servo* servo_ptr = va_arg(marker, Servo);
    //}
    while (cnt != 0) {
      cnt->MyNode()->Motion.Adv.TriggerGroup(10);
      cnt = va_arg(marker, Servo *);
    }
    va_end(marker);
  }
};
