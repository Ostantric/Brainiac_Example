
#include "Brainiac.h"
#include "Logging.h"

Brainiac::Brainiac(vector<Servo *> theServos, SysManager &myMgr)
    : m_sysMgr(myMgr) {
  // mutex1=std::ref(main_mutex);
  m_servos = theServos;
  m_state = BRAINIAC_IDLE;
  m_quitting = false;
  servos_idle = false;
  m_servoIdle.fill(true);
  m_servoReady.fill(true);
  m_servoCommandDone.fill(true);
  m_servoCommandSent.fill(true);
  // create list of flags to keep track of the nodes for
  // various conditions
  for (Uint16 i = 0; i < m_servos.size(); i++) {
    // servor[i]=false;
    // m_servoIdle.push_back(false);
    // m_servoReady.push_back(false);
    // m_servoCommanded.push_back(false);
    // m_servoCommandDone.push_back(false);
    m_servoIdle.at(i) = false;
    m_servoReady.at(i) = false;
    m_servoCommandSent.at(i) = false;
    m_servoCommandDone.at(i) = false;
    // m_servoIdle[i]=false;
    // m_servoReady[i]=false;
    // m_servoCommanded[i]=false;
    // m_servoCommandDone[i]=false;

    m_servos.at(i)->Addr = m_servos.at(i)->MyNode()->Info.Ex.Addr();
    // cout << m_servos.at(i)->UserID << endl;
    if (m_servos.at(i)->UserID.compare("Y1") == 0) {
      Y_1 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("Y2") == 0) {
      Y_2 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("X1") == 0) {
      X_1 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("Z1") == 0) {
      Z_1 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("Z2") == 0) {
      Z_2 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("Z3") == 0) {
      Z_3 = m_servos.at(i);
    } else if (m_servos.at(i)->UserID.compare("E1") == 0) {
      E_1 = m_servos.at(i);
    } else {
      printf("unmatched servo name\n");
    }
  }
}

void Brainiac::SignalReset() { ResetCondition(m_commandReady); }

void Brainiac::SignalIdle(Uint16 nodeNum) {
  SignalPerNode(m_servoIdle, m_nodesIdle, nodeNum);
}

void Brainiac::SignalReady(Uint16 nodeNum) {
  SignalPerNode(m_servoReady, m_nodesReady, nodeNum);
}

void Brainiac::WaitForCommand(CommandType &command_type) {
  WaitForCondition(m_commandReady);
  command_type = this->command_type;
}

void Brainiac::WaitForDone() { WaitForCondition(m_nodesDone); }

void Brainiac::WaitForIdle() { WaitForCondition(m_nodesIdle); }

bool Brainiac::ReturnDoneCondition() { return ReturnCondition(m_nodesDone); }

void Brainiac::SignalReadyCommand() { SetCondition(m_commandReady); }

void Brainiac::SignalCommandSent(Uint16 nodeNum) {
  SignalPerNode(m_servoCommandSent, m_nodesCommandSent, nodeNum);
}

void Brainiac::SignalCommandDone(Uint16 nodeNum) {
  SignalPerNode(m_servoCommandDone, m_nodesDone, nodeNum);
}

// The Brainiac's state machine thread. Creates threads for each servo
void Brainiac::BrainiacMain() {
  // Get all conditions to a known state
  ResetCondition(m_servoIdle, m_nodesIdle);
  ResetCondition(m_servoCommandSent, m_nodesCommandSent);
  ResetCondition(m_servoCommandDone, m_nodesDone);
  ResetCondition(m_commandReady);
  // ResetCondition(m_nodesCommanded);
  command_done = false;
  command_type = Idle;
  command_requested = false;
  servos_idle = false;

  try {
    // Create threads for all the axes
    for (Uint16 i = 0; i < m_servos.size(); i++) {
      // printf(" axis[%d]: userID=%s\n", iAxis,
      // theAxes.at(iAxis)->MyNode()->Info.UserID.Value());
      m_servos.at(i)->CreateThread(this);
    }
  } catch (...) {
    printf("Supervisor failed initializing threads\n");
    Quit();
  }
  // for (Uint16 i = 0; i < m_servos.size(); i++) {
  //
  // }

  // Get the state machine going
  while (!m_quitting) {
    try {
      // For debugging
      m_lastState = m_state;
      switch (m_state) {
      case BRAINIAC_IDLE:
        // Reset each node to idle
        for (Uint16 iAxis = 0; iAxis < m_servos.size(); iAxis++) {
          m_servos.at(iAxis)->ResetToIdle();
          // cout << "reset_servo" << endl;
        }
        m_state = BRAINIAC_WAIT_FOR_ALL;
        break;
      case BRAINIAC_WAIT_FOR_ALL:

        if (m_quitting)
          continue;
        LOG_BRAINIAC("Waiting Servos to Be IDLE");
        // WaitForCondition(m_nodesIdle);
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_cond.wait(lock_m, [&] { return m_nodesIdle; });
        }
        if (m_quitting)
          continue;
        ResetCondition(m_servoIdle, m_nodesIdle);
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          servos_idle = true;
          main_cond->notify_all();
        }
        m_state = BRAINIAC_WAIT_FOR_COMMAND;
        break;

      case BRAINIAC_WAIT_FOR_COMMAND:
        LOG_BRAINIAC("Waiting for a command");
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          main_cond->wait(lock_m, [&] { return command_requested; });
          command_requested = false;
        }
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          servos_idle = false;
        }
        LOG_BRAINIAC("Got a command");
        switch (HOME_RQ) {
        case 0x00: { // XYZE_RQ states
          command_type = Move;
          switch (XYZE_RQ) {
          case 0x01: // x    0001
            X_1->MyNode()->Motion.Adv.TriggerGroup(1);
            X_1->KickServo();
            SetCondition(m_commandReady);
            LOG_BRAINIAC("Waiting X1 MoveCommand to be Sent");
            {
              std::unique_lock<std::mutex> lock_m(m_mutex);
              m_cond.wait(lock_m, [&] { return m_servoCommandSent.at(0); });
            }
            {
              std::unique_lock<std::mutex> lock_m(m_mutex);
              m_servoCommandSent.at(0) = false;
            }
            m_state = BRAINIAC_WAIT_FOR_X_MOVE_SENT;
            break;
          case 0x02: // y    0010
            Y_1->MyNode()->Motion.Adv.TriggerGroup(2);
            Y_2->MyNode()->Motion.Adv.TriggerGroup(2);
            Y_1->KickServo();
            Y_2->KickServo();
            SetCondition(m_commandReady);
            LOG_BRAINIAC("Waiting Y1&Y2 MoveCommand to be Sent");
            {
              std::unique_lock<std::mutex> lock_m(m_mutex);
              m_cond.wait(lock_m, [&] {
                return (m_servoCommandSent.at(0) && m_servoCommandSent.at(1));
              });
            }
            {
              std::unique_lock<std::mutex> lock_m(m_mutex);
              m_servoCommandSent.at(0) = false;
              m_servoCommandSent.at(1) = false;
            }

            m_state = BRAINIAC_WAIT_FOR_Y_MOVE_SENT;
            break;
          case 0x03: // xy   0011
            SetTriggerGroup(X_1, Y_1);
            m_state = BRAINIAC_WAIT_FOR_XY_MOVE_SENT;
            break;
          case 0x04: // z    0100
            SetTriggerGroup(Z_1, Z_2, Z_3);
            m_state = BRAINIAC_WAIT_FOR_Z_MOVE_SENT;
            break;
          case 0x05: // xz   0101
            SetTriggerGroup(X_1, Z_1, Z_2, Z_3);
            m_state = BRAINIAC_WAIT_FOR_XZ_MOVE_SENT;
            break;
          case 0x06: // yz   0110
            SetTriggerGroup(Y_1, Y_2, Z_1, Z_2, Z_3);
            m_state = BRAINIAC_WAIT_FOR_YZ_MOVE_SENT;
            break;
          case 0x07: // xyz  0111
            SetTriggerGroup(X_1, Y_1, Y_2, Z_1, Z_2, Z_3);
            m_state = BRAINIAC_WAIT_FOR_XYZ_MOVE_SENT;
            break;
          case 0x08: // e    1000
            SetTriggerGroup(E_1);
            m_state = BRAINIAC_WAIT_FOR_E_MOVE_SENT;
            break;
          case 0x09: // xe   1001
            SetTriggerGroup(X_1, E_1);
            m_state = BRAINIAC_WAIT_FOR_XE_MOVE_SENT;
            break;
          case 0x0a: // ye   1010
            SetTriggerGroup(Y_1, Y_2, E_1);
            m_state = BRAINIAC_WAIT_FOR_YE_MOVE_SENT;
            break;
          case 0x0b: // xye  1011
            SetTriggerGroup(X_1, Y_1, Y_2, E_1);
            m_state = BRAINIAC_WAIT_FOR_XYE_MOVE_SENT;
            break;
          case 0x0c: // ze   1100
            SetTriggerGroup(Z_1, Z_2, Z_3, E_1);
            m_state = BRAINIAC_WAIT_FOR_ZE_MOVE_SENT;
            break;
          case 0x0d: // xze  1101
            SetTriggerGroup(X_1, Z_1, Z_2, Z_3, E_1);
            m_state = BRAINIAC_WAIT_FOR_XZE_MOVE_SENT;
            break;
          case 0x0e: // yze  1110
            SetTriggerGroup(Y_1, Y_2, Z_1, Z_2, Z_3, E_1);
            m_state = BRAINIAC_WAIT_FOR_YZE_MOVE_SENT;
            break;
          case 0x0f: // xyze 1111
            SetTriggerGroup(X_1, Y_1, Y_2, Z_1, Z_2, Z_3, E_1);
            m_state = BRAINIAC_WAIT_FOR_XYZE_MOVE_SENT;
            break;
          default:
            command_type = Idle;
            m_state = BRAINIAC_IDLE;
            break;
          }
        } break;
        case 0x01: { // HOME X
          command_type = Home;
          X_1->KickServo();
          SetCondition(m_commandReady);
          {
            std::unique_lock<std::mutex> lock_m(m_mutex);
            m_cond.wait(lock_m, [&] { return m_servoCommandSent.at(0); });
          }
          {
            std::unique_lock<std::mutex> lock_m(m_mutex);
            m_servoCommandSent.at(0) = false;
          }

          // WaitForFlag(m_servoCommandSent, 1, X_1);
          // ResetCondition(m_servoCommandSent, m_nodesCommandSent);
          X_1->Home();
          m_state = BRAINIAC_WAIT_FOR_HOMING_X;
        } break;
        case 0x02: { // HOME Y
          command_type = Home;
          Y_1->KickServo();
          Y_2->KickServo();
          LOG_BRAINIAC("HOME Y1&Y2 command");
          SetCondition(m_commandReady);
          {
            std::unique_lock<std::mutex> lock_m(m_mutex);
            m_cond.wait(lock_m, [&] {
              return (m_servoCommandSent.at(0) && m_servoCommandSent.at(1));
            });
          }
          {
            std::unique_lock<std::mutex> lock_m(m_mutex);
            m_servoCommandSent.at(0) = false;
            m_servoCommandSent.at(1) = false;
          }
          Y_1->Home();
          Y_2->Home();
          m_state = BRAINIAC_WAIT_FOR_HOMING_Y;
        } break;
        case 0x03: { // HOME XY
          command_type = Home;
          m_state = BRAINIAC_WAIT_FOR_HOMING_XY;
        } break;
        case 0x04: { // HOME Z
          command_type = Home;
          m_state = BRAINIAC_WAIT_FOR_HOMING_Z;
        } break;
        case 0x05: { // HOME XZ
          command_type = Home;
          m_state = BRAINIAC_WAIT_FOR_HOMING_XZ;
        } break;
        case 0x06: { // HOME YZ
          command_type = Home;
          m_state = BRAINIAC_WAIT_FOR_HOMING_YZ;
        } break;
        case 0x07: { // HOME XYZ
          command_type = Home;
          m_state = BRAINIAC_WAIT_FOR_HOMING_XYZ;
        } break;
        default: {
          command_type = Idle;
          m_state = BRAINIAC_IDLE;
        } break;
        }

        break;

      case BRAINIAC_WAIT_FOR_HOMING_X:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("Waiting X1 to be homed");
        // WaitForFlag(m_servoCommandDone, 1, X_1);
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_cond.wait(lock_m, [&] { return m_servoCommandDone.at(0); });
        }
        LOG_BRAINIAC("Done with the X1 HomeCommand");
        // ResetCondition(m_servoCommandDone, m_nodesDone);
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_servoCommandDone.at(0) = false;
        }
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          command_done = true;
          main_cond->notify_all();
        }
        // WaitForFlag(m_servoCommandDone.at(X_1->Addr));
        m_state = BRAINIAC_WAIT_FOR_ALL;
        break;
      case BRAINIAC_WAIT_FOR_HOMING_Y:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("Waiting Y1&Y2 to be homed");
        // WaitForFlag(m_servoCommandDone, 1, X_1);
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_cond.wait(lock_m, [&] {
            return (m_servoCommandDone.at(0) && m_servoCommandDone.at(1));
          });
        }
        LOG_BRAINIAC("Done with the Y1&Y2 HomeCommand");
        // ResetCondition(m_servoCommandDone, m_nodesDone);
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_servoCommandDone.at(0) = false;
          m_servoCommandDone.at(1) = false;
        }
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          command_done = true;
          main_cond->notify_all();
        }
        // WaitForFlag(m_servoCommandDone.at(X_1->Addr));
        m_state = BRAINIAC_WAIT_FOR_ALL;
        break;
      case BRAINIAC_WAIT_FOR_X_MOVE_SENT:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("X-MoveCommand Sent,Ready for trigger");
        X_1->MyNode()->Port.Adv.TriggerMovesInGroup(1);
        m_state = BRAINIAC_WAIT_FOR_X_MOVE_DONE;
        break;
      case BRAINIAC_WAIT_FOR_Y_MOVE_SENT:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("Y-MoveCommand Sent,Ready for trigger");
        Y_1->MyNode()->Port.Adv.TriggerMovesInGroup(2);
        Y_2->MyNode()->Port.Adv.TriggerMovesInGroup(2);
        m_state = BRAINIAC_WAIT_FOR_Y_MOVE_DONE;
        break;
      case BRAINIAC_WAIT_FOR_X_MOVE_DONE:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("Waiting X1 MoveCommand to be Done");
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_cond.wait(lock_m, [&] { return m_servoCommandDone.at(0); });
        }
        LOG_BRAINIAC("Done with the X1 MoveCommand");
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_servoCommandDone.at(0) = false;
        }
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          command_done = true;
          main_cond->notify_all();
        }
        // WaitForFlag(m_servoCommandDone.at(X_1->Addr));
        m_state = BRAINIAC_WAIT_FOR_ALL;
        break;
      case BRAINIAC_WAIT_FOR_Y_MOVE_DONE:
        if (m_quitting)
          continue;
        LOG_BRAINIAC("Waiting Y1&Y2 MoveCommand to be Done");
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_cond.wait(lock_m, [&] { return (m_servoCommandDone.at(0) && m_servoCommandDone.at(1)); });
        }
        LOG_BRAINIAC("Done with the Y1&Y2 MoveCommand");
        {
          std::unique_lock<std::mutex> lock_m(m_mutex);
          m_servoCommandDone.at(0) = false;
          m_servoCommandDone.at(1) = false;
        }
        {
          std::unique_lock<std::mutex> lock_m(*mutex_ptr);
          command_done = true;
          main_cond->notify_all();
        }
        m_state = BRAINIAC_WAIT_FOR_ALL;
        break;

      default:

        break;
      } // Try block
    } catch (mnErr &err) {
      fprintf(stderr, "Supervisor thread failed: %s\n", err.ErrorMsg);
      Quit();
    } catch (...) {
      fprintf(stderr, "Supervisor thread failed.\n");
      Quit();
    }
  }
  // cout << "here" << endl;
  m_sysMgr.Delay(0);

  // SetCondition(m_nodesCommandSent);
  WaitForCondition(m_nodesIdle);

  m_state = BRAINIAC_EXITED;
  // Wait for threads to end
  for (Uint16 iAxis = 0; iAxis < m_servos.size(); iAxis++) {
    m_servos.at(iAxis)->Join();
  }
  return;
}
//																			   *
//******************************************************************************
