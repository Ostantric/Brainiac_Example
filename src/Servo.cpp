#include "Servo.h"
#include "Brainiac.h"
#include "Logging.h"
#include <stdio.h>
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#endif

Servo::Servo(INode *node) : m_node(node), m_moveCount(0), m_quitting(false) {

  // Save the config file before starting
  // This would typically be performed right after tuning each axis.
  // Chosing a file name that contains the address and serial number
  // can ensure that the system does not get miswired.
  // Using the UserID as a filename key can allow swapping motors with
  // minimal setup required. This requires that no invalid characters
  // are present in the UserID.
  char tempPath[MAX_CONFIG_PATH_LENGTH];
#if defined(_WIN32) || defined(_WIN64)
  GetTempPathA(MAX_CONFIG_PATH_LENGTH, tempPath);
#else
  strncpy(tempPath, "/tmp", sizeof(tempPath));
#endif
  snprintf(m_configFile, sizeof(m_configFile), "%s/config-%02d-%d.mtr",
           tempPath, m_node->Info.Ex.Addr(), m_node->Info.SerialNumber.Value());
  // sprintf(m_configFile, "%s\config-%s.mtr", tempPath,
  // m_node->Info.UserID.Value());

  if (m_node->Setup.AccessLevelIsFull())
    m_node->Setup.ConfigSave(m_configFile);
  this->UserID = m_node->Info.UserID.Value();
};

Servo::~Servo() {
  // Print out the move statistics one last time
  PrintStats();

  // If we don't have full access, there's nothing to do here
  if (!m_node->Setup.AccessLevelIsFull()) {
    return;
  }

  // Disable the node and restore the config file
  attnReg attnReq;
  attnReq.cpm.Disabled = 1;
  m_node->Adv.Attn.ClearAttn(attnReq);
  m_node->EnableReq(false);

  m_node->Status.RT.Refresh();
  if (m_node->Status.RT.Value().cpm.Enabled) {
    attnReg theAttn = m_node->Adv.Attn.WaitForAttn(attnReq, 3000);
    if (theAttn.cpm.Enabled) {
      printf("Error: Timed out waiting for disable\n");
      return;
    }
  }
  // Restore the original config file
  // This would typically be performed when setting up the machine.
  m_node->Setup.ConfigLoad(m_configFile);
}

void Servo::KickServo() { SetCondition(kick); }
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME
//* 		Axis::Enable
//
//	DESCRIPTION:
//		Clear alerts and get the node ready to roll
//
void Servo::Enable() {
  if (m_node != NULL) {
    // Clear alerts and node stops
    m_node->Status.AlertsClear();
    m_node->Motion.NodeStop(STOP_TYPE_ABRUPT);
    m_node->Motion.NodeStopClear();

    // Wait for node to be ready to take commands
    attnReg attnReq;
    attnReq.cpm.Ready = 1;
    m_node->Adv.Attn.ClearAttn(attnReq);

    m_node->EnableReq(true);

    // If the node is not currently ready, wait for it to get there
    if (!m_node->Status.IsReady()) {
      attnReg theAttn = m_node->Adv.Attn.WaitForAttn(attnReq, 3000);
      if (!theAttn.cpm.Ready) {
        mnErr eInfo;
        eInfo.ErrorCode = MN_ERR_TIMEOUT;
        eInfo.TheAddr = m_node->Info.Ex.Addr();
        snprintf(eInfo.ErrorMsg, sizeof(eInfo.ErrorMsg),
                 "Error: Timed out waiting for enable\n");
        throw eInfo;
      }
    }
    attnReq.cpm.Ready = 0;
    attnReq.cpm.WasHomed = 1;

    m_node->Adv.Attn.ClearAttn(attnReq);
    /*
    // If node is set up to home, start homing
    if (m_node->Motion.Homing.HomingValid()) {
            m_node->Motion.Homing.Initiate();

                    attnReg theAttn = m_node->Adv.Attn.WaitForAttn(attnReq,
    30000); if (!theAttn.cpm.WasHomed) { mnErr eInfo; eInfo.ErrorCode =
    MN_ERR_TIMEOUT; eInfo.TheAddr = m_node->Info.Ex.Addr();
                            snprintf(eInfo.ErrorMsg, sizeof(eInfo.ErrorMsg),
                                    "Error: Timed out waiting for home\n");
                            throw eInfo;
                    }
    }*/
  }
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME
//* 		Axis::InitMotionParams
//
//	DESCRIPTION:
//		Initialize accLim, velLim, etc
//
void Servo::InitMotionParams() {
  // Set the user units to RPM and RPM/s
  m_node->VelUnit(INode::RPM);
  m_node->AccUnit(INode::RPM_PER_SEC);
  m_node->Motion.VelLimit = VEL_LIM_RPM;
  m_node->Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
  //m_node->Motion.JrkLimit = RAS_CODE;
  // m_node->Limits.PosnTrackingLimit.Value(TRACKING_LIM);
  m_node->Limits.TrqGlobal.Value(TRQ_PCT);
  // Set up the move itself
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME
//* 		Axis::Move
//
//	DESCRIPTION:
//		Issue the move
//

int Servo::Move() {
  // Clear out any lingering move done attentions
  attnReg attnMask;
  attnMask.cpm.MoveDone = 1;
  attnMask.cpm.Disabled = 1;
  attnMask.cpm.NotReady = 1;
  m_node->Adv.Attn.ClearAttn(attnMask);

  // Uncomment the following line to enable breakpoint debugging. The lock
  // allows the application to break without having timeout errors be thrown.
  // WARNING: While stepping through, leaving the scope this lock is
  // declared in (ie this function) will disengage the lock,
  // and may result in timeouts.
  // SysManager::ThreadLock lock;

  if (m_moveVel > 3800) {
    m_moveVel = 3800;
  }
  m_node->Motion.VelLimit = m_moveVel;
  return m_node->Motion.Adv.MovePosnStart(m_move.value, true, true, false);
}

void Servo::Home() {
  // Clear out any lingering move done attentions
  attnReg attnMask;
  attnMask.cpm.WasHomed = 1;
  attnMask.cpm.Disabled = 1;
  attnMask.cpm.NotReady = 1;
  m_node->Adv.Attn.ClearAttn(attnMask);

  m_node->Motion.Homing.Initiate();
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME
//* 		Axis::WaitForMove
//
//	DESCRIPTION:
//		Wait for attention that move has completed
//
bool Servo::WaitForMove(::int32_t timeoutMs) {
  if (m_node == NULL) {
    return false;
  }
  attnReg attnMask;
  attnReg attnSeen;
  attnMask.cpm.MoveDone = 1;
  attnMask.cpm.Disabled = 1;
  attnMask.cpm.NotReady = 1;
  attnSeen = m_node->Adv.Attn.WaitForAttn(attnMask, timeoutMs, false);
  // printf("  End move axis %d: %f\n", m_node->Info.Ex.Addr(), infcCoreTime());
  if (attnSeen.cpm.Disabled) {
    printf("  ERROR: [%d] disabled during move\n", m_node->Info.Ex.Addr());
    return false;
  }
  if (attnSeen.cpm.NotReady) {
    printf("  ERROR: [%d] went NotReady during move\n", m_node->Info.Ex.Addr());
    return false;
  }
  return attnSeen.cpm.MoveDone;
};

bool Servo::WaitForHome(::int32_t timeoutMs) {
  if (m_node == NULL) {
    return false;
  }
  attnReg attnMask;
  attnReg attnSeen;
  attnMask.cpm.WasHomed = 1;
  // attnMask.cpm.Homing = 1;
  attnSeen = m_node->Adv.Attn.WaitForAttn(attnMask, timeoutMs, false);
  // printf("  End move axis %d: %f\n", m_node->Info.Ex.Addr(), infcCoreTime());
  if (attnSeen.cpm.Disabled) {
    printf("  ERROR: [%d] disabled during move\n", m_node->Info.Ex.Addr());
    return false;
  }
  if (attnSeen.cpm.NotReady) {
    printf("  ERROR: [%d] went NotReady during move\n", m_node->Info.Ex.Addr());
    return false;
  }
  return attnSeen.cpm.WasHomed;
};
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME
//* 		Axis::AxisMain
//
//	DESCRIPTION:
//		The main work of the axis class - initialize the node and run
//		the state machine
//
void Servo::ServoMain(Brainiac *theBrainiac) {
  bool moveDone;
  bool homeDone;
  string output_log;
  ostringstream output_stream;
  Brainiac::CommandType command;

  m_brainiac = theBrainiac;

  // If we don't have nodes, exit
  if (m_node == NULL) {
    printf("Thread: No node, exiting\n");
    return;
  }

  try {
    attnReg attnMask;
    attnMask.cpm.MoveDone = 1;
    attnMask.cpm.Disabled = 1;
    attnMask.cpm.Enabled = 1;
    attnMask.cpm.Ready = 1;
    attnMask.cpm.WasHomed = 1;
    attnMask.cpm.InHardStop = 1;
    m_node->Adv.Attn.Mask = attnMask;

    // Get the node ready to go
    Enable();
    // Initialize the motion values
    InitMotionParams();

  } catch (mnErr &err) {
    fprintf(stderr, "Thread Setup link error: %s\n", err.ErrorMsg);
    m_brainiac->Quit();
  } catch (...) {
    fprintf(stderr, "Thread setup failed generically.\n");
    m_brainiac->Quit();
  }

  /*if (!m_quitting) {
    // Zero out the measured position
    m_node->Motion.PosnMeasured.Refresh();
    double posn = m_node->Motion.PosnMeasured.Value();
    m_node->Motion.AddToPosition(-posn);
    m_node->Motion.PosnMeasured.Refresh();
    PrintStats(true);
  }*/
  // PrintStats(true);
  m_state = STATE_IDLE;
  m_brainiac->SignalReady(m_node->Info.Ex.Addr());

  // Start the machine cycling
  while (!m_quitting) {
    try {

      // Save last state for debugging purposes
      m_lastState = m_state;
      switch (m_state) {
      case STATE_IDLE:
        // cout << "axis idle" << endl;
        // Update idle state
        m_brainiac->SignalIdle(m_node->Info.Ex.Addr());
        // cout << "servo_state_idle" << endl;
        LOG_SERVO(this->UserID,"IDLE");
        WaitForCondition(kick);
        ResetCondition(kick);
        // Quitting?
        if (m_quitting)
          continue;
        m_state = STATE_WAIT_FOR_COMMAND;
        break;
      case STATE_WAIT_FOR_COMMAND:
        // cout << "waiting kicked" << endl;
        m_brainiac->WaitForCommand(command);
        //m_brainiac->SignalReset();
        //m_brainiac->SignalReset();
        
        LOG_SERVO(this->UserID,"Got a command");
        // Out of here?
        if (m_quitting)
          continue;
        if (command == Brainiac::CommandType::Home) { // Homing requested
          m_state = STATE_SEND_HOME;
        } else if (command == Brainiac::CommandType::Move) { // Move requested
          m_state = STATE_SEND_MOVE;
        } else { // idle no request
        }

        break;
      case STATE_SEND_HOME:
        // Initiate Homing
        m_homingTimeoutMs = TIME_TILL_TIMEOUT;
        // Home();
        output_stream<<"Sending the home command,Homing Timeout:"<<m_homingTimeoutMs<<"ms";
        LOG_SERVO(this->UserID,output_stream.str());
        output_stream.str("");
        m_state = STATE_SIGNAL_HOME_SENT;
        break;

      case STATE_SEND_MOVE:
        // Initiate the motionMove
        
        ::int32_t moveDuration;
        empty_buffer_space = Move();
        moveDuration = (::int32_t)m_node->Motion.Adv.MovePosnDurationMsec(
            m_move.value, true);
        m_moveTimeoutMs = moveDuration + 200;
        output_stream<<"Sending the move command,Move Duration:"<<moveDuration<<"ms";
        LOG_SERVO(this->UserID,output_stream.str());
        output_stream.str("");
        m_state = STATE_SIGNAL_MOVE_SENT;
        break;
      case STATE_SIGNAL_MOVE_SENT:
        m_brainiac->SignalCommandSent(m_node->Info.Ex.Addr());
        LOG_SERVO(this->UserID,"Move command sent");
        m_state = STATE_WAIT_FOR_MOVE_DONE;
        break;
      case STATE_SIGNAL_HOME_SENT:
        m_brainiac->SignalCommandSent(m_node->Info.Ex.Addr());
        LOG_SERVO(this->UserID,"Home command sent");
        m_state = STATE_WAIT_FOR_HOME_DONE;
        break;
      case STATE_WAIT_FOR_HOME_DONE:
        homeDone = WaitForHome(m_homingTimeoutMs);

        
        if (!homeDone) {
          printf("ERROR: [%2d] timed out waiting for home done\n",
                 m_node->Info.Ex.Addr());
          m_brainiac->Quit();
          return;
        } else {
          LOG_SERVO(this->UserID,"Homed");
          m_brainiac->SignalCommandDone(m_node->Info.Ex.Addr());
          m_state = STATE_IDLE;
        }
        if (m_quitting)
          continue;
        break;
      case STATE_WAIT_FOR_MOVE_DONE:
        moveDone = WaitForMove(m_moveTimeoutMs);
        
        if (!moveDone) {
          printf("ERROR: [%2d] timed out waiting for move done\n",
                 m_node->Info.Ex.Addr());
          m_brainiac->Quit();
          return;
        } else {
          LOG_SERVO(this->UserID,"Moved");
          m_brainiac->SignalCommandDone(m_node->Info.Ex.Addr());
          m_moveCount++;
          m_state = STATE_IDLE;
        }
        if (m_quitting)
          continue;
        break;
      default:
        fprintf(stderr, "ThreadFunction: Illegal state");
        return;
      }
    } // Try block
    catch (mnErr &err) {
      fprintf(stderr, "Thread link error: %s\n", err.ErrorMsg);
      m_brainiac->Quit();
      // m_brainiac->SignalMoveSent(m_node->Info.Ex.Addr());
      // m_brainiac->SignalMoveDone(m_node->Info.Ex.Addr());
      m_brainiac->SignalCommandDone(m_node->Info.Ex.Addr());
      m_brainiac->SignalCommandSent(m_node->Info.Ex.Addr());
      ResetToIdle();
      continue;
    } catch (...) {
      fprintf(stderr, "Thread failed generically.\n");
      m_brainiac->Quit();
      // m_brainiac->SignalMoveSent(m_node->Info.Ex.Addr());
      // m_brainiac->SignalMoveDone(m_node->Info.Ex.Addr());
      m_brainiac->SignalCommandDone(m_node->Info.Ex.Addr());
      m_brainiac->SignalCommandSent(m_node->Info.Ex.Addr());
      ResetToIdle();
    }
  } // while (!m_quitting)

  m_state = STATE_EXITED;
  ResetToIdle();
  m_brainiac->SignalIdle(m_node->Info.Ex.Addr());
  printf("[%s] Thread Exited...\n", m_node->Info.UserID.Value());
  return;
};
//																			   *
//******************************************************************************
