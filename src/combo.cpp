///*******************************************************************************
/**
    Example-MultiThreaded

        The main function for Multi-Threaded ClearPath-SC example. The only
command line argument is the port number where the network is attached. This
        main function opens the port, prints some basic information about the
        nodes that are found, checks that they are all in full-access mode,
        then creates the Supervisor object (which has its own thread) to run
        the show. This main thread then waits for the user to hit a key to
        end the program.
**/
//******************************************************************************
#include "Logging.h"
#include "file.h"
mutex main_mutex;
mutex mutex_x, mutex_y, mutex_z;
condition_variable cond_boss, cond_x, cond_y, cond_z;
bool signal_command, isdone, supervisor_idle;
Servo *X1, *Y1, *Y2, *Z1, *Z2, *Z3;
uint8_t XYZE_RQ = 0x00;
uint8_t HOME_RQ = 0x00;

// Send message and wait for newline
void msgUser(const char *msg) {
  LOG_MAIN(msg);
  getchar();
}
string filename = "";
void MN_DECL AttentionDetected(const mnAttnReqReg &detected) {
  // Make a local, non-const copy for printing purposes
  mnAttnReqReg myAttns = detected;
  // Create a buffer to hold the attentionReg information
  char attnStringBuf[512];
  // Load the buffer with the string representation of the attention information
  myAttns.AttentionReg.StateStr(attnStringBuf, 512);
  // Print it out to the console
  printf("  --> ATTENTION: port %d, node=%d, attn=%s\n",
         detected.MultiAddr >> 4, detected.MultiAddr, attnStringBuf);
}

#if _MSC_VER
#pragma warning(disable : 4996)
#endif
// A nice way of printing out the system time
string CurrentTimeStr() {
  time_t now = time(NULL);
  return string(ctime(&now));
}
#define CURRENT_TIME_STR CurrentTimeStr().c_str()
#if _MSC_VER
#pragma warning(default : 4996)
#endif

int main(int argc, char *argv[]) {
  if (argc != 2) // argc should be 2 for correct execution
    // We print argv[0] assuming it is the program name
    cout << "usage: " << argv[0] << " <filename>\n";
  else {
    filename = argv[1];
  }

  cout << filename << endl;
  std::ifstream *file = new ifstream(filename);
  bool readflag = false;
  exiting = false;
  startprint = false;
  state_machine = IDLE;
  cout << "usage:" << endl;

  int count_line = 0;

  size_t portCount = 0;

  std::vector<std::string> comHubPorts;

  // Create the SysManager object. This object will coordinate actions among
  // various ports
  // and within nodes. In this example we use this object to setup and open our
  // port.
  SysManager myMgr; // Create System Manager myMgr

  // This will try to open the port. If there is an error/exception during the
  // port opening, the code will jump to the catch loop where detailed
  // information regarding the error will be displayed; otherwise the catch loop
  // is skipped over

  SysManager::FindComHubPorts(comHubPorts);
  printf("Found %d SC Hubs\n", comHubPorts.size());

  for (portCount = 0;
       portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
       portCount++) {

    myMgr.ComHubPort(
        portCount,
        comHubPorts[portCount]
            .c_str()); // define the first SC Hub port (port 0) to be associated
                       // with COM portnum (as seen in device manager)
  }

  if (portCount > 0) {
    // printf("\n I will now open port \t%i \n \n", portnum);
    myMgr.PortsOpen(portCount); // Open the port

    for (size_t i = 0; i < portCount; i++) {
      IPort &myPort = myMgr.Ports(i);

      printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(),
             myPort.OpenState(), myPort.NodeCount());
    }
  } else {
    printf("Unable to locate SC hub port\n");

    msgUser(
        "Press any key to continue."); // pause so the user can see the error
                                       // message; waits for user to press a key

    return -1; // This terminates the main program
  }

  // Assume that the nodes are of the right type and that this app has full
  // control
  bool nodeTypesGood = true, accessLvlsGood = true;

  for (size_t iPort = 0; iPort < portCount; iPort++) {
    // Get a reference to the port, to make accessing it easier
    IPort &myPort = myMgr.Ports(iPort);

    // Enable the attentions for this port
    myPort.Adv.Attn.Enable(true);
    // The attentions will be handled by the individual nodes, but register
    // a handler at the port level, just for illustrative purposes.
    myPort.Adv.Attn.AttnHandler(AttentionDetected);

    for (unsigned iNode = 0; iNode < myPort.NodeCount(); iNode++) {

      // Get a reference to the node, to make accessing it easier
      INode &theNode = myPort.Nodes(iNode);
      theNode.Motion.AccLimit = 100000;

      // Make sure we are talking to a ClearPath SC
      if (theNode.Info.NodeType() != IInfo::CLEARPATH_SC_ADV) {
        printf(
            "---> ERROR: Uh-oh! Node %d is not a ClearPath-SC Advanced Motor\n",
            iNode);
        nodeTypesGood = false;
      }

      if (nodeTypesGood) {
        // Create an axis for this node
        // listOfAxes.push_back(new Axis(&theNode));

        if (!theNode.Setup.AccessLevelIsFull()) {
          printf("---> ERROR: Oh snap! Access level is not good for node %u\n",
                 iNode);
          accessLvlsGood = false;
        } else {

          if (strcmp(theNode.Info.UserID.Value(), (const char *)"Y1") == 0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "Y1 init" << endl;
            // theNode.Motion.Adv.TriggerGroup(2);
            Y1 = listOfServos.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"Y2") ==
                     0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "Y2 init" << endl;
            // theNode.Motion.Adv.TriggerGroup(2);
            Y2 = listOfServos.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"X1") ==
                     0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "X1 init" << endl;
            X1 = listOfServos.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"Z1") ==
                     0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "Z1 init" << endl;
            Z1 = listOfServos.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"Z2") ==
                     0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "Z2 init" << endl;
            Z2 = listOfServos.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"Z3") ==
                     0) {
            listOfServos.push_back(new Servo(&theNode));
            cout << "Z3 init" << endl;
            Z3 = listOfServos.back();
          } else {
            // theNode.Motion.Adv.TriggerGroup(2);
            cout << "unkown axis" << endl;
            return -1;
          }
          // cout << theNode.Info.UserID.Value() << endl;
        }
      }
    }
  }
  if (nodeTypesGood && accessLvlsGood) {

    // Create the supervisor thread, giving it access to the list of axes
    printf("\nMachine starting: %s\n", CURRENT_TIME_STR);

    // Everything is running - wait for the user to press a key to end it
    // msgUser("Press any key to stop the machine."); //pause so the user can
    // see the error message; waits for user to press a key
    signal_command = false;

    BOSS = new Brainiac(listOfServos, myMgr);
    BOSS->mutex_ptr = &main_mutex;
    BOSS->main_cond = &cond_boss;
    BOSS->CreateThread();

    while (!exiting) {
      try {
        switch (state_machine) {
        case IDLE:
          file = new ifstream(filename);
          if (file->is_open()) {
            msgUser("Enter to start printing");
            state_machine = START_READING;
          } else {
            cout << "ERROR: can't open the file" << endl;
          }

          break;
        case WAIT_FOR_BOSS_IDLE:
          // wait for idle
          {
            std::unique_lock<std::mutex> lk(main_mutex);
            cond_boss.wait(lk, [&] { return BOSS->servos_idle; });
          }
          LOG_MAIN("SERVOS ARE IDLE");
          state_machine = SEND_COMMAND_TO_BOSS;
          break;
        case SEND_COMMAND_TO_BOSS:
          // notify there is a new command request
          // cout << "MAIN: Here" << endl;
          {
            std::unique_lock<std::mutex> lk(main_mutex);
            BOSS->XYZE_RQ = XYZE_RQ;
            BOSS->HOME_RQ = HOME_RQ;
            XYZE_RQ = 0x00;
            HOME_RQ = 0x00;
            BOSS->command_requested = true;
            cond_boss.notify_all();
          }

          LOG_MAIN("COMMAND REQUEST SENT TO BOSS");
          state_machine = WAIT_FOR_BOSS_COMMANDS_DONE;
          break;

        case WAIT_FOR_BOSS_COMMANDS_DONE:
          LOG_MAIN("WAITING FOR COMMAND TO BE DONE");
          {
            std::unique_lock<std::mutex> lk(main_mutex);
            cond_boss.wait(lk, [&] { return BOSS->command_done; });
          }
          {
            std::unique_lock<std::mutex> lk(main_mutex);
            BOSS->command_done = false;
            lk.unlock();
          }
          LOG_MAIN("COMMAND IS DONE");
          state_machine = START_READING;
          break;

        case WAIT_FOR_SUPER_IDLE:
          // theSuper->WaitForDone();
          break;
        case START_READING:
          if (!read_file(file)) {
            state_machine = ENDOFFILE;
            count_line = 0;
          } else {
            // state_machine = WAIT_FOR_BUFFER;
          }

          break;
        case ENDOFFILE:
          // cout << "end of file" << endl;
          file->close();

          state_machine = IDLE;
          // theSuper->Quit();
          // theSuper->Terminate();
          // return -1;

          break;

        default:
          break;
        }
      } catch (...) {
      }
    }

    printf("Machine stopping: %s\n", CURRENT_TIME_STR);

    // Tell the supervisor to stop

    // printf("\nFinalStats:\tnumMoves\t\tPosition\n");
  } else {
    // If something is wrong with the nodes, tell the user about it and quit
    if (!nodeTypesGood) {
      printf(
          "\n\tFAILURE: Please attach only ClearPath-SC Advanced nodes.\n\n");
    } else if (!accessLvlsGood) {
      printf("\n\tFAILURE: Please get full access on all your nodes.\n\n");
    }
  }
}

bool read_file(std::ifstream *file) {
  std::string line;
  std::string temp;
  std::string s_xpos, s_ypos, s_zpos, s_epos, s_orginal_feedrate;
  std::string s_target;
  std::string s_command;
  size_t pos = 0;
  size_t endpoint = 0;
  bool xflag = false;
  bool yflag = false;
  bool zflag = false;
  bool eflag = false;
  if (getline(*file, line)) {
    if ((endpoint = line.find(blank)) != (std::string::npos)) {
      s_command = line.substr(1, endpoint);
    } else { // end of line
      s_command = line.substr(1, line.length());
    }
    cout << line << endl;
    switch (line.at(0)) {
    case 'G':
      command = stoi(s_command);
      // cout << command << endl;
      switch (command) {
      case 1: // linear move
              // cout << "(" << i << ")" << line.substr(0, (line.length() - 1));
        if ((pos = line.find(delimiterF)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          s_orginal_feedrate = temp;
          original_feedrate = atof(s_orginal_feedrate.c_str());
          final_feedrate = original_feedrate * feedreate_multiplier / 100;
        }
        if ((pos = line.find(delimiterX)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_xpos = temp.substr(0, endpoint);
          } else { // end of line
            s_xpos = temp;
          }
          xpos = atof(s_xpos.c_str());
          //X1->SetMove(xpos, final_feedrate);
          //XYZE_RQ = XYZE_RQ + 1;
        }
        if ((pos = line.find(delimiterY)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_ypos = temp.substr(0, endpoint);
          } else { // end of line
            s_ypos = temp;
          }
          ypos = atof(s_ypos.c_str());
          Y1->SetMove(ypos, final_feedrate);
          Y2->SetMove(ypos, final_feedrate);
          XYZE_RQ = XYZE_RQ + (1 << 1);
        }
        if ((pos = line.find(delimiterZ)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_zpos = temp.substr(0, endpoint);
          } else { // end of line
            s_zpos = temp;
          }
          zpos = atof(s_zpos.c_str());
          //Z1->SetMove(zpos, final_feedrate);
          //Z2->SetMove(zpos, final_feedrate);
          //Z3->SetMove(zpos, final_feedrate);
          //XYZE_RQ = XYZE_RQ + (1 << 2);
        }
        if ((pos = line.find(delimiterE)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_epos = temp.substr(0, endpoint);
          } else { // end of line
            s_epos = temp;
          }
          epos = atof(s_epos.c_str());
          //XYZE_RQ = XYZE_RQ + (1 << 3);
        }
        HOME_RQ = 0x00;
        state_machine = WAIT_FOR_BOSS_IDLE;
        break;
      case 28:
        if ((pos = line.find(delimiterX)) != (std::string::npos)) {
          HOME_RQ = HOME_RQ + 1;
        }
        if ((pos = line.find(delimiterY)) != (std::string::npos)) {
          HOME_RQ = HOME_RQ + (1 << 1);
        }
        if ((pos = line.find(delimiterZ)) != (std::string::npos)) {
          HOME_RQ = HOME_RQ + (1 << 2);
        }

        state_machine = WAIT_FOR_BOSS_IDLE;
        break;

      default:
        break;
      }
      break;
    case 'M':
      command = stoi(s_command);
      // cout << command << endl;
      switch (command) {
      case 82: // Estep Absolute
        // This command is used to override G91 and put the E axis into
        // absolute mode independent of the other axes.
        break;
      case 84: // Disable Steppers
        break;
      case 104: // Set Hotend Temp no waiting
        break;
      case 106: // Set Fan Speed. *8bit value
        break;
      case 109: // Set Hotend Temperature and wait for it to reach
        break;
      case 140: // Set Bed Temperature, no waiting
        break;
      case 190: // Set Bed Temperature and wait for it to reach
        break;
      case 220: // Set Feedrate Percentage
        if ((pos = line.find(delimiterS)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_target = temp.substr(0, endpoint);
          } else { // end of line
            s_target = temp;
          }
          feedreate_multiplier = atof(s_target.c_str());
          // cout << " feedrate percentage:" << feedreate_multiplier<<endl;
        }
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}
