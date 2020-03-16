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
#include "file_old.h"
mutex main_mutex;
condition_variable cond;
bool signal_command, isdone, supervisor_idle;
Axis *X, *Y1, *Y2, *Z1, *Z2, *Z3;

// Send message and wait for newline
void msgUser(const char *msg) {
  std::cout << msg;
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
        listOfAxes.push_back(new Axis(&theNode));

        if (!theNode.Setup.AccessLevelIsFull()) {
          printf("---> ERROR: Oh snap! Access level is not good for node %u\n",
                 iNode);
          accessLvlsGood = false;
        } else {
          // Set the move distance based on where it is in the network
          // listOfAxes.at(iNode)->SetMoveRevs((iNode + 1) * 2);
          // Set the trigger group indicator
          //

          if (strcmp(theNode.Info.UserID.Value(), (const char *)"Y1") == 0) {
            cout << "Y1 init" << endl;
            theNode.Motion.Adv.TriggerGroup(2);
            Y1 = listOfAxes.back();
          } else if (strcmp(theNode.Info.UserID.Value(), (const char *)"Y2") ==
                     0) {
            cout << "Y2 init" << endl;
            theNode.Motion.Adv.TriggerGroup(2);
            Y2 = listOfAxes.back();
          } else {
            theNode.Motion.Adv.TriggerGroup(2);
            cout << "unkown axis" << endl;
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

    while (!exiting) {
      try {
        switch (state_machine) {
        case IDLE:

          file = new ifstream(filename);
          if (file->is_open()) {
            msgUser("enter to start printing");
            theSuper = new Supervisor(listOfAxes, myMgr);
            theSuper->mutex1 = &main_mutex;
            theSuper->main_cond = &cond;

            theSuper->CreateThread();
            /*{
              std::unique_lock<std::mutex> lk(main_mutex, defer_lock);
              lk.lock();
              // lk.try_lock_for(std::chrono::milliseconds(100));
              //supervisor_idle = &theSuper->super_idle;
              cout << "super_idle: "<< theSuper->super_idle << endl;
              cond.wait(lk, [] { return theSuper->super_idle; });
              cout << "super_idle: "<< theSuper->super_idle << endl;
            }*/
            state_machine = START_READING;

          } else {
            cout << "file error" << endl;
          }

          break;
        case WAIT_FOR_BUFFER: {
          {
            std::unique_lock<std::mutex> lk(main_mutex, defer_lock);
            lk.lock();
            // lk.try_lock_for(std::chrono::milliseconds(100));
            // cout << "command_done: "<< theSuper->command_done << endl;
            cond.wait(lk, [] { return theSuper->command_done; });
            // cout << "command_done: "<< theSuper->command_done << endl;
            // cout << "endk" << endl;
          }
          {
            std::unique_lock<std::mutex> lk(main_mutex, defer_lock);
            lk.lock();
            // lk.try_lock_for(std::chrono::milliseconds(100));
            // supervisor_idle = &theSuper->super_idle;
            theSuper->command_ready = false;
            // cout << "super_idle: "<< theSuper->super_idle << endl;
            cond.wait(lk, [] { return theSuper->super_idle; });
            // cout << "super_idle: "<< theSuper->super_idle << endl;
          }

          state_machine = START_READING;
        }
        // cout<<"hi"<<endl;
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
          /*switch (count_line) {
          case 10:
            count_line = 0;
            //state_machine = WAIT_FOR_BUFFER;
            break;
          default:
            theSuper->SignalReadyCommand();
            if (!read_file(file)) {
              state_machine = ENDOFFILE;
              count_line = 0;
            } else {

              count_line++;
            }

            break;
          }*/
          break;
        case ENDOFFILE:
          cout << "end of file" << endl;
          file->close();
          state_machine = IDLE;
          theSuper->Quit();
          theSuper->Terminate();

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
          // cout << " feedrate:" << final_feedrate<<endl;
        }
        if ((pos = line.find(delimiterX)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_xpos = temp.substr(0, endpoint);
          } else { // end of line
            s_xpos = temp;
          }
          xpos = atof(s_xpos.c_str());
          // listOfAxes.at(0)->SetMove(xpos, final_feedrate);
          // listOfAxes.at(1)->SetMove(xpos, final_feedrate);
          // listOfAxes.at(0)->Move();
          // command_ready=true;
          // m_cond.notify_one();
          // theSuper->SignalReadyCommand();
          // std::cout<<"ready"<<endl;
          // cout << " x:" << xpos;
        }

        if ((pos = line.find(delimiterY)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_ypos = temp.substr(0, endpoint);
          } else { // end of line
            s_ypos = temp;
          }
          ypos = atof(s_ypos.c_str());
          listOfAxes.at(0)->SetMove(ypos, final_feedrate);
          // cout << " y:" << ypos;
          // Y2->SetMove(xpos, final_feedrate);
          // Y1->SetMove(xpos, final_feedrate);
          // lk->try_lock_for(std::chrono::milliseconds(100));
          {
            std::unique_lock<std::mutex> lk(main_mutex, defer_lock);

            lk.lock();
            // theSuper->command_ready = &signal_command;
            theSuper->command_ready = true;
            theSuper->super_idle = false;
            theSuper->command_type = Supervisor::CommandType::Move;
            
            // lk.unlock();
            // cout << "unlock_main" << endl;

            cond.notify_all();
          }
          state_machine = WAIT_FOR_BUFFER;
        }

        if ((pos = line.find(delimiterZ)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_zpos = temp.substr(0, endpoint);
          } else { // end of line
            s_zpos = temp;
          }
          zpos = atof(s_zpos.c_str());
          // cout << " z:" << zpos;
        }

        if ((pos = line.find(delimiterE)) != (std::string::npos)) {
          temp = line.substr(pos + 1, line.length());
          if ((endpoint = temp.find(blank)) != (std::string::npos)) {
            s_epos = temp.substr(0, endpoint);
          } else { // end of line
            s_epos = temp;
          }
          epos = atof(s_epos.c_str());
          // cout << " e:" << epos;
        }

        // cout << "\n";
        break;
      case 28:
        
        if ((pos = line.find(delimiterX)) != (std::string::npos)) {
            //X->Home();
        }
        if ((pos = line.find(delimiterY)) != (std::string::npos)) {
            //Y->Home();
        }
        if ((pos = line.find(delimiterZ)) != (std::string::npos)) {

        }
        {
            std::unique_lock<std::mutex> lk(main_mutex, defer_lock);

            lk.lock();
            // theSuper->command_ready = &signal_command;
            theSuper->command_ready = true;
            theSuper->super_idle = false;
            theSuper->command_type = Supervisor::CommandType::Home;
            // lk.unlock();
            // cout << "unlock_main" << endl;

            cond.notify_all();
          }
        //listOfAxes.at(0)->Home();
        state_machine = WAIT_FOR_BUFFER;
        
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
