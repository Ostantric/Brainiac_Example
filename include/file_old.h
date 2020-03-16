#include "Axis.h"
#include "Brainiac.h"
#include "Servo.h"
#include "Supervisor.h"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>

#define READ_LINES 16

std::string blank = " ";
std::string delimiterX = "X";
std::string delimiterY = "Y";
std::string delimiterZ = "Z";
std::string delimiterE = "E";
std::string delimiterF = "F";
std::string delimiterS = "S";

std::string::size_type size; // alias of size_t

double xpos, ypos, zpos, epos, final_feedrate;
double feedreate_multiplier = 100; //%100 for starting
int original_feedrate, command;

bool startprint;
bool exiting; // Axis quitting
// Supervisor for Teknic
Supervisor *theSuper;
Brainiac *BOSS;
Brainiac *Brainiac_X;
Brainiac *Brainiac_Y;
Brainiac *Brainiac_Z;
// Create a list of axes - one per node
vector<Axis *> listOfAxes;
vector<Servo *> listOfServos;
vector<Servo *> listOfX;
vector<Servo *> listOfY;
vector<Servo *> listOfZ;
// State machine information
enum StateEnum {
  IDLE,
  WAIT_FOR_BUFFER,
  WAIT_FOR_BOSS_IDLE,
  SEND_COMMAND_TO_BOSS,
  WAIT_FOR_BOSS_COMMANDS_DONE,
  WAIT_FOR_X,
  WAIT_FOR_Y,
  WAIT_FOR_Z,
  WAIT_FOR_E,
  WAIT_FOR_XY,
  WAIT_FOR_XZ,
  WAIT_FOR_XE,
  WAIT_FOR_YZ,
  WAIT_FOR_YE,
  WAIT_FOR_ZE,
  WAIT_FOR_XYZ,
  WAIT_FOR_XYE,
  WAIT_FOR_XZE,
  WAIT_FOR_YZE,
  WAIT_FOR_XYZE,
  START_READING,
  ENDOFFILE,
  WAIT_FOR_SUPER_IDLE
};

StateEnum state_machine;

bool read_file(std::ifstream *file);
