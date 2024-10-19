#pragma once

#include <sys/ioctl.h>
#include <unistd.h>
#include "cyt_console_kit_config.hpp"
#include <ncurses.h>
#include <panel.h>
#include <sstream>
#include <iostream>
#include <chrono>



namespace coyot3{
namespace logger{

namespace Terminal{
  bool Initialize(int cols = 1,int rows = 1);
  bool InitializeModule(int module_index = 0);
  

  bool appendText(std::ostringstream& s, int cpair,int windex = 0);
  bool appendInfo(std::ostringstream& s, int windex = 0);
  bool appendWarn(std::ostringstream& s, int windex = 0);
  bool appendError(std::ostringstream& s, int windex = 0);
  bool appendDebug(std::ostringstream& s, int windex = 0);

}


}//eons logger
}//eons coyot3

#define CCONSOLE_COLOR_INFO   1
#define CCONSOLE_COLOR_WARN   2
#define CCONSOLE_COLOR_ERROR  3
#define CCONSOLE_COLOR_DEBUG  4


// to be called once at main().
#define CLOG_TERMINALS_INITIALIZE(c,r) coyot3:logger::Terminal::Initialize(c,r);

// Sets the terminal index to assign to the module that prints "stuffs"
#define CLOG_MODULE_INITIALIZE(t)      coyot3:logger::Terminal::module_terminal_index = t;