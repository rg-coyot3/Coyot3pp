#pragma once

#include <string>
#include <sstream>
#include <ctime>
#include <chrono>
#include <iostream>






#include <iostream>
#ifndef CY_SIMPLELOGGER_DEFAULT_DEBUG_LEVEL
    #define CY_SIMPLELOGGER_DEFAULT_DEBUG_LEVEL 3
#endif


#define __SIMPLELOGGER_COLOR_BLACK_        "\033[30m"
#define __SIMPLELOGGER_COLOR_RED_          "\033[31m"
#define __SIMPLELOGGER_COLOR_GREEN_        "\033[32m"
#define __SIMPLELOGGER_COLOR_YELLOW_       "\033[33m"
#define __SIMPLELOGGER_COLOR_BLUE_         "\033[34m"
#define __SIMPLELOGGER_COLOR_MAGENTA_      "\033[35m"
#define __SIMPLELOGGER_COLOR_CYAN_         "\033[36m"
#define __SIMPLELOGGER_COLOR_WHITE_        "\033[37m"
#define __SIMPLELOGGER_STDOUTCOLORS_RESET_ "\033[0m"

namespace coyot3{
namespace logger{
    
        namespace Terminal{
            static int module_terminal_index = 0;
        }
        class SimpleLoggerClass{
        public:
            static int GlobalLogDebugLevel;
            enum class LogLineType{
                INFO=0,
                WARNING = 1,
                ERROR = 2,
                DEBUG = 3
            };
            static const char* LogLineTypeToString(LogLineType t);
            SimpleLoggerClass(LogLineType tp);
            SimpleLoggerClass(LogLineType tp,const ::std::string& header);
            SimpleLoggerClass(LogLineType tp, int tid = 0);


            virtual ~SimpleLoggerClass();

            template<typename T>
            SimpleLoggerClass& operator << (const T& data){
                slcstream << data;
                return *this;
            }

        private:
            ::std::ostringstream slcstream;
            LogLineType t;
            int _tid;
    };


}//eons logger
}//eons coyot3


#undef CLOG_MODULE_PREFIX_ // remove definition for any prefy at the beggining 
                           //   of any module when including the tools

#undef CLOG_MODULE_IS_MUTED_   //




#ifdef CLOG_MODULE_IS_MUTED_


  #define CLOG_INFO(data) //
  #define CLOG_WARN(data) //
  #define CLOG_ERROR(data) //
  #define CLOG_DEBUG(DEBUGL,data) //

#else

  #ifndef CLOG_MODULE_PREFIX_
    #define CLOG_INFO(data) {\
            ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::INFO, coyot3::logger::Terminal::module_terminal_index);\
            slc << data;\
        }

    #define CLOG_WARN(data) {\
        ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::WARNING, coyot3::logger::Terminal::module_terminal_index);\
        slc << data;\
        } 


    #define CLOG_ERROR(data) {\
        ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::ERROR, coyot3::logger::Terminal::module_terminal_index);\
        slc << data;\
        }


    #define CLOG_DEBUG(DEBUGL,data) \
        if(DEBUGL <= coyot3::logger::SimpleLoggerClass::GlobalLogDebugLevel)\
        {\
            ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::DEBUG, coyot3::logger::Terminal::module_terminal_index);\
            slc << data;\
        }
  #else

  #define CLOG_INFO(data) {\
          ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::INFO, coyot3::logger::Terminal::module_terminal_index);\
          slc << CLOG_MODULE_PREFIX_ << " ; " << data;\
      }

  #define CLOG_WARN(data) {\
      ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::WARNING, coyot3::logger::Terminal::module_terminal_index);\
      slc << CLOG_MODULE_PREFIX_ << " ; " << data;\
      } 


  #define CLOG_ERROR(data) {\
      ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::ERROR, coyot3::logger::Terminal::module_terminal_index);\
      slc << CLOG_MODULE_PREFIX_ << " ; " << data;\
      }


  #define CLOG_DEBUG(DEBUGL,data) \
      if(DEBUGL <= coyot3::logger::SimpleLoggerClass::GlobalLogDebugLevel)\
      {\
          ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::DEBUG, coyot3::logger::Terminal::module_terminal_index);\
          slc << CLOG_MODULE_PREFIX_ << " ; " << data;\
      }
  #endif
#endif




#define CLOG_DEBUG_LEVEL_SET(level) ::coyot3::logger::SimpleLoggerClass::GlobalLogDebugLevel = level;

// a wrapper : needs the definition of a positive integer that contains the 
//             number of warnings to be shown.
#define CLOG_WARN_THEN_DEBUG(CLOG_VAR_COUNTER,DEBUGL,data) \
  if(CLOG_VAR_COUNTER > 0){ \
    CLOG_VAR_COUNTER --;    \
    CLOG_WARN(data);        \
  }else{                    \
    CLOG_DEBUG(DEBUGL,data);\
  }

  
#define CLOG_WARN_THEN_SILENCE(CLOG_VAR_COUNTER,data) \
  if(CLOG_VAR_COUNTER > 0){  \
    CLOG_VAR_COUNTER --;    \
    CLOG_WARN(data);        \
  }

#define CLOG_INFO_OR_ALERT(is_alert,prefix,postfix_on_info,postfix_on_alert) \
  if((is_alert)){\
    CLOG_WARN(prefix << " : " << postfix_on_alert)\
  }else{\
    CLOG_INFO(prefix << " : " << postfix_on_info)\
}

#define CLOG_EVALUATION(assertion,prefix,postfix_on_info,postfix_on_alert) \
  if(assertion){\
    CLOG_INFO(prefix << " : " << postfix_on_info)\
  }else{\
    CLOG_WARN(prefix << " : " << postfix_on_alert)\
}
