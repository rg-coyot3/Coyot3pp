#include <Coyot3pp/Cor3/module/LoggerCapability.hpp>



#define __SIMPLELOGGER_COLOR_BLACK_        "\033[30m"
#define __SIMPLELOGGER_COLOR_RED_          "\033[31m"
#define __SIMPLELOGGER_COLOR_GREEN_        "\033[32m"
#define __SIMPLELOGGER_COLOR_YELLOW_       "\033[33m"
#define __SIMPLELOGGER_COLOR_BLUE_         "\033[34m"
#define __SIMPLELOGGER_COLOR_MAGENTA_      "\033[35m"
#define __SIMPLELOGGER_COLOR_CYAN_         "\033[36m"
#define __SIMPLELOGGER_COLOR_WHITE_        "\033[37m"
#define __SIMPLELOGGER_STDOUTCOLORS_RESET_ "\033[0m"




namespace ct = coyot3::tools;


namespace coyot3{
namespace mod{



  LoggerLine::LoggerLine(Level l)
  :level_(l)
  {

  }

  LoggerLine::~LoggerLine(){
    
  }

  std::string     LoggerLine::str() const{return ostr_.str();}
  LoggerLine::Level      LoggerLine::level() const{return level_;}
  


  LoggerCapability::LoggerCapability(const std::string& iname)
  :instance_name_(iname)
  ,file_log_target_()
  ,file_log_active_(false)

  ,stdout_log_active_(true)
  ,stdout_colored_(true)
  {

  }

  LoggerCapability::~LoggerCapability(){}



  // LoggerLine& LoggerCapability::log_info(){
  //   LoggerLine ll(LoggerLine::Level::CLC_INFO);
  //   return ll;
  // }
  // LoggerLine& LoggerCapability::log_warn(){
  //   LoggerLine ll(LoggerLine::Level::CLC_WARN);
  //   return ll;
  // }
  // LoggerLine& LoggerCapability::log_err(){
  //   LoggerLine ll(LoggerLine::Level::CLC_ERROR);
  //   return ll;
  // }
  // LoggerLine& LoggerCapability::log_debug(int vlevel){
  //   LoggerLine ll(static_cast<LoggerLine::Level>(static_cast<int>(LoggerLine::Level::CLC_DEBUG)+ vlevel));
  //   return ll;
  // }

  LoggerLine::Level LoggerCapability::modlog_verbosity() const{return level_;}
  LoggerLine::Level LoggerCapability::modlog_verbosity(LoggerLine::Level l){return level_ = l;}


  void LoggerCapability::operate_line_(const LoggerLine& l){
    if(l.level() > level_)return;
    if(stdout_log_active_==true)
    if(stdout_colored_)operate_stdout_colors_(l);
    else               operate_stdout_nocolors_(l);
  }

  
  std::ostringstream LoggerCapability::o(){return std::ostringstream();}
  void        
  LoggerCapability::log_info(const std::ostringstream& o){
    log_info(o.str());
  }
  void        
  LoggerCapability::log_info(const std::string& o){
    //std::cout << " info" << std::endl;
    LoggerLine ll(LoggerLine::Level::CLC_INFO);
    ll << o;
    operate_line_(ll);
  }

  void        LoggerCapability::log_warn(const std::ostringstream& o){
    log_warn(o.str());
  }
  void        LoggerCapability::log_warn(const std::string& o){
    LoggerLine ll(LoggerLine::Level::CLC_WARN);
    ll << o;
    operate_line_(ll);
  }

  void        LoggerCapability::log_err(const std::ostringstream& o){
    log_err(o.str());
  }
  void        LoggerCapability::log_err(const std::string& o){
    LoggerLine ll(LoggerLine::Level::CLC_ERROR);
    ll << o;  
    operate_line_(ll);
  }

  void        LoggerCapability::log_debug(int vlevel, const std::ostringstream& o){
    log_debug(vlevel,o.str());
  }
  void        LoggerCapability::log_debug(int vlevel, const std::string& o){
    LoggerLine ll(static_cast<LoggerLine::Level>(static_cast<int>(LoggerLine::Level::CLC_DEBUG)+ vlevel));
    ll << o;
    operate_line_(ll);
  }
  void LoggerCapability::operate_stdout_colors_(const LoggerLine& l){
    std::stringstream sstr;
    
    switch(l.level()){
      case LoggerLine::Level::CLC_ERROR:
        std::cout << __SIMPLELOGGER_COLOR_RED_ << ct::get_current_utc_string() 
        << " : ERR" << prefix_ << l.str() 
        << __SIMPLELOGGER_STDOUTCOLORS_RESET_ << std::endl;
        break;
      case LoggerLine::Level::CLC_WARN:
        std::cout << __SIMPLELOGGER_COLOR_YELLOW_ << ct::get_current_utc_string() 
        << " : WRN" << prefix_<< l.str()
        << __SIMPLELOGGER_STDOUTCOLORS_RESET_ << std::endl;
        break;
      case LoggerLine::Level::CLC_INFO:
        std::cout << __SIMPLELOGGER_COLOR_GREEN_ << ct::get_current_utc_string() 
        << " : INF" << prefix_ << l.str()
        << __SIMPLELOGGER_STDOUTCOLORS_RESET_ << std::endl;
        break;
      case LoggerLine::Level::CLC_DEBUG:
      default:
        if(l.level() <= verbosity_)
        std::cout << __SIMPLELOGGER_COLOR_BLUE_ << ct::get_current_utc_string() 
        << " : DBG(" << l.level() << ")" << prefix_ << l.str()
        << __SIMPLELOGGER_STDOUTCOLORS_RESET_ << std::endl;
        break;
    }
    

  }
  void LoggerCapability::operate_stdout_nocolors_(const LoggerLine& l){
    std::stringstream sstr;
    
    switch(l.level()){
      case LoggerLine::Level::CLC_ERROR:
        std::cout << ct::get_current_utc_string() 
        << " : ERR : " << instance_name_ << " : " << l.str()
        << std::endl;
        break;
      case LoggerLine::Level::CLC_WARN:
        std::cout << ct::get_current_utc_string() 
        << " : WRN : " << instance_name_ << " : " << l.str()
        << std::endl;
        break;
      case LoggerLine::Level::CLC_INFO:
        std::cout << ct::get_current_utc_string() 
        << " : INF : " << instance_name_ << " : " << l.str()
         << std::endl;
        break;
      case LoggerLine::Level::CLC_DEBUG:
      default:
        if(l.level() <= verbosity_)
        std::cout << ct::get_current_utc_string() 
        << " : DBG(" << l.level() << ") : " << instance_name_ << " : " << l.str()
        << std::endl;
        break;
    }
    

  }


  std::string LoggerCapability::instance_name() const{return instance_name_;}
  std::string LoggerCapability::instance_name(const std::string& in){
    instance_name_ = in;
    prefix_conf_();
    return instance_name_;
  }
  std::string LoggerCapability::class_name() const{return class_name_;}
  std::string LoggerCapability::class_name(const std::string& cn){
    class_name_ = cn;
    prefix_conf_();
    return class_name_;
  }
  void LoggerCapability::prefix_conf_(){
    if((instance_name_.size() != 0) && (class_name_.size() != 0)){
      prefix_ = std::string(" : ") + instance_name_ + " : " + class_name_ + " : ";
    }else if((instance_name_.size() == 0) && (class_name_.size() == 0)){
      prefix_ = " : ";
    }else if(instance_name_.size() == 0){
      prefix_ = std::string(" : ") + class_name_ + " : ";
    }else if(class_name_.size() == 0){
      prefix_ = std::string(" : ") + instance_name_ + " : ";
    }
    prefix_ = " : WTF! :";
  }
}
}