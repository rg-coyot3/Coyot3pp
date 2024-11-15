#pragma once



#include "../Coyot3.hpp"



namespace coyot3{
namespace mod{


  
class LoggerCapability;


enum LogLevel{
    CLC_ERROR = 1,
    CLC_WARN  = 2,
    CLC_INFO  = 3,
    CLC_DEBUG = 4,
  };
  
struct LoggerLine{

  LoggerLine(LogLevel l = LogLevel::CLC_INFO);
  virtual ~LoggerLine();

  template<typename T>
  LoggerLine& operator << (const T& data){
    ostr_ << data;
    return *this;
  }


  std::string    str() const;
  LogLevel                  log_level() const;
  private:
    mutable ::std::ostringstream ostr_;
    LogLevel                 log_level_;

};



class LoggerCapability{

  friend class LoggerLine;
  protected:
    LoggerCapability(const std::string& iname = std::string());


  public:
    virtual ~LoggerCapability();
    LogLevel        modlog_verbosity() const;
    LogLevel        modlog_verbosity(LogLevel l);
    LogLevel        modlog_verbosity(int l);



  protected:
    // LoggerLine& log_info();
    // LoggerLine& log_warn();
    // LoggerLine& log_err();
    // LoggerLine& log_debug(int vLogLevel = 1);
    
    std::ostringstream o();

    void log_info(const std::ostringstream& o);
    void log_info(const std::string& o);
    void log_warn(const std::ostringstream& o);
    void log_warn(const std::string& o);
    void log_err(const std::ostringstream& o);
    void log_err(const std::string& o);
    void log_debug(int vLogLevel,const std::ostringstream& o);
    void log_debug(int vLogLevel,const std::string& o);
    template<typename T, typename U>
    void log_eval(bool assertion, const T& cont_assert_ok, const U& cont_assert_ko){
      if(assertion == true){
        log_info(cont_assert_ok);
      }else{
        log_warn(cont_assert_ko);
      }
    }

    std::string instance_name() const;
    std::string instance_name(const std::string& in);
    std::string class_name() const;
    std::string class_name(const std::string& cn);





  private:
    std::string       instance_name_;
    std::string       class_name_;
    std::string       prefix_;
      void prefix_conf_();
    int               verbosity_;
    LogLevel          log_level_;

    std::string   file_log_target_;
    bool          file_log_active_;

    bool          stdout_log_active_;
    bool          stdout_colored_;

    void    operate_line_(const LoggerLine& l) const;


    void    operate_stdout_nocolors_(const LoggerLine& l)const ;
    void    operate_stdout_colors_(const LoggerLine& l) const;

};


}
}