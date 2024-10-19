#pragma once



#include "../Coyot3.hpp"



namespace coyot3{
namespace mod{



struct LoggerLine{
  enum Level{
    CLC_ERROR = 0,
    CLC_WARN  = 1,
    CLC_INFO  = 2,
    CLC_DEBUG = 3,
    
  };
  LoggerLine(Level l = Level::CLC_INFO);
  virtual ~LoggerLine();

  template<typename T>
  LoggerLine& operator << (const T& data){
    ostr_ << data;
    return *this;
  }


  std::string    str() const;
  Level                  level() const;
  private:
    ::std::ostringstream ostr_;
    Level                 level_;
};



class LoggerCapability{

  friend class LoggerLine;
  protected:
    LoggerCapability(const std::string& iname = std::string());


  public:
    virtual ~LoggerCapability();




  protected:
    // LoggerLine& log_info();
    // LoggerLine& log_warn();
    // LoggerLine& log_err();
    // LoggerLine& log_debug(int vlevel = 1);
    
    std::ostringstream o();
    void log_info(const std::ostringstream& o);
    void log_info(const std::string& o);
    void log_warn(const std::ostringstream& o);
    void log_warn(const std::string& o);
    void log_err(const std::ostringstream& o);
    void log_err(const std::string& o);
    void log_debug(int vlevel,const std::ostringstream& o);
    void log_debug(int vlevel,const std::string& o);
    std::string instance_name() const;
    std::string instance_name(const std::string& in);

    LoggerLine::Level        modlog_verbosity() const;
    LoggerLine::Level        modlog_verbosity(LoggerLine::Level l);



  private:
    std::string       instance_name_;
    int               verbosity_;
    LoggerLine::Level level_;

    std::string   file_log_target_;
    bool          file_log_active_;

    bool          stdout_log_active_;
    bool          stdout_colored_;

    void    operate_line_(const LoggerLine& l);


    void    operate_stdout_nocolors_(const LoggerLine& l);
    void    operate_stdout_colors_(const LoggerLine& l);

};


}
}