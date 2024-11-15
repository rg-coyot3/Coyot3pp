#include <Coyot3pp/Sqlit3/Sqlit3Connector/Sqlit3Connector.hpp>


namespace coyot3{
namespace ddbb{
namespace sqlite{


  Sqlit3Connector::Sqlit3Connector(const std::string& name)
  :ModuleBase(name)
  ,db_own_(nullptr)
  ,connector_ext_(nullptr)
  {
    class_name("sqlit3connector");
    add_task_start(std::bind(&Sqlit3Connector::start,this));
    add_task_stop(std::bind(&Sqlit3Connector::start,this));
    log_info("created");
  }



  std::string Sqlit3Connector::database_name() const{
    return database_name_;
  }
  std::string Sqlit3Connector::database_name(const std::string& tn){
    return database_name_ = tn;
  }


  bool Sqlit3Connector::start(){
    if(connector_ext_ != nullptr){
      log_warn("start : master database attached. ignoring operation");
      return true;
    }
    return _open_database();
  }
  bool Sqlit3Connector::stop(){
    if(connector_ext_ != nullptr){
      log_warn("stop : master database attached. ignoring operation");
      return true;
    }
    return _close_database();
  }

  bool Sqlit3Connector::_open_database(){
    int rc = sqlite3_open(database_name_.c_str(),&db_own_);

    if(rc == SQLITE_OK){
      log_info(o() << "open-database : OK [" << database_name_ << "]");
    }else{
      log_warn(o() << "open-database : ERROR! code [" << rc << "]");
    }
    return rc == SQLITE_OK;
  }

  bool Sqlit3Connector::_close_database(){
    if(db_own_ == nullptr){
      log_warn("close database, trying to close non existant database...!");
      return false;
    }
    sqlite3_close(db_own_);
    log_info(o() << "database closed : OK [" << database_name_ << "]");
    return true;
  }

  bool Sqlit3Connector::make_query(const std::string& q){
    std::string err;
    return make_query(q,err);
  }
  bool Sqlit3Connector::make_query(const std::string& q, std::string& err){
    log_debug(5, o() << "make-query : " << q);
    
    if(connector_ext_ != nullptr){
      log_debug(3,"make-query : external master");
      return connector_ext_->make_query(q,err);
    }

    std::lock_guard<std::mutex> guard(mtx_db_);
    char* errMsg = nullptr;
    int rc = sqlite3_exec(db_own_, q.c_str(), nullptr, nullptr, &errMsg);
    err = errMsg;

  }














}
}
}
