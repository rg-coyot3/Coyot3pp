#include <Coyot3pp/Mqtt/Client/Client.hpp>


namespace ct = coyot3::tools;
namespace coyot3::communication::mqtt{


  bool Client::init_(){
    
    th_main_ = new(std::nothrow) ct::ControlThread(
      std::bind(&Client::task_controller,this),
      "controller-" + name());
    if(th_main_ == nullptr){
      log_err("init- : fatal error reserving memory to controller task thread");
      return false;
    }
    th_main_->setInterval(CONTROLLER_INTERVAL_NORMAL);
    log_debug(3,"init- : created main controller thread instance");
    return true;
  }

  bool Client::start_(){
    bool r = th_main_->start();
    log_eval(r,
      "start- : controller thread launched",
      "start- : error launching controller thead!");
    return r;
  }

  bool Client::pause_(){
    log_info("pause- : pausing connector activities");
    return true;
  }

  bool Client::stop_(){
    bool r = th_main_->stop();
    log_eval(r,
      "stop- : controller thread stopped",
      "stop- : error stopping controller thread!");
    log_eval(disconnect_from_broker_(),
      "stop- : disconnected from broker",
      "stop- : error disconnecting from broker!");
    return r;
  }

  bool Client::end_(){
    log_warn("end- : freeing resources");
    delete th_main_;
    th_main_ = nullptr;
    disconnect_from_broker_();
    message_stack_repub_.clear();
    message_stack_prim_.clear();
    return true;
  }


}