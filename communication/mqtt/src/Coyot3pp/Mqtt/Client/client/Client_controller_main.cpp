#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace ct = coyot3::tools;

namespace coyot3::communication::mqtt{
  
  void Client::task_controller(){
    int64_t now = ct::get_current_timestamp();
    bool allsubscribed = true;
    switch(model.state()){
      case ec::MosqClientState::DISCONNECTED:
        allsubscribed = false;
        log_info("controller- : disconnected : invoking connection");
        full_reset_connection();
        break;
      case ec::MosqClientState::CONNECTING:
        if(now - model.ts_last_transition() > (config_.timeout() * 1000)){
          log_warn(o() << "controller- : pulse : connecting for too long("
          << config_.timeout() << "). forcing reconnection");
          model.ts_last_transition(now);

          full_reset_connection();
        }
        break;
      case ec::MosqClientState::CONNECTED:
        log_debug(8,o() << "controller- : pulse : connected");
        break;
      case ec::MosqClientState::ERROR:
        log_warn("controller- : error state : forcing reconnection");
        full_reset_connection();
        break;
    }

    if(ec::MosqClientState::CONNECTED != model.state()){
      return;
    }

    if(allsubscribed == false){
      make_subscriptions_();
      allsubscribed = all_subscriptions_are_done_();
    }

  }

  
}