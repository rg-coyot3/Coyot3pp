#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace ct = coyot3::tools;
namespace coyot3{
namespace communication{
namespace mqtt{

  void Client::task_controller(){

    switch(client_state_){
      case ec::MosqClientState::DISCONNECTED:
        
        break;
      case ec::MosqClientState::CONNECTING:

        break;
      case ec::MosqClientState::CONNECTED:

        break;
      case ec::MosqClientState::ERROR:

        break;
    }

    if(ec::MosqClientState::CONNECTED != client_state_){
      return;
    }

    if(subscriptions_to_do_.size()){
      th_main_->setInterval(CONTROLLER_INTERVAL_INTENSIVE);
      make_subscriptions_();
    }else{
      th_main_->setInterval(CONTROLLER_INTERVAL_NORMAL);
    }

  }

  
  bool Client::make_subscriptions_(){
    std::lock_guard<std::mutex> guard(client_tx_mtx_);
    Subscription current = subscriptions_to_do_.first();
    Subscription& source = subscriptions_config_.get(current.topic());
    int subscriptionId;
    int rc = mosquitto_subscribe(client_,&subscriptionId,
                                 current.topic().c_str(),
                                 current.mosquitto_qos());
    switch(rc){
      
    }

    source.ts_connection_launch(ct::get_current_timestamp());
    source.mosquitto_subid(subscriptionId);
  }

}
}
}