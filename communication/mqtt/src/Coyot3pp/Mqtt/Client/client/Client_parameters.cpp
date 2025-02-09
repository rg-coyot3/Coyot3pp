#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace coyot3::communication::mqtt{

  int 
  Client::on_connection_callback_add(CallbackOnEventSimple cb){
    int id = 0;
    for(const CallbacksOnEventSimplePair& it : model.callbacks_on_connection()){
        id = std::max(id,it.first);
    }
    model.callbacks_on_connection().insert(std::make_pair(id,cb));
    log_info(o() << "on-connection-callback-add : added onConnection "
    "callback [" << id << "]");
    return id;
  }

  
  int 
  Client::on_disconnection_callback_add(CallbackOnEventSimple cb){
    int id = 0;
    for(const CallbacksOnEventSimplePair& it : model.callbacks_on_disconnection()){
        id = std::max(id,it.first);
    }
    model.callbacks_on_disconnection().insert(std::make_pair(id,cb));
    log_info(o() << "on-connection-callback-add : added onDisconnection "
    "callback [" << id << "]");
    return id;

  }



  int
  Client::on_error_callback_set(){
    // TO-DO
    return -1;
  }


}
