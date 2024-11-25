#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace coyot3{
namespace communication{
namespace mqtt{


  ClientConfiguration& Client::config(const ClientConfiguration& configuration){
    config_ = configuration;
    return config_;
  }

  const ClientConfiguration& Client::config() const{
    return config_;
  }

  bool 
  Client::register_subscription(const std::string& topic,
                                int mqttQos,
                                int64_t& id,
                                MqttClientOnMessageCallback cb){
    Subscription sub;
    if(model.subscriptions_config().is_member(topic)== true){
      log_warn(o() << "register-subscription- : topic is already registered : ["
      << topic << "]");
      return false;
    }
    id = 0;
    model.subscriptions_config().for_each([&](const Subscription& s){
      id = std::max(id,s.id());
      return true;
    });
    id = sub.id(id + 1);
    sub.topic(topic);
    sub.active(true);
    sub.mosquitto_qos(mqttQos);
    sub.callback = cb;
    
    model.subscriptions_config().insert(sub);
    log_info(o() << "register-subscription- : registered [" << id << "]["
    << topic << "]");
    return true;
  }

  bool 
  Client::register_publisher(const std::string& topic,
                                 int mqttQos,int64_t& id){
    if(model.publishers_config().is_member(topic) == true){
      log_warn(o() << "register-subscription- : publisher already registered : "
      "[" << topic << "]");
      return false;
    }
    Publisher pub;
    id = 0;
    model.publishers_config().for_each([&](const Publisher& p){
      id = std::max(id,p.id());
      return true;
    });
    
    id = pub.id(id + 1);
    pub.active(true);
    pub.topic(topic);
    pub.mosquitto_qos(mqttQos);
    model.publishers_config().insert(pub);
    log_info(o() << "register-publisher- : registered [" << id << "][" 
    << topic << "]");

    return true;
  }



  int 
  Client::on_connection_callback_add(CallbackOnEventSimple cb){
    int id = 0;
    for(const CallbackOnEventSimplePair& it : model.callbacks_on_connection()){
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
    for(const CallbackOnEventSimplePair& it : model.callbacks_on_disconnection()){
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
}
}
