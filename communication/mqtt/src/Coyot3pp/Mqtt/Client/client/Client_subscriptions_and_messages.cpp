#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace ct = coyot3::tools;
namespace coyot3::communication::mqtt{

void Client::on_mosq_client_message(const struct mosquitto_message* message){
  std::string topic(message->topic);
  log_debug(5, o() << "on-msg-received : for topic [" << topic << "]");
  if(model.subscriptions_active().is_member(topic) == true){
    model.subscriptions_active().get(topic).subs().for_each([&](Subscription& s){
      s.ts_last_msg(ct::get_current_timestamp());
      s.msg_count()++;
      s.callback()(topic,(uint8_t*)message->payload,message->payloadlen);
      return true;
    });
    return;
  }

  //searches the reason...
  log_debug(3, o() << "on-msg-received : for topic [" << topic << "] : "
  "not currently active. Searching root.");
  model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& tr){
    std::size_t p;
    if((p = tr.topic().find_first_of('#')) == std::string::npos)return false;
    if(p == 0){//subscription to the root
      cmqc_subscriptions_tree ntr(tr);
      ntr.topic(topic);
      model.subscriptions_active().insert(ntr);
      return true;
    }
    std::string trtopicb = tr.topic().substr(0,p);
    if(topic.find(trtopicb) != 0)return false; //not included
    //this root is the one searched.
    cmqc_subscriptions_tree ntr(tr);
    ntr.topic(topic);
    model.subscriptions_active().insert(ntr);
    return true;
  });
  //SHOULD BE registered at active stack... so the callbacks are invoked.
  if(model.subscriptions_active().is_member(topic) == true){
    model.subscriptions_active().get(topic).subs().for_each([&](Subscription& s){
      s.ts_last_msg(ct::get_current_timestamp());
      s.msg_count()++;
      s.callback()(topic,(uint8_t*)message->payload,message->payloadlen);
      return true;
    });
    return;
  }
}



  int __subscription_id_patch = 0;

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
                                MqttClientOnMessageCallback cb){
    log_debug(5,o() << "register-subscription : no-reg-id : [" << topic << "]");
    int64_t id;
    return register_subscription(topic,mqttQos,id,cb);
  }
  
  bool 
  Client::register_subscription(const std::string& topic,
                                int mqttQos,
                                int64_t& id,
                                MqttClientOnMessageCallback cb){
    Subscription sub;
    bool         taskDone = false;
    log_info(o() << "register-subscription : topic [" << topic << "] : begin");
    
    sub.topic(topic);
    sub.mosquitto_qos(mqttQos);
    sub.callback(cb);
    sub.active(true);
    id = sub.id(__subscription_id_patch++);

    if(model.subscriptions_config().is_member(topic)){
      log_info(o() << "register-subscription : topic [" << topic << "] : "
        "adding callback to existing topic.");
      model.subscriptions_config().get(topic).subs().insert(sub);
      log_info(o() << "register-subscription : topic [" << topic << "] : "
        "adding callback to existing topic. Total callbacks = [" << 
        model.subscriptions_config().get(topic).subs().size() << "]");
      return true;
    }

    std::string topicbuf;
    bool inputIsRoot = false;
    if(topic.find_first_of('#') != std::string::npos){
      topicbuf = topic.substr(0,topic.find_first_of('#'));
      inputIsRoot = true;
    }else{
      topicbuf = topic;
    }
    
    //searching if incoming subscription is already included in some other sub
    model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& tt){
      std::string ttp;std::size_t s;
      if((s = tt.topic().find_first_of('#')) == std::string::npos)return false;
      ttp = tt.topic().substr(0,s);
      if(topicbuf.find(ttp) == 0){
        //incoming is included in existent
        log_info(o() << "register-subscription : topic [" << topic << "] : "
        "registering at same stack as [" << tt.topic() << "]");
        tt.subs().insert(sub);
        taskDone = true;
        return true;
      }
      return false;
    });
    if(taskDone == true){
      return true;
    }


    //checking if incoming subscription includes any other existent
    if(inputIsRoot){
      std::string existentIncluded("");
      model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& tt){
        if(existentIncluded.size() != 0)return false;
        std::string ttp;std::size_t s;
        if((s = tt.topic().find_first_of('#')) == std::string::npos){
          ttp = tt.topic(); //no wildcard
        }else{
          ttp = tt.topic().substr(0,s); // has wildcard
        }
        
        if(ttp.find(topicbuf) == 0){
          //incoming includes existent
          log_info(o() << "register-subscription : topic [" << topic << "] : "
          "found existent topic that is included in this new subscription . "
          " will be ported from [" << tt.topic() << "]");
          existentIncluded = tt.topic();
          taskDone = true;
          return true;
        }
        return false;
      });
      if(existentIncluded.size() != 0){
        cmqc_subscriptions_tree tr(model.subscriptions_config().get(existentIncluded));
        tr.topic(topic);
        tr.mosquitto_qos(std::max(mqttQos,tr.mosquitto_qos()));
        tr.subs().insert(sub);
        model.subscriptions_config().remove(existentIncluded);
        model.subscriptions_config().insert(tr);
        log_info(o() << "register-subscription : topic [" << topic << "] : "
        "removed [" << existentIncluded << "] in favor of new one");
      }
    }
    if(taskDone == true)return true;

    //no equal, no included in other, this one does not include other... then
    // it is a new one
    cmqc_subscriptions_tree tr;
    tr.topic(topic); //with the wildcard if exists
    tr.topic_index(topic);
    tr.active(true);
    tr.subscribed(false);
    tr.mosquitto_qos(mqttQos);
    tr.ts_connection_launch(0);
    tr.ts_connection_done(0);
    tr.msg_count(0);
    tr.subs().insert(sub);
    model.subscriptions_config().insert(tr); //inserting new one.
    log_info(o() << "register-subscription : topic [" << topic << "] created "
    "tree.");
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


  bool Client::all_subscriptions_are_done_(){
    bool allgood = true;
    model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& s){
      allgood &= s.subscribed();
      return true;
    });
    return allgood;
  }

  bool Client::make_subscriptions_(){
    std::lock_guard<std::mutex> guard(client_tx_mtx_);
    int64_t now = ct::get_current_timestamp();
    bool opdone = false;
    bool topics_to_subscribe = false;
    size_t total_ops;
    bool all_subscribed = true;
    total_ops = model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& s){
      all_subscribed &= s.subscribed();
      if(s.subscribed() == true)return false;

      //if(s.ts_connection_done() != 0){return false;}
      if((now - s.ts_connection_launch()) < (5000 * 1000)){
        //last try to connect was earlier than 5 seconds... wait for potential
        // response
        return false;
      }
      int subscriptionId;
      int rc = mosquitto_subscribe(client_,
                                  &subscriptionId,
                                  s.topic().c_str(),
                                  s.mosquitto_qos());
      if(rc != MOSQ_ERR_SUCCESS){
        log_warn(o() << "make-subscription- : error making subscription for "
        "topic [" << s.topic() << "]. Reason=" << mosq_rc_stringify(rc));
        return false;
      }
      s.ts_connection_launch(ct::getCurrentTimestamp());
      s.mosquitto_subid(subscriptionId);
      log_debug(5,o() << "make-subscription- : obtained mosquitto-sub-id [" 
      << s.mosquitto_subid() << "] for topic [" 
      << s.topic() << "]");
      return opdone = true;
    });
    

    return true;
  }

  void 
  Client::on_mosq_client_subscribe(int message_id,
                                   int qos_count, 
                                   const int* granted_qos){
    std::size_t r;
    r = model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& s){
      if(s.mosquitto_subid() != message_id){
        return false;
      }
      s.subscribed(true);
      s.ts_connection_done(ct::get_current_timestamp());
      log_debug(1,o() << "on-mosq-client-subscribe- : subscription id [" 
      << message_id << "] recognized for topic [" << s.topic() << "]. Current "
      "total subscriptions = " << qos_count);
      return true;
    });
    if(r == 0){
      log_warn(o() << "on-mosq-client-subscribe- : subscription confirmed with "
      "mid = [" << message_id << "] but it is not recognize any attempt to "
      "make the subscriptions with that ID");
    }
  }

  bool 
  Client::prepare_subscriptions_(){
    log_debug(3,"connection-to-broker- : clearing active subscriptions map");
    model.subscriptions_active().clear();
    model.subscriptions_config().for_each([&](cmqc_subscriptions_tree& s){
      s.ts_connection_launch(0);
      s.ts_connection_done(0);
      s.subscribed(false);
      return true;
    });
    return true;
  }



  bool 
  Client::full_reset_connection_(){
    std::lock_guard<std::mutex> guard1(client_tx_mtx_);
    std::lock_guard<std::mutex> guard2(client_models_mtx_);
    message_stack_prim_.for_each([&](const Message& m){
      message_stack_repub_.push_back(m);
      return true;
    });
    message_stack_prim_.clear();

    disconnect_from_broker_();
    return true;
  }

}