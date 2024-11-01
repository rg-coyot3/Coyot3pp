/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * */


#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>
namespace coyot3{
namespace communication{
namespace mqtt{

void MqttGateway::_debug_level_set_mod_callbacks(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
}
/**
 * When a priority message has been published, it has been stocked in the 
 *  priority messsages queue. After published, an ACK will be received from the
 *  broker with the ID of that message.
 * that message_id will be used to search at the priority messages queue the message
 * 
 * 
 * */
void MqttGateway::on_mosq_client_published(int message_id)
{
  bool  found = false;


  if((found = stack_priority_messages_primary.remove(message_id)) == true)
  {
    CLOG_DEBUG(3,"mqtt-gw : on-mosq-client-published-msg : correctly "
      "removed message id(" << message_id << ") from priority stack");
  }else{
    CLOG_DEBUG(5,"mqtt-gw : on-mosq-client-published-msg : received ack "
      "of the reception at the broker for message id(" << message_id << ")");
  }
  

  if(config.show_debug_msgs())
  if(!found)
  {
      CLOG_DEBUG(5,"mqtt-gw : on published : ACK received for message ID " << 
          message_id << " but no priority message was found with that ID");
  }
}

/**
 * @brief - internal
 * 
 * */

void MqttGateway::on_mosq_client_logs(int level,const char* msg)
{
  if(config.show_debug_msgs())
  {
      CLOG_INFO("mqtt-gw : client debug : mosq log msg level[" << level <<
          "] :: msg[" << msg << "]");
  }
  //pubrec
  //pubcomp
  //pingresp
  std::string m(msg);
  if(
    (m.find("PINGRESP") != std::string::npos)||
    (m.find("PUBREC")   != std::string::npos)||
    (m.find("PUBCOMP")  != std::string::npos)
  ){
    last_confirmed_activity_ts_ = coyot3::tools::get_current_timestamp();
  }

}

// /
void MqttGateway::on_mosq_client_subscribe(int message_id,
                                            int qos_count,
                                            const int* granted_qos)
{
  CLOG_INFO("mqtt-gw : onSubscription : subscription ok for the subscription "
      "ID [" << message_id << "]");
  return;
}
//
void MqttGateway::on_mosq_client_unsubscribe(int message_id)
{
  /* to do : 
      recover the topic unsubscription from the message id
  */
  CLOG_INFO("mqtt-gw : onUnsubscription : received ACK for unsubscription "
      "with the ID [" << message_id << "]");
  return;
}
int MqttGateway::on_mosq_key_file_passphrase_request(char* buff,
                                                    int size,
                                                    int rwflag)
{
  CLOG_INFO("mqtt-gw : encripted key file : Setting key pass phrase"
      " : size = " << size);
  // char *pw = pass_phrase.c_str();
  if(config.certificates_passphrase().size() < static_cast<unsigned int>(size))
  {
      strncpy(buff, config.certificates_passphrase().c_str()
      , config.certificates_passphrase().size());
          buff[config.certificates_passphrase().size()] = '\0';
  }
  else
  {
      strncpy(buff,config.certificates_passphrase().c_str(),
          size);
  }
  
  int l= strlen(buff);
  
  
  CLOG_INFO("mqtt-gw : Current connection passphrase length " << 
      config.certificates_passphrase() << " [" 
      << config.certificates_passphrase() << "]");

  CLOG_INFO("mqtt-gw : Passphrase to be used (max size=" << size <<
      "passSize= " << l << "[" << buff << "]");
  return strlen(buff);
}

/**
 * 
 * 
 * */
void MqttGateway::on_mosq_client_disconnects(int reason)
{
    
  CLOG_WARN("mqtt-gw : client has disconnected - ");
  this->gateway_state = GatewayState::DISCONNECTED;
  this->mosqclient_ts_last_disconnection = coyot3::tools::getCurrentTimestamp();
  bool expected_disconnection = (reason == 0);
  //invoking managers event
  std::for_each(on_disconnect_callbacks.begin(),on_disconnect_callbacks.end(),
      [&](OnDisconnectionCallback callback){
          callback(expected_disconnection);
      });
  
}
bool MqttGateway::on_mosq_message_search_relation(const std::string& in,  std::string& out){
  CLOG_INFO("mqtt-gw : on-mosq-message : searching relation for [" << in << "]");
  
  TopicSubscriptionsStack::iterator     icoll;
  TopicSubscriptionsRelations::iterator irels;
  
  for(irels = topic_subs_relations.begin(); irels != topic_subs_relations.end(); ++irels)
  {
    if(irels->first == irels->second){
      out = irels->second;
      return true;
    }
  }
  CLOG_INFO("mqtt-gw : on-mosq-message : creating new relation for [" << in << "]");
  for(icoll = topic_subs_collection.begin(); icoll != topic_subs_collection.end(); ++ icoll){
    if(icoll->first[icoll->first.size() - 1] != '#'){
      //not a wildcard
      CLOG_INFO("--debug not for [" << icoll->first << "]");
      continue;
    }
    std::string buff = icoll->first.substr(0, icoll->first.size()-1);

    if(in.find(buff) != 0){
      //not found
      continue;
    }
    CLOG_INFO("mqtt-gw : on-mosq-message : found match at [" << in << "] for [" 
      << icoll->first << "]. Adding at the relations stack.")
    
    topic_subs_relations.insert(std::make_pair(in, icoll->first));
    out = icoll->first;
    return true;
  }
  CLOG_WARN("mqtt-gw : on-mosq-message : unable to find mach from [" << in << "]");
  return false;
}

//
void MqttGateway::on_mosq_message(const char* t, const void* p, size_t ps)
{
    last_confirmed_activity_ts_ = coyot3::tools::get_current_timestamp();
    std::string topic = t;
    TopicSubscriptionsStack::iterator it;
    if(lock_interface_io() == true){
      CLOG_DEBUG(5,"mqtt-gw : on-mosq-message : activity is locked for this "
      "connector. Ignoring message received at topic [" << topic << "]")
      return;
    }
    it = topic_subs_collection.find(topic);
    if(it == topic_subs_collection.end())
    {
        CLOG_WARN("MQTT GW : no callback found for received message at " << 
            "topic [" << topic << "] searching relation!");
        std::string buffer;
        if(on_mosq_message_search_relation(topic,buffer) == false){
          CLOG_WARN("MQTT GW : on-mosq-message- : no solution for topic [" 
            << topic << "]");
          return;          
        }
        it = topic_subs_collection.find(buffer);
        if(it == topic_subs_collection.end()){
          CLOG_ERROR("mqtt-gw : on-mosq-message- : error! found solution for [" 
          << topic << "](" << buffer << ") does not give result at the current table!");
          return;
        }
    }
    //searching indexations
    
    std::for_each(it->second.begin(),it->second.end(),[&](OnMessageCallback cb){
            cb(topic,static_cast<const uint8_t*>(p),ps);
        });
}

/**
 * @brief : internal - will be invoked by the mosquitto lib instance
 * */
void MqttGateway::on_mosq_client_connects(int result)
{   

    if(mosqclient_full_reset_flag == true)
    {
        CLOG_INFO("mqtt-gw : on connect : first connection for current "
          "mosquitto client");
        make_mosq_subscriptions();
        republish_secondary_queue();
    }else{
      //protection against the case of fast broker-reboot : to test : short disconnection in case of bad tcp link
      topic_qos_rel_todo.clear();
      TopicQosRelations::iterator it;
      for(it = topic_qos_relations.begin();it != topic_qos_relations.end(); ++it)
      {
          topic_qos_rel_todo.insert(std::make_pair(it->first,it->second));
      }
      make_mosq_subscriptions();
    }
    
    mosqclient_full_reset_flag = false;



    this->gateway_state = GatewayState::CONNECTED;
    this->mosqclient_ts_last_connection = coyot3::tools::getCurrentTimestamp();

    switch(result)
    {
        case MOSQ_ERR_SUCCESS : 
            CLOG_INFO("mqtt-gw : client connected OK");
            this->gateway_state = GatewayState::CONNECTED;
            break;
        case 1 :
            CLOG_ERROR("mqtt-gw : CONNECTION REFUSED BY THE BROKER : "
                "unacceptable protocol version");
            this->gateway_state = GatewayState::DISCONNECTED;
            break;
        case 2 : 
            CLOG_ERROR("mqtt-gw : CONNECTION REFUSED BY THE BROKER : "
                "identifier rejected");
            this->gateway_state = GatewayState::DISCONNECTED;
            break;
        case 3 : 
            CLOG_ERROR("mqtt-gw : CONNECTION UNAVAILABLE : broker "
                "is unavailable");
            this->gateway_state = GatewayState::DISCONNECTED;
            break;
        case MOSQ_ERR_CONN_REFUSED:
            CLOG_ERROR("mqtt-gw : CONNECTION CONNECTION REFUSED! : broker "
              "refused connection.");
            this->gateway_state = GatewayState::DISCONNECTED;
        default :
            CLOG_ERROR("mqtt-gw : CONNECTION UNAVAILABLE : error code ["
                << result <<"]");
            this->gateway_state = GatewayState::DISCONNECTED;
    }
    CLOG_INFO("mqtt-gw : client connected. invoking callbacks for managers. "
        "Total = " << on_connect_callbacks.size());

    for(OnConnectFuncCollection::iterator it = on_connect_callbacks.begin();
        it != on_connect_callbacks.end() ; ++it)
    {
        CLOG_INFO("mqtt-gw : on connect : invoking callback");
        (*it)(static_cast<bool>(result == 0));
    }

}


}
}// end of namespace wrappers
}//end of namespace coyot3