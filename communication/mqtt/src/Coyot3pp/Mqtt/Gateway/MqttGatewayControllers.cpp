/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * */

#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>
namespace coyot3{
namespace communication{
namespace mqtt{



void MqttGateway::_debug_level_set_mod_controllers(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
}
void MqttGateway::priority_messages_controller_pulse()
{
    int pub_result;
    //std::lock_guard<std::mutex> guard(mtx_republish_queue);
    // if(queue_priority_messages_primary.empty())
    // {
    //     if(mosquitto_client_debug)
    //     CLOG_INFO("mqtt-gw : prior msgs republish : queue is empty");
    //     //nothing to do.
    //     return;
    // }

    if(stack_priority_messages_primary.empty())
    {
      CLOG_DEBUG(5,"mqtt-gw : priority messages controller pulse : stack is empty");
      return;
    }
    
    std::vector<int> toDelete;
    int64_t nowTS = coyot3::tools::get_current_timestamp(),diff;
    MqttMessagesStack updater;
    if(gateway_state != GatewayState::CONNECTED)
    {
      CLOG_DEBUG(3,"mqtt-gw : priority-messages-controller-pulse : gw is not connected");
      return;
    }

    stack_priority_messages_primary.forEach([&](MqttMessage& msg){
      diff = nowTS - msg.creation_time;
      if(diff < config.important_messages_timeout_ms())
      {
        //nothing to do with this message
        return false;
      }
      toDelete.push_back(msg.messageID);
      if(msg.retries > config.important_messages_max_retries())
      {
        //nothing to do... just delete the message... and inform of error?
        if(this->user_callback_on_priority_message_send_error)
        {
          CLOG_DEBUG(2,"mqtt-gw : priority-messages-controller-pulse : "
            "exceeded number of retries to send message to topic [" 
            << msg.topic << "]. Invoking user callback.");
          this->user_callback_on_priority_message_send_error(msg.topic,msg.payload,msg.size);
        }
        return false;
      }
      //create new msg
      MqttMessage newMsg(msg);
      newMsg.setCreationTimestampNow();
      newMsg.incrementRetries();
      
      //---------- guard---------------
      {
        std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
        pub_result = mosquitto_publish(
          client,
          &newMsg.messageID,
          newMsg.topic.c_str(),
          newMsg.size,
          static_cast<const void*>(newMsg.payload),
          0,false);
      }
      //---------- guard---------------
      updater.push(newMsg);

      switch(pub_result){
        case MOSQ_ERR_SUCCESS:
          if(config.show_debug_msgs())
          CLOG_INFO("mqtt-gw : priority-messages-controller-pulse : "
              "reports success for current message ID [" 
              << newMsg.messageID << "]");
          break;
        case MOSQ_ERR_INVAL:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : WARN "
              "- invalid input params to send a stream at topic :" <<
              newMsg.topic);
          break;
        case MOSQ_ERR_NO_CONN:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : "
            "no connection to mqtt broker to send stream at topic: " 
            << newMsg.topic);
          break;
        case MOSQ_ERR_PROTOCOL:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : PROTOCOL "
            "ERROR COMMUNICATING with broker while sending payload at topic :"
              << newMsg.topic);
            break;
        case MOSQ_ERR_PAYLOAD_SIZE:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : "
            "ERROR! message to big (size=" << newMsg.size << ") at topic: " 
            << newMsg.topic);
          break;
        case MOSQ_ERR_MALFORMED_UTF8:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : "
            "ERROR! invalid UTF-8 descriptor to send message to topic: " << 
              newMsg.topic);
          break;
        default:
          CLOG_WARN("mqtt-gw : priority-messages-controller-pulse : "
            "unhandled error while publishing a payload");
      }
      return true;
    });   

    for(int id : toDelete)
    {
      //removing id...
      stack_priority_messages_primary.remove(id);
    }
    stack_priority_messages_primary+=updater;
    
   
}

bool MqttGateway::queue_controller()
{   
    if(queue_controller_thread != nullptr)
    {
        //
        CLOG_WARN("mqtt-gw : controller launch : the queue controller "
            "thread is already launched for this gateway!");
        if(queue_controller_keepalive_flag == false){
            queue_controller_thread->join();
            delete queue_controller_thread;
            queue_controller_thread = nullptr;
        }else{
            CLOG_ERROR("mqtt-gw : controller launch : the gateway "
                "controller is not able to be stopped");
            return false;    
        }   
    }

    queue_controller_thread = new(std::nothrow) std::thread([&](){
        CLOG_INFO("mqtt-gw : queue controller : launched OK");

        queue_controller_keepalive_flag = true;

        while(queue_controller_keepalive_flag){   
            sleep(MQTT_GATEWAY_INTERNAL_QUEUE_CONTROLLER_PULSE_SECONDS); 
            priority_messages_controller_pulse();
        }
        CLOG_INFO("mqtt-gw : queue controller : ended!");
    });

    if(queue_controller_thread== nullptr){
        CLOG_ERROR("mqtt-gw : queue controller launch : ERROR LAUNCHING "
          "CONTROLLER");
        return false;
    }

    CLOG_INFO("mqtt-gw : queue controller launch ended");
    return true;
   
}

bool MqttGateway::gateway_controller()
{
    if(gateway_controller_thread != nullptr){
        //
        CLOG_WARN("mqtt-gw : controller launch : the gateway controller "
            "thread is already launched for this gateway");
        if(gateway_controller_keepalive_flag == false)
        {
            gateway_controller_thread->join();
            delete gateway_controller_thread;
            gateway_controller_thread = nullptr;
        }
        else
        {
            CLOG_ERROR("mqtt-gw : controller launch : the gateway "
                "controller is not able to be stopped");
            return false;    
        }
    }
    
    gateway_controller_thread = new(std::nothrow) std::thread([&](){
        CLOG_INFO("mqtt-gw : gateway controller : launched OK");
        gateway_controller_keepalive_flag = true;

        int counter_connected = 0;
        int counter_disconnected = 0;
        int counter_connecting = 0;
        int loop_counter = 0;

        int64_t current_ts = coyot3::tools::get_current_timestamp();
        int64_t buffernum;
 
        while(gateway_controller_keepalive_flag)
        {   
            sleep(1);
            ++loop_counter;
            current_ts = coyot3::tools::get_current_timestamp();
            if((current_ts - last_confirmed_activity_ts_) > (config.timeout() * 1000)){
              CLOG_WARN("mqtt-gw : controller : NO ACTIVITY FOUND AFTER [" << 
              config.timeout() << "] seconds (" 
              << (current_ts - last_confirmed_activity_ts_) << " msecs) "
              "UNDETECTED DISCONNECTION?. FORCING REBOOT.")
              gateway_state = GatewayState::DISCONNECTED;
              mosqclient_ts_last_disconnection = current_ts - (mosqclient_disconnected_timeout_secs * 2000);
            }
            if(gateway_state!=GatewayState::CONNECTED){
              CLOG_INFO("mqtt-gw : controller : PULSE [" << 
                loop_counter << "] GW state : [" 
                << MqttGateway::GatewayStateToString(gateway_state)<<"]");
            }
            switch(gateway_state)
            {
                case GatewayState::CONNECTED:
                    if(counter_connected == 0)
                    {
                        counter_connected++;
                        counter_disconnected = 0;
                        counter_connecting = 0;
                        pulse_mosq_subscribe();
                    }
                    break;
                case GatewayState::DISCONNECTED:
                    buffernum = (current_ts  - mosqclient_ts_last_disconnection) / 1000;
                    if(buffernum > mosqclient_disconnected_timeout_secs)
                    {
                      CLOG_WARN("mqtt-gw : controller : Gateway "
                        "disconnected for too long [more than " << 
                        mosqclient_disconnected_timeout_secs 
                        << "] MOSQUITTO CLIENT FULL RESET" );
                      counter_connected = counter_disconnected 
                        = counter_connecting = loop_counter = 0;
                      full_reset_connection();
                    }
                    break;
                case GatewayState::CONNECTING:
                    ++counter_connecting;
                    counter_disconnected = 0;

                    if(counter_connecting > 10)
                    {
                      CLOG_WARN("mqtt-gw : controller : Gateway connecting "
                      "for too long (after 10 pulses cycles), now rebooting "
                      "the mosquitto client");
                      counter_connected = counter_disconnected = counter_connecting = loop_counter = 0;
                      full_reset_connection();
                    }
                    break;
                default:
                    CLOG_INFO("mqtt-gw : controller : case default");

            }
        }
        CLOG_INFO("mqtt-gw : controller : ended!");
    });
    if(gateway_controller_thread == nullptr)
    {
        CLOG_ERROR("mqtt-gw : controller launch : ERROR LAUNCHING CONTROLLER")
        return false;
    }

    CLOG_INFO("mqtt-gw : gateway controller : launched OK")
    return true;

}


}
}
}// end of namespace ac3s_ascs_communications