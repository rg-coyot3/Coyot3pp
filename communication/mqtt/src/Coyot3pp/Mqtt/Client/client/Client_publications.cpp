#include <Coyot3pp/Mqtt/Client/Client.hpp>


namespace  cm = coyot3::mod;
namespace  ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{


bool 
Client::publish(
  const std::string& topic, 
  const uint8_t* payload, 
  std::size_t length, 
  ec::MessagePriorityLevel priority){
    return publish_(topic,payload,length,priority);
}
bool 
Client::publish(
  const std::string& topic, 
  const uint8_t* payload, 
  std::size_t length, 
  int qos){
    return publish_(topic,payload,length,qos);
}

bool 
Client::publish_(
  const std::string& topic, 
  const uint8_t* payload, 
  std::size_t length, 
  ec::MessagePriorityLevel priority){
    
    int mid;
    int qos;
    switch (priority)
    {
      case ec::MessagePriorityLevel::LOW: qos = 0; break;
      case ec::MessagePriorityLevel::MEDIUM: qos = 1; break;
      case ec::MessagePriorityLevel::HIGH: 
      case ec::MessagePriorityLevel::HIGH_WHEN_AVAILABLE: qos = 2; break;
      case ec::MessagePriorityLevel::DEFAULT: qos = config_.qos(); break;
    }    

    if(qos == 0){
      //nothing to do. do not follow even the message publication.
      return publish_(topic,payload,static_cast<int>(length),qos);
    }

    Message msg(topic,payload,length);

    if(
      publish_payload_v3_(topic,payload,static_cast<int>(length),qos,&mid) 
      == false){
      if(priority == ec::MessagePriorityLevel::HIGH_WHEN_AVAILABLE){
        // the user wants it to be sent when it is connected.
        message_stack_repub_.push_back(msg);
        return true;
      }
      //tell the user that it did not work
      return false;
    }

    msg.id(mid);
    msg.last_activity(ct::get_current_timestamp());
    msg.priority(priority);
    if(message_stack_prim_.is_member(mid) == true){
      log_warn(o() << "publish- : " << priority << " : mqtt message id already "
      "exists... removing old!");
      message_stack_prim_[mid] = msg;
    }else{
      //saved to check ack from broker.
      message_stack_prim_.insert(msg);
    }

    return true;
}

bool
Client::publish_(
  const std::string& topic, 
  const uint8_t* payload, 
  std::size_t length, 
  int qos){

    int mid;
    if(qos == 0){
      return publish_payload_v3_(topic,payload,static_cast<int>(length),qos,nullptr);
    }
    if(publish_payload_v3_(topic,
                          payload,
                          static_cast<int>(length),
                          qos,
                          nullptr) == false){
      return false;
    }
    Message msg(topic,payload,length);
    msg.id(mid);
    msg.last_activity(ct::get_current_timestamp());
    ec::MessagePriorityLevel l;
    switch (qos)
    {
    case 1: msg.priority(ec::MessagePriorityLevel::MEDIUM);break;
    case 2: msg.priority(ec::MessagePriorityLevel::HIGH);break;
    
    default:
      break;
    }
    if(message_stack_prim_.is_member(mid) == true){
      log_warn(o() << "publish- : qos=" << qos << " : mqtt message id already "
      "exists... removing old!");
      message_stack_prim_[mid] = msg;
    }else{
      //saved to check ack from broker.
      message_stack_prim_.insert(msg);
    }
    return true;
}



bool 
Client::publish_payload_v3_(
  const std::string& t, const uint8_t* p, int s, int q, int* mid
){
  std::lock_guard<std::mutex> guard(client_tx_mtx_);
  if(model.state() != ec::MosqClientState::CONNECTED){
    //not connected state
    return false;
  }
  if(client_ == nullptr){
    //redundant control... at this point it should never be null, but if it 
    // happens, then it is an error
    log_err("publish-payload- : trying to publish but mosquitto-client-instance"
    " is null!");
    model.state(ec::MosqClientState::ERROR);
    return false;
  }

  // retain last message if qos is not zero.
  int r = mosquitto_publish(client_, mid, t.c_str(), s, p, q, (q == 2));
  if(r != MOSQ_ERR_SUCCESS){
    log_warn(o() << "error publishing msg for topic [" << t 
    << "] size(" << s << ") : reason [" << mosq_rc_stringify(r) << "]");
  }
  return (r == MOSQ_ERR_SUCCESS);
}


void
Client::on_mosq_client_published(int message_id){
  if(message_stack_prim_.is_member(message_id) == false){
    //nothing to do.
    return;
  }
  log_debug(5,o() << "on-mosq-msg-published- : ");
  message_stack_prim_.remove(message_id);
}

void Client::backup_stack_prim_to_repub_(){
  if(message_stack_prim_.size() == 0)return;
  log_debug(1,"backup-stack-prim-to-repub-");
  message_stack_prim_.for_each([&](const Message& m){
    log_debug(3,"backup-stack-prim-to-repub- : backing up for repub msg (t=" <<
    m.topic() << ",p=" << m.payload_stringify() << ")");
    message_stack_repub_.push_back(m);
    return true;
  });
}


}
}
}
