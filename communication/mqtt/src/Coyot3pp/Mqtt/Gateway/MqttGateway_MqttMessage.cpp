/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * 
 * @author : Ricardo GONZALEZ - Phong PHAM
 * @email : ricardogonalm@ltlab.net 
 *  
 * GPLv2+
 * */


#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>




namespace coyot3{
namespace communication{
namespace mqtt{


void 
MqttGateway::_debug_level_set_mod_msg(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
}

MqttGateway::MqttMessage::MqttMessage()
:messageID(0)
,creation_time(0)
,topic()
,payload(nullptr)
,size(0)
,prio(MqttGateway::Priority::DEFAULT)
,retries(0)
{
    CLOG_INFO("MQTT MESSAGE ::: CONSTRUCTOR COPY INVOKED");
    this->creation_time = coyot3::tools::getCurrentTimestamp();
}

MqttGateway::MqttMessage::MqttMessage(const MqttGateway::MqttMessage& o)
{
  payload = nullptr;
  CLOG_DEBUG(7,"mqtt-gw-msg : constructor (cont mqtt-message& o)");
  *this = o;
}

MqttGateway::MqttMessage::MqttMessage(MqttGateway::MqttMessage& o)
{
  payload = nullptr;
  CLOG_DEBUG(7,"mqtt-gw-msg : constructor (mqtt-message& o)");
  *this = o;
}

MqttGateway::MqttMessage::MqttMessage(int pmsgid
                                      ,const std::string& ptopic
                                      , const uint8_t* pptr
                                      ,size_t psize
                                      ,Priority pprio)
:messageID(pmsgid)
,creation_time(0)
,topic(ptopic)
,payload(nullptr)
,size(0)
,prio(pprio)
,retries(0)
{
  
  if(!setPayload(pptr,psize))
  {
    CLOG_ERROR("mqtt-gw-msg : constructor-from-params : ERROR "
      "allocating string to store payload. MAY BE FATAL!");
  }
}
MqttGateway::MqttMessage::~MqttMessage()
{
    CLOG_DEBUG(7,"mqtt-gw-msg : destroying message id [" << messageID << "]");

    if(this->payload!=nullptr)
    {
        delete this->payload;
    }
}



MqttGateway::MqttMessage& 
MqttGateway::MqttMessage::operator=(const MqttGateway::MqttMessage& o)
{

  CLOG_DEBUG(5,"mqtt-gw-msg : copy (const msg& o)");
  this->creation_time = o.creation_time;
  if(payload != nullptr)
  {
    CLOG_DEBUG(8,"mqtt-gw-msg : copy (const msg& o)  deleting previous "
      "loaded payload");
    delete payload;
    payload = nullptr;
  }
  this->payload = new(std::nothrow) uint8_t[o.size];
  if(payload== nullptr)
  {   
      CLOG_ERROR("mqtt-gw-msg : COPY CONSTRUCTOR ! NO ENAUGH MEMORY TO "
        "ALLOCATE STREAM");
  }
  else
  {
      memcpy(this->payload,o.payload,o.size);
  }
  
  this->size = o.size;
  this->prio = o.prio;
  this->messageID = o.messageID;
  this->topic = o.topic;
  this->retries = o.retries;


  return *this;
}

MqttGateway::MqttMessage& 
MqttGateway::MqttMessage::operator=(MqttGateway::MqttMessage& o)
{

  CLOG_DEBUG(5,"mqtt-gw-msg : copy (const msg& o)");
  this->creation_time = o.creation_time;
  if(payload != nullptr)
  {
    delete payload;
    payload = nullptr;
  }
  this->payload = new(std::nothrow) uint8_t[o.size];
  if(payload== nullptr)
  {   
      CLOG_ERROR("mqtt-gw-msg : COPY CONSTRUCTOR ! NO ENAUGH MEMORY TO "
        "ALLOCATE STREAM");
  }
  else
  {
      memcpy(this->payload,o.payload,o.size);
  }
  
  this->size = o.size;
  this->prio = o.prio;
  this->messageID = o.messageID;
  this->topic = o.topic;
  this->retries = o.retries;


  return *this;
}


bool 
MqttGateway::MqttMessage::setPayload(const uint8_t* pptr, size_t psize)
{
  if(payload != nullptr)
  {
    delete payload;
    payload = nullptr;
  }
  payload = new(std::nothrow) uint8_t[psize];
  if(!payload)
  {
    CLOG_ERROR("mqtt-gw-msg : ERROR allocating memory for message content!");
    return false;
  }
  memcpy(payload,pptr,psize);
  return true;
}

void 
MqttGateway::MqttMessage::setCreationTimestampNow()
{
  creation_time = coyot3::tools::getCurrentTimestamp();
}

void 
MqttGateway::MqttMessage::incrementRetries()
{
  ++retries;
}

MqttGateway::MqttMessagesStack::MqttMessagesStack()
:msg_map()
{

}
MqttGateway::MqttMessagesStack::MqttMessagesStack(const MqttMessagesStack& o)
{
  *this = o;
}
MqttGateway::MqttMessagesStack::~MqttMessagesStack()
{

}

MqttGateway::MqttMessagesStack& 
MqttGateway::MqttMessagesStack::operator=(const MqttGateway::MqttMessagesStack& o)
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  msg_map.clear();
  msg_map = o.msg_map;
  return *this;
}

MqttGateway::MqttMessagesStack& 
MqttGateway::MqttMessagesStack::operator+=(const MqttGateway::MqttMessagesStack& o)
{
  o.forEach([&](const MqttMessage& m){
    return push(m);
  });
  return *this;
}

MqttGateway::MqttMessagesStack& 
MqttGateway::MqttMessagesStack::operator+=(const MqttGateway::MqttMessage& o)
{
  push(o);
  return *this;
}

size_t 
MqttGateway::MqttMessagesStack::size() const
{
  return msg_map.size();
}
void 
MqttGateway::MqttMessagesStack::clear()
{
  msg_map.clear();
}

size_t 
MqttGateway::MqttMessagesStack::size()
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  return msg_map.size();
}

bool 
MqttGateway::MqttMessagesStack::empty() const
{
  return msg_map.empty();
}

bool 
MqttGateway::MqttMessagesStack::push(const MqttMessage& m)
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  MqttMessagesMapIterator mi;
  if((mi = msg_map.find(m.messageID)) != msg_map.end())
  {
    return false;
  }
  msg_map.insert(std::make_pair(m.messageID,m));
  return true;
}

bool 
MqttGateway::MqttMessagesStack::push(const MqttMessagesStack& o)
{
  size_t pushed = o.forEach([&](const MqttGateway::MqttMessage& m){
    return push(m);
  });
  return (pushed == o.size());
}

bool 
MqttGateway::MqttMessagesStack::exists(const MqttMessage& o)
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  return (msg_map.find(o.messageID) != msg_map.end());
}

bool 
MqttGateway::MqttMessagesStack::exists(int messageId)
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  return (msg_map.find(messageId) != msg_map.end());
}

bool 
MqttGateway::MqttMessagesStack::remove(const MqttMessage& m)
{
  return remove(m.messageID);
}
bool 
MqttGateway::MqttMessagesStack::remove(int m)
{
  std::lock_guard<std::mutex> guard(stack_mtx_);
  MqttMessagesMapIterator mit;
  if((mit = msg_map.find(m)) == msg_map.end())
  {
    return false;
  }
  msg_map.erase(mit);
  return true;
}

size_t 
MqttGateway::MqttMessagesStack::forEach(std::function<bool(MqttMessage& m)>func)
{
  size_t res = 0;
  std::lock_guard<std::mutex> guard(stack_mtx_);
  for(MqttMessagesMapIterator it = msg_map.begin();it != msg_map.end();++it)
  {
    res+=static_cast<size_t>(func(it->second));
  }
  return res;
}
size_t
MqttGateway::MqttMessagesStack::forEach(std::function<bool(const MqttMessage& m)>func) const
{
  size_t res = 0;
  for(MqttMessagesMapConstIterator it = msg_map.begin();it != msg_map.end();++it)
  {
    res+=static_cast<size_t>(func(it->second));
  }
  return res;
}


}//end of namespace mqtt
}//end of namespace communication
}//end of namespace coyot3;


