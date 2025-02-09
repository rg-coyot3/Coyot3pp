#include <Coyot3pp/Mqtt/Client/message/Message.hpp>

namespace ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{



  CYT3MACRO_enum_class_definitions(
    MessagePriorityLevel
    ,
    , LOW
    , MEDIUM
    , HIGH
    , HIGH_WHEN_AVAILABLE
    , DEFAULT
  )



CYT3MACRO_model_class_definitions(
  Message
  ,
  , ( 
    void gents_now(),
    void activity_ts_now(),
    std::vector<uint8_t>& payload(const uint8_t* p COMMA() std::size_t s),
    std::vector<uint8_t>& payload(const std::string& p),
    std::string payload_stringify() const,
    std::basic_string<uint8_t> payload_bstringify() const,
    Message(const std::string& topic COMMA() const uint8_t* p COMMA() std::size_t s),
    Message(const std::string& topic COMMA() const std::string& p),
    std::string to_string()
  )
  , ( )
    , id            , int64_t               , 0
    , creation_time , int64_t               , 0
    , last_activity , int64_t               , 0
    , topic         , std::string           , 
    , payload       , std::vector<uint8_t>  , 
    , priority      , ec::MessagePriorityLevel  , ec::MessagePriorityLevel::DEFAULT
    , retries       , int                   , 0
    , token         , int                   , 0
)



  CYT3MACRO_model_class_set_mapped_definitions(Message, id)
  CYT3MACRO_model_class_set_stack_definitions(Message, )


  Message::Message(const std::string& t, const uint8_t* p, std::size_t s){
    topic(t);
    payload(p,s);
    gents_now();
    retries(0);
    token(0);
  }
  Message::Message(const std::string& t, const std::string& p){
    topic(t);
    payload(p);
    gents_now();
    retries(0);
    token(0);
  }


  void Message::gents_now(){
    creation_time(ct::get_current_timestamp());
  }
  void Message::activity_ts_now(){
    last_activity(ct::get_current_timestamp());
  }

  std::vector<uint8_t>& Message::payload(const uint8_t* p , std::size_t s)
  {
    payload_.clear();
    for(std::size_t i = 0;i < s;++i){
      payload_.push_back(p[i]);
    }
    return payload_;
  }
  
  std::vector<uint8_t>& Message::payload(const std::string& p)
  {
    payload_.reserve(p.size());
    payload_.assign(p.begin(),p.end());
    
    return payload_;
  }

  std::string Message::payload_stringify() const {
    std::string str;
    
    for(const uint8_t& c: payload_){
      str.push_back(c);
    }
    return str;
  }
  std::basic_string<uint8_t> Message::payload_bstringify() const {
    std::basic_string<uint8_t> str;
    
    for(const uint8_t& c: payload_){
      str.push_back(c);
    }
    return str;
  }
  

  
  


}
}
}


