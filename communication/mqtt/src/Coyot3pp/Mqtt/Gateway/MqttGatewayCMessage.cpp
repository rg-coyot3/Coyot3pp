#include <Coyot3pp/Mqtt/Gateway/MqttGatewayCMessage.hpp>

namespace ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{


  CYT3MACRO_enum_class_definitions(
    CMessagePriority
    ,
    , LOW
    , MEDIUM
    , HIGH
    , DEFAULT
  )



CYT3MACRO_model_class_definitions(
  CMqttMessage
  ,
  , ( 
    void gents_now(),
    void activity_ts_now(),
    std::vector<uint8_t> payload(const uint8_t* p COMMA() std::size_t s),
    std::vector<uint8_t> payload(const std::string& p),
    std::string payload_stringify() const,
    std::basic_string<uint8_t> payload_bstringify() const,
    CMqttMessage(const uint8_t* p COMMA() std::size_t s),
    CMqttMessage(const std::string& p),
    std::string to_string()
  )
  , ( )
    , id            , int64_t               , 0
    , creation_time , int64_t               , 0
    , last_activity , int64_t               , 0
    , topic         , std::string           , 
    , payload       , std::vector<uint8_t>  , 
    , priority      , ec::CMessagePriority  , ec::CMessagePriority::DEFAULT
    , retries       , int                   , 0
)



  CYT3MACRO_model_class_set_mapped_definitions(CMqttMessage, id)
  

  CMqttMessage::CMqttMessage(const uint8_t* p, std::size_t s){
    payload(p,s);
    gents_now();
  }
  CMqttMessage::CMqttMessage(const std::string& p){
    payload(p);
    gents_now();
  }


  void CMqttMessage::gents_now(){
    creation_time(ct::get_current_timestamp());
  }
  void CMqttMessage::activity_ts_now(){
    last_activity(ct::get_current_timestamp());
  }

  std::vector<uint8_t> CMqttMessage::payload(const uint8_t* p , std::size_t s)
  {
    payload_.clear();
    for(std::size_t i = 0;i < s;++i){
      payload_.push_back(p[i]);
    }
    return payload_;
  }
  
  std::vector<uint8_t> CMqttMessage::payload(const std::string& p)
  {
    payload_.clear();
    for(const char& c : p){
      payload_.push_back(static_cast<uint8_t>(c));
    }
    return payload_;
  }

  std::string CMqttMessage::payload_stringify() const {
    std::string str;
    
    for(const uint8_t& c: payload_){
      str.push_back(c);
    }
    return str;
  }
  std::basic_string<uint8_t> CMqttMessage::payload_bstringify() const {
    std::basic_string<uint8_t> str;
    
    for(const uint8_t& c: payload_){
      str.push_back(c);
    }
    return str;
  }

}
}
}