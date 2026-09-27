#pragma once


#include <Coyot3pp/Cor3/Coyot3.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{

  COYOT3PP_ENUM_CLASS_DECLARATIONS(
    MessagePriorityLevel
    ,
    , LOW                 , 0
    , MEDIUM              , 1
    , HIGH                , 2
    , HIGH_WHEN_AVAILABLE , 3
    , DEFAULT             , 10
  )


  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    Message
  ,
  , ( 
    void gents_now(),
    void activity_ts_now(),
    std::vector<uint8_t>& payload(const uint8_t* p COMMA() std::size_t s),
    std::vector<uint8_t>& payload(const std::string& p),
    std::string payload_stringify() const,
    std::basic_string<uint8_t> payload_bstringify() const,
      Message(const std::string& t COMMA() const uint8_t* p COMMA() std::size_t s),
      Message(const std::string& t COMMA() const std::string& p),
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

  COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(Message, id)
  COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(Message, )
  


}
}
}