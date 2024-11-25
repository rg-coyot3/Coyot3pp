#include <Coyot3pp/Mqtt/Client/topics/Subscription.hpp>

namespace coyot3{
namespace communication{
namespace mqtt{


  CYT3MACRO_model_class_definitions(
    Subscription
    ,
    , ( 
        typedef std::function<bool(const std::string& COMMA() const std::vector<uint8_t>&)> OnMessageCallback
        , OnMessageCallback callback
      )
    , ( )
      , id              , int64_t           , 
      , active          , bool              , true
      , subscribed      , bool              , false
      , topic           , std::string       , ""
      , mosquitto_qos   , int               , 
      , ts_last_msg     , int64_t           , 0
      , msg_count       , int64_t           , 0
  )

    
    CYT3MACRO_model_class_set_mapped_declarations(Subscription,topic)



}
}
}