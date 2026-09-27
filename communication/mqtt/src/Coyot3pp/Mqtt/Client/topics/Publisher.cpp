#include <Coyot3pp/Mqtt/Client/topics/Publisher.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{


  COYOT3PP_MODEL_CLASS_DEFINITIONS(
    Publisher
    ,
    , ( 
      
    )
    , ( )
      , id            , int64_t         , 
      , active        , bool            , true
      , topic         , std::string     , 
      , mosquitto_qos , int             , 
      , ts_last_msg   , int64_t         , 0
      , msg_count     , int64_t         , 0
  )


    COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(Publisher,topic)


}
}
}