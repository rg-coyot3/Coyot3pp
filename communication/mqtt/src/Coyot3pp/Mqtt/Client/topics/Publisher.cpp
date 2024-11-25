#include <Coyot3pp/Mqtt/Client/topics/Publisher.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{


  CYT3MACRO_model_class_definitions(
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


    CYT3MACRO_model_class_set_mapped_definitions(Publisher,topic)


}
}
}