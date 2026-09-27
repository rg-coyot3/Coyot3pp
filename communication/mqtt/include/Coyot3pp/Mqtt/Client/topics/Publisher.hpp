#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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
    

    COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(Publisher,topic)

}
}
}