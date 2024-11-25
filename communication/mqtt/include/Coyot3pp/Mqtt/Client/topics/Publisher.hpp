#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{

  CYT3MACRO_model_class_declarations(
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
    

    CYT3MACRO_model_class_set_mapped_declarations(Publisher,topic)

}
}
}