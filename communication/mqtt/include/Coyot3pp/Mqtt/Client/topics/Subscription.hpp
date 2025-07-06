#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>



namespace coyot3::communication::mqtt{

  typedef std::function<bool(const std::string&, 
                                 const uint8_t*, 
                                 std::size_t)>   MqttClientOnMessageCallback;

  CYT3MACRO_model_class_declarations(
    Subscription
    ,
    , ( )
    , ( )
      , id                    , int64_t                     , 
      , active                , bool                        , true
      , subscribed            , bool                        , false
      , topic                 , std::string                 , ""
      , mosquitto_qos         , int                         , 
      , mosquitto_subid       , int                         , 
      , ts_last_msg           , int64_t                     , 0
      , msg_count             , int64_t                     , 0
      , callback              , MqttClientOnMessageCallback , 
  )


    CYT3MACRO_model_class_set_mapped_declarations(Subscription,id)



  CYT3MACRO_model_class_declarations(
    cmqc_subscriptions_tree
    ,
    , ( )
    , ( )
      , topic                 , std::string                 , 
      , topic_index           , std::string                 , 
      , active                , bool                        , true
      , subscribed            , bool                        , false
      , mosquitto_qos         , int                         , 
      , mosquitto_subid       , int                         , 
      , ts_connection_launch  , int64_t                     , 0
      , ts_connection_done    , int64_t                     , 0
      , ts_last_msg           , int64_t                     , 0
      , msg_count             , int64_t                     , 0
      , subs                  , SubscriptionMappedSet       , 
  )

    CYT3MACRO_model_class_set_mapped_declarations(cmqc_subscriptions_tree, topic)

}