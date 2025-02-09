#include <Coyot3pp/Mqtt/Client/topics/Subscription.hpp>

namespace coyot3::communication::mqtt
{
  
  CYT3MACRO_model_class_definitions_no_eq_overload(
    Subscription
    ,
    , ( 
        MqttClientOnMessageCallback callback
      )
    , ( )
      , id                    , int64_t                     , 
      , active                , bool                        , true
      , subscribed            , bool                        , false
      , topic                 , std::string                 , ""
      , mosquitto_qos         , int                         , 
      , mosquitto_subid       , int                         , 
      , ts_connection_launch  , int64_t                     , 0
      , ts_connection_done    , int64_t                     , 0
      , ts_last_msg           , int64_t                     , 0
      , msg_count             , int64_t                     , 0
      , callback              , MqttClientOnMessageCallback ,
  
  )


    bool Subscription::operator==(const Subscription &o) const 
    { return 
      (  (id_ == o.id_) 
      && (active_ == o.active_) 
      && (topic_ == o.topic_) 
      && (mosquitto_qos_ == o.mosquitto_qos_) 
      && (ts_last_msg_ == o.ts_last_msg_) 
      && (msg_count_ == o.msg_count_)
      ); 
    }



CYT3MACRO_model_class_set_mapped_definitions(Subscription, id)

CYT3MACRO_model_class_definitions(
  cmqc_subscriptions_tree
  ,  
  , ()
  , ()
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

CYT3MACRO_model_class_set_mapped_definitions(cmqc_subscriptions_tree, topic)
}