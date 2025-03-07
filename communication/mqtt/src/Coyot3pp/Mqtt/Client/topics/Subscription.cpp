#include <Coyot3pp/Mqtt/Client/topics/Subscription.hpp>

namespace coyot3::communication::mqtt
{
  
  CYT3MACRO_model_class_definitions_no_opsoverload(
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


    bool Subscription::operator==(const Subscription &o) const 
    { return 
      (  
          (id_ == o.id())
        &&(active_ == o.active())
        &&(subscribed_ == o.subscribed())
        &&(topic_ == o.topic())
        &&(mosquitto_qos_ == o.mosquitto_qos())
        &&(mosquitto_subid_ == o.mosquitto_subid())
        &&(ts_last_msg_ == o.ts_last_msg())
        &&(msg_count_ == o.msg_count())
      ); 
    }
    Subscription& Subscription::operator=(const Subscription& o){
      id(o.id());
      active(o.active());
      subscribed(o.subscribed());
      topic(o.topic());
      mosquitto_qos(o.mosquitto_qos());
      mosquitto_subid(o.mosquitto_subid());
      ts_last_msg(o.ts_last_msg());
      msg_count(o.msg_count());
      callback(o.callback());
      return *this;
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