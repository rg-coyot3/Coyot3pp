#pragma once
#include "../topics/Publisher.hpp"
#include "../topics/Subscription.hpp"


namespace coyot3{
namespace communication{
namespace mqtt{

  CYT3MACRO_enum_class_declarations(
    MosqClientState
    ,
      , DISCONNECTED = 0
      , CONNECTING = 1
      , CONNECTED = 2
      , ERROR = 3
  )

  typedef coyot3::tools::ControlThread ControlThread;
  typedef std::function<void(bool)>                         MqttClientCallbackOnEventSimple;
  typedef std::map<int ,MqttClientCallbackOnEventSimple>    MqttClientCallbacksOnEventSimple;
  typedef MqttClientCallbacksOnEventSimple::iterator        MqttClientCallbacksOnEventSimpleIter;
  typedef std::pair<int ,MqttClientCallbackOnEventSimple>   MqttClientCallbacksOnEventSimplePair;

  CYT3MACRO_model_class_declarations(
    ClientDataModel
    ,
    , ( 

        void set_state(ec::MosqClientState s)
    )
    , ( )
      , ts_creation                 , int64_t                           , 0
      , state                       , ec::MosqClientState               , ec::MosqClientState::UNKNOWN_OR_UNSET
      , mosquitto_id                , std::string                       , 
      , ts_last_transition          , int64_t                           , 0
      , ts_first_connection         , int64_t                           , 0
      , ts_last_connection          , int64_t                           , 0
      , ts_last_disconnection       , int64_t                           , 0
      , total_num_reconnections     , int64_t                           , 0

      , subscriptions_config        , SubscriptionMappedSet             ,
      , publishers_config           , PublisherMappedSet                , 

      , subscriptions_pending       , SubscriptionMappedSet             , 

      , callbacks_on_connection     , MqttClientCallbacksOnEventSimple  , 
      , callbacks_on_disconnection  , MqttClientCallbacksOnEventSimple  , 
  )

  


}
}
}