#pragma once
#include "../topics/Publisher.hpp"
#include "../topics/Subscription.hpp"


namespace coyot3::communication::mqtt{


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

  // example variadic arguments... c-style
  struct CientDataModelError : std::exception{
      char text[1000];

      CientDataModelError(char const* fmt, ...) __attribute__((format(printf,2,3)));
      

      char const* what() const noexcept { return text; }
  };

  CYT3MACRO_model_class_declarations(
    ClientDataModel
    ,
    , ( 

        void set_state(ec::MosqClientState s),
        Subscription& get_subscription_by_id(int64_t id)
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

      , subscriptions_config        , cmqc_subscriptions_treeMappedSet  ,
      , subscriptions_active        , cmqc_subscriptions_treeMappedSet  ,
      , publishers_config           , PublisherMappedSet                , 

      , subscriptions_pending_ids   , std::vector<int64_t>              , 
      , subscriptions_pending       , SubscriptionMappedSet             , 

      , callbacks_on_connection     , MqttClientCallbacksOnEventSimple  , 
      , callbacks_on_disconnection  , MqttClientCallbacksOnEventSimple  , 
  )




}
