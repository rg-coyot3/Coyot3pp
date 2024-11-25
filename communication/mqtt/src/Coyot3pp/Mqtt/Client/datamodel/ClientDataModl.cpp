#include <Coyot3pp/Mqtt/Client/Client.hpp>



namespace ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{

  CYT3MACRO_enum_class_definitions(
    MosqClientState
    ,
      , DISCONNECTED
      , CONNECTING
      , CONNECTED
      , ERROR
  )

  CYT3MACRO_model_class_definitions(
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
      , total_num_reconnections     , int64_t                           , 0

      , subscriptions_config        , SubscriptionMappedSet             ,
      , publishers_config           , PublisherMappedSet                , 

      , subscriptions_pending       , SubscriptionMappedSet             , 

      , callbacks_on_connection     , MqttClientCallbacksOnEventSimple  , 
      , callbacks_on_disconnection  , MqttClientCallbacksOnEventSimple  , 
  )

  void ClientDataModel::set_state(ec::MosqClientState s){
    ts_last_transition(ct::get_current_timestamp());
    if(s == ec::MosqClientState::CONNECTED){
      if(ts_first_connection() == 0)ts_first_connection(ts_last_transition());
      ts_last_connection(ts_last_transition());
      state(s);
    }else if(s == ec::MosqClientState::DISCONNECTED){
      ts_last_disconnection(ts_last_transition());
    }
  }

}
}
}