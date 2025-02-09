#include <Coyot3pp/Mqtt/Client/datamodel/ClientDataModel.hpp>



namespace ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{



    CientDataModelError::CientDataModelError(char const* fmt, ...)  {
              std::va_list ap;
              va_start(ap, fmt);
              vsnprintf(text, sizeof text, fmt, ap);
              va_end(ap);
    }



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

  Subscription& ClientDataModel::get_subscription_by_id(int64_t id){
    std::string t; size_t s;
    Subscription* res = nullptr;
    s = subscriptions_config().for_each([&](cmqc_subscriptions_tree& subt){
      if(res != nullptr)return false;
      if(subt.subs().is_member(id) == true){
        res = &subt.subs().get(id);
        return true;
      }
      return false;
    });
    if(s == 0){
      CLOG_WARN("client-data-model- : ATTENTION! : NO SUBSCRIPTION "
      "GOTTEN WITH ID [" << id << "] !!! UNDEFINED-BEHABE!!!");
      std::stringstream sstr; sstr << "Error! : Subscription not found with id "
      "[" << id << "]";
      throw CientDataModelError(sstr.str().c_str());
    }
    return *res;
  }




}
}
}