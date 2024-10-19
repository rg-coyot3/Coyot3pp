#include <Coyot3pp/Mqtt/Gateway/AcrpTopicManagementInfo.hpp>



namespace coyot3{
namespace communication{
namespace mqtt{


  CYT3MACRO_enum_class_definitions(
    AcrpTopicRole
    , 
    , SUBSCRIBER
    , EMITTER   
  )



  CYT3MACRO_model_class_definitions(
    AcrpTopicInformation
    ,
    , ( 
      typedef MqttGateway::OnMessageCallback  OnMessageCallback
    )
    , ( 
      role                              , ec::AcrpTopicRole , ec::AcrpTopicRole::UNKNOWN_OR_UNSET
    )
    , timeout_packet_resend               , int               , 5000
    , timeout_packet_send_failed          , int               , 3000
    , automatic_resend_time               , int               , 1000
    , timeout_receiver_alive_checkpacket  , int               , 10000
    , noprot_msg_noactivity_timeout       , int               , 3000
    , publish_dynop_interval              , int               , 5000
    , postfix_req                         , std::string       , "crp_req"
    , postfix_res                         , std::string       , "crp_res"

  )


/////////
///////// ACRP 
/////////

const char*                 AcrpTopicManagementInfo::Defaults::postfix_req         = "crp_req";
const char*                 AcrpTopicManagementInfo::Defaults::postfix_res         = "crp_res";
const bool                  AcrpTopicManagementInfo::Defaults::active              = true;
const MqttGateway::Priority AcrpTopicManagementInfo::Defaults::qos = MqttGateway::Priority::HIGH;


const char* AcrpTopicManagementInfo::Config::JsField::acrp_is_active = "acrp_is_active";
const char* AcrpTopicManagementInfo::Config::JsField::acrp_timeout_packet_resend = "acrp_timeout_packet_resend";
const char* AcrpTopicManagementInfo::Config::JsField::acrp_timeout_packet_send_failed = "acrp_timeout_packet_send_failed";
const char* AcrpTopicManagementInfo::Config::JsField::acrp_automatic_resend_time = "acrp_automatic_resend_time";
const char* AcrpTopicManagementInfo::Config::JsField::acrp_timeout_receiver_alive_checkpacket = "acrp_timeout_receiver_alive_checkpacket";
const char* AcrpTopicManagementInfo::Config::JsField::acrp_noprot_msg_noactivity_timeout = "acrp_noprot_msg_noactivity_timeout";

AcrpTopicManagementInfo::Config::Config()
:acrp_is_active(false)
,acrp_timeout_packet_resend(1000)
,acrp_timeout_packet_send_failed(15000)
,acrp_automatic_resend_time(13999)
,acrp_timeout_receiver_alive_checkpacket(15000)
,acrp_noprot_msg_noactivity_timeout(120000)
{

}
AcrpTopicManagementInfo::Config::Config(const AcrpTopicManagementInfo::Config& o)
{
  *this = o;
}
AcrpTopicManagementInfo::Config::~Config(){}

AcrpTopicManagementInfo::Config& AcrpTopicManagementInfo::Config::operator=(const AcrpTopicManagementInfo::Config& o)
{
  acrp_is_active = o.acrp_is_active;
  acrp_timeout_packet_resend = o.acrp_timeout_packet_resend;
  acrp_timeout_packet_send_failed = o.acrp_timeout_packet_send_failed;
  acrp_automatic_resend_time = o.acrp_automatic_resend_time;
  acrp_timeout_receiver_alive_checkpacket = o.acrp_timeout_receiver_alive_checkpacket;
  acrp_noprot_msg_noactivity_timeout = o.acrp_noprot_msg_noactivity_timeout;

  return *this;
}
bool AcrpTopicManagementInfo::Config::operator==(const AcrpTopicManagementInfo::Config& o)
{
  return (
       (acrp_is_active == o.acrp_is_active)
    && (acrp_timeout_packet_resend == o.acrp_timeout_packet_resend)
    && (acrp_timeout_packet_send_failed == o.acrp_timeout_packet_send_failed)
    && (acrp_automatic_resend_time == o.acrp_automatic_resend_time)
    && (acrp_timeout_receiver_alive_checkpacket == o.acrp_timeout_receiver_alive_checkpacket)
    && (acrp_noprot_msg_noactivity_timeout == o.acrp_noprot_msg_noactivity_timeout)
  );
}


bool        AcrpTopicManagementInfo::Config::fromJson(const Json::Value& o)
{
  if(
         !(coyot3::tools::json_contains_member(o,JsField::acrp_is_active))
      || !(coyot3::tools::json_contains_member(o,JsField::acrp_timeout_packet_resend))
      || !(coyot3::tools::json_contains_member(o,JsField::acrp_timeout_packet_send_failed))
      || !(coyot3::tools::json_contains_member(o,JsField::acrp_automatic_resend_time))
      || !(coyot3::tools::json_contains_member(o,JsField::acrp_timeout_receiver_alive_checkpacket))
      || !(coyot3::tools::json_contains_member(o,JsField::acrp_noprot_msg_noactivity_timeout))
  ){
    return false;
  }
  try{
    acrp_is_active = o[JsField::acrp_is_active].asBool();
    acrp_timeout_packet_resend = o[JsField::acrp_timeout_packet_resend].asLargestInt();
    acrp_timeout_packet_send_failed = o[JsField::acrp_timeout_packet_send_failed].asLargestInt();
    acrp_automatic_resend_time = o[JsField::acrp_automatic_resend_time].asLargestInt();
    acrp_timeout_receiver_alive_checkpacket = o[JsField::acrp_timeout_receiver_alive_checkpacket].asLargestInt();
    acrp_noprot_msg_noactivity_timeout = o[JsField::acrp_noprot_msg_noactivity_timeout].asLargestInt();
  }catch(const Json::Exception& e)
  {
    CLOG_WARN("mqtt-manager-base : config : from-json : error parsing data [" << e.what() << "]");
    return false;
  }catch(...){
    CLOG_WARN("mqtt-manager-base : config : from-json : error parsing data [unknown-exception]");
    return false;
  }
  return true;
}
Json::Value AcrpTopicManagementInfo::Config::toJson() const
{
  Json::Value js;

  js[JsField::acrp_is_active] = acrp_is_active;
  js[JsField::acrp_timeout_packet_resend] = static_cast<Json::LargestInt>(acrp_timeout_packet_resend);
  js[JsField::acrp_timeout_packet_send_failed] = static_cast<Json::LargestInt>(acrp_timeout_packet_send_failed);
  js[JsField::acrp_automatic_resend_time] = static_cast<Json::LargestInt>(acrp_automatic_resend_time);
  js[JsField::acrp_timeout_receiver_alive_checkpacket] = static_cast<Json::LargestInt>(acrp_timeout_receiver_alive_checkpacket);
  js[JsField::acrp_noprot_msg_noactivity_timeout] = static_cast<Json::LargestInt>(acrp_noprot_msg_noactivity_timeout);

  return js;
}


//
// CONFIG : end
//

const char* AcrpTopicManagementInfo::RoleToString(Role r)
{
  switch(r)
  {
    case Role::EMITTER: return "EMITTER";break;
    case Role::SUBSCRIBER: return "SUBSCRIBER";break;
    default:
      return "role-value-unknown";
  }
}

/**
 * @brief : just simply does nothing with the ACK... only log it in case that
 *  the debug mode is up to level 6.
 * */
void AcrpTopicManagementInfo::on_published_placebo_callback(const std::string& topic, bool result,const Json::Value& source)
{
  CLOG_DEBUG(6,"ACRP TOPIC MANAGEMENT INFO : on pub do-nothing-callback : "
    "received evaluation ["
    << (result == true?"OK":"ERROR") << "for message ["
    << source << "]");
}



/**
 * @brief : constructor publisher
 * */
AcrpTopicManagementInfo::AcrpTopicManagementInfo(
  const std::string& base
  ,const std::string& preq
  ,const std::string& pres
)
:message_stack()
,topic_base()
,topic_crp_req()
,topic_crp_res()
,acrp_timeout_packet_send_failed(0)
,acrp_automatic_resend_time(0)
,acrp_last_incoming_activity_ts(0)
,acrp_last_packet_send(0)

,acrpConfig()
{
  topic_base = base;
  if(preq.size())
  {
    topic_crp_req = (base + "/" + preq);
  }else{
    topic_crp_req = (base + "/" + Defaults::postfix_req);
  }
  
  if(preq.size())
  {
    topic_crp_res = (base + "/" + pres);
  }else{
    topic_crp_res = (base + "/" + Defaults::postfix_res);
  }
  role = Role::EMITTER;
  
  on_data_callback = on_message_placebo_callback;

  publish_op_result_callback = &AcrpTopicManagementInfo::on_published_placebo_callback;

  CLOG_INFO("ACRP TOPIC MANAGEMENT INFO : constructor : PUBLISHER : base ["
    "topic [" << topic_base << "];req[" << topic_crp_req << "];res["
    << topic_crp_res << "]");
}

AcrpTopicManagementInfo::AcrpTopicManagementInfo(
  const std::string& base
  ,OnPublisherOperationResultCallbackType callbackOnPublishResult
  ,const std::string& preq
  ,const std::string& pres
)
:message_stack()
,topic_base()
,topic_crp_req()
,topic_crp_res()
,acrp_timeout_packet_send_failed(0)
,acrp_automatic_resend_time(0)
,acrp_last_incoming_activity_ts(0)
,acrp_last_packet_send(0)

,acrpConfig()
{
  topic_base = base;
  if(preq.size())
  {
    topic_crp_req = (base + "/" + preq);
  }else{
    topic_crp_req = (base + "/" + Defaults::postfix_req);
  }
  
  if(preq.size())
  {
    topic_crp_res = (base + "/" + pres);
  }else{
    topic_crp_res = (base + "/" + Defaults::postfix_res);
  }
  role = Role::EMITTER;
  
  on_data_callback = on_message_placebo_callback;

  publish_op_result_callback = callbackOnPublishResult;

  CLOG_INFO("ACRP TOPIC MANAGEMENT INFO : constructor : EMITTER : base ["
    "topic [" << topic_base << "];req[" << topic_crp_req << "];res["
    << topic_crp_res << "]");
}

/**
 * @brief : constructor subscriber
 * */
AcrpTopicManagementInfo::AcrpTopicManagementInfo(
  const std::string& base
  ,OnSubscriberDataCallbackType callback
  ,OnSubscriberParsedDataCallbackType callbackParsedData
  ,const std::string& preq
  ,const std::string& pres
):message_stack()
,topic_base()
,topic_crp_req()
,topic_crp_res()
,acrp_timeout_packet_send_failed(0)
,acrp_automatic_resend_time(0)
,acrp_last_incoming_activity_ts(0)
,acrp_last_packet_send(0)

,acrpConfig()
{
    
  topic_base = base;
  if(preq.size())
  {
    topic_crp_req = (base + "/" + preq);
  }else{
    topic_crp_req = (base + "/" + Defaults::postfix_req);
  }
  
  if(preq.size())
  {
    topic_crp_res = (base + "/" + pres);
  }else{
    topic_crp_res = (base + "/" + Defaults::postfix_res);
  }
  role = Role::SUBSCRIBER;
  

  on_data_callback = callback;
  on_parsed_data_callback = callbackParsedData;
  
  publish_op_result_callback = &AcrpTopicManagementInfo::on_published_placebo_callback;

  CLOG_INFO("ACRP TOPIC MANAGEMENT INFO : constructor : SUBSCRIBER : base "
    "topic [" << topic_base << "];req[" << topic_crp_req << "];res[" 
    << topic_crp_res << "]");
}

AcrpTopicManagementInfo::~AcrpTopicManagementInfo()
{
  CLOG_WARN("ACRP TOPIC MANAGEMENT INFO : destructor : for [" << topic_base << "]");
}

bool AcrpTopicManagementInfo::setTimeoutPacketResendConfigTime()
{
  acrp_timeout_packet_send_failed = acrpConfig.acrp_timeout_packet_send_failed;
  acrp_automatic_resend_time = acrpConfig.acrp_automatic_resend_time;
  return true;
}
bool AcrpTopicManagementInfo::setTimeoutPacketResendReducedTime()
{
  acrp_timeout_packet_send_failed = 2 * acrpConfig.acrp_timeout_packet_resend - 1;
  acrp_automatic_resend_time = acrpConfig.acrp_timeout_packet_resend - 1;
  return true;
}





/////
///// MISCELANEUS : BEGIN
/////



const std::string&  AcrpTopicManagementInfo::getTopicBase() const 
{
  return topic_base;
}
const std::string& AcrpTopicManagementInfo::getTopicCrpReq() const
{
  return topic_crp_req;
}
const std::string& AcrpTopicManagementInfo::getTopicCrpRes() const
{
  return topic_crp_res;
}
bool AcrpTopicManagementInfo::isActive()
{
  return acrpConfig.acrp_is_active;
}
bool AcrpTopicManagementInfo::isActive(bool state)
{
  return acrpConfig.acrp_is_active = state;
}

bool AcrpTopicManagementInfo::setTopicBase(const std::string& t)
{
  topic_base = t;
  return true;
}
bool AcrpTopicManagementInfo::setTopicCrpReq(const std::string& t)
{
  topic_crp_req = t;
  return true;
}
bool AcrpTopicManagementInfo::setTopicCrpRes(const std::string& t)
{
  topic_crp_res = t;
  return true;
}


bool AcrpTopicManagementInfo::setRole(AcrpTopicManagementInfo::Role r)
{
  role = r;
  return true;
}
AcrpTopicManagementInfo::Role AcrpTopicManagementInfo::getRole()
{
  return role;
}
MqttGateway::Priority AcrpTopicManagementInfo::getPriority()
{
  return qos_priority;
}
bool AcrpTopicManagementInfo::setPriority(MqttGateway::Priority p)
{
  qos_priority = p;
  return true;
}
AcrpPacket* AcrpTopicManagementInfo::getPacket(int64_t token,int64_t sec)
{
  return message_stack.find(token,sec);
}
AcrpPacket* AcrpTopicManagementInfo::getPacket(const AcrpPacket& source)
{
  return getPacket(source.timestampOrigToken,source.tokenSec);
}
AcrpPacket* AcrpTopicManagementInfo::registerPacket(const AcrpPacket& source)
{
  return message_stack.registerPacket(source);
}
bool AcrpTopicManagementInfo::unregisterPacket(int64_t token,int64_t sec)
{
  return message_stack.removeFromStack(token,sec);
}
bool AcrpTopicManagementInfo::unregisterPacket(const AcrpPacket& acrp)
{
  return unregisterPacket(acrp.timestampOrigToken,acrp.tokenSec);
}

size_t AcrpTopicManagementInfo::forEachMessage(std::function<bool(AcrpPacket*)> func)
{
  return message_stack.forEach(func);
}

void AcrpTopicManagementInfo::updateOnPublishedCallback(OnPublisherOperationResultCallbackType callback){
  publish_op_result_callback = callback;
}


bool  AcrpTopicManagementInfo::setConfig(const Json::Value& js)
{
  bool res = acrpConfig.fromJson(js);
  acrp_timeout_packet_send_failed = acrpConfig.acrp_timeout_packet_send_failed;
  acrp_automatic_resend_time = acrpConfig.acrp_automatic_resend_time;
  return res;
}
AcrpTopicManagementInfo::Config& AcrpTopicManagementInfo::getConfig()
{
  return acrpConfig;
}

int64_t AcrpTopicManagementInfo::getLastIncomingActivityTs()
{
  return acrp_last_incoming_activity_ts;
}
void AcrpTopicManagementInfo::markIncomingActivity()
{
  acrp_last_incoming_activity_ts = coyot3::tools::getCurrentTimestamp();
}

int64_t AcrpTopicManagementInfo::getTimeoutPacketResend()
{
  return acrpConfig.acrp_timeout_packet_resend;
}
int64_t AcrpTopicManagementInfo::getTimeoutPacketSendFailed()
{
  return acrp_timeout_packet_send_failed;
}
int64_t AcrpTopicManagementInfo::getAutomaticResendTime()
{
  return acrp_automatic_resend_time;
}
int64_t AcrpTopicManagementInfo::getPacketStoreExpirationTimeout()
{
  return acrpConfig.acrp_noprot_msg_noactivity_timeout;
}
int64_t AcrpTopicManagementInfo::getDynopPublishInterval()
{
  return acrpConfig.acrp_publish_dynop_interval;
}
int64_t AcrpTopicManagementInfo::getLastPacketSend()
{
  return acrp_last_packet_send;
}
void    AcrpTopicManagementInfo::markLastPacketSend()
{
  acrp_last_packet_send = coyot3::tools::getCurrentTimestamp();
}


//statics
bool AcrpTopicManagementInfo::on_message_placebo_callback(const std::string& topic, const uint8_t* d,size_t s)
{
  return true;
}

/////
///// MISCELANEUS : END
/////

}
}//eons wrappers
}// end of namespace cyt_tools

std::ostream& operator<<(std::ostream& o, coyot3::communication::mqtt::AcrpTopicManagementInfo::Role s)
{
  return (o << coyot3::communication::mqtt::AcrpTopicManagementInfo::RoleToString(s));
}



