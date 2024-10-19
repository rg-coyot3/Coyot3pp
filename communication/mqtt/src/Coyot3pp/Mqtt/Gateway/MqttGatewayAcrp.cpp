#include <Coyot3pp/Mqtt/Gateway/MqttGatewayAcrp.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{



CYT3MACRO_model_class_definitions(
  AcrpConfigObject
  , 
    , ( )
    , ( )
      , is_active                           , bool              , 
      , use_defaults                        , bool              , 
      , timeout_packet_resend               , int64_t           , 
      , timeout_packet_send_failed          , int64_t           , 
      , automatic_resend_time               , int64_t           , 
      , timeout_receiver_alive_checkpacket  , int64_t           , 
      , noprot_msg_noactivity_timeout       , int64_t           , 
      , publish_dynop_interval              , int64_t           , 
      , default_posftix_request             , std::string       , 
      , default_postfix_response            , std::string       , 
)

CYT3MACRO_model_class_serializable_json_definitions(
  AcrpConfigObject
  , 
  , 
    , ( ) 
    , ( )
      , is_active                           , "is_active"                           , 
      , use_defaults                        , "use_defaults"                        , 
      , timeout_packet_resend               , "timeout_packet_resend"               , 
      , timeout_packet_send_failed          , "timeout_packet_send_failed"          , 
      , automatic_resend_time               , "automatic_resend_time"               , 
      , timeout_receiver_alive_checkpacket  , "timeout_receiver_alive_checkpacket"  , 
      , noprot_msg_noactivity_timeout       , "noprot_msg_noactivity_timeout"       , 
      , publish_dynop_interval              , "publish_dynop_interval"              , 
      , default_posftix_request             , "default_posftix_request"             , 
      , default_postfix_response            , "default_postfix_response"            , 
)


CYT3MACRO_model_class_definitions(
  MqttGatewayAcrpConfig
    , MqttGatewayConfigObject
    , ( )
    , ( )
    , acrp   , AcrpConfigObject , 
)

CYT3MACRO_model_class_serializable_json_definitions(
  MqttGatewayAcrpConfig
  , MqttGatewayConfigObject
  , 
  , ( acrp , "acrp", AcrpConfigObject)
  , ( )
)





const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_is_active = "is_active";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_use_defaults = "use_defaults";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_timeout_packet_resend = "timeout_packet_resend";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_timeout_packet_send_failed = "timeout_packet_send_failed";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_automatic_resend_time = "automatic_resend_time";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_timeout_receiver_alive_checkpacket = "timeout_receiver_alive_checkpacket";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_noprot_msg_noactivity_timeout = "noprot_msg_noactivity_timeout"; 
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_publish_dynop_interval = "publish_dynop_interval";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_default_posftix_request = "default_posftix_request";
const char* MqttGatewayAcrp::AcrpConfig::JsField::acrp_default_postfix_response = "default_postfix_response";

const bool    MqttGatewayAcrp::AcrpConfig::Defaults::acrp_use_defaults = false;
const bool    MqttGatewayAcrp::AcrpConfig::Defaults::acrp_is_active = false;
const char*   MqttGatewayAcrp::AcrpConfig::Defaults::acrp_default_posftix_request = "/req";
const char*   MqttGatewayAcrp::AcrpConfig::Defaults::acrp_default_postfix_response = "/res";
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_timeout_packet_resend = 5;
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_timeout_packet_send_failed = 15;
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_automatic_resend_time = 5;
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_timeout_receiver_alive_checkpacket = 5;
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_noprot_msg_noactivity_timeout = 10;
const int64_t MqttGatewayAcrp::AcrpConfig::Defaults::acrp_publish_dynop_interval = 5;


const char* MqttGatewayAcrp::AcrpConfig::get_help(){
  std::stringstream s;
  s << " - acrp-config fields : " << std::endl;
  s << " - " << JsField::acrp_use_defaults << "(bool)" << std::endl;
  s << " - " << JsField::acrp_is_active << "(bool)" << std::endl;
  s << " - " << JsField::acrp_default_posftix_request << "(string)" << std::endl;
  s << " - " << JsField::acrp_default_postfix_response << "(string)" << std::endl;
  s << " - " << JsField::acrp_timeout_packet_resend << "(int64_t)" << std::endl;
  s << " - " << JsField::acrp_timeout_packet_send_failed << "(int64_t)" << std::endl;
  s << " - " << JsField::acrp_automatic_resend_time << "(int64_t)" << std::endl;
  s << " - " << JsField::acrp_timeout_receiver_alive_checkpacket << "(int64_t)" << std::endl;
  s << " - " << JsField::acrp_noprot_msg_noactivity_timeout << "(int64_t)" << std::endl;
  s << " - " << JsField::acrp_publish_dynop_interval << "(int64_t)" << std::endl;
  s << std::endl;
  return s.str().c_str();
}

MqttGatewayAcrp::AcrpConfig::AcrpConfig()
:acrp_is_active(Defaults::acrp_is_active)
,acrp_use_defaults(Defaults::acrp_use_defaults)
,acrp_timeout_packet_resend(Defaults::acrp_timeout_packet_resend)
,acrp_timeout_packet_send_failed(Defaults::acrp_timeout_packet_send_failed)
,acrp_automatic_resend_time(Defaults::acrp_automatic_resend_time)
,acrp_timeout_receiver_alive_checkpacket(Defaults::acrp_timeout_receiver_alive_checkpacket)
,acrp_noprot_msg_noactivity_timeout(Defaults::acrp_noprot_msg_noactivity_timeout)
,acrp_publish_dynop_interval(Defaults::acrp_publish_dynop_interval)
,acrp_default_posftix_request(Defaults::acrp_default_postfix_response)
,acrp_default_postfix_response(Defaults::acrp_default_postfix_response)
{

}
MqttGatewayAcrp::AcrpConfig::AcrpConfig(const AcrpConfig& o)
{
  *this = o;
}
MqttGatewayAcrp::AcrpConfig::~AcrpConfig(){}

MqttGatewayAcrp::AcrpConfig& MqttGatewayAcrp::AcrpConfig::operator=(const AcrpConfig& o)
{
  acrp_is_active = o.acrp_is_active;
  acrp_use_defaults = o.acrp_use_defaults;
  acrp_timeout_packet_resend = o.acrp_timeout_packet_resend;
  acrp_timeout_packet_send_failed = o.acrp_timeout_packet_send_failed;
  acrp_automatic_resend_time = o.acrp_automatic_resend_time;
  acrp_timeout_receiver_alive_checkpacket = o.acrp_timeout_receiver_alive_checkpacket;
  acrp_noprot_msg_noactivity_timeout = o.acrp_noprot_msg_noactivity_timeout;
  acrp_publish_dynop_interval = o.acrp_publish_dynop_interval;
  acrp_default_posftix_request = o.acrp_default_posftix_request;
  acrp_default_postfix_response = o.acrp_default_postfix_response;

  return *this;
}

bool MqttGatewayAcrp::AcrpConfig::operator==(const AcrpConfig& o)
{
  return (
    (acrp_is_active == o.acrp_is_active) &&
    (acrp_use_defaults == o.acrp_use_defaults) &&
    (acrp_timeout_packet_resend == o.acrp_timeout_packet_resend) &&
    (acrp_timeout_packet_send_failed == o.acrp_timeout_packet_send_failed) &&
    (acrp_automatic_resend_time == o.acrp_automatic_resend_time) &&
    (acrp_timeout_receiver_alive_checkpacket == o.acrp_timeout_receiver_alive_checkpacket) &&
    (acrp_noprot_msg_noactivity_timeout == o.acrp_noprot_msg_noactivity_timeout) &&
    (acrp_publish_dynop_interval == o.acrp_publish_dynop_interval) &&
    (acrp_default_posftix_request == o.acrp_default_posftix_request) &&
    (acrp_default_postfix_response == o.acrp_default_postfix_response) 
  );
}

bool MqttGatewayAcrp::AcrpConfig::fromJson(const Json::Value& js)
{
  bool res = true;
  res &= coyot3::tools::json_import_value(js,JsField::acrp_use_defaults,acrp_use_defaults);
  if(!res){
    CLOG_ERROR("mqtt-gateway-acrp : acrp-config : from-json : loading configuration : ERROR!");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : acrp-config : from-json : loading configuration");
  if(acrp_use_defaults == true){
    CLOG_WARN("mqtt-gateway-acrp : acrp-config : from-json : will use default "
    "configuration. the rest of the parameters will not be read");
    return true;
  }
  
  
  res &= coyot3::tools::json_import_value(js,JsField::acrp_is_active,acrp_is_active);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_timeout_packet_resend,acrp_timeout_packet_resend);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_timeout_packet_send_failed,acrp_timeout_packet_send_failed);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_automatic_resend_time,acrp_automatic_resend_time);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_timeout_receiver_alive_checkpacket,acrp_timeout_receiver_alive_checkpacket);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_noprot_msg_noactivity_timeout,acrp_noprot_msg_noactivity_timeout);
  res &= coyot3::tools::json_import_value(js,JsField::acrp_publish_dynop_interval,acrp_publish_dynop_interval);
  if(res == true){
    CLOG_INFO("mqtt-gateway-acrp : acrp-config : from-json : loading configuration : OK");
  }else{
    CLOG_ERROR("mqtt-gateway-acrp : acrp-config : from-json : loading configuration : ERROR!");
  }
  return res;
}
Json::Value MqttGatewayAcrp::AcrpConfig::toJson() const
{
  Json::Value js;

  js[JsField::acrp_is_active]                          = acrp_is_active;
  js[JsField::acrp_timeout_packet_resend]              = static_cast<Json::Int>(acrp_timeout_packet_resend);
  js[JsField::acrp_timeout_packet_send_failed]         = static_cast<Json::Int>(acrp_timeout_packet_send_failed);
  js[JsField::acrp_automatic_resend_time]              = static_cast<Json::Int>(acrp_automatic_resend_time);
  js[JsField::acrp_timeout_receiver_alive_checkpacket] = static_cast<Json::Int>(acrp_timeout_receiver_alive_checkpacket);
  js[JsField::acrp_noprot_msg_noactivity_timeout]      = static_cast<Json::Int>(acrp_noprot_msg_noactivity_timeout);
  js[JsField::acrp_publish_dynop_interval]             = static_cast<Json::Int>(acrp_publish_dynop_interval);
  js[JsField::acrp_default_posftix_request]            = acrp_default_posftix_request;
  js[JsField::acrp_default_postfix_response]           = acrp_default_postfix_response;

  return js;
}

bool MqttGatewayAcrp::AcrpConfig::setAtmiConfig(AcrpTopicManagementInfo::Config& cfg)
{
  cfg.acrp_timeout_packet_resend              = acrp_timeout_packet_resend;
  cfg.acrp_timeout_packet_send_failed         = acrp_timeout_packet_send_failed;
  cfg.acrp_automatic_resend_time              = acrp_automatic_resend_time;
  cfg.acrp_timeout_receiver_alive_checkpacket = acrp_timeout_receiver_alive_checkpacket;
  cfg.acrp_noprot_msg_noactivity_timeout      = acrp_noprot_msg_noactivity_timeout;
  cfg.acrp_publish_dynop_interval             = acrp_publish_dynop_interval;
  cfg.postfix_req                             = acrp_default_posftix_request;
  cfg.postfix_res                             = acrp_default_postfix_response;

  return true;
}

void MqttGatewayAcrp::AcrpConfig::print_config_state()
{
  CLOG_INFO("acrp-config : state : ");
  CLOG_INFO(" : acrp_is_active = " << acrp_is_active);
  CLOG_INFO(" : acrp_use_defaults = " << acrp_use_defaults);
  CLOG_INFO(" : acrp_automatic_resend_time = " << acrp_automatic_resend_time);
  CLOG_INFO(" : acrp_default_posftix_request = " << acrp_default_posftix_request);
  CLOG_INFO(" : acrp_default_postfix_response = " << acrp_default_postfix_response);
  CLOG_INFO(" : acrp_timeout_packet_resend = " << acrp_timeout_packet_resend);
  CLOG_INFO(" : acrp_timeout_packet_send_failed = " << acrp_timeout_packet_send_failed);
  CLOG_INFO(" : acrp_automatic_resend_time = " << acrp_automatic_resend_time);
  CLOG_INFO(" : acrp_timeout_receiver_alive_checkpacket = " << acrp_timeout_receiver_alive_checkpacket);
  CLOG_INFO(" : acrp_noprot_msg_noactivity_timeout = " << acrp_noprot_msg_noactivity_timeout);
  CLOG_INFO(" : acrp_publish_dynop_interval = " << acrp_publish_dynop_interval);
}




//to-do
MqttGatewayAcrp::MqttGatewayAcrp()
:MqttGateway()
,acrpConfig()
,tmiStack()
,manager_checker_th_(nullptr)
,acrp_dynopub_last_operation(0)
{
  CLOG_INFO("mqtt-gateway-acrp : constructor");
}

MqttGatewayAcrp::MqttGatewayAcrp(const Json::Value& config_source)
:MqttGateway(config_source)
,acrpConfig()
,tmiStack()
,manager_checker_th_(nullptr)
,acrp_dynopub_last_operation(0)
{
  CLOG_INFO("mqtt-gateway-acrp : constructor (json-source)");
  if(acrpConfig.fromJson(config_source["acrp"]) == false)
  {
    CLOG_ERROR("mqtt-gateway-acrp : constructor : (json-source) : error loading acrp config from json source");
    gateway_state = GatewayState::ERROR;
  }else{
    CLOG_ERROR("mqtt-gateway-acrp : constructor : (json-source) : load-ok");
  }
  
}
MqttGatewayAcrp::~MqttGatewayAcrp()
{
  CLOG_WARN("TO-DO");
}

bool 
MqttGatewayAcrp::import_acrp_from_config_struct()
{
  acrpConfig.acrp_is_active             = config_acrp.acrp().is_active();
  acrpConfig.acrp_use_defaults          = config_acrp.acrp().use_defaults();
  acrpConfig.acrp_automatic_resend_time = config_acrp.acrp().automatic_resend_time();
  acrpConfig.acrp_default_posftix_request = config_acrp.acrp().default_posftix_request();
  acrpConfig.acrp_default_postfix_response = config_acrp.acrp().default_postfix_response();
  acrpConfig.acrp_timeout_packet_resend = config_acrp.acrp().timeout_packet_resend();
  acrpConfig.acrp_timeout_packet_send_failed = config_acrp.acrp().timeout_packet_send_failed();
  acrpConfig.acrp_automatic_resend_time = config_acrp.acrp().automatic_resend_time();
  acrpConfig.acrp_timeout_receiver_alive_checkpacket = config_acrp.acrp().timeout_receiver_alive_checkpacket();
  acrpConfig.acrp_noprot_msg_noactivity_timeout = config_acrp.acrp().noprot_msg_noactivity_timeout();
  acrpConfig.acrp_publish_dynop_interval = config_acrp.acrp().publish_dynop_interval();
  CLOG_INFO("mqtt-gateway-acrp : import-acrp-from-config-struct : done");
  acrpConfig.print_config_state();
  return true;
}


bool 
MqttGatewayAcrp::load_acrp_configuration(){
  Json::Value acrpCfgSource = _config_source["acrp"];
  if(acrpCfgSource.type() == Json::nullValue){
    CLOG_INFO("mqtt-gateway-acrp : load-acrp-configuration : importing from v2"
      "struct");
    return import_acrp_from_config_struct();
  }
  CLOG_INFO("mqtt-gateway-acrp : load-acrp-configuration : loading");
  if(acrpConfig.fromJson(acrpCfgSource) == false)
  {
    CLOG_ERROR("mqtt-gateway-acrp : load-acrp-configuration : error loading "
    "acrp config");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : load-acrp-configuration : loading : OK");
  return true;
}


bool 
MqttGatewayAcrp::set_configuration(const MqttGatewayAcrpConfig& conf){
  bool res = true;
  config_acrp = conf;
  res &= MqttGateway::set_configuration(conf);
  return res; 
}



bool 
MqttGatewayAcrp::Init(){
  CLOG_INFO("mqtt-gateway-acrp : init : initializing base layer");
  if(MqttGateway::Init() == false){
    CLOG_ERROR("mqtt-gateway-acrp : init : base layer initialization failed");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : init : loading acrp config");
  if(load_acrp_configuration() == false){
    CLOG_ERROR("mqtt-gateway-acrp : init : acrp configuration load error");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : init : done");
  return true;
}
bool MqttGatewayAcrp::Start(){

  CLOG_INFO("mqtt-gateway-acrp : start : initializing acrp topics");
  if(mqtt_gateway_init_acrp() == false){
    CLOG_ERROR("mqtt-gateway-acrp : start : acrp initialization error");
    return false;
  }

  CLOG_INFO("mqtt-gateway-acrp : start : starting base layer");
  if(MqttGateway::Start() == false){
    CLOG_ERROR("mqtt-gateway-acrp : start : error starting base layer");
    return false;
  }
  if(acrpConfig.acrp_is_active == true){
    
  }
  return true;
}
bool MqttGatewayAcrp::Stop(){
  return false;
}

bool MqttGatewayAcrp::acrp_register_publisher(const std::string& topic_base
                            ,bool is_active
                            ,int qos
                            ,const std::string& topic_postfix_req
                            ,const std::string& topic_postfix_res)
{
  CLOG_DEBUG(3,"mqtt-gateway-acrp : acrp register publisher - no callback : topic "
  "base [" << topic_base << "], postfix-req[" << topic_postfix_req << "], "
  "postfix-res[" << topic_postfix_res << "]. IS ACTIVE? : "
  << (is_active == true?"[true]":"[false]"));

  bool res;
  res = tmiStack.addPublisher(topic_base
                    ,topic_postfix_req
                    ,topic_postfix_res
                    ,std::bind(&MqttGatewayAcrp::on_publisher_operation_result_callback
                            ,this
                            ,std::placeholders::_1
                            ,std::placeholders::_2
                            ,std::placeholders::_3));
  if(!res){
    CLOG_WARN("mqtt-gateway-acrp : acrp register publisher - no callback : topic "
      "base [" << topic_base << "] error registering topic!");
      return false;
  }
  //acrpConfig.setAtmiConfig(tmiStack.getByBase(topic_base)->getConfig());
  tmiStack.getByBase(topic_base)->isActive(is_active);
    
  CLOG_INFO("mqtt-gateway-acrp : acrp register publisher : topic base [" 
    << topic_base << "] : registered OK");
  return true;
}                            

bool MqttGatewayAcrp::acrp_register_publisher(const std::string& topic_base
                            ,bool is_active
                            ,AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType onPublishResultCallback
                            ,int qos
                            ,const std::string& topic_postfix_req
                            ,const std::string& topic_postfix_res)
{
  CLOG_DEBUG(3,"mqtt-gateway-acrp : acrp-register-publisher - with callback : topic "
  "base [" << topic_base << "], postfix-req[" << topic_postfix_req << "], "
  "postfix-res[" << topic_postfix_res << "]. IS ACTIVE? : "
  << (is_active == true?"[true]":"[false]"));
  bool res;
  
  res = tmiStack.addPublisher(topic_base
                    ,topic_postfix_req
                    ,topic_postfix_res
                    ,onPublishResultCallback);
  if(!res){
    CLOG_WARN("mqtt-gateway-acrp : acrp-register-publisher : topic base ["
      << topic_base << "] error registering topic!");
      return false;
  }
  acrpConfig.setAtmiConfig(tmiStack.getByBase(topic_base)->getConfig());
  tmiStack.getByBase(topic_base)->isActive(is_active);
  
  CLOG_INFO("mqtt-gateway-acrp : acrp-register-publisher : topic base [" 
    << topic_base << "] : registered OK");
  return true;
}              


bool 
MqttGatewayAcrp::acrp_register_subscriber(const std::string& topic_base
  ,AcrpTopicManagementInfo::OnSubscriberDataCallbackType callback
  ,AcrpTopicManagementInfo::OnSubscriberParsedDataCallbackType parsedDataCallback
  , bool is_active
  , int qos
  ,const std::string& topic_postfix_req
  ,const std::string& topic_postfix_res)
{
  CLOG_DEBUG(3,"mqtt-manager-base : acrp register subscriber : topic "
    "base [" << topic_base << "], postfix-req[" << topic_postfix_req << "], "
    "postfix-res[" << topic_postfix_res << "]. IS ACTIVE? : "
    << (is_active == true?"[true]":"[false]"));
  bool res;
  res = tmiStack.addSubscriber(
       topic_base
      ,topic_postfix_req
      ,topic_postfix_res
      ,callback
      ,parsedDataCallback);
  
  if(!res){
    CLOG_WARN("mqtt-manager-base : acrp register subscriber : topic base ["
      << topic_base << "] error registering topic!");
      return false;
  }
  acrpConfig.setAtmiConfig(tmiStack.getByBase(topic_base)->getConfig());
  tmiStack.getByBase(topic_base)->isActive(is_active);
  CLOG_INFO("mqtt-manager-base : acrp register subscriber : topic base [" 
    << topic_base << "] : registered OK");
  return true;
}


bool 
MqttGatewayAcrp::acrp_publish(const std::string& topic_base, 
                  const Json::Value& packet, 
                  MqttGateway::Priority p)
{
  AcrpTopicManagementInfo* atmi;
  bool res = true;
  if(acrpConfig.acrp_is_active==false)
  {
    CLOG_DEBUG(7,"mqtt-gateway-acrp : publish : acrp is off. Direct "
      "publish at [" << topic_base << "]");
    return MqttGateway::publish(topic_base,packet,p);
  }

  atmi = tmiStack.getByBase(topic_base);
  if(atmi == nullptr)
  {
    //topic is not managed. will not publish.
    CLOG_WARN("mqtt-gateway-acrp : publish : error sending. topic is not "
      "registered! publishing at raw topic [" << topic_base << "]");
    return MqttGateway::publish(topic_base,packet,p);
  }
  
  if(atmi->isActive() == false){
    //topic is inactive. publishing at base topic.
    
    CLOG_DEBUG(5,"mqtt-gateway-acrp : publish : acrp off for a publication"
      "at topic [" << topic_base << "]");
    res &= MqttGateway::publish(topic_base,packet,p);
    return res;
  }

  AcrpPacket acrp(packet);  
  CLOG_DEBUG(5,"mqtt-gateway-acrp : publish : acrp on : registering " 
    " for a publication addressed to a topic base ["
    << atmi->getTopicBase() << "]=>[" << atmi->getTopicCrpReq() << "] ");
  
  res &= MqttGateway::publish(
                  atmi->getTopicCrpReq()
                  ,acrp.to_json()
                  ,p);
  //register the packet.
  if(!atmi->registerPacket(acrp))
  {
    //error registering packet.
    CLOG_WARN("mqtt-gateway-acrp : publish : acrp on : impossible to store"
      " a copy of the acrp packet for further use");
    return false;
  }
  return res;
}              

bool 
MqttGatewayAcrp::acrp_publisher_ack_common_callback(
          const std::string& topic_response
        , const uint8_t* data
        , size_t data_length)
{
  AcrpTopicManagementInfo* atmi;
  atmi = tmiStack.getByRes(topic_response);
  //CLOG_INFO("*** to-delete : pub common ack : searching [" << topic_response << "]");
  if(!atmi)
  {
    CLOG_WARN("mqtt-gateway-acrp : acrp-publisher-ack-common-callback : warning"
      " : received acknowledge packet but there's no registered topic at [" 
      << topic_response << "] but there are no registered publisher?");
    return true;
  }
  //CLOG_INFO("*** to-delete : received message at topic [" << topic_response << "]");
  atmi->markIncomingActivity();
  //
  CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp-publisher-ack-common-callback : "
    "received data for related publisher [" << atmi->getTopicBase() << "]");
  AcrpPacket acrp;
  bool res;
  res = acrp.from_stream(data,data_length);
  if(!res)
  {
    CLOG_WARN("mqtt-gateway-acrp : acrp-publisher-ack-common-callback : "
      "received corrupted data at publisher-ack-topic for base: [" 
      << atmi->getTopicBase() << "]");
    return true;
  }
  if(acrp.timestampOrigToken == 0)
  {
    //CLOG_INFO("*** to-delete : received message at topic IS DYNOP [" << topic_response << "]");
    CLOG_DEBUG(7,"mqtt-gateway-acrp : acrp-publisher-ack-common-callback : "
      "dynamic timeout optimization packet");
    return true;
  }
  //search if has been treated.
  AcrpPacket* acrpPtr;
  acrpPtr = atmi->getPacket(acrp.timestampOrigToken,acrp.tokenSec);

  if(acrpPtr == nullptr)
  {
    //already managed??
    CLOG_DEBUG(6,"mqtt-gateway-acrp : acrp-publisher-ack-common-callback : "
      "received acknowledge of reception for packet [" 
      << acrp.timestampOrigToken << "," << acrp.tokenSec << "], but may be "
      "redundant");
    return true;
  }
  if(acrpPtr->isTreated()== false)
  {
    //call the result callback.  
    if(atmi->publish_op_result_callback){
      atmi->publish_op_result_callback(
        atmi->getTopicBase()
        ,true
        ,acrpPtr->getPayload());
    }else{
      //just let go
    }
  }
  acrpPtr->jobDone(); // sets isTreated as true. (the packet will be "forgotten" at some moment)
  return true; //don't keep it at stack.
}                  

bool 
MqttGatewayAcrp::on_publisher_operation_result_callback(const std::string& topic
                                    , bool result
                                    , const Json::Value& sourceData)
{
  CLOG_DEBUG(7,"mqtt-gateway-acrp : on publisher operation result callback : "
    " result [" << (result == true?"true":"false") << "] for topic : ["
    << topic << "]");
  return true;
}

bool 
MqttGatewayAcrp::register_subscriber(
  const std::string& topic_base
  ,AcrpTopicManagementInfo::OnSubscriberDataCallbackType       callback
  ,AcrpTopicManagementInfo::OnSubscriberParsedDataCallbackType parsedDataCallback
  ,bool is_active
  ,const std::string& topic_postfix_req
  ,const std::string& topic_postfix_res)
{
  CLOG_DEBUG(3,"mqtt-gateway-acrp : register-subscriber : topic(" << topic_base 
  << ")(acrp:" << (is_active==true?"true":"false") << "(psreq:" 
  << topic_postfix_req << ";psres:" << topic_postfix_res <<")"); 
  bool res = true;
  res = tmiStack.addSubscriber(
    topic_base
    ,topic_postfix_req
    ,topic_postfix_res
    ,callback
    ,parsedDataCallback);
  if(!res){
    CLOG_WARN("mqtt-gateway-acrp : register-subscriber : topic(" 
    << topic_base << ")(acrp:" << (is_active==true?"true":"false")
    << " : ERROR preparing subscription to topic");
    return false;
  }
  // ? la configuración de la subscripción modifica la configuración global?
  tmiStack.getByBase(topic_base)->isActive(is_active);
  CLOG_DEBUG(3,"mqtt-gateway-acrp : register-subscriber : topic(" << topic_base 
  << ")(acrp:" << (is_active==true?"true":"false") << "(psreq:" 
  << topic_postfix_req << ";psres:" << topic_postfix_res <<") : DONE"); 
  return true;
}                                    

bool MqttGatewayAcrp::acrp_subscription_common_callback(const std::string& topic
                                    ,const uint8_t* data
                                    ,size_t dataSize)
{
  AcrpTopicManagementInfo* atmi;
  atmi = tmiStack.getByReq(topic);
  bool res;
  if(!atmi){
    CLOG_WARN("mqtt-gateway-acrp : acrp-subscription-common-callback : error "
    "obtaining current ATMI for topic-req [" << topic << "]");
    return true;
  }
  atmi->markIncomingActivity();
  AcrpPacket acrp;
  res = acrp.from_stream(data,dataSize);
  if(!res){
    CLOG_WARN("mqtt-gateway-acrp : acrp-subscription-common-callback : error "
    "parsing incoming packet for topic [" << topic << "]: data((" 
    << coyot3::tools::toString(data,dataSize) <<"))");
    return true;
  }
  AcrpPacketAcknowledgement acrpAck(acrp);
  CLOG_DEBUG(7,"mqtt-gateway-acrp :-subscription-common-callback : "
  "sending acknowledgement for packet [" << acrp.timestampOrigToken << "](iter:" 
  << acrp.getIteration() << ")");
  if(publish(atmi->getTopicCrpRes(),acrpAck.to_json(),atmi->getPriority()) == true){
    CLOG_DEBUG(7,"mqtt-gateway-acrp : acrp-sub-common-callback : "
    "ack sent for (" << acrp.timestampOrigToken << ") at topic(base:" 
    << atmi->getTopicBase() << ")");
    atmi->markLastPacketSend();
  }else{
    CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp-sub-common-callback : "
    "ack NOT sent, awaiting iteration for (" << acrp.timestampOrigToken 
    << ") at topic(base:" << atmi->getTopicBase() << ")");
  }
  //5.4 of the document : timestampOrigToken = 0
  if(acrpAck.timestampOrigToken == 0){
    CLOG_DEBUG(7,"mqtt-gateway-acrp : acrp-sub-common-callback : "
      "this is just a dyn-opt packet for acrp. doing nothing");
    return true;
  }

  AcrpPacket* acrpSourcePtr = atmi->getPacket(acrp);
  acrpSourcePtr = atmi->getPacket(acrp);
  if(!acrpSourcePtr){
    //first incoming packet for this request: must invoke initial callback.
    acrpSourcePtr = atmi->registerPacket(acrp);
    if(!acrpSourcePtr)
    {
      CLOG_ERROR("mqtt-gateway-acrp : acrp-sub-common-callback : error"
        " registering Acrp packet for topic base [" << atmi->getTopicBase()
        << "]");

      //TO-DO : INCLUDE SOME KIND OF ON-FATAL-ERROR CALLBACK HERE
      return false;
    }
    //invoke the callback
    if(atmi->on_parsed_data_callback){
      CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp-sub-common-callback : invoking "
      "callback")
      atmi->on_parsed_data_callback(atmi->getTopicBase(),acrp.getPayload());
    }else{
      CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp-sub-common-callback : no parsed-data-callback!");
      //std::function<bool(const std::string&,const uint8_t*,size_t)>
      if(atmi->on_data_callback){
        std::string pl = acrp.getPayloadStringified();
        atmi->on_data_callback(atmi->getTopicBase(),(const uint8_t*)(pl.c_str()),pl.size());
      }else{
        CLOG_WARN("mqtt-gateway-acrp : acrp-sub-common-callback : NO CALLBACK SET!");
      }
    }
    acrpSourcePtr->jobDone();
  }
  return true;
}              

bool MqttGatewayAcrp::acrpInitialize()
{
  CLOG_INFO("mqtt-gateway-acrp : acrp-initialize : begin");
  if(acrpConfig.acrp_is_active == false){
  CLOG_DEBUG(2, "mqtt-gateway-acrp : acrp-initialize : acrp is INACTIVE : "
    "searching for subscribers");
    size_t resd;
    resd = tmiStack.forEach([&](AcrpTopicManagementInfo* tmi){
      CLOG_DEBUG(2, "mqtt-gateway-acrp : acrp-initialize : checking TMI : [" 
        << tmi->getTopicBase() << "]");
      if(tmi->getRole() == AcrpTopicManagementInfo::Role::SUBSCRIBER)
      {
        //directly subscribe the topic to the final callback
        CLOG_INFO("mqtt-gateway-acrp : acrp-initialize : ACRP OFF : preparing "
          "subscription to topic [" << tmi->getTopicBase() << "](raw-data)");
        if(subscribe_to_topic(tmi->getTopicBase()
          , tmi->on_data_callback
          , tmi->getPriority())== false)
        {
          CLOG_WARN("mqtt-gateway-acrp : acrp-initialize : ACRP OFF : error "
            "preparing subscription to base topic [" 
            << tmi->getTopicBase() << "](raw-data)");
          return false;
        }
      }
      return true;
    });
    if(resd != tmiStack.size())
    {
      CLOG_WARN("mqtt-gateway-acrp : acrp-initialize : ACRP OFF : error "
        "making raw subscriptions");
      return false;
    }
    CLOG_INFO("mqtt-gateway-acrp : acrp-initialize : ACRP OFF : prepared.");
    return true;
  }
  //if acrp is active, we set-up the gateway.
  if(acrpConfig.acrp_is_active == false)
  {
    CLOG_INFO("mqtt-gateway-acrp : acrp is not initialized : all ended ok");
    return true;
  }

  bool res;
  res = mqtt_gateway_init_acrp();
  if(!res)
  {
    CLOG_WARN("mqtt-gateway-acrp : acrp-initialize : error initializing the "
      "mqtt-gateway for ACRP communication");
  }
  acrp_start_check();
  return res;
}

bool MqttGatewayAcrp::mqtt_gateway_init_acrp()
{
  size_t res = 0;
  CLOG_INFO("mqtt-gateway-acrp : mqtt-gateway-init-acrp : begin : checking for "
    << tmiStack.size() << " items");
  res = tmiStack.forEach([&](AcrpTopicManagementInfo* tmi){
    switch(tmi->getRole())
    {
      case AcrpTopicManagementInfo::Role::SUBSCRIBER:
        return mqtt_gateway_init_acrp_subscriber(tmi);
        break;
      case AcrpTopicManagementInfo::Role::EMITTER:
        return mqtt_gateway_init_acrp_publisher(tmi);
        break;
      default:
        CLOG_WARN("mqtt-gateway-acrp : start mqtt gateway init acrp : error "
          "obtaining the role of acrp-topic-management-info for ["
          << tmi->getTopicBase() << "]");
        return false;
    }
    return true;
  });
  if(res != tmiStack.size())
  {
    CLOG_WARN("mqtt-gateway-acrp : mqtt-gateway-init-acrp : not all atmis were "
      "correctly initialized!!! You may need to check the configuration");
  }
  return (res == tmiStack.size());
}
bool MqttGatewayAcrp::mqtt_gateway_init_acrp_publisher(AcrpTopicManagementInfo* tmi)
{
  bool res;
  CLOG_DEBUG(3,"mqtt-gateway-acrp : set mqtt gateway init acrp publisher : "
    "registering publisher [" << tmi->getTopicBase() << "] at the mqtt gateway");
  if(tmi->isActive() == false)
  {
    CLOG_INFO("mqtt-gateway-acrp : set mqtt gateway init acrp publisher : "
      "publisher is not acrp. Ignoring further data for this publisher : ["
      << tmi->getTopicBase() << "]");
    return true;
  }
  CLOG_INFO("mqtt-gateway-acrp : set mqtt gateway init acrp publisher : "
      "publisher is acrp. Subcribing to response topic : ["
      << tmi->getTopicCrpRes() << "]");
  res = subscribe_to_topic(tmi->getTopicCrpRes()
      ,std::bind(&MqttGatewayAcrp::acrp_publisher_ack_common_callback
      ,this
      ,std::placeholders::_1
      ,std::placeholders::_2
      ,std::placeholders::_3)
      ,tmi->getPriority());

  if(res == true)
  {
    CLOG_INFO("mqtt-gateway-acrp : start mqtt gateway : registered [" 
          << tmi->getTopicBase() 
          << "]");
  }else{
    CLOG_WARN("mqtt-gateway-acrp : start mqtt gateway : reg acrp publisher : "
      "error registering subscription to ACRP-RES topic [" 
      << tmi->getTopicCrpRes() << "]");
    return false;
  }
  
  return true;
}
bool 
MqttGatewayAcrp::mqtt_gateway_init_acrp_subscriber(AcrpTopicManagementInfo* tmi)
{
  //subscribe to ACK
  CLOG_DEBUG(4,"mqtt-gateway-acrp : mqtt-gateway-init-acrp-subscriber : "
    "topic base : [" << tmi->getTopicBase() << "]");
  bool res;
  if(tmi->isActive() == false)
  {
    //not managed. link directly done to the user class
    CLOG_INFO("mqtt-gateway-acrp : set mqtt-gateway-init-acrp-subscriber : not "
     "active for topic [" << tmi->getTopicBase() << "]");
    res = subscribe_to_topic(tmi->getTopicBase()
                                ,tmi->on_data_callback
                                ,tmi->getPriority());
    if(res == false)
    {
      CLOG_WARN("mqtt-gateway-acrp : set mqtt-gateway-init-acrp-subscriber : "
        "error making subscription to gateway for topic ["
        << tmi->getTopicBase() << "]");
      return false;
    }
  }

  CLOG_INFO("mqtt-gateway-acrp : mqtt-gateway-init-acrp-subscriber : "
  "making acrp subscription for [" << tmi->getTopicBase() << "] at (" 
  << tmi->getTopicCrpReq() << ")");

  res = subscribe_to_topic(tmi->getTopicCrpReq()
          ,std::bind(&MqttGatewayAcrp::acrp_subscription_common_callback
            ,this
            ,std::placeholders::_1
            ,std::placeholders::_2
            ,std::placeholders::_3)
          ,tmi->getPriority());
  if(!res)
  {
    CLOG_WARN("mqtt-gateway-acrp : set mqtt-gateway-init-acrp-subscriber : "
      "error preparing the subscription to topic [" << tmi->getTopicCrpReq() 
      << "]");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : start mqtt-gateway-init-acrp-subscriber : "
    "OK : topic base : [" << tmi->getTopicBase() << "]");
  return true;
}

bool MqttGatewayAcrp::acrp_start_check()
{
  if(acrpConfig.acrp_is_active == false)
  {
    CLOG_INFO("mqtt-gateway-acrp : acrp-start-check : acrp is disabled for "
      "this module. Ignoring stage");
    return true;
  }
  if(!manager_checker_th_)
  {
    manager_checker_th_ = new(std::nothrow) coyot3::tools::ControlThread(
      std::bind(&MqttGatewayAcrp::acrp_management_pulse,this),"acrp-thread"
    );  
    if(!manager_checker_th_)
    {
      CLOG_ERROR("mqtt-gateway-acrp : acrp start check : ERROR : FATAL? "
        "impossible to allocate timer");
      exit(1);
    }
    // if(!connect(timerManagerChecker,&QTimer::timeout
    //         ,this,&MqttGatewayAcrp::acrp_management_pulse))
    // {
    //   CLOG_ERROR("mqtt-gateway-acrp : acrp start check : ERROR : FATAL : "
    //     "impossible to connect acrp checker timer to this instance");
    //   exit(1);
    // }
    manager_checker_th_->setInterval(std::min(
               acrpConfig.acrp_timeout_packet_resend/2
              ,acrpConfig.acrp_publish_dynop_interval/2));
  }
    
  manager_checker_th_->start();
  CLOG_INFO("mqtt-gateway-acrp : acrp start check : timer initiated");
  return true;
}
bool MqttGatewayAcrp::acrp_stop_check()
{
  if(!manager_checker_th_)
  {
    CLOG_WARN("mqtt-gateway-acrp : acrp-stop-check : there's no active timer");
    return false;
  }
  CLOG_INFO("mqtt-gateway-acrp : acrp-stop-check : stopping check");
  manager_checker_th_->stop();
  delete manager_checker_th_;
  manager_checker_th_ = nullptr;
  return true;
}

/**
 * @brief: check one by one every topic
 *              for subscribers:
 *                    - remove messages without activity
 *              for publishers : 
 *                    - republish if not acknowledged
 *                    - remove acknowledged with no activity
 *      
 * */
void MqttGatewayAcrp::acrp_management_pulse()
{
  //CLOG_INFO("*** to delete : acrp management pulse");
  tmiStack.forEach([&](AcrpTopicManagementInfo* atmi){
    std::vector<std::pair<int64_t,int64_t>> toDel;

    switch(atmi->getRole())
    {
      case AcrpTopicManagementInfo::Role::EMITTER:
        {
          //remember: foreachMessage is mutually exclusive
          toDel.clear();
          //CLOG_INFO("*** to delete : checking emitter [" << atmi->getTopicBase() << "]");
          atmi->forEachMessage([&](AcrpPacket* packet){
            //CLOG_INFO("*** to delete : checking emitter [" << atmi->getTopicBase() << "] ::: message [" << packet->timestampOrigToken << "," << packet->tokenSec << "]");
            bool res;
            res = acrp_management_pulse_check_msg_publisher(atmi,packet);
            if(res == false)
            {
              //prepare to delete.
              toDel.push_back(std::make_pair(packet->timestampOrigToken,packet->tokenSec));
            }
            return true;
          });
          //delete olds
          for(const std::pair<int64_t,int64_t>& p:toDel)
          {
            //CLOG_INFO("*** TO DELETE : unregistering publisher packet [" << p.first << "," << p.second << "]");
            atmi->unregisterPacket(p.first,p.second);
          }
        }
        break;
      case AcrpTopicManagementInfo::Role::SUBSCRIBER:
        {
          toDel.clear();
          //remember: foreachMessage is mutually exclusive
          atmi->forEachMessage([&](AcrpPacket* packet){
            bool res;
            res = acrp_management_pulse_check_msg_subscriber(atmi,packet);
            if(res == false){
              //prepare message for deletion
              toDel.push_back({packet->timestampOrigToken,packet->tokenSec});
            }
            return true;
          });
          //delete messages
          for(const std::pair<int64_t,int64_t>& p : toDel)
          {
            atmi->unregisterPacket(p.first,p.second);
          }
        }
        break;
      default:
        CLOG_ERROR("mqtt-gateway-acrp : acrp management pulse : error "
          "error evaluating topic role. Base=[" << atmi->getTopicBase() << "]");
    }

    for(std::vector<std::pair<int64_t,int64_t>>::iterator it = toDel.begin();it != toDel.end();++it)
    {
      bool res;
      CLOG_DEBUG(4,"mqtt-gateway-acrp : acrp management pulse : removing "
        "message [" << it->first << "] from stack of topic base [" 
        << atmi->getTopicBase() << "]");
      res = atmi->unregisterPacket(it->first,it->second);
      if(!res)
      {
        CLOG_WARN("mqtt-gateway-acrp : acrp management pulse : warning : unable"
          "to remove message [" << it->first << "] from stack of topic-base"
          << atmi->getTopicBase() << "]");
      }
    }
    return true;
  });
  acrp_dynopub_make_operations();
}
/**
 * @brief 
 * 
 * @param tmi 
 * @param acrpPtr 
 * @return true : keeps the packet in stack
 * @return false : removes the packet from stack
 */
bool MqttGatewayAcrp::acrp_management_pulse_check_msg_publisher(
  AcrpTopicManagementInfo* tmi
  ,AcrpPacket* acrpPtr)
{
  bool toRet = false; //by default, delete this packet.
  int64_t thisMoment = coyot3::tools::getCurrentTimestamp();
  int64_t diff = thisMoment - acrpPtr->timestampOrigToken;
  if(acrpPtr->isTreated() == true)
  {
    //packet already treated, remove from stack... no further evaluations are needed.
    return false;
  }
  
  //republish if(late)
  //CLOG_INFO("*** to delete : thismoment = " << thisMoment);
  //CLOG_INFO("*** to delete : acrpPtr->timestampOrigToken = " << acrpPtr->timestampOrigToken);
  //CLOG_INFO("*** to delete : diff = " << thisMoment - acrpPtr->timestampOrigToken);
  //CLOG_INFO("*** to delete : tmi->getTimeoutPacketSendFailed() = " << tmi->getTimeoutPacketSendFailed());
  if(diff > tmi->getTimeoutPacketSendFailed())
  {
    if(tmi->publish_op_result_callback)
    {
      CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp-management-pulse-check-publisher : "
        "packet [" << acrpPtr->timestampOrigToken << "," << acrpPtr->tokenSec 
        << "] publish error, invoking on publish-error callback");
      tmi->publish_op_result_callback(tmi->getTopicBase(),false,acrpPtr->to_json());
    }
    //removes the packet from the stack
    return false;
  }
  //else if we are out of the interval when we are resending messages.
  else if(diff > tmi->getAutomaticResendTime())
  {
    //nothing to do at this point.. just let the time go.
    //CLOG_INFO("*** to delete : error publishing message : ignoring : ((" << acrpPtr->to_json() << "))");
    CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp pulse check publisher msg : "
      "packet not acknowledged, out of resend window. doing nothing");
    return true;
  }
  //else if it is needed to republish the message.
  else if((thisMoment - acrpPtr->getLastInternalActivityTs()) > tmi->getTimeoutPacketResend())
  {
    bool res;
    CLOG_DEBUG(5,"mqtt-gateway-acrp : acrp pulse check publisher msg : "
      "for message token [" << acrpPtr->timestampOrigToken << "] republishing");
    acrpPtr->updateIteration(); //and updates last internal activity
    res = publish(tmi->getTopicCrpReq(),acrpPtr->to_json(),tmi->getPriority());
    if(!res)
    {
      CLOG_WARN("mqtt-gateway-acrp : acrp pulse check publisher msg : "
        "for messge token [" << acrpPtr->timestampOrigToken << "] ")
    }
    return true; // don't delete the packet
  }
  //no need to resend or treat in any way the packet
  return true;
}

bool MqttGatewayAcrp::acrp_management_pulse_check_msg_subscriber(AcrpTopicManagementInfo* tmi,AcrpPacket* acrpPtr)
{
  //The "only" thing to do is to remove messages with no activity.
  int64_t diff = coyot3::tools::getCurrentTimestamp() - acrpPtr->timestampOrigToken;
  if(diff < tmi->getPacketStoreExpirationTimeout())
  {
    return false;
  }
  return true;
}

int  MqttGatewayAcrp::acrp_dynopub_make_operations()
{
  int64_t thisMoment = coyot3::tools::getCurrentTimestamp();
  int ret = 0;
  tmiStack.forEach([&](AcrpTopicManagementInfo* tmi){
    if(tmi->getRole() == AcrpTopicManagementInfo::Role::SUBSCRIBER)
    {
      ++ret;
      return true;
    }
    if(tmi->isActive() == false)
    {
      ++ret;
      return true;
    }
    AcrpPacketDynamicOptimization dynopt;
    // if there
    if(
      ((thisMoment - tmi->getLastIncomingActivityTs()) > tmi->getDynopPublishInterval())
      &&((thisMoment - tmi->getLastPacketSend()) > tmi->getDynopPublishInterval())
    ){
      CLOG_DEBUG(3,"mqtt-gateway-acrp : acrp dynopub_make_operations : "
        "publishing acrp-dyn-timeout-optimization packet");
      
      publish(tmi->getTopicCrpReq()
                          ,dynopt.to_json()
                          ,coyot3::communication::mqtt::MqttGateway::Priority::LOW);
    }
    

    if((thisMoment - tmi->getLastIncomingActivityTs()) > 
      CYTOOLS_MQTTACRP_DYNOP_NOCOMMFACYTOR * tmi->getDynopPublishInterval())
    {
      //CLOG_INFO("*** to-delete : TIEMPO DE ERROR DE ENVÍO OPTIMIZADO" << tmi->getTopicBase());
      tmi->setTimeoutPacketResendReducedTime();
    }else{
      //CLOG_INFO("*** to-delete : TIEMPO DE ERROR DE ENVÍO COMO CONFIGURACIÓN [" << tmi->getTopicBase());
      tmi->setTimeoutPacketResendConfigTime();
    }
    ++ret;
    return true;
  });
  return ret;
}






}
}
}