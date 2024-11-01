/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * */


#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>

namespace coyot3{
namespace communication{
namespace mqtt{




CYT3MACRO_model_class_definitions(
  MqttGatewayConfigObject
  , 
  , ( virtual void print_current_config_state() )
  , ( )
    , debug_mode                          , bool        , false 
    , host_address                        , std::string , ""
    , host_port                           , int32_t     , 0
    , qos                                 , int         , -1
    , qos_max                             , int         , 2
    , timeout                             , int         , 30
    , show_debug_msgs                     , bool        , false
    , important_messages_timeout_ms       , int         , 5
    , important_messages_max_retries      , int         , 5
    , clean_session                       , bool        , true
    , client_id                           , std::string , ""
    , client_id_postfix_rand_str_length   , int         , 10
    , user                                , std::string , ""
    , password                            , std::string , ""
    , base_path                           , std::string , ""
    , certificates_relative_path          , bool        , true
    , certificates_ca                     , std::string , ""
    , certificates_cert                   , std::string , ""
    , certificates_key                    , std::string , ""
    , certificates_passphrase             , std::string , ""
    , tls_version                         , std::string , ""
    , ciphers                             , std::string , "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384:DHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:DHE-RSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:DHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA384:DHE-RSA-AES256-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES256-SHA:ECDHE-RSA-AES256-SHA:DHE-RSA-AES256-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES128-SHA:DHE-RSA-AES128-SHA:RSA-PSK-AES256-GCM-SHA384:DHE-PSK-AES256-GCM-SHA384:RSA-PSK-CHACHA20-POLY1305:DHE-PSK-CHACHA20-POLY1305:ECDHE-PSK-CHACHA20-POLY1305:AES256-GCM-SHA384:PSK-AES256-GCM-SHA384:PSK-CHACHA20-POLY1305:RSA-PSK-AES128-GCM-SHA256:DHE-PSK-AES128-GCM-SHA256:AES128-GCM-SHA256:PSK-AES128-GCM-SHA256:AES256-SHA256:AES128-SHA256:ECDHE-PSK-AES256-CBC-SHA384:ECDHE-PSK-AES256-CBC-SHA:SRP-RSA-AES-256-CBC-SHA:SRP-AES-256-CBC-SHA:RSA-PSK-AES256-CBC-SHA384:DHE-PSK-AES256-CBC-SHA384:RSA-PSK-AES256-CBC-SHA:DHE-PSK-AES256-CBC-SHA:AES256-SHA:PSK-AES256-CBC-SHA384:PSK-AES256-CBC-SHA:ECDHE-PSK-AES128-CBC-SHA256:ECDHE-PSK-AES128-CBC-SHA:SRP-RSA-AES-128-CBC-SHA:SRP-AES-128-CBC-SHA:RSA-PSK-AES128-CBC-SHA256:DHE-PSK-AES128-CBC-SHA256:RSA-PSK-AES128-CBC-SHA:DHE-PSK-AES128-CBC-SHA:AES128-SHA:PSK-AES128-CBC-SHA256:PSK-AES128-CBC-SHA"
)






void MqttGatewayConfigObject::print_current_config_state()
{
  CLOG_INFO("    config : debug_mode                        = " << debug_mode());
  CLOG_INFO("    config : host_address                      = " << host_address());
  CLOG_INFO("    config : host_port                         = " << host_port());
  CLOG_INFO("    config : qos                               = " << qos());
  CLOG_INFO("    config : qos_max                           = " << qos_max());
  CLOG_INFO("    config : timeout                           = " << timeout());
  CLOG_INFO("    config : show_debug_msgs                   = " << show_debug_msgs());
  CLOG_INFO("    config : important_messages_timeout_secs   = " << important_messages_timeout_ms());
  CLOG_INFO("    config : important_messages_max_retries    = " << important_messages_max_retries());
  CLOG_INFO("    config : clean_session                     = " << clean_session());
  CLOG_INFO("    config : client_id                         = " << client_id());
  CLOG_INFO("    config : user                              = " << user());
  CLOG_INFO("    config : password                          = " << password());
  CLOG_INFO("    config : base_path                         = " << base_path());
  CLOG_INFO("    config : certificates_relative_path        = " << certificates_relative_path());
  CLOG_INFO("    config : certificates_ca                   = " << certificates_ca());
  CLOG_INFO("    config : certificates_cert                 = " << certificates_cert());
  CLOG_INFO("    config : certificates_key                  = " << certificates_key());
  CLOG_INFO("    config : certificates_passphrase           = " << certificates_passphrase());
  CLOG_INFO("    config : tls_version                       = " << tls_version());
  CLOG_INFO("    config : ciphers                           = " << ciphers());
}

CYT3MACRO_model_class_serializable_json_definitions(
  MqttGatewayConfigObject
  , 
  , 
  , ( )
  , ( )
    , debug_mode                          , "debug_mode"                      , 
    , host_address                        , "host_address"                    , 
    , host_port                           , "host_port"                       , 
    , qos                                 , "qos"                             , 
    , qos_max                             , "qos_max"                         , 
    , timeout                             , "timeout"                         , 
    , show_debug_msgs                     , "show_debug_msgs"                 , 
    , important_messages_timeout_ms       , "important_messages_timeout_ms"   , 
    , important_messages_max_retries      , "important_messages_max_retries"  , 
    , clean_session                       , "clean_session"                   , 
    , client_id                           , "client_id"                       , 
    , user                                , "user"                            , 
    , password                            , "password"                        , 
    , base_path                           , "base_path"                       , 
    , certificates_relative_path          , "certificates_relative_path"      , 
    , certificates_ca                     , "certificates_ca"                 , 
    , certificates_cert                   , "certificates_cert"               , 
    , certificates_key                    , "certificates_key"                , 
    , certificates_passphrase             , "certificates_passphrase"         , 
    , tls_version                         , "tls_version"                     , 
    , ciphers                             , "ciphers"                         , 
)


  
  
  
void MqttGateway::_debug_level_set_mod_configurations(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
}
bool MqttGateway::set_configuration(const Json::Value& source)
{
  CLOG_DEBUG(3,"mqtt-gw : set configuration : begin");
  _config_source = source;
  return (_load_configuration_from_jsoncpp_struct()? true : set_configuration_(source));
}

bool MqttGateway::set_configuration(const MqttGatewayConfigObject& conf){
  config = conf;
  return update_config_from_struct_();
}

bool
MqttGateway::update_config_from_struct_()
{
  bool checkGlobal = true, checkOp;
  std::stringstream sstr;
  checkGlobal &= checkOp = (config.host_address().size() != 0);
  if(!checkOp)sstr << "host address cannot be empty ;";
  checkGlobal &= checkOp = (config.host_port() != 0);
  if(!checkOp)sstr << "host port MUST be set ;";
  has_user = (config.user().size() != 0);
  has_password = (config.password().size() != 0);
  checkGlobal &= checkOp = (has_user == has_password);
  if(!checkOp)sstr << "user and password MUST BE BOTH set, or NOT set ;";

  has_ca_file = (config.certificates_ca().size() != 0);
  has_cert_file = (config.certificates_cert().size() != 0);
  has_key_file = (config.certificates_key().size() != 0);
  has_key_passphrase = (config.certificates_passphrase().size() != 0);

  checkGlobal &= checkOp = (has_cert_file == has_key_file);
  if(!checkOp)sstr << "certificate and key file paths MUST BE BOTH set, "
    "or NOT set ";
  
  checkGlobal &= checkOp = (!has_key_passphrase) || (((has_key_passphrase && has_key_file) || (!has_key_file)));
  if(!checkOp)sstr << "if passphrase is defined, you MUST also define CERT and KEY ;";

  has_tls_version = (config.tls_version().size() != 0);
  has_ciphers     = (config.ciphers().size() != 0);

  checkGlobal &= checkOp = ((has_tls_version && has_ciphers) == (has_key_file && has_cert_file));
  if(!checkOp)sstr << "if tls version is set, cert and key files path MUST BE defined, OR NONE of them ;";
  
  checkOp &= (     (config.important_messages_max_retries() >= 1)
                && (config.important_messages_max_retries() <= 30) );
  if(!checkOp){
    CLOG_WARN("mqtt-gw : set-config-v2- : important-messages-republish times "
      "MUST BE an integer between 1 and 30. Now setting default = 5");
    config.important_messages_max_retries(5);
  }
  checkOp &= (     (config.important_messages_timeout_ms() >= 5000)
                && (config.important_messages_max_retries() <= 60000) );
  if(!checkOp){
    CLOG_WARN("mqtt-gw : set-config-v2- : important-messages-broker-rx-ack time"
      " MUST BE an integer between 5 and 60. Now setting default = 30");
    config.important_messages_max_retries(30000);
  }

  //certs-base-path - no need

  checkGlobal &= checkOp = (config.qos() != -1) 
    && ((config.qos() >=0) || (config.qos() <=2));
  if(!checkOp){
    sstr << "QoS MUST BE set, and MUST BE a value = 0, 1 or 2 ;";
  }
  checkOp = ((config.qos_max() >= 0) && (config.qos_max() <= 2));
  if(!checkOp){
    CLOG_WARN("mqtt-gw : set-config- : max-qos MUST BE 0, 1 or 2. "
      "setting default = 2");
    config.qos_max(2);
  }
  

  if(config.timeout() < 10){
    CLOG_WARN("mqtt-gw : connection timeout is lower than 10 seconds. setting it to 10");
    config.timeout(10);
  }
  if(config.timeout() > 60){
    CLOG_WARN("mqtt-gw : connection timeout is higher than 60 seconds. setting it to 60");
    config.timeout(60);
  }

  if(!checkGlobal){
    CLOG_WARN("mqtt-gw: set-config-v2 : there were errors setting the "
      "configuration. Please, check the following notice lines to be able to "
      "debug your configuration:");
    CLOG_WARN(" : " << sstr.str());
    
  }else{
    CLOG_INFO("mqtt-gw : set-config-v2 : configuration is set.");
  }
  CLOG_INFO("mqtt-gw : set-config-v2 : current configuration ==>");
  config.print_current_config_state();
  return checkGlobal;
}

bool 
MqttGateway::set_configuration_(const Json::Value& confStruct){
  
  CLOG_INFO_OR_ALERT((config << confStruct) == false
    ,"mqtt-gw : set-configuration- : deserialization "
    , "success"
    ,"WARN : may have been incomplete or may not contain all data.")
  
  return update_config_from_struct_();

  
}
bool MqttGateway::_load_configuration_from_jsoncpp_struct()
{
  CLOG_DEBUG(3,"mqtt-gw : load configuration : begin");
  
  if(configuration.fromJson(_config_source) == false){
    CLOG_WARN("mqtt-gw : load-configuration : new version : error importing data")
  }
  std::string buffer;
  bool toret= true;
  //init parameters flags.
  has_user = has_password = has_ca_file = has_cert_file = has_key_file = 
      has_key_passphrase = false;
  
  bool load_not_err_ = true;
  
  CLOG_INFO("mqtt-gw : load config : loaded mqtt gateway params. Now "
  "reading...");
  std::string bufferStr;
  int         bufferInt;
  int         bufferBool;


  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_PACKAGE_LOCATION,bufferStr);
    config.base_path(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_MQTT_DEBUG,bufferBool);
    config.debug_mode(bufferBool);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_ADDRESS,bufferStr);
    config.host_address(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_PORT,bufferInt);
    config.host_port(bufferInt);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_QOS,bufferInt);
    config.qos(bufferInt);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_QOS_MAX,bufferInt);
    config.qos_max(bufferInt);
    
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_TIMEOUT,bufferInt);
    config.timeout(bufferInt);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_MOSQUITTO_DEBUG,bufferBool);
    config.show_debug_msgs(bufferBool);
  
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CONTROLLER_REPUBLISH_MSGTIMEOUT,bufferInt);
    config.important_messages_timeout_ms(bufferInt);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CONTROLLER_REPUBLISH_MAXRETRIES,bufferInt);
    config.important_messages_max_retries(bufferInt);

  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CLEAN_SESSION,bufferBool);
    config.clean_session(bufferBool);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CLIENT_IDENTIFIER,bufferStr);
    config.client_id(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_USER,bufferStr);
    config.user(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_PASSWORD,bufferStr);
    config.password(bufferStr);

  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CA_FILE,bufferStr);
    config.certificates_ca(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CERT_FILE,bufferStr);
    config.certificates_cert(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_KEY_FILE,bufferStr);
    config.certificates_key(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_PASSPHRASE,bufferStr);
    config.certificates_passphrase(bufferStr);

    
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_TLS_VERSION,bufferStr);
    config.tls_version(bufferStr);
  load_not_err_ &= coyot3::tools::json_import_value(_config_source,MQTT_GATEWAY_CONFIG_CIPHERS,bufferStr);
    config.ciphers(bufferStr);


  if(load_not_err_ == false){
    CLOG_ERROR("cyt-mqtt-gateway : load-config : error loading parameters for "
      "the unit, please, review your config object");
    return false;
  }
  if(config.host_address().size() == 0){
    CLOG_ERROR("cyt-mqtt-gateway : load-config : error : 'host' MUST be described");
    return false;
  }
  if(config.qos() < 0 || config.qos() > 2 || config.qos_max() < 0 || config.qos_max() > 2){
    CLOG_ERROR("cyt-mqtt-gateway : load-config : error : 'qos' and 'max_qos' must be 0, 1 or 2");
    return false;
  }
  if(config.timeout() < 10){
    CLOG_WARN("cyt-mqtt-gateway : load-config : warning : 'timeout' will be "
      "set from (" << config.timeout() << ") to 10");
    config.timeout(10);
  }

  if((config.user().size() != 0) && (config.password().size() != 0)){
    CLOG_INFO("cyt-mqtt-gateway : load-config : user & password are defined.");
    has_user = has_password = true;
  }else if((config.user().size() == 0) && (config.password().size() != 0)){
    CLOG_WARN("cyt-mqtt-gateway : load-config : warning : 'password' is set while no username is specified! no user/pass auth will be done");
    has_user = false;
    has_password = false;
  }else if((config.user().size() != 0) && (config.password().size() != 0)){
    CLOG_WARN("cyt-mqtt-gateway : load-config : warning : 'user' is set but password is not configured. Basic auth will be done");
    has_user = true;
    has_password = false;
  }else{
    CLOG_INFO("cyt-mqtt-gateway : load-config : no user/pass auth.")
    has_user = has_password = false;
  }


  if(
    (config.certificates_ca().size() != 0) 
    && (config.certificates_cert().size() != 0) 
    && (config.certificates_key().size() != 0))
  {
    CLOG_INFO("cyt-mqtt-gateway : load-config : communication certificates config have been found");
    has_ca_file = has_cert_file = has_key_file = true;
    if(config.certificates_passphrase().size() != 0){
      CLOG_INFO("cyt-mqtt-gateway : load-config : certificates passphrase has been found");
      has_key_passphrase = true;
    }
    if(config.base_path().size() != 0){
      CLOG_INFO("cyt-mqtt-gateway : load-config : certificates base-path has been found");
    }
    if(config.tls_version().size() == 0){
      CLOG_ERROR("cyt-mqtt-gateway : load-config : certificates are defined. TLS VERSION MUST BE SET");
      return false;
    }
    if(config.ciphers().size() == 0){
      CLOG_ERROR("cyt-mqtt-gateay : load-config : certificates are defined. CIPHERS STRING MUST BE DEFINED");
      return false;
    }
  }else if(
    (config.certificates_ca().size() == 0) 
    && (config.certificates_cert().size() == 0) 
    && (config.certificates_key().size() == 0))
  {
    CLOG_INFO("cyt-mqtt-gateay : load-config : no ssl communication will be done using certificates");
  }else{
    CLOG_ERROR("cyt-mqtt-gateay : load-config : review certificates "
      "configuration at [" MQTT_GATEWAY_CONFIG_CA_FILE "], [" MQTT_GATEWAY_CONFIG_CERT_FILE 
      "] and [" MQTT_GATEWAY_CONFIG_KEY_FILE "]. You must define them all, or none.");
    return false;
  }
  print_config();
  if(toret == false){
    CLOG_ERROR("cyt-mqtt-gateway : load-config : there were errors at the configuration object.");
  }
  return toret;
}
bool MqttGateway::load_configuration()
{
  return (update_config_from_struct_() ? true : _load_configuration_from_jsoncpp_struct() );
}
void MqttGateway::set_certificate_location_path(const std::string& l)
{
    config.base_path(l);
}

void MqttGateway::print_config()
{
  CLOG_INFO("----------------------------------------- --- - ");
  CLOG_INFO("mqtt-gw : address       : " << config.host_address());
  CLOG_INFO("mqtt-gw : host          : " << config.host_port());
  CLOG_INFO("mqtt-gw : client id     : " << (config.client_id().size() == 0?"not set":config.client_id().c_str()));
  CLOG_INFO("mqtt-gw : qos           : " << config.qos());
  CLOG_INFO("mqtt-gw : timeout       : " << config.timeout());
  CLOG_INFO("mqtt-gw : mosquitto log : " << (config.show_debug_msgs()==true?"true":"false"));
  CLOG_INFO("mqtt-gw : clean session : " << (config.clean_session() == true?"true":"false"));
  if(has_user || has_password)
  {
      CLOG_INFO("mqtt-gw : username    : " << config.user());
      CLOG_INFO("mqtt-gw : password    : " << config.password());
  }else{
      CLOG_INFO("mqtt-gw : no username and password set");
  }

  if(has_ca_file){
      CLOG_INFO("mqtt-gw : ca file     : " << config.certificates_ca());
  }else{
      CLOG_INFO("mqtt-gw : ca file     : not defined");
  }
  
  if(has_cert_file){
      CLOG_INFO("mqtt-gw : cert file   : " << config.certificates_cert());
  }else{
      CLOG_INFO("mqtt-gw : cert file   : not defined");
  }
  
  if(has_key_file){
      CLOG_INFO("mqtt-gw : key file    : " << config.certificates_key());
  }else{
      CLOG_INFO("mqtt-gw : key file    : not defined");
  }
  if(has_ca_file || has_cert_file || has_key_file){
      
      CLOG_INFO("mqtt-gw : tls version : " << config.tls_version());
  }else{
      CLOG_INFO("mqtt-gw : tls version : not needed");
  }
  
  if(has_key_passphrase){
    CLOG_INFO("mqtt-gw : k. passphrase : " << config.certificates_passphrase());
  }else{
    if(has_key_file){
        CLOG_INFO("mqtt-gw : k. passphrase : not defined");
    }else{
        CLOG_WARN("mqtt-gw : k. passphrase : not defined");
    }
  }
  CLOG_INFO("----------------------------------------- --- - ");
}

}
}
}//end namespace