#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <rapidjson/document.h>

const char* example_json=R"JSONEXAMPLE(
{
  "uno" : 1,
  "dos" : "dos",
  "tres" : 3.33,
  "cuatro" : true,
  "cinco" : {
    "seis" : 6
  },
  "siete" : [ 1,2,3,4,5]
}
)JSONEXAMPLE";

const char* example_json_long=R"JSONLONGEXAMPLE(
[

{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,
{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}
,{
  "debug_mode" : true
  ,"host_address" : "hola-mundo"
  ,"host_port" : 1234
  ,"qos" : 123
  ,"qos_max" : 123 
  ,"timeout" : 123123
  ,"show_debug_msgs" : true
  ,"important_messages_timeout_ms" : 123
  ,"important_messages_max_retries" : 123
  ,"clean_session" : false
  ,"client_id" : "123123aaa"
  ,"client_id_postfix_rand_str_length" : 22
  ,"user" : "afa"
  ,"password" : "efefe"
  ,"base_path" : "eefefee"
  ,"certificates_relative_path" : false
  ,"certificates_ca" : "efefeff"
  ,"certificates_cert" : "efffe"
  ,"certificates_key" : "eefefefe"
  ,"certificates_passphrase" : "ffefefefef"
  ,"tls_version" : "efefef"
  ,"ciphers" : "aaafffaffaf"
}



]
)JSONLONGEXAMPLE";


CYT3MACRO_model_class_declarations(
  MqttClientConfigObject
  , 
    , ( virtual void print_current_config_state())
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


CYT3MACRO_model_class_serializable_json_declarations(
  MqttClientConfigObject
  ,
  , 
  , ( )
  , ( )
    , debug_mode                        , "debug_mode"                        , 
    , host_address                      , "host_address"                      , 
    , host_port                         , "host_port"                         , 
    , qos                               , "qos"                               , 
    , qos_max                           , "qos_max"                           , 
    , timeout                           , "timeout"                           , 
    , show_debug_msgs                   , "show_debug_msgs"                   , 
    , important_messages_timeout_ms     , "important_messages_timeout_ms"     , 
    , important_messages_max_retries    , "important_messages_max_retries"    , 
    , clean_session                     , "clean_session"                     , 
    , client_id                         , "client_id"                         , 
    , client_id_postfix_rand_str_length , "client_id_postfix_rand_str_length" , 
    , user                              , "user"                              , 
    , password                          , "password"                          , 
    , base_path                         , "base_path"                         , 
    , certificates_relative_path        , "certificates_relative_path"        , 
    , certificates_ca                   , "certificates_ca"                   , 
    , certificates_cert                 , "certificates_cert"                 , 
    , certificates_key                  , "certificates_key"                  , 
    , certificates_passphrase           , "certificates_passphrase"           , 
    , tls_version                       , "tls_version"                       , 
    , ciphers                           , "ciphers"                           , 
)

CYT3MACRO_model_class_set_stack_declarations(MqttClientConfigObject,)
CYT3MACRO_model_class_set_stack_serializable_json_declarations(MqttClientConfigObject)
CYT3MACRO_model_class_set_stack_definitions(MqttClientConfigObject,)
CYT3MACRO_model_class_set_stack_serializable_json_definitions(MqttClientConfigObject)


CYT3MACRO_model_class_definitions(
  MqttClientConfigObject
  , 
    , ( virtual void print_current_config_state())
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

CYT3MACRO_model_class_serializable_json_definitions(
  MqttClientConfigObject
  ,
  , 
  , ( )
  , ( )
    , debug_mode                        , "debug_mode"                        , 
    , host_address                      , "host_address"                      , 
    , host_port                         , "host_port"                         , 
    , qos                               , "qos"                               , 
    , qos_max                           , "qos_max"                           , 
    , timeout                           , "timeout"                           , 
    , show_debug_msgs                   , "show_debug_msgs"                   , 
    , important_messages_timeout_ms     , "important_messages_timeout_ms"     , 
    , important_messages_max_retries    , "important_messages_max_retries"    , 
    , clean_session                     , "clean_session"                     , 
    , client_id                         , "client_id"                         , 
    , client_id_postfix_rand_str_length , "client_id_postfix_rand_str_length" , 
    , user                              , "user"                              , 
    , password                          , "password"                          , 
    , base_path                         , "base_path"                         , 
    , certificates_relative_path        , "certificates_relative_path"        , 
    , certificates_ca                   , "certificates_ca"                   , 
    , certificates_cert                 , "certificates_cert"                 , 
    , certificates_key                  , "certificates_key"                  , 
    , certificates_passphrase           , "certificates_passphrase"           , 
    , tls_version                       , "tls_version"                       , 
    , ciphers                           , "ciphers"                           , 
)


void MqttClientConfigObject::print_current_config_state()
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






CYT3MACRO_enum_class_declarations(
  EnumExample1
  ,
    , MY_STATE_0 = 0
    , MY_STATE_1 = 1
    , MY_STATE_2 = 2)


CYT3MACRO_enum_class_definitions(
  EnumExample1
  ,
    , MY_STATE_0 
    , MY_STATE_1 
    , MY_STATE_2 )

CYT3MACRO_model_class_declarations(
  MyClassPriv_
  , 
  , ( )
  , ( )
    , number          , int             , 0
    , cadena          , std::string     , ""
)

CYT3MACRO_model_class_definitions(
  MyClassPriv_
  , 
  , ( )
  , ( )
    , number          , int             , 0
    , cadena          , std::string     , ""
)


class MyClass : public MyClassPriv_{
  public:
    MyClass():MyClassPriv_(){}
    MyClass(const MyClassPriv_& o):MyClassPriv_(o){}
    virtual ~MyClass(){}

    CYT3MACRO_enum_class_declarations(
      EnumExample3
      , MyClass
      , MY_STATE_0 = 0
      , MY_STATE_1 = 1
      , MY_STATE_2 = 2
    )
};


CYT3MACRO_model_class_declarations(
  RapidjsonDevSubclase
  , 
  , ( )
  , ( )
    , seis          , int       , -1
)

CYT3MACRO_model_class_serializable_json_declarations(
  RapidjsonDevSubclase
  ,
  ,
  , ( )
  , ( )
    , seis        , "seis"      , 
)

CYT3MACRO_model_class_declarations(
  RapidjsonDev
  ,
  , ( )
  , ( )
    , uno         , int                     , 0
    , dos         , std::string             , ""
    , tres        , double                  , 0.0
    , cuatro      , bool                    , true
    , cinco       , RapidjsonDevSubclase    , 
)


CYT3MACRO_model_class_serializable_json_declarations(
  RapidjsonDev
  , 
  ,
  , ( 
    cinco       , "cinco"       , RapidjsonDebSubclase
  )
  , ( )
    , uno         , "uno"         ,
    , dos         , "dos"         ,
    , tres        , "tres"        ,
    , cuatro      , "cuatro"      ,
)




CYT3MACRO_model_class_definitions(
  RapidjsonDevSubclase
  , 
  , ( )
  , ( )
    , seis          , int       , -1
)

CYT3MACRO_model_class_serializable_json_definitions(
  RapidjsonDevSubclase
  ,
  ,
  , ( )
  , ( )
    , seis        , "seis"      , 
)

CYT3MACRO_model_class_definitions(
  RapidjsonDev
  ,
  , ( )
  , ( )
    , uno         , int                     , 0
    , dos         , std::string             , ""
    , tres        , double                  , 0.0
    , cuatro      , bool                    , true
    , cinco       , RapidjsonDevSubclase    , 
)

CYT3MACRO_model_class_serializable_json_definitions(
  RapidjsonDev
  , 
  ,
  , ( 
    cinco       , "cinco"       , RapidjsonDevSubclase
  )
  , ( )
    , uno         , "uno"         ,
    , dos         , "dos"         ,
    , tres        , "tres"        ,
    , cuatro      , "cuatro"      ,
)




int main(int argv, char** argc){
  ec::EnumExample1 enumtype1;

  rapidjson::Document doc;

  std::string content;
  //coyot3::tools::load_content_from_file_text("../doc/example_sources/json/example.json",content);
  content = example_json;

  doc.Parse(content.c_str(),content.size());

  CLOG_INFO(" : documento cargado" << content);


  
  RapidjsonDev rjsontest;
  // Json::Value js;
  // js << content;
  // Json::Reader reader;
  // reader.parse(content,js);
  // CLOG_INFO(" js = " << js);
  (rjsontest << content);
  // if((rjsontest << content) ==  false)
  // {
  //   CLOG_ERROR(" - error parseando clase de test desde string")
  //   exit(1);
  // }

  CLOG_INFO(" - clase test parseada correctamente : " << rjsontest.uno())
  CLOG_INFO(" - clase test parseada correctamente : " << rjsontest.dos())
  CLOG_INFO(" - clase test parseada correctamente : " << rjsontest.tres())
  CLOG_INFO(" - clase test parseada correctamente : " << rjsontest.cuatro())


  RapidjsonDevJsIO rjt2;

  CLOG_INFO("punto 00");

  
  for(rapidjson::Value::ConstMemberIterator it = doc.MemberBegin();
      it != doc.MemberEnd(); ++it){
    CLOG_INFO(" members : " << it->name.GetString())
  }
    CLOG_INFO("punto 01");

  int i;
  
  coyot3::tools::rjson_import_value(doc,"uno",i);
  CLOG_INFO("rjson import int=" << i);

  //bool res = rjt2.from_json(doc);
  bool res = (rjt2 << content);
  CLOG_INFO( " fromrjson = " << res);

  

  CLOG_INFO(" - clase test parseada correctamente : " << rjt2.uno())
  CLOG_INFO(" - clase test parseada correctamente : " << rjt2.dos())
  CLOG_INFO(" - clase test parseada correctamente : " << rjt2.tres())
  CLOG_INFO(" - clase test parseada correctamente : " << rjt2.cuatro())


  rapidjson::Document doc2;
  
  rapidjson::Value val;
  val = doc["cinco"];

  int seis;
  
  doc2.Move();

  coyot3::tools::rjson_import_value(doc2,"cinco",seis);

  CLOG_INFO(" - clase test seis: " << rjt2.cinco().seis())

  
  Json::Value control;
  std::string cont;
  cont = example_json_long;
  //coyot3::tools::load_content_from_file_text("../doc/example_sources/json/example2.json",cont);
  // if(coyot3::tools::load_json_from_file("../doc/example_sources/json/example2.json",control) == false){
  //   CLOG_ERROR("error cargando json")
  //   exit(1);
  // }else{
  //   CLOG_INFO("cargado control. tipo = " << control.type())
  //   CLOG_INFO("cargado control. tipo = " << control.isArray())
  //   sleep(2);
  // }

  MqttClientConfigObjectStackJsIO test1;
  MqttClientConfigObjectStackJsIO test2;

  int64_t total_veces= 10000;
  int64_t test1_init,test1_fin;
  int64_t test2_init,test2_fin;

  res = true;
  rapidjson::Value::ConstValueIterator itr;
  

  CLOG_INFO(" - test performance : " << cont)
  CLOG_INFO(" - inicia test jsoncpp")
  test1_init = coyot3::tools::now();
  for(int64_t i = 0; i < total_veces; i++){
    test1.clear();
    res &= test1.from_json_string(cont);
  }
  test1_fin = coyot3::tools::now();
  CLOG_INFO(" - allgood = " << res << " : fin test json : msecs=" << (test1_fin - test1_init))

  CLOG_INFO(" - inicia test rjson")
  test2_init = coyot3::tools::now();
  res = true;
  for(int64_t i = 0; i < total_veces; i++){
    test2.clear();
    res &= test2.from_rjson_string(cont);
  }
  test2_fin = coyot3::tools::now();
  CLOG_INFO(" - fin test rjson : msecs=" << (test2_fin - test2_init))
  CLOG_INFO(" - allgood " << res)

  
}