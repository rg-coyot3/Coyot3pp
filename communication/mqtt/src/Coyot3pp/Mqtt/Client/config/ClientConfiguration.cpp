#include <Coyot3pp/Mqtt/Client/config/ClientConfiguration.hpp>



namespace coyot3::communication::mqtt{


CYT3MACRO_model_class_definitions(
  ClientConfiguration
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
  ClientConfiguration
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


void ClientConfiguration::print_current_config_state(){
  CLOG_INFO(" - config state: ")
  CLOG_INFO("   debug_mode                        :" << debug_mode() )
  CLOG_INFO("   host_address                      :" << host_address() )
  CLOG_INFO("   host_port                         :" << host_port() )
  CLOG_INFO("   qos                               :" << qos() )
  CLOG_INFO("   qos_max                           :" << qos_max() )
  CLOG_INFO("   timeout                           :" << timeout() )
  CLOG_INFO("   show_debug_msgs                   :" << show_debug_msgs() )
  CLOG_INFO("   important_messages_timeout_ms     :" << important_messages_timeout_ms() )
  CLOG_INFO("   important_messages_max_retries    :" << important_messages_max_retries() )
  CLOG_INFO("   clean_session                     :" << clean_session() )
  CLOG_INFO("   client_id                         :" << client_id() )
  CLOG_INFO("   client_id_postfix_rand_str_length :" << client_id_postfix_rand_str_length() )
  CLOG_INFO("   user                              :" << user() )
  CLOG_INFO("   password                          :" << password() )
  CLOG_INFO("   base_path                         :" << base_path() )
  CLOG_INFO("   certificates_relative_path        :" << certificates_relative_path() )
  CLOG_INFO("   certificates_ca                   :" << certificates_ca() )
  CLOG_INFO("   certificates_cert                 :" << certificates_cert() )
  CLOG_INFO("   certificates_key                  :" << certificates_key() )
  CLOG_INFO("   certificates_passphrase           :" << certificates_passphrase() )
  CLOG_INFO("   tls_version                       :" << tls_version() )
  CLOG_INFO("   ciphers                           :" << ciphers() )
}

}