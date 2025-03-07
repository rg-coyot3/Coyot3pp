#include <Coyot3pp/H264RtspServer/RtspServerConfigObject.hpp>



namespace coyot3{
namespace av{
namespace rtsp{


  CYT3MACRO_model_class_definitions(
    RtspStreamConfig
      ,
      , ( )
      , ( )
        , stream_name           , std::string                        , ""
        , path                  , std::string                        , ""
        , is_active             , bool                               , true
        , show_debug_preview    , bool                               , false 
        , compress_ratio        , double                             , 0.0
        , width                 , int                                , 960
        , height                , int                                , 541
        , update_frequence      , int                                , 15
        , gst_pipeline_chain    , coyot3::tools::CytStringSet        ,
  );

  CYT3MACRO_model_class_serializable_json_definitions(
    RtspStreamConfig
    ,
    ,
    , ( gst_pipeline_chain    , "gst_pipeline_chain"    , coyot3::tools::CytStringSet )
    , ( )
      , stream_name           , "stream_name"           ,
      , path                  , "path"                  ,
      , is_active             , "is_active"             ,
      , show_debug_preview    , "show_debug_preview"    ,
      , compress_ratio        , "compress_ratio"        ,
      , width                 , "width"                 ,
      , height                , "height"                ,
      , update_frequence      , "update_frequence"      ,
    )
  


  CYT3MACRO_model_class_set_stack_definitions(
    RtspStreamConfig,100);
  
  CYT3MACRO_model_class_set_stack_serializable_json_definitions(
    RtspStreamConfig);



  CYT3MACRO_model_class_definitions(
    RtspServerConfig
      , 
      , ( )
      , ( )
        , server_name                   , std::string             , ""
        , server_port                   , int                     , 8554
        , base_path                     , std::string             , "/tmp"
        , ssl_on                        , bool                    , false
        , file_ssl_ca                   , std::string             , ""
        , file_ssl_cert                 , std::string             , ""
        , file_ssl_key                  , std::string             , ""
        , auth_global_use               , bool                    , false
        , auth_user                     , std::string             , ""
        , auth_pass                     , std::string             , ""
        , auth_credentials_use          , bool                    , false
        , auth_client_credentials_file  , std::string             , ""
        , debug_level                   , int                     , 1
        , key_auth_use                  , bool                    , false
        , key_auth_string               , std::string             , "12345ABCDEF"
        , key_auth_autogen              , bool                    , false
        , max_clients                   , int                     , 0
        , streams_collection         , RtspStreamConfigStack   , 
  );
  // JSON - 
      


  CYT3MACRO_model_class_serializable_json_definitions(
    RtspServerConfig
      , 
      , 
      , ( 
          streams_collection         , "streams_collection"         , RtspStreamConfigStack  
      )
      , ( )
        , server_name                   , "server_name"                   ,
        , server_port                   , "server_port"                   ,
        , base_path                     , "base_path"                     ,
        , ssl_on                        , "ssl_on"                        ,
        , file_ssl_ca                   , "file_ssl_ca"                   ,
        , file_ssl_cert                 , "file_ssl_cert"                 ,
        , file_ssl_key                  , "file_ssl_key"                  ,
        , auth_global_use               , "auth_global_use"               ,
        , auth_user                     , "auth_user"                     ,
        , auth_pass                     , "auth_pass"                     ,
        , auth_credentials_use          , "auth_credentials_use"          ,
        , auth_client_credentials_file  , "auth_client_credentials_file"  ,
        , debug_level                   , "debug_level"                   ,
        , key_auth_use                  , "key_auth_use"                  ,
        , key_auth_string               , "key_auth_string"               ,
        , key_auth_autogen              , "key_auth_autogen"              ,
        , max_clients                   , "max_clients"                   ,
  );



}
}
}