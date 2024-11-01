#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>



namespace coyot3{
namespace av{
namespace rtsp{

  

   CYT3MACRO_model_class_definitions(
    RtspServerParams
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

        , ts_start                      , int64_t                     , 0
        , ts_end                        , int64_t                     , 0

        , current_key                   , std::string                 , ""
  );

  RtspH264Server::ServerGstStruct::ServerGstStruct()
  :gstreamer_main_loop(nullptr)
  ,gstloop_thread(nullptr)
  ,rtsp_server(nullptr)
  ,rtsp_server_source_id(0)
  ,server_port(0){}
  
  RtspH264Server::ServerGstStruct::~ServerGstStruct(){}

}
}
}