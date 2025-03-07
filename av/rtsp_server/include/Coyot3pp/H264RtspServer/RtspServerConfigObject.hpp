#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>

namespace coyot3{
namespace av{
namespace rtsp{

  CYT3MACRO_model_class_declarations(
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

  CYT3MACRO_model_class_serializable_json_declarations(
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
  


  CYT3MACRO_model_class_set_stack_declarations(
    RtspStreamConfig,100);
  
  CYT3MACRO_model_class_set_stack_serializable_json_declarations(
    RtspStreamConfig);



  CYT3MACRO_model_class_declarations(
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
      


  CYT3MACRO_model_class_serializable_json_declarations(
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


/*

  "rtsp_server_params" : {
    "version" : "1.0"
    ,"server_name"                : ""
    , "base_path"                 : ""
    ,"server_port"                : 8554
    ,"font_size"                  : 1.0
    ,"status_publication_freq"    : 1
    ,"debug_level"                : 1
    ,"key_auth"                   : false
    ,"key_auth_string"            : "af3z7n6vSo9iGznRvp0JsaU0on"
    ,"key_auth_autogen"           : false
    ,"max_clients_per_flux"       : 1
    ,"metadata_band_show"         : true
    ,"image_mark_show"            : false
    ,"image_mark_location"        : "images/logo_main_02.png"
    ,"image_background_show"      : true
    ,"image_background_location"  : "images/logo_background_1080.png"
    ,"latency_control_active"     : true
    ,"latency_control_targeturl"  : "10.100.100.1"
    ,"streams_collection" : 
    [
      {
          "stream_name"           : "video_camera_center_raw"
        , "path"                  : "/av"
        , "show_debug_preview"    : false
        , "show_info_texts"       : false
        , "include_timestamp"     : false
        , "include_date_time"     : true
        , "compress_ratio"        : 0.0
        , "update_frequence"      : 15
        , "resize_img"            : true 
        , "resized_width"         : 700
        , "resized_height"        : 700
        , "preserve_aspect_ratio" : true
        , "sources" : [
          {
              "name"                : "center"
            , "source_info"         : "/camera_front_center_tele/image_raw"
            , "source_type"         : "ROS2_IMAGE"
            , "rotation"            : 0
            , "min_interval"        : 250
            , "show_title"          : true
            , "mx"                  : 0.0
            , "my"                  : 0.0
            , "mw"                  : 0.0
            , "mh"                  : 0.0
          },
          {
              "name"                : "center_BIS"
            , "source_info"         : "/camera_front_center_tele/image_raw"
            , "source_type"         : "ROS2_IMAGE"
            , "rotation"            : 10
            , "min_interval"        : 250
            , "show_title"          : true
            , "mx"                  : 0.0
            , "my"                  : 0.0
            , "mw"                  : 0.0
            , "mh"                  : 0.0
          },
          {
            "name"                  : "center_TRIS"
          , "source_info"           : "/camera_front_center_tele/image_raw"
          , "source_type"           : "ROS2_IMAGE"
          , "rotation"              : -90
          , "min_interval"          : 250
          , "show_title"            : true
          , "mx"                    : 0.0
          , "my"                    : 0.0
          , "mw"                    : 0.0
          , "mh"                    : 0.0
          }
        ]
        , "gst_pipeline_chain_test" : [
          "( video/x-raw,width=400,height=700 "
          ,"! x264enc tune=zeronlatency bitrate=1000 "
          ,"key-int-max=30 ! video/x-h264, profile=baseline "
          ,"! rtph264pay name=pay0 pt=96)"
      ]
        , "gst_pipeline_chain" : [
            "( appsrc "
            , "name=imagesrc "
            , "do-timestamp=true "
            , "min-latency=0 "
            , "max-latency=0 "
            , "max-bytes=1000 "
            , "is-live=true "
            , "!"
            , "videoconvert "
            , "!"
            , "x264enc "
            , "pass=quant "
            , "bitrate=1000 "
            , "byte-stream=true "
            , "key-int-max=15 "
            , "intra-refresh=false "
            , "subme=5 "
            , "sliced-threads=true "
            , "speed-preset=ultrafast "
            , "quantizer=25 "
            , "tune=zerolatency "
            , "! "
            , "h264parse "
            , "! "
            , "rtph264pay "
            , "pt=96 "
            , "sync=false "
            , "name=pay0 )"
        ]
        ,
       "gst_pipeline_chain_tests_" : [
              "videotestsrc",
              " ! ", 
              "video/x-raw,width=1920,height=1080",
              " ! ",
              "x264enc",
              " ! ",
              "h264parse",
              " ! ",
              "rtph264pay name=pay0 config-interval=1"
      ]
      }
    ]
  }



*/