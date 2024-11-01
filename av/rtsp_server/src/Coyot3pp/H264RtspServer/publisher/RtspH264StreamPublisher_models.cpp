#include <Coyot3pp/H264RtspServer/RtspH264StreamPublisher.hpp>

namespace ct = coyot3::tools;
namespace ci = coyot3::av::image;
namespace cm = coyot3::mod;

namespace coyot3{
namespace av{
namespace rtsp{

  
  CYT3MACRO_model_class_definitions(
    RtspH264PublisherParams
    ,
    , ( )
    , ( )
      , name                        , std::string         , ""
      , id                          , int                 , 0
      
      , pipeline                    , std::string         , ""
      , path                        , std::string         , ""

      , text_contents_show          , bool                , false
      , text_size_scale             , double              , 1.0
      , width                       , int                 , 300
      , height                      , int                 , 200
      
      , max_clients                 , int                 , 0

      , compress_ratio              , double              , 1.0
      , cv_color_conversion         , int                 , -1
      , quality                     , int                 , 1

      , production_interval         , int                 , 100

  ) 

  
  CYT3MACRO_model_class_definitions(
    RtspClientInformation
    ,
    , ( int64_t client_connection_time_total())
    , ( )
      , host              , std::string     , ""
      , port              , uint32_t        , 0
      , uri               , std::string     , ""
      , abs_path          , std::string     , ""
      , query             , std::string     , ""
      , key               , std::string     , ""
      , connection_ts     , int64_t         , 0
      , disconnection_ts  , int64_t         , 0
      
      , gst_client    , GstRTSPClient*      , nullptr
      , gst_context   , GstRTSPContext*     , nullptr
  )

  CYT3MACRO_model_class_set_mapped_definitions(RtspClientInformation,
                                                gst_client)





  RtspH264StreamPublisher::Handlers::Handlers()
  :owner(nullptr)
  ,gst_application(nullptr)
  ,gst_application_alt(nullptr)
  ,mounts(nullptr)
  ,factory(nullptr)
  ,media_connection(0)
  ,gst_timestamp(0)
  ,gst_type()
  ,gst_format()
  ,stream_thread(nullptr){}

  RtspH264StreamPublisher::Handlers::~Handlers(){}





}
}
}