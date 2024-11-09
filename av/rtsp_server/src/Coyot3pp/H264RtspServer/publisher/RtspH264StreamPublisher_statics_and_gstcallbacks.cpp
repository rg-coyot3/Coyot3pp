#include <Coyot3pp/H264RtspServer/RtspH264StreamPublisher.hpp>

namespace ct = coyot3::tools;
namespace ci = coyot3::av::image;
namespace cm = coyot3::mod;

namespace coyot3{
namespace av{
namespace rtsp{


  // statics
  void 
  RtspH264StreamPublisher::static_factory_media_configure(
    GstRTSPMediaFactory*  factory
    ,GstRTSPMedia*        media
    ,RtspH264StreamPublisher* publisher){

    
    CLOG_INFO("rtsp-h264-stream : static-factory-media-configure : configuring")
    publisher->factory_media_configure(factory,media);
  }



  void 
  RtspH264StreamPublisher::factory_media_configure(
    GstRTSPMediaFactory*  factory
    ,GstRTSPMedia*        media)
  {
    log_info("factory-media-configure :  begin");
    GstElement* pipeline         = gst_rtsp_media_get_element(media);
    //CLOG_INFO("TO-DELETE : factory media conf bin get by name : " << gst_bin_get_by_name(GST_BIN(pipeline),"imagesrc"))
    handlers.gst_application_alt = (GstAppSrc*)gst_bin_get_by_name(GST_BIN(pipeline),"imagesrc");
    handlers.gst_application = handlers.gst_application_alt;
    gst_util_set_object_arg(G_OBJECT(handlers.gst_application_alt),"format","time");
    gst_object_unref(pipeline);
    log_info("factory-media-configure : DONE");
  }






}
}
}
