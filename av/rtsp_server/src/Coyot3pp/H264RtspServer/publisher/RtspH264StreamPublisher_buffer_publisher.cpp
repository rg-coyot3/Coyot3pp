#include <Coyot3pp/H264RtspServer/RtspH264StreamPublisher.hpp>

namespace ct = coyot3::tools;
namespace ci = coyot3::av::image;
namespace cm = coyot3::mod;

namespace coyot3{
namespace av{
namespace rtsp{

void RtspH264StreamPublisher::stream_image_publish(){
  CLOG_DEBUG_LEVEL_SET(0)
  gst_buffer_publisher();
}

bool RtspH264StreamPublisher::gst_buffer_publisher(){
  //CLOG_INFO("to-delete : publisher call")
  if(handlers.gst_application == nullptr){
    //ignore, all good
    log_debug(7,"gst-buffer-publisher- : gst-app not set. ignoring iteration.");
    return true;
  }
  GstBuffer*        buff = NULL;
  void*             imgdata;

  GstFlowReturn     gstFlowReturn;
  IplImage*         iplImagePtr;
  guchar*           gucharData1;
  GstMapInfo        gstMapInfo,map;
  GstCaps*          gstCaps;
  
  log_debug(7,"gst-buffer-publisher- : preparing output");
  if(!source_image->prepare_output()){
    log_warn("gst-buffer-publisher- : error preparing data source!");
  }
  log_debug(7,"gst-buffer-publisher- : creating caps");
  handlers.gst_type = "video/x-raw";
  handlers.gst_format = "BGR";
  gstCaps = gst_caps_new_simple(
    handlers.gst_type.c_str(),
    "format", G_TYPE_STRING, handlers.gst_format.c_str(),
    "width" , G_TYPE_INT   , params_.width(),
    "height", G_TYPE_INT   , params_.height(),
    nullptr
  );
  log_debug(7,"gst-buffer-publisher- : setting caps");
  gst_app_src_set_caps(handlers.gst_application,gstCaps);

  if(source_image != nullptr)
  {
    std::lock_guard guard(source_image->getMutexRef());
    log_debug(7,"gst-buffer-publisher- : creating buffer from source image");
    stream_image = source_image->getImageRef();
    buff = gst_buffer_new_wrapped_full(
      (GstMemoryFlags)0,
      (gpointer)(stream_image.data),
      (stream_image.cols * stream_image.rows * stream_image.channels()),
      0,
      (stream_image.cols * stream_image.rows * stream_image.channels()),
      NULL,
      NULL
    );
  }else{
    log_debug(7,"gst-buffer-publisher- : creating buffer from blank");
    buff = gst_buffer_new_wrapped_full(
      (GstMemoryFlags)0,
      (gpointer)(stream_image.data),
      (stream_image.cols * stream_image.rows * stream_image.channels()),
      0,
      (stream_image.cols * stream_image.rows * stream_image.channels()),
      NULL,
      NULL
    );
  }
  // cv::imshow("now-streaming",stream_image);
  // cv::waitKey(1);
  log_debug(7,"gst-buffer-publisher- : mapping");
  gst_buffer_map(buff,&gstMapInfo,GST_MAP_WRITE);
  log_debug(7,"gst-buffer-publisher- : copying");
  //memcpy((guchar*)gstMapInfo.data,gucharData1,gst_buffer_get_size(buff));
  memcpy((guchar*)gstMapInfo.data,stream_image.data,gst_buffer_get_size(buff));
  GST_BUFFER_PTS(buff) = handlers.gst_timestamp;
  GST_BUFFER_DURATION(buff) = params_.production_interval()*1000;
  handlers.gst_timestamp+=GST_BUFFER_DURATION(buff);
  gst_buffer_unmap(buff,&gstMapInfo);
  log_debug(7,"gst-buffer-publisher- : pushing");
  gst_app_src_push_buffer(handlers.gst_application,buff);
  log_debug(5,"gst-buffer-publisher- : image published");
  //CLOG_INFO("to-delete : PUBLISHED!!!")
  return true;
}





}
}
}