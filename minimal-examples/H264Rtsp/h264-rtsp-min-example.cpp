
#include <Coyot3pp/Imag3/ImageContent/ImageContentTestImage.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentImgSequence.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentComposition.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentCvVideocap.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentDisplayImage.hpp>
#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>
// #include <X11/Xlib.h>

#ifndef IMAG3_EXAMPLES_DIRECTORY
#define IMAG3_EXAMPLES_DIRECTORY ""
#endif

namespace cai = coyot3::av::image;
namespace car = coyot3::av::rtsp;
namespace ct = coyot3::tools;
bool dostuff = true;

void on_control_c(int sig)
{
  CLOG_WARN("ending")
  dostuff = false;
}
int main(int argv, char **argc)
{
  XInitThreads();
  cai::ImageContentParams params;

  //cai::ImageContentTestImage img;
  cai::ImageContentCvVideocap img("video","/dev/video0");


  params.width(500);
  params.height(500);
  params.base_path("");
  params.show_preview(false);
  params.show_name(true);

  img.set_params(params);
  // img.name("testSource");
  img.id(2);
  img.debug_level_set(1);


  img.connection_retry_interval(5000);

  img.Init();
  img.Start();


  cai::ImageContentDisplayImage display("display");
  display.set_params(params);
  display.preserve_aspect_ratio(true);
  display.Init();
  display.Start();

  cai::ImageContentComposition composition("composition");
  params.width(800);

  composition.source_attach(img);
  composition.source_attach(display);


  car::RtspServerConfig serverConf;
  serverConf.server_name("hello-world");
  serverConf.server_port(8554);
  

  car::RtspH264Server server("rtsp-server");
  //server = new car::RtspH264Server();
  server.set_configuration(serverConf);
  server.modlog_verbosity(10);
  
  
  car::RtspStreamConfig streamConf;
  streamConf.stream_name("hello");
  streamConf.path("/hello");
  streamConf.is_active(true);
  streamConf.show_debug_preview(false);
  streamConf.width(800);
  streamConf.height(500);
  streamConf.update_frequence(15);
  CLOG_INFO("pipeline : " << car::RtspH264Server::pipeline_preset_ull)
  streamConf.gst_pipeline_chain().push_back(car::RtspH264Server::pipeline_preset_ull);

  server.stream_create(streamConf);
  if(!server.stream_get_by_name("hello")){
    CLOG_WARN("oh ooooooh....")
  }
  server.stream_get_by_name("hello")->image_source_set(composition);
  server.Init();
  server.Start();


  while(dostuff){
    usleep(75000);
    //CLOG_INFO("pulse");
  }


  // img.End(true);
  // seq.End(true);
}