#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>


namespace ct = coyot3::tools;

namespace coyot3{
namespace av{
namespace rtsp{

  
  const std::string RtspH264Server::pipeline_preset_ull 
                    = "appsrc name=imagesrc "
                      "do-timestamp=true "
                      "min-latency=0 "
                      "max-latency=0 "
                      "max-bytes=1000 "
                      "is-live=true  "
                      "! "
                      "videoconvert "
                      "! x264enc "
                      "pass=quant "
                      "bitrate=1000 "
                      "byte-stream=true "
                      "key-int-max=15 "
                      "intra-refresh=false "
                      "subme=5 "
                      "sliced-threads=true "
                      "speed-preset=ultrafast "
                      "quantizer=25 "
                      "tune=zerolatency "
                      "! "
                      "h264parse "
                      "! "
                      "rtph264pay "
                      "pt=96 "
                      "sync=false "
                      "name=pay0 ";

  const std::string RtspH264Server::pipeline_preset_tst
                    = "videotestsrc"
                      " ! "
                      "video/x-raw,width=800,height=800"
                      " ! "
                      "x264enc"
                      " ! "
                      "h264parse"
                      " ! "
                      "rtph264pay "
                      "name=pay0 "
                      "config-interval=1";

  const std::string RtspH264Server::pipeline_preset_dsk
                    = "ximagesrc "
                      "! video/x-raw,framerate=5/1 "
                      "! videoconvert "
                      "! theoraenc "
                      "! oggmux "
                      "! filesink location=desktop.ogg";

  int              
  RtspH264Server::connected_clients_get_total(){
    StreamsMap::iterator it;
    int totalc = 0;
    for(it = streams.begin();it != streams.end();++it){
      totalc+=it->second->clients_get_num();
    }
    return totalc;
  }
  GstRTSPClient*   RtspH264Server::connected_clients_get_oldest(){
    RtspClientInformation cli;
    cli.connection_ts(ct::get_current_timestamp());
    int64_t reference = coyot3::tools::get_current_timestamp();
    StreamsMap::iterator sit;
    for(sit = streams.begin();sit != streams.end(); ++sit){
      RtspClientInformation buffer;
      if(sit->second->oldest_client_obtain(buffer)==true){
        if(cli.connection_ts() > buffer.connection_ts()){
          cli = buffer;
        }
      }
    }
    return cli.gst_client();
  }

  void 
  RtspH264Server::set_cb_on_client_connected(
    std::function<void(RtspClientInformation)> cb){
      cb_on_client_connects = cb;
  }
  
  void 
  RtspH264Server::set_cb_on_client_disconnected(
    std::function<void(RtspClientInformation)> cb){
      cb_on_client_disconnect = cb;
  }

  RtspServerParams& RtspH264Server::params(){return params_;}
  RtspServerParams  RtspH264Server::params() const {return params_;}

}
}
}