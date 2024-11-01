#include <Coyot3pp/H264RtspServer/RtspH264StreamPublisher.hpp>

namespace ct = coyot3::tools;
namespace ci = coyot3::av::image;
namespace cm = coyot3::mod;

namespace coyot3{
namespace av{
namespace rtsp{
// misc
  int RtspH264StreamPublisher::id() const{return params_.id();}
  int RtspH264StreamPublisher::id(int i){return params_.id(i);}
  

  bool RtspH264StreamPublisher::set_owner(GstRTSPServer* owner){
    handlers.owner = owner;
    return (handlers.owner != nullptr);
  }


  bool 
  RtspH264StreamPublisher::oldest_client_obtain(RtspClientInformation& cli){
    bool res = false;
    ClientsMap::iterator it;
    int64_t compts = ct::get_current_timestamp();

    for(it = clients.begin();it != clients.end();++it){
      if(it->second.connection_ts() < compts){
        cli = it->second;
        res = true;
      }  
    }
    return res;
  }

  bool
  RtspH264StreamPublisher::image_source_set(ImageContent* src){
    if(source_image != nullptr){
      log_warn("image-source-set : source is already set. must detach first!");
      return false;
    }
    source_image = src;
    source_image->set_dimensions(params_.width(),
                                  params_.height(),
                                  src->preserve_aspect_ratio());
    log_info(o() << "image-source-set : source attached (" << src->name() 
      << ")");
    source_image->prepare_output();
    return true;
  }
  bool
  RtspH264StreamPublisher::image_source_set(ImageContent& src){
    return image_source_set(&src);
  }

  ci::ImageContent* RtspH264StreamPublisher::image_source_detach(){
    ImageContent* p = source_image;
    source_image = nullptr;
    if(!p){
      log_warn("image-source-detach : source is empty");
    }else{
      stream_image = cv::Mat(params_.width(),
                            params_.height(),
                            CV_8UC3,
                            cv::Scalar(100,0,0));
    }

    return p;
  }

}
}
}
