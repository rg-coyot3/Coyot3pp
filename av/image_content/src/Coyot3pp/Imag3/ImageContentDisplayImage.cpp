#include <Coyot3pp/Imag3/ImageContent/ImageContentDisplayImage.hpp>



namespace coyot3{
namespace av{
namespace image{


  ImageContentDisplayImage::ImageContentDisplayImage(const std::string& name)
  :ImageContent(name)
  ,display_(nullptr)
  ,window_root_(0)
  ,window_attributes_{0}{
    class_name("imc-display");
    log_info("constructor" );
    add_task_start(std::bind(&ImageContentDisplayImage::task_start,this));
    add_task_stop(std::bind(&ImageContentDisplayImage::task_stop,this));
  }
  ImageContentDisplayImage::~ImageContentDisplayImage(){
    log_warn("destructor");
  }

  bool ImageContentDisplayImage::task_start(){
    
    log_info("start : obtaining display");
    display_ = XOpenDisplay(nullptr);
    if(!display_){
      log_warn("start : error obtaining display!");
      return false;
    }
    log_info("start : obtaining root window");
    window_root_ = DefaultRootWindow(display_);
    return true;
  }

  bool ImageContentDisplayImage::task_stop(){
    log_info("stop : closing display");
    XCloseDisplay(display_);
    return true;
  }


  bool ImageContentDisplayImage::prepare_output(){
    if(!get_image_from_display()){
      log_warn("prepare-output : impossible to get image");
      return false;
    }
    return ImageContent::prepare_output();
  }
  
  bool ImageContentDisplayImage::get_image_from_display(){
    log_debug(7,o() << "get-image-from-display : obtaining window attributes");
    XGetWindowAttributes(display_,window_root_, &window_attributes_);
    
    log_debug(5, o() << "get-image-from-display : making capture [" 
      << window_attributes_.width << ", " << window_attributes_.height << "]");
    XImage* img = XGetImage(display_,
                            window_root_,
                            0,0,
                            window_attributes_.width,window_attributes_.height,
                            AllPlanes,ZPixmap);
    if(!img){
      log_warn("get-img-from-display : error making capture");
      return false;
    }
    bits_per_pixel_ = img->bits_per_pixel;
    pixels_.resize(window_attributes_.width * 
                    window_attributes_.height * 
                    4);
    log_debug(3, o() << "get-image-from-display : copying (" 
      <<  window_attributes_.width << ", " << window_attributes_.height 
      << "]bpp(" << bits_per_pixel_ << ")");
    
    memcpy(&pixels_[0], img->data, pixels_.size());
    log_debug(5, "get-image-from-display : destroying source");
    XDestroyImage(img);
    try{
      int too = image_source.type();
      image_source = cv::Mat(window_attributes_.height,window_attributes_.width,
                        (bits_per_pixel_ > 24? CV_8UC4 : CV_8UC3), &pixels_[0]);
      if(too != image_source.type()){
        cv::cvtColor(image_source, image_source,cv::COLOR_BGRA2BGR);
      }
    }catch(const cv::Exception& e){
      log_warn(o() << "get-img-from-display : error importing capture : "
        "cv-exception(" << e.what() << ")");
      return false;
    }catch(...){
      log_warn("get-img-from-display : error importing capture : "
        "exception(UNKNOWN)");
      return false;
    }
    return true;
  }

}
}
}