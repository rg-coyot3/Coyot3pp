#include <libcoyot3/services/rtsp_server/VideoStreamMetadataBand.h>



namespace coyot3{
namespace wrappers{



CYT3MACRO_enum_class_definitions(
  MetadataBandFormat
  ,
    , HORIZONTAL
    , VERTICAL
)
CYT3MACRO_enum_class_definitions(
  MetadataBandMode
  ,
    , SIN_AUTOGEN
    , SIN_AUTOGEN_THREADED
    , EXTERNAL_VALUE
)
CYT3MACRO_enum_class_definitions(
  MetadataBandShiftDirection
  ,
    , LEFT_TO_RIGHT
    , RIGHT_TO_LEFT
    , TOP_TO_DOWN
    , DOWN_TO_TOP
    , STATIC
)


  const char* 
  VideoStreamMetadataBand::FormatToString(VideoStreamMetadataBand::Format f)
  {
    switch(f){
      case Format::HORIZONTAL: return "HORIZONTAL";break;
      case Format::VERTICAL: return "VERTICAL";break;
      default:
        return "no-known-type";
    }
  }
  const char* 
  VideoStreamMetadataBand::ModeToString(VideoStreamMetadataBand::Mode f)
  {
    switch(f){
      case Mode::SIN_AUTOGEN: return "SIN_AUTOGEN";break;
      case Mode::SIN_AUTOGEN_THREADED: return "SIN_AUTOGEN_THREADED";break;
      
      default:
        return "no-known-mode";
    }
  }
  const char* 
  VideoStreamMetadataBand::ShiftDirectionToString(VideoStreamMetadataBand::ShiftDirection f)
  {
    switch(f){
      case ShiftDirection::LEFT_TO_RIGHT: return "LEFT_TO_RIGHT"; break;
      case ShiftDirection::RIGHT_TO_LEFT: return "RIGHT_TO_LEFT"; break;
      case ShiftDirection::TOP_TO_DOWN: return "TOP_TO_DOWN"; break;
      case ShiftDirection::DOWN_TO_TOP: return "DOWN_TO_TOP"; break;
      case ShiftDirection::STATIC: return "STATIC";break;
      default:
        return "no-known-shift-direction";
    }
  }

  void 
  VideoStreamMetadataBand::_initialize_values()
  {
    value_ = 0;
    speed_ = 0;
    width_ = 0;
    height_ = 0;
    format_ = Format::VERTICAL;
    mode_ = Mode::SIN_AUTOGEN;
    shift_direction_ = ShiftDirection::RIGHT_TO_LEFT;
    last_update_ts_ = 0;
    threaded_update_period_ = 0;
    cth_ = nullptr;

    tmp_auto_mark_active_ = false;
    tmp_auto_mark_last_mark_ts_ = 0;
    tmp_auto_mark_interval_ = 1000;
    tmp_auto_mark_marker_ = "o";
    tmp_auto_mark_marker_ = 4;
  }


  VideoStreamMetadataBand::VideoStreamMetadataBand()
  {
    _initialize_values();
  }
  VideoStreamMetadataBand::VideoStreamMetadataBand(int h,int w){
    _initialize_values();
    if(!setDimensions(h,w))
    {
      CLOG_WARN("video-stream-metadata-band : constructor("<< h << "," << w << ")");
    }
  }
  VideoStreamMetadataBand::~VideoStreamMetadataBand(){

  }
  double VideoStreamMetadataBand::value(){return value_;}
  double VideoStreamMetadataBand::value(double v){return (value_ = v);}
  
  double VideoStreamMetadataBand::updateSpeed(){return speed_;}
  double VideoStreamMetadataBand::updateSpeed(double s){return (speed_ = s);}

  VideoStreamMetadataBand::ShiftDirection VideoStreamMetadataBand::shiftDirection(){return shift_direction_;}
  VideoStreamMetadataBand::ShiftDirection VideoStreamMetadataBand::shiftDirection(VideoStreamMetadataBand::ShiftDirection d){return (shift_direction_ = d);}
  
  cv::Mat& VideoStreamMetadataBand::getImageRef(){return image_product_;}
  cv::Mat  VideoStreamMetadataBand::getImageCopy(){
    cv::Mat r;
    {
      std::lock_guard<std::mutex> guard(mtx_);
      r = image_product_;
    }
  }
  std::mutex& VideoStreamMetadataBand::getMutexRef(){return mtx_;}



  bool VideoStreamMetadataBand::setDimensions(int w,int h){
    std::lock_guard<std::mutex> guard(mtx_);
    if((w <= 0) || (h <= 0))
    {
      CLOG_WARN("vstream-metadata-band : set-dimensions : error! (" << w << "," << "h");
      return false;
    }
    width_ = w;  
    height_ = h;
    return generate_background_black_();
  }


  bool VideoStreamMetadataBand::generate_background_black_()
  {
    try{
      background_base_ = cv::Mat(cv::Size(width_,height_),CV_8UC3,cv::Scalar(0));
      image_product_ = background_base_;
    }catch(const cv::Exception& e){
      CLOG_ERROR("vstream-metadata-band : gen-background-black : error "
        "generating (" << width_ << "," << height_ << ") : [" << e.what() << "]");
      return false;
    }catch(...){
      CLOG_ERROR("vstream-metadata-band : gen-background-black : error "
        "generating (" << width_ << "," << height_ << ") : [UNKNOWN]");
      return false;
    }
    return true;
  }

  bool VideoStreamMetadataBand::shift_image_(int desp){
    int num_px = static_cast<int>(std::ceil(static_cast<double>(desp) * (speed_/10.0)));
    if(num_px==0){
      num_px = 1;
    }
    
    cv::Mat tmp= cv::Mat(cv::Mat::zeros(image_product_.size(),image_product_.type()));
    try{
      switch(shift_direction_)
      {
        case ShiftDirection::LEFT_TO_RIGHT:
          image_product_(cv::Rect(0, 0, image_product_.cols, image_product_.rows - num_px))
            .copyTo(tmp(cv::Rect(num_px, 0, tmp.cols, tmp.rows - num_px)));        
          break;
        case ShiftDirection::RIGHT_TO_LEFT:
          image_product_(cv::Rect(num_px, 0, image_product_.cols - num_px, image_product_.rows))
            .copyTo(tmp(cv::Rect(0, 0, image_product_.cols - num_px, image_product_.rows)));
          break;
        case ShiftDirection::TOP_TO_DOWN:
          image_product_(cv::Rect(0, 0, image_product_.cols, image_product_.rows - num_px))
            .copyTo(tmp(cv::Rect(0, num_px, image_product_.cols, image_product_.rows - num_px)));
          break;
        case ShiftDirection::DOWN_TO_TOP:
          image_product_(cv::Rect(num_px, 0, image_product_.cols - num_px, image_product_.rows))
            .copyTo(tmp(cv::Rect(0, 0, image_product_.cols - num_px, image_product_.rows)));
          break;
        case ShiftDirection::STATIC:
        default:
          CLOG_DEBUG(5,"video-stream-metadata-band : shift-image : mode is STATIC");
          return true;
          break;
      }
      image_product_ = tmp;
      
      //from [https://stackoverflow.com/a/21813828/5817105](antonis io)
    }catch(const cv::Exception& e){
      CLOG_WARN("video-stream-metadata-band : shift-image : EXCEPT-RISED "
        "shifting [" <<shift_direction_ << "](" << num_px << ") : "
        "except((" << e.what() << ")");
    }catch(...){
      CLOG_WARN("video-stream-metadata-band : shift-image : EXCEPT-RISED "
        "shifting [" <<shift_direction_ << "](" << num_px << ") : "
        "except((UNKNOWN))");
    }

    return true;
  }
  bool VideoStreamMetadataBand::product_regen_()
  {
    try{
      image_product_ = background_base_;
    }catch(const cv::Exception& e){
      CLOG_WARN("video-stream-metadata-band : product-regen : EXCEPT-RISED "
        " : exept((" << e.what() << "))");
      return false;
    }catch(...){
      CLOG_WARN("video-stream-metadata-band : product-regen : EXCEPT-RISED "
        " : exept((UNKNOWN))");
      return false;
    }
    return true;
  }


  bool VideoStreamMetadataBand::add_colored_line_(double q,int r,int g,int b)
  {
      int x0,y0,xf,yf;
      switch(shift_direction_){
        case ShiftDirection::LEFT_TO_RIGHT:
          x0 = 0;
          y0 = image_product_.rows;
          xf = 0;
          yf = image_product_.rows - static_cast<int>(static_cast<double>(image_product_.rows)* q);
          break;
        case ShiftDirection::RIGHT_TO_LEFT:
          
          x0 = (image_product_.cols-1);
          y0 = image_product_.rows-1;
          xf = (image_product_.cols-1);
          yf = image_product_.rows -1 - static_cast<int>(static_cast<double>(image_product_.rows)* q);
          break;
        case ShiftDirection::TOP_TO_DOWN:
          x0 = 0;
          y0 = 0;
          xf = static_cast<int>(static_cast<double>(image_product_.cols) * q);
          yf = 0;
          break;
          break;
        case ShiftDirection::DOWN_TO_TOP:
          x0 = 0;
          y0 = image_product_.rows;
          xf = static_cast<int>(static_cast<double>(image_product_.cols) * q);
          yf = image_product_.rows;
          break;
      }
      try{
        CLOG_DEBUG(5,"video-stream-metadata-band : add-colored-line : ["
          << shift_direction_ << "](" << x0 << "," << y0 << " => " << xf << "," << yf << ")");
        cv::line(image_product_,cv::Point(x0,y0),cv::Point(xf,yf),cv::Scalar(r,g,b));
      }catch(const cv::Exception& e){
        CLOG_WARN("video-stream-metadata-band : add-colored-line [" << shift_direction_ 
          << "](" << x0 << "," << y0 << " => " << xf << "," << yf << ") : EXCEPT-RISED "
          " : exept((" << e.what() << "))");
        return false;
      }catch(...){
        CLOG_WARN("video-stream-metadata-band : add-colored-line [" << shift_direction_ 
          << "](" << x0 << "," << y0 << " => " << xf << "," << yf << ") : EXCEPT-RISED "
          " : exept((UNKNOWN))");
        return false;
      }

      return true;
  }

  bool VideoStreamMetadataBand::autogenThreadStart(){
    CLOG_WARN("video-stream-metadata-band : autogen-thread-start : NOT YET IMPLEMENTED");
    return false;
  }
  bool VideoStreamMetadataBand::autogenThreadStop(){
    CLOG_WARN("video-stream-metadata-band : autogen-thread-stop : NOT YET IMPLEMENTED");
    return false;
  }


bool VideoStreamMetadataBand::add_mark_point_()
{
  int x,y;
  switch(shift_direction_)
  {
    case ShiftDirection::TOP_TO_DOWN:
    case ShiftDirection::LEFT_TO_RIGHT:
      x = 5;
      y = 5;
      break;
    case ShiftDirection::RIGHT_TO_LEFT:
      x = image_product_.cols-5;
      y = 5;
      break;
    case ShiftDirection::DOWN_TO_TOP:
      x = 5;
      y = image_product_.rows-5;
      break;
    case ShiftDirection::STATIC:
    default:
      x = image_product_.cols / 2;
      y = image_product_.rows / 2;
      break;
  }
  
  CLOG_INFO("tick automark (" << x << "," << y << ")");
  cv::circle(image_product_,cv::Point(x,y),2,CV_RGB(52,103,152),2,2);
  return true;
}

static const std::vector<std::vector<int>> color_predefined_selects = 
  {
   {200,200,20}
  ,{15,234,172}
  };

bool VideoStreamMetadataBand::add_mark_line_(int percent)
{

  double pc = static_cast<double>(abs(percent%100)) / 100.0;
  int colorSelection = percent / 100;
  colorSelection = colorSelection%(sizeof(color_predefined_selects));
  return add_colored_line_(pc,color_predefined_selects[colorSelection][0],color_predefined_selects[colorSelection][1],color_predefined_selects[colorSelection][2]);
}
bool VideoStreamMetadataBand::add_mark_(const std::string& m,int markSize){
  CLOG_DEBUG(7,"video-stream-metadata-band : add-mark : adding ("<< m << ")(s:" 
    << markSize << ")");
  
  if(markSize<0)
  {
    return add_mark_point_();
  }


  return add_mark_line_(markSize);
  
}


static double _sin_gen_offset = 0.0;
static double _sin_gen_offset_period = 2000.0; // one second = one cycle
    

  bool VideoStreamMetadataBand::refreshImage(){
    bool res = true;
    if(last_update_ts_ == 0){
      last_update_ts_ = coyot3::tools::getCurrentTimestamp();
    }

    int64_t now = coyot3::tools::getCurrentTimestamp();
    int64_t dif = now - last_update_ts_;
    last_update_ts_ = now;

    switch (mode_)
    {
      case Mode::SIN_AUTOGEN:
      case Mode::SIN_AUTOGEN_THREADED:
        _sin_gen_offset+=dif;
        CLOG_DEBUG(7,"tick : diff = " << dif << " singenoffset : " << _sin_gen_offset);

        if(_sin_gen_offset>=_sin_gen_offset_period)
        {
          _sin_gen_offset-=_sin_gen_offset_period;
        }
        value_ = ((sin(2*M_PI * (_sin_gen_offset / _sin_gen_offset_period)) + 1.0)/2.0) / 2.0;
        //CLOG_DEBUG(9," refresh-image : value = " << value_);
        break;
      case Mode::EXTERNAL_VALUE:
      default:
        //we do nothing
        value_;
        break;
    }

    {
      std::lock_guard<std::mutex> guard(mtx_);
      if(!shift_image_(dif)){
        CLOG_WARN("video-stream-metadata-band : refresh-image : error shifting image!");
        res = false;
      }else if(!add_colored_line_(value_)){
        
        CLOG_WARN("video-stream-metadata-band : refresh-image : error adding marker!");
        res = false;
      }

      if(tmp_auto_mark_active_ == true){
        if((now - tmp_auto_mark_last_mark_ts_) > tmp_auto_mark_interval_){
          tmp_auto_mark_last_mark_ts_ = now;
          add_mark_(tmp_auto_mark_marker_,tmp_auto_mark_marker_size_);
        }
      }

    }



    return res;
  }



  bool VideoStreamMetadataBand::addMark(const std::string& m, int markSize)
  {
    std::lock_guard<std::mutex> guard(mtx_);
    return add_mark_(m,markSize);
  }

  bool VideoStreamMetadataBand::autoMark(bool activation,int64_t interval,const std::string& mark,int markSize)
  {
    tmp_auto_mark_active_ = activation;
    tmp_auto_mark_interval_ = interval;
    if(mark.size()!= 0){
      tmp_auto_mark_marker_ = mark;
    }
    if(markSize != 0){
      tmp_auto_mark_marker_size_ = markSize;
    }
    return true;
  }

}//eons
}

std::ostream& operator<<(std::ostream& o,coyot3::wrappers::VideoStreamMetadataBand::Format f)
{
  return o << coyot3::wrappers::VideoStreamMetadataBand::FormatToString(f);
}
std::ostream& operator<<(std::ostream& o,coyot3::wrappers::VideoStreamMetadataBand::Mode f)
{
  return o << coyot3::wrappers::VideoStreamMetadataBand::ModeToString(f);
}
std::ostream& operator<<(std::ostream& o,coyot3::wrappers::VideoStreamMetadataBand::ShiftDirection f)
{
  return o << coyot3::wrappers::VideoStreamMetadataBand::ShiftDirectionToString(f);
}