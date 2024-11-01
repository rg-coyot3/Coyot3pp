#include <Coyot3pp/Imag3/ImageContent/ImageContent.hpp>

#include <thread>
#include <X11/Xlib.h>

bool __CYT_OPENCV_PREVIEWS_ARE_INTIIALIZED = false;


namespace ct = coyot3::tools;

namespace coyot3{
namespace av{
namespace image{


  void ImageContent::InitializeOpenCvVisualPreviews(){
    if(!__CYT_OPENCV_PREVIEWS_ARE_INTIIALIZED){
      XInitThreads();
    }
    __CYT_OPENCV_PREVIEWS_ARE_INTIIALIZED=true;
  }

  CYT3MACRO_model_class_definitions(
    SourceImageConfig
      ,
      , ()
      , ()
        , name        , std::string   , ""
        , is_active   , bool          , true
        , source_type , std::string   , ""
        , source_info , std::string   , ""
        , rotation    , double        , 0.0
        , show_title  , bool          , true

  );

  // vector like from SourceImageConfig = SourceImageConfigStack
  CYT3MACRO_model_class_set_stack_definitions(
    SourceImageConfig, 100);


  // JSON - 

          // serialization for SourceImageConfig
        CYT3MACRO_model_class_serializable_json_definitions( 
          SourceImageConfig
          , 
          ,
          , ()
          , ()
            , name         , "name"          , 
            , is_active    , "is_active"     ,    
            , source_type  , "source_type"   , 
            , source_info  , "source_info"   , 
            , show_title   , "show_title"    ,   
            , rotation     , "rotation"      , 

        );

  // serialization for SourceImageConfigStack = SourceImageConfigStackJsIO
  CYT3MACRO_model_class_set_stack_serializable_json_definitions(SourceImageConfig);




/**
 * @brief ImageContentParams son un conjunto de parámetros comunes para todas 
 *        las clases heredadas... aunque hay muchas propiedades que 
 *        probablemente acabe quitando...
 * 
 */

CYT3MACRO_model_class_definitions(
  ImageContentEffectsParams
  , 
  , ( std::string to_string() const)
  , ( )
    , transparency        , double          , 0.0
    , rotation            , int             , 0
    , src_shifting_x      , int             , 0
    , src_shifting_y      , int             , 0
    , black_and_white     , bool            , false
    , border_width        , int             , 0
)

std::string ImageContentEffectsParams::to_string() const{
  std::stringstream sst;
  sst << "transparency=" << transparency_ <<
          ";rotation=" << rotation_ <<
          ";src_shifting_x=" << src_shifting_x_ <<
          ";src_shifting_y=" << src_shifting_y_ <<
          ";black_and_white=" << black_and_white_ <<
          ";border_width=" << border_width_;
  return sst.str();
}


CYT3MACRO_model_class_definitions(
  ImageContentParams
  , 
  , ( void cross_image_dimensions_with_rels(int dest_width, 
                                            int dest_height, 
                                            bool preserve_ar), 
      std::string to_string(),
      double aspect_ratio(),
      void* user_data_,
      void* user_data() const,
      void*& user_data(),
      void*& user_data(const void* p)
  )
  , ( )
    , id                    , int               , 0
    , name                  , std::string       , ""
    , src_type              , std::string       , ""

    , width                 , int               , 0     
    , height                , int               , 0  

    , effects               , ImageContentEffectsParams ,


    , encoding_source       , std::string       , ""    
    , encoding_target       , std::string       , ""
    , cvt_color_target      , int               , -1
    , encoding_found        , bool              , false
    , color_conversion_do   , bool              , false

    , last_update_ts        , int64_t           , 0
    , last_update_source_ts , int64_t           , 0

    , update_interval       , int64_t           , 100
    , show_preview          , bool              , false
    , show_name             , bool              , false

    , pos_x                 , int               , 0
    , pos_y                 , int               , 0

    , do_resize             , bool              , false
    , preserve_aspect_ratio , bool              , false

    
    
    , is_static             , bool              , false
    , rxi                   , double            , 0.0 
    , ryi                   , double            , 0.0
    , rxf                   , double            , 0.0
    , ryf                   , double            , 0.0

    , base_path             , std::string       , ""
    , debug_level           , int               , 0

)

 void* ImageContentParams::user_data() const{return user_data_;}
void*& ImageContentParams::user_data() {return user_data_;}
void*& ImageContentParams::user_data(const void* p){user_data_ = const_cast<void*>(p);return user_data_;}


std::string ImageContentParams::to_string(){
  std::stringstream sst;

  sst << 
  " id=" << id_ <<
  ";name=" << name_ <<
  ";src_type=" << src_type_ <<
  ";width=" << width_ <<
  ";height=" << height_ <<
  ";effects=(" << effects_.to_string() <<
  ")"
  ";encoding_source=" << encoding_source_ << 
  ";encoding_target=" << encoding_target_ << 
  ";cvt_color_target=" << cvt_color_target_ << 
  ";encoding_found=" << encoding_found_ << 
  ";color_conversion_do=" << color_conversion_do_ << 
  ";last_update_ts=" << last_update_ts_ << 
  ";last_update_source_ts=" << last_update_source_ts_ << 
  ";update_interval=" << update_interval_ << 
  ";show_preview=" << show_preview_ << 
  ";show_name=" << show_name_ << 
  ";user_data=" << user_data_ << 
  ";pos_x=" << pos_x_ << 
  ";pos_y=" << pos_y_ << 
  ";do_resize=" << do_resize_ << 
  ";preserve_aspect_ratio=" << preserve_aspect_ratio_ << 
  ";rxi=" << rxi_ << 
  ";ryi=" << ryi_ << 
  ";rxf=" << rxf_ << 
  ";ryf=" << ryf_ << 
  ";base_path=" << base_path_;
  return sst.str();
}
double ImageContentParams::aspect_ratio(){
  return static_cast<double>(width()) / static_cast<double>(height());
}



void 
  ImageContentParams::cross_image_dimensions_with_rels(
  int dest_width,
  int dest_height,
  bool preserve_ar)
{
  pos_x(std::floor(static_cast<double>(dest_width)   * rxi()));
  pos_y(std::floor(static_cast<double>(dest_height)  * ryi()));
  width(std::floor(static_cast<double>(dest_width)   * (rxf() - rxi())));
  height(std::floor(static_cast<double>(dest_height) * (ryf() - ryi())));
  preserve_aspect_ratio(preserve_ar);
  CLOG_INFO("imc-params : croos-dim-rel :[" << name() << "]: (" << dest_width << "," 
  << dest_height << ") = [" << to_string() << "]")
}

ImageContent::ImageContent(const std::string& instanceName)
:ModuleBase(instanceName)
,params_()
,image_product()
,image_source()
{
  class_name("img-content");
  ImageContent::InitializeOpenCvVisualPreviews();
  log_info("image-content : constructor : new instance created with no params");

  //module config 
  conf_task_init_(std::bind(&ImageContent::task_init,this));
}

ImageContent::~ImageContent(){}

bool ImageContent::task_init(){
  img_product_initialize();
  img_source_initialize();
  CLOG_DEBUG_LEVEL_SET(params_.debug_level());
  log_info("imc : init : product initialized");
  return true;
}

void ImageContent::setLogDebugLevel(int level){
  log_warn(o() << "imc : set-log-debug-level : "<< level);
  CLOG_DEBUG_LEVEL_SET(level);
}

cv::Mat& ImageContent::getImageRef(){
  return image_product;
}

cv::Mat ImageContent::getImageCopy(){
  cv::Mat i;
  {
    std::lock_guard<std::mutex> guard(mtx_output_);
    i = image_product.clone();
  }
  return i;
}

bool ImageContent::set_config(const SourceImageConfig& config){
  log_info("imc : set-configuration ");
  params_.name(config.name());
  params_.effects().rotation(config.rotation());
  params_.show_name(config.show_title());

  return true;
}


bool ImageContent::prepare_output(){
  bool opres = true;
  //resize

  //shifting;
  if(params_.effects().src_shifting_x() || params_.effects().src_shifting_y()){
    if(!shift_source(params_.effects().src_shifting_x(), params_.effects().src_shifting_y())){
      log_warn("imc :  update : error shifting source");
      return false;
    }
  }

  if(!update_img_to_product()){
    log_warn("imc : update : error importing to product");
    return false;
  }

  if(!(opres &= imgprod_rotate())){
    log_warn("imc : update : img rotation error");
  }
  if(!(opres &= imgprod_show_title())){
    log_warn("imc : update : error drawing title");
  }

  
  if(show_preview()){
    cv::resizeWindow(name()+"_prod", params_.width(),params_.height());
    cv::imshow(name()+"_prod",image_product);
    cv::waitKey(1);
  
  }
  return opres;
}

bool ImageContent::update_img_to_product(){
  bool opres = true;
  std::lock_guard guard_output(mtx_output_);
  std::lock_guard guard_source(mtx_source_);
  int     f_height;
  int     f_width;
  int     desp_x = 0;
  int     desp_y = 0;
  double prop_orig,prop_dest;
  try{
    if(params_.preserve_aspect_ratio() == false){
      cv::resize(
        image_source,
        image_product,
        cv::Size(params_.width(),params_.height()),cv::INTER_LINEAR);
    }else{
      prop_orig = source_aspect_ratio();
      prop_dest = product_aspect_ratio();

      bool fifif;
      if(prop_dest >= prop_orig)
      {
        fifif = true;
        //dest width >= dest height
        f_height = params_.height();
        f_width  = static_cast<int>(static_cast<double>(params_.height()) * prop_orig);
        desp_x = (params_.width() - f_width) / 2;
        if(desp_x < 0){
          desp_y = static_cast<int>(static_cast<double>(std::abs(desp_x)) * prop_dest);
          f_height = std::floor(static_cast<double>(params_.width()) * prop_dest);
          f_width = params_.width();
          desp_x = 0;
        }
      }else{
        fifif = false;
        //dest width < dest height
        f_width = params_.width();
        f_height = static_cast<int>(static_cast<double>(params_.width()) / prop_orig);
        desp_y = (params_.height() - f_height) / 2;
        if(desp_y < 0){
          desp_x = static_cast<int>(static_cast<double>(std::abs(desp_y)) / prop_dest);
          f_width = std::float_denorm_style(static_cast<double>(params_.height()) / prop_dest);
          f_height = params_.height();
          desp_y = 0;
        }
      }
      // CLOG_INFO("----C: " << fifif << " " << name() << "(" << image_source.cols << " x " << image_source.rows
      //   << ")p:" << prop_orig << " => (" << params_.width() << " x " << params_.height() 
      //   << ")p:" << prop_dest << "[" << desp_x << "," << desp_y << "]")
      cv::Mat buffer = cv::Mat(f_width,
                            f_height,
                            CV_8UC3,
                            cv::Scalar(0,0,0));
      cv::resize(image_source,
                  buffer,cv::Size(f_width,f_height),cv::INTER_LINEAR);
      image_product = cv::Mat::zeros(cv::Size(params_.width(),
                                    params_.height()),
                                    buffer.type());
      buffer.copyTo(image_product(cv::Rect(desp_x,desp_y,f_width,f_height)));
      CLOG_DEBUG(5,"image-content : -resize-current :[" << name() 
      << "]: image resized to (" << f_width << " x " << f_height << ")");
      // CLOG_INFO("image-content : resized to product :[" << name() 
      // << "]: from original [" <<  image_source.cols << "," << image_source.rows 
      // << "] to [" << image_product.cols << "," << image_product.rows << "]")
    }
  }catch(const cv::Exception& e){
    CLOG_WARN("image-content : update-img-to-product- :[" << name() << "]: "
    "error resizing (" <<f_width << ", " << f_height << "; offset(" 
    << desp_x << "," << desp_y << ")) : "
    << "(prop_orig=" << prop_orig << ";prop_dest=" << prop_dest << ")"
    "except(" << e.what() << ") : " 
    << params_.to_string())
    opres = false;
  }catch(...){
    CLOG_WARN("image-content : update-img-to-product- :[" << name() << "]: "
    "error resizing (" <<f_width << ", " << f_height << "; offset(" 
    << desp_x << "," << desp_y << ")) : except( NOT HANDLED ) : " 
    << params_.to_string())
    opres = false;
  }
  return opres;
}

bool ImageContent::imgprod_rotate(){
  if(params_.effects().rotation() == 0){
    return true;
  }
  std::lock_guard guard(mtx_output_);
  bool opres = true;
  try{
    log_debug(7,o() << "imc : imgprod-rotate : rotating (" 
      << params_.effects().rotation() << ")");
    switch(params_.effects().rotation())
    {
      case 90:
        cv::rotate(image_product,image_product,cv::ROTATE_90_CLOCKWISE);
        break;
      case 180:
        cv::rotate(image_product,image_product,cv::ROTATE_180);
        break;
      case -90:
      case 270: 
        cv::rotate(image_product,image_product,cv::ROTATE_90_COUNTERCLOCKWISE);
        break;
      default:
      {
        cv::Point2f center((image_product.cols -1)/2.0,
                           (image_product.rows - 1)/2.0);
        cv::Mat rot_matrix = cv::getRotationMatrix2D(
                                center,
                                static_cast<double>(params_.effects().rotation()),
                                1.0);
        //cv::Mat ri;
        cv::warpAffine(image_product,image_product,rot_matrix,image_product.size());
      }
    }
  }catch(const cv::Exception& e){
    log_warn(o() << "imc : imgprod-rotate : error rotating image [" 
      << params_.effects().rotation() << " deg]. exception(" 
      << e.what() << ")");
    opres = false;
  }catch(...){
    log_warn(o() << "imc : imgprod-rotate : error rotating image [" 
      << params_.effects().rotation() << " deg]. exception( UNKNOWN )");
    opres = false;
  }
  return opres;
}

const bool ImageContent::show_preview() const {return params_.show_preview();}
bool ImageContent::show_preview(bool p){return params_.show_preview(p);}

double 
ImageContent::product_aspect_ratio() const{
  if(params_.last_update_ts() == 0){
    return 1;
  }
  return static_cast<double>(static_cast<double>(image_product.cols) 
         / static_cast<double>(image_product.rows));
}

double
ImageContent::source_aspect_ratio() const{
  return static_cast<double>(static_cast<double>(image_source.cols) 
         / static_cast<double>(image_source.rows));
}

bool 
ImageContent::set_dimensions(
  int width,
  int height,
  bool preserve_aspect_ratio)
{

  if((width < 0) || (height < 0))
  {
    CLOG_WARN("image-content : resize-image : setting wrong params "
      "for final resized image => [w:" << width << ",h:" << height << "]");
    return false;
  }
  bool operate = false;
  params_.preserve_aspect_ratio(preserve_aspect_ratio);
  if(width == params_.width() && height == params_.height()){
    return true;
  }
  params_.width(width);
  params_.height(height);
  
  
  return img_product_initialize(params_.width(),params_.height());
}


//misc

void ImageContent::cross_image_dimensions_with_rels(
      int dest_width,
      int dest_height,
      bool preserve_ar){
  params_.cross_image_dimensions_with_rels(dest_width,dest_height,preserve_ar);
  img_product_initialize(params_.width(),params_.height());
}
bool ImageContent::preserve_aspect_ratio() const{
  return params_.preserve_aspect_ratio();
}
bool ImageContent::preserve_aspect_ratio(bool preserve){
  return params_.preserve_aspect_ratio(preserve);
}

bool ImageContent::set_params(const ImageContentParams& isdm){
  params_ = isdm;
  CLOG_INFO("image-content : set-params : " << params_.to_string())
  return true;
}


bool ImageContent::imgprod_resize(){
  std::lock_guard guard(mtx_output_);
  bool opres = true;
  try{
    cv::resize(image_product,
              image_product,
              cv::Size(params_.width(),params_.height()));
  }catch(const cv::Exception& e){
    CLOG_WARN("image-content : imgprod-resize- : resize to (" 
    << params_.width() << "," << params_.height() << ") resulted in error."
    " exception(" << e.what() << ")")
    opres = false;
  }catch(...){
    CLOG_WARN("image-content : imgprod-resize- : resize to (" 
    << params_.width() << "," << params_.height() << ") resulted in error."
    " exception( UNKNOWN )")
    opres = false;
  }
  return opres;
}

void 
ImageContent::set_relative_position(
  double x0,double y0,double xf,double yf, bool preserve_ar){
    CLOG_DEBUG(5,"image-content : set-rel-pos- : (" << x0 << "," << y0 
      << " => " << xf << "," << yf << "): par(" << (preserve_ar?"true":"false") 
      <<")")
    params_.rxi(x0);params_.ryi(y0);params_.rxf(xf);params_.ryf(yf);
    params_.preserve_aspect_ratio(preserve_ar);
}

int ImageContent::get_width() const{
  return static_cast<int>(image_product.cols); 
}

int ImageContent::get_height() const{
  return static_cast<int>(image_product.rows); 
}


std::mutex& ImageContent::getMutexRef(){return mtx_output_;}



ImageContentParams& ImageContent::params(){return params_;}
ImageContentParams  ImageContent::params()const{return params_;}

int ImageContent::id() const{return params_.id();}
int ImageContent::id(int i){return params_.id(i);}


bool ImageContent::img_product_initialize(int x,int y){
  std::lock_guard guard(mtx_output_);
  bool allgood = true;
  if(x == 0 || y == 0){
    x = params_.width();y = params_.height();
  }
  if( x == image_product.cols && y == image_product.rows){
    //nothing to do.
    return true;
  }
  try{
    image_product = cv::Mat(y,x,CV_8UC3,cv::Scalar(0,0,0));
    log_debug(5,o() << "img-prod-initialize : to(" << x << "," << y << ")[" 
      << image_product.rows << "," 
      << image_product.cols << "] :: [[" << params_.to_string() << "]]");
  }catch(const cv::Exception& e){
    log_warn(o() << "img-prod-initialize : error initializing product image for"
      "dimensions [" << x << "," << y << "]. exception(" << e.what() << ")");
    allgood = false;
  }catch(...){
    log_warn(o() << "img-prod-initialize : error initializing product image for"
      " dimensions [" << x << "," << y << "]. exception( UNKNOWN!)");
    allgood = false;
  }
  return allgood;
}
bool ImageContent::img_source_initialize(int x,int y){
  std::lock_guard guard(mtx_source_);
  bool allgood = true;
  if(x == 0 || y == 0){
    x = params_.width();y = params_.height();
  }
  if(x == image_source.cols && y == image_source.rows){

    return true;
  }
  try{
    image_source = cv::Mat(y,x,CV_8UC3,cv::Scalar(0,0,0));
    CLOG_DEBUG(5,"image-content : img-source-initialize :[" << name() << "]: "
    "to(" << x << "," << y << ")[" << image_source.cols << "," 
    << image_source.rows << "] :: [[" << params_.to_string() << "]]")
  }catch(const cv::Exception& e){
    CLOG_WARN("image-content : img-source-initialize :[" << name() << " : "
    "error initializing product image for dimensions [" << x << "," << y << "]."
    " exception(" << e.what() << ")")
    allgood = false;
  }catch(...){
    CLOG_WARN("image-content : img-source-initialize :[" << name() << " : "
    "error initializing product image for dimensions [" << x << "," << y << "]."
    " exception( UNKNOWN!)")
    allgood = false;
  }
  return allgood;
}

void ImageContent::debug_level_set(int l){CLOG_DEBUG_LEVEL_SET(l)}

bool ImageContent::imgprod_show_title(){
  if(params_.show_name() == false)return true;
  cv::Point loc(params_.width() / 30,params_.height() / 30);
  double sz = static_cast<double>(params_.width()) / 600.;
  cv::putText(image_product,
              name(),
              loc,
              cv::FONT_HERSHEY_COMPLEX,
              sz,
              CV_RGB(0,0,0),
              4);
  
  cv::putText(image_product,
              name(),
              loc,
              cv::FONT_HERSHEY_COMPLEX,
              sz,
              CV_RGB(255,255,255),
              2);
  
  return true;
}


bool ImageContent::shift_product(int x, int y){
  if(x == 0 && y == 0){return true;}
  if(abs(x) > params_.width() || abs(y) > params_.height())return false;
  std::lock_guard<std::mutex> guard(mtx_output_);
  cv::Mat tmp= cv::Mat(cv::Mat::zeros(image_product.size(),image_product.type()));
  int xs0 , ys0, xd0 , yd0, fw, fh;

  if(x >= 0){
    xs0 = 0;
    fw = params_.width() - x - 1;
    xd0 = x;
  }else{
    xs0 = std::abs(x);
    fw = params_.width() + x - 1;
    xd0 = 0;
  }
  if(y >= 0){
    ys0 = 0;
    fh = params_.height() - y - 1;
    yd0 = y;
  }else{
    ys0 = std::abs(y);
    fh = params_.height() + y - 1;
    yd0 = 0;
  }

  image_product(cv::Rect(xs0,ys0,fw,fh)).copyTo(tmp(cv::Rect(xd0,yd0,fw,fh)));
  image_product = tmp;
  return true;
}

bool ImageContent::shift_source(int x, int y){
  if(x == 0 && y == 0){return true;}
  if(abs(x) > image_source.cols || abs(y) > image_source.rows)return false;
  std::lock_guard<std::mutex> guard(mtx_source_);
  cv::Mat tmp= cv::Mat(cv::Mat::zeros(image_source.size(),image_source.type()));
  int xs0 , ys0, xd0 , yd0, fw, fh;

  if(x >= 0){
    xs0 = 0;
    fw = image_source.cols - x - 1;
    xd0 = x;
  }else{
    xs0 = std::abs(x);
    fw = image_source.cols + x - 1;
    xd0 = 0;
  }
  if(y >= 0){
    ys0 = 0;
    fh = image_source.rows - y - 1;
    yd0 = y;
  }else{
    ys0 = std::abs(y);
    fh = image_source.rows + y - 1;
    yd0 = 0;
  }

  image_source(cv::Rect(xs0,ys0,fw,fh)).copyTo(tmp(cv::Rect(xd0,yd0,fw,fh)));
  image_source = tmp;
  return true;
}


}
}
}