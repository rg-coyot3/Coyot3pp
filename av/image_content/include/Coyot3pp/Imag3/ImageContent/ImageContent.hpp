#pragma once






#include <string>
#include <opencv2/opencv.hpp>


#include <vector>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/background_segm.hpp> /// for motion detector

//#include <cv_bridge/cv_bridge.h>



#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <Coyot3pp/Cor3/ModuleBase.hpp>



namespace coyot3{
namespace av{
namespace image{


    // SourceImageConfig
  CYT3MACRO_model_class_declarations(
    SourceImageConfig
      ,
      , ( )
      , ( )
        , name        , std::string   , ""
        , is_active   , bool          , true
        , source_type , std::string   , ""
        , source_info , std::string   , ""
        , rotation    , double        , 0.0
        , show_title  , bool          , true

  );

  // vector like from SourceImageConfig = SourceImageConfigStack
  CYT3MACRO_model_class_set_stack_declarations(
    SourceImageConfig, 100);


  // JSON - 

          // serialization for SourceImageConfig
        CYT3MACRO_model_class_serializable_json_declarations( 
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
  CYT3MACRO_model_class_set_stack_serializable_json_declarations(SourceImageConfig);


CYT3MACRO_model_class_declarations(
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


CYT3MACRO_model_class_declarations(
  ImageContentParams
  , 
  , ( void cross_image_dimensions_with_rels(int dest_width COMMA() 
                                            int dest_height COMMA() 
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


/**
 * @brief contiene la imagen que será "pegada" en la imagen del stream final.
 *        Tiene el formato original 
*/
class ImageContent : public coyot3::mod::ModuleBase{
  public:
    static void InitializeOpenCvVisualPreviews();

    ImageContent(const std::string& instanceName = std::string());
    virtual ~ImageContent();

    /**
     * @brief tests state to init. initializes both source and product mats with 
     *  params.width and height. transits state to initialized.
     * 
     * @return true 
     * @return false 
     */
    bool task_init();

    /**
     * @brief prepares the image stored at image-source, for image-product
     * 
     * @return true 
     * @return false 
     */
    virtual bool prepare_output();

    /**
     * @brief Get the Image product reference.
     * 
     * @return cv::Mat& 
     */
    cv::Mat&    getImageRef();

    /**
     * @brief Get the product mutex reference.
     * 
     * @return std::mutex& 
     */
    std::mutex& getMutexRef();

    /**
     * @brief Get the Image Copy. locks the product mutex
     * 
     * @return cv::Mat 
     */
    cv::Mat     getImageCopy();
    

    // virtual bool config_from_json(const Json::Value& config);
    virtual bool set_config(const SourceImageConfig& config);
            bool set_params(const ImageContentParams& isdm);


    bool preserve_aspect_ratio() const;
    bool preserve_aspect_ratio(bool preserve);
    
    /**
     * @brief Set the image resize params. it is a virtual method so inherited
     *        classes may just want to operate directly at the product. 
     *        resizes the product image.
     * 
     * @param width 
     * @param height 
     * @param preserve_aspect_ratio 
     * @return true 
     * @return false 
     */
    virtual bool set_dimensions(
      int width,
      int height,
      bool preserve_aspect_ratio = true);

    /**
     * @brief crosses relative positions with final destination and height
     * 
     * @param dest_width 
     * @param dest_height 
     * @param preserve_ar 
     */
    void cross_image_dimensions_with_rels(
      int dest_width,
      int dest_height,
      bool preserve_ar = true);
    /**
     * @brief Set the relative position of the image. Metadata
     * 
     * @param x0 0.0 - 1.0 ==> 0% - 100%
     * @param y0 0.0 - 1.0 ==> 0% - 100%
     * @param xf 0.0 - 1.0 ==> 0% - 100%
     * @param yf 0.0 - 1.0 ==> 0% - 100%
     * @param preserve_ar 
     */
    void set_relative_position(
      double x0,double y0,
      double xf,double yf,
      bool preserve_ar = true);

    int get_width() const;
    int get_height() const;

    const bool show_preview() const;
    bool show_preview(bool p);


    double product_aspect_ratio() const;
    double source_aspect_ratio() const;
    void   setLogDebugLevel(int level);

    virtual void   debug_level_set(int l);
    
    int         id() const;
    int         id(int i);

    ImageContentParams   & params();
    ImageContentParams     params() const;
    
    bool        shift_product(int x,int y);
    bool        shift_source(int x,int y);
    
  protected:
    ImageContentParams    params_;
    cv::Mat               image_product;
    cv::Mat               image_source;
    std::mutex            mtx_output_;
    std::mutex            mtx_source_;


    bool update_img_to_product();
    bool imgprod_resize(); //mutex lock
    bool imgprod_rotate(); //mutex lock

    bool imgprod_show_title();    
    //bool find_encodings();


    bool img_product_initialize(int x = 0,int y = 0); //mutex lock
    bool img_source_initialize(int x = 0,int y = 0); //mutex lock

    
  private:


};

}
}
}

