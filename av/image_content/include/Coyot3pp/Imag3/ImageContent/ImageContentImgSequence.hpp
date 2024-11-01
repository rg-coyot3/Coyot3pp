#pragma once

#include "ImageContent.hpp"

namespace coyot3{
namespace av{
namespace image{


  CYT3MACRO_enum_class_declarations(
    ImageContentImgSequenceMode
    , 
      , SEQUENCE_NORMAL = 0
      , RANDOM = 1
      , SEQUENCE_INVERSE = 2
      , STATIC = 3
  )

  CYT3MACRO_model_class_declarations(
    ImageContentImgSequenceParamsImage
    , 
    , ( )
    , ( )
    , name      , std::string       , ""
    , path      , std::string       , ""
    , interval  , int               , -1
  )

  CYT3MACRO_model_class_set_stack_declarations(
    ImageContentImgSequenceParamsImage,100)

  CYT3MACRO_model_class_declarations(
    ImageContentImgSequenceParams
    , 
    , ( )
    , ( )
    , base_path      , std::string                              , ""
    , interval       , int                                      , 5000
    , sequence       , ImageContentImgSequenceParamsImageStack  , 
    , current_index  , int                                      , 0
    , mode           , ec::ImageContentImgSequenceMode          , ec::ImageContentImgSequenceMode::SEQUENCE_NORMAL
  )


  CYT3MACRO_model_class_serializable_json_declarations(
    ImageContentImgSequenceParamsImage
    , 
    , 
    , ( )
    , ( )
    , name            , "name"            , 
    , path            , "path"            , 
    , interval        , "interval"        ,
  )
  CYT3MACRO_model_class_set_stack_serializable_json_declarations(
    ImageContentImgSequenceParamsImage)

  CYT3MACRO_model_class_serializable_json_declarations(
    ImageContentImgSequenceParams
    , 
    , 
    , ( sequence , "sequence" , ImageContentImgSequenceParamsImageStack)
    , ( )
    , base_path       , "base_path"    , 
    , interval         , "interval"    ,
  )



  class ImageContentImgSequence : public ImageContent{
    public:
      ImageContentImgSequence(const std::string& name = std::string());
      virtual ~ImageContentImgSequence();

      bool    set_configuration(const ImageContentImgSequenceParams& conf);

      bool next();
      bool previous();

      ImageContentImgSequenceParams& params_imgsec();
      
      

    protected:
      bool task_init();
      bool task_start();
      bool task_pause();
      bool task_stop();
      bool task_end();
      
      ImageContentImgSequenceParams params_imgsec_;
      coyot3::tools::WorkerThread*  wth_;
      std::mutex                    mtx_isec_;

      void                            image_sequence_task();
      void                            calculate_next_index();
                          bool          just_launched_;
                          std::string   current_file_path_;

    private:
      




  }; 


}
}
}