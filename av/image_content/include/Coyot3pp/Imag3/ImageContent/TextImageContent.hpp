#pragma once



#include "ImageContent.hpp"

namespace coyot3{
namespace av{
namespace image{

  COYOT3PP_ENUM_CLASS_DECLARATIONS(
    TextHorizontalAlign
    ,
      , UNDEFINED   , 0
      , RIGHT       , 1
      , CENTER      , 2
      , LEFT        , 3
  );

  COYOT3PP_ENUM_CLASS_DECLARATIONS(
    TextVerticalAlign
    ,
      , UNDEFINED , 0
      , TOP       , 1
      , MIDDLE    , 2
      , BOTTOM    , 3
  );

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    ImageTextContent
    , 
    , ( )
    , ( )
      , name            , std::string           , ""
      , id              , int                   , 0
      , active          , bool                  , false
      , content         , std::string           , ""
      
      , font            , std::string           , ""
      , size            , double                , 12.0
      , size_multipl    , double                , 1.0
      , bold            , bool                  , false
      , underlined      , bool                  , false
      , crossed         , bool                  , false
      , wrap_contents   , bool                  , false
      , align_hor       , ec::TextHorizontalAlign   , ec::TextHorizontalAlign::UNDEFINED
      , align_ver       , ec::TextVerticalAlign     , ec::TextVerticalAlign::UNDEFINED

      , fg_color_val    , int                   , 0
      , fg_color_str    , std::string           , ""
      , bg_color_val    , int                   , 1
      , bg_color_str    , std::string           , ""
      , border_width    , int                   , 0
      , bo_color_val    , int                   , 1
      , bo_color_str    , std::string           , ""


      , x               , double                , 0.0
      , y               , double                , 0.0
      , w               , double                , 0.0
      , h               , double                , 0.0
      , transparency    , double                , 0.0
  )

  COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(ImageTextContent,id);



}
}
}
