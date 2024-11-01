#pragma once

#include "WebsocketsServerGateway.hpp"
#include <Coyot3pp/Cor3/ModuleBase.hpp>

namespace coyot3::services::webapp{
  
  
  
  //data.content.desktop.ICON
  CYT3MACRO_model_class_declarations(
    ModSpecContHtmlDesktopIconObj
    , 
    , ( )
    , ( )
      , active          , bool        , false 
      , icon            , std::string , ""
  )
          CYT3MACRO_model_class_serializable_json_declarations(
            ModSpecContHtmlDesktopIconObj
            , 
            ,
            , ( )
            , ( )
              , active      , "active"          , 
              , icon        , "icon"            , 
          )

  //data.content.DESKTOP
  CYT3MACRO_model_class_declarations(
    ModSpecContHtmlDesktopObj
    , 
    , ( )
    , ( )
      , start_menu          , bool                              , false
      , toolbar             , bool                              , false
      , desktop_icon        , ModSpecContHtmlDesktopIconObj     , 
  )
          CYT3MACRO_model_class_serializable_json_declarations(
            ModSpecContHtmlDesktopObj, , 
            , (
               desktop_icon         , "desktop_icon"      , ModSpecContHtmlDesktopIconObj
            )
            , ( )
              , start_menu          , "start_menu"        , 
              , toolbar             , "toolbar"           ,
          )
  
  //data.content.html.FORMAT
  CYT3MACRO_model_class_declarations(
    ModSpecContHtmlFormatObj
    ,
    , ( )
    , ( )
      , maximized         , bool                        , false
      , minimized         , bool                        , false
      , x                 , std::string                 , "0"
      , y                 , std::string                 , "0"
      , w                 , std::string                 , "0"
      , h                 , std::string                 , "0"
      , classes           , coyot3::tools::CytStringSet , 
  )
          CYT3MACRO_model_class_serializable_json_declarations(
            ModSpecContHtmlFormatObj, , 
            , ( classes     , "classes"       , coyot3::tools::CytStringSet )
            , ( )
                , maximized         , "maximized"         , 
                , minimized         , "minimized"         , 
                , x                 , "x"                 , 
                , y                 , "y"                 , 
                , w                 , "w"                 , 
                , h                 , "h"                 , 
          )


  //data.content.HTML
  CYT3MACRO_model_class_declarations(
    ModSpecObjContHtml
    , 
    , ( )
    , ( )
      , title             , std::string               , ""
      , source            , std::string               , ""
      , icon              , std::string               , ""
      , alias             , std::string               , ""
      , format            , ModSpecContHtmlFormatObj  , 
  )
            CYT3MACRO_model_class_serializable_json_declarations(
              ModSpecObjContHtml, , 
              , ( 
                  format            , "format"            , ModSpecContHtmlFormatObj
              )
              , ( )
                , title             , "title"             , 
                , source            , "source"            , 
                , icon              , "icon"              , 
                , alias             , "alias"             , 
            )

  //data.content.STYLE_SHEET
  CYT3MACRO_model_class_declarations(
    ModSpecContStyleSheetsObj
    , 
    , ( )
    , ( )
      , name              , std::string                 , ""
      , sheets            , coyot3::tools::CytStringSet , 
  )
            CYT3MACRO_model_class_serializable_json_declarations(
              ModSpecContStyleSheetsObj, , 
              , (
                  sheets            ,"sheets"             , coyot3::tools::CytStringSet
              )
              , ( )
                , name              , "name"              , 
            )
  //data.content.STYLE_SHEET[]
  CYT3MACRO_model_class_set_stack_declarations(ModSpecContStyleSheetsObj, 1000)
            
            CYT3MACRO_model_class_set_stack_serializable_json_declarations(ModSpecContStyleSheetsObj)



  //data.content.JS
  CYT3MACRO_model_class_declarations(
    ModSpecContJavascriptObj
    , 
    , ( )
    , ( )
      , script              , std::string         , ""
      , init                , std::string         , ""
  )
            CYT3MACRO_model_class_serializable_json_declarations(
              ModSpecContJavascriptObj, , , ( ), ( )
                , script          , "script"          , 
                , init            , "init"            , 
            )
  //data.content.JS
  CYT3MACRO_model_class_set_stack_declarations(ModSpecContJavascriptObj, 1000)

            CYT3MACRO_model_class_set_stack_serializable_json_declarations(ModSpecContJavascriptObj)

  CYT3MACRO_model_class_declarations(
    ModSpecContentObject
    , 
    , ( )
    , ( )
      , desktop         , ModSpecContHtmlDesktopObj         , 
      , html            , ModSpecObjContHtml                , 
      , style_sheets    , ModSpecContStyleSheetsObjStack    , 
      , js              , ModSpecContJavascriptObjStack     , 
      , data            , coyot3::tools::CytStringSet       , 
  )

            CYT3MACRO_model_class_serializable_json_declarations(
              ModSpecContentObject, , 
              , (
                      desktop         , "desktop"         , ModSpecContHtmlDesktopObj
                    , html            , "html"            , ModSpecObjContHtml
                    , style_sheets    , "style_sheets"    , ModSpecContStyleSheetsObjStack
                    , js              , "js"              , ModSpecContJavascriptObjStack
                    , data            , "data"            , coyot3::tools::CytStringSet
              )
              , ( )

            )

  CYT3MACRO_model_class_declarations(
    WebModuleSpecificationObject
    , 
    , ( )
    , ( )
      , id              , std::string           , ""
      , name            , std::string           , ""
      , description     , std::string           , ""
      , active          , bool                  , false
      , content         , ModSpecContentObject  , 
  ) 

              CYT3MACRO_model_class_serializable_json_declarations(
                WebModuleSpecificationObject, , 
                , (
                  content       , "content"         , ModSpecContentObject
                )
                , ( )
                    , id              , "id"              , 
                    , name            , "name"            , 
                    , description     , "description"     , 
                    , active          , "active"          , 
                    , content         , "content"         , 
              )

  CYT3MACRO_model_class_declarations(
    Collag3WebappServerConfObj
    , 
    , ( )
    , ( )
      , id                      , int           , 666
      , name                    , std::string   , "collag3-server"
      
      , server_port             , int           , 9000
           
      , content_path_abs        , std::string   , ""
      , content_path_rel        , std::string   , ""
      , content_path_use_abs    , bool          , false
      
  )
  
  class Collag3WebappServer
  :public coyot3::services::websocket::WebsocketsServerGateway
  , public coyot3::mod::ModuleBase{
    public :
      Collag3WebappServer();
      virtual ~Collag3WebappServer();

      bool set_server_configuration(const Collag3WebappServerConfObj& conf);
      bool set_modules_configuration(const ModSpecContentObject& conf);




    protected:

      Collag3WebappServerConfObj  config;
      ModSpecContentObject        modules;


    private:


  };

}