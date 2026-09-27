#pragma once

#include "WebsocketsServerGateway.hpp"
#include <Coyot3pp/Cor3/ModuleBase.hpp>

namespace coyot3::services::webapp{
  
  
  
  //data.content.desktop.ICON
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    ModSpecContHtmlDesktopIconObj
    , 
    , ( )
    , ( )
      , active          , bool        , false 
      , icon            , std::string , ""
  )
          COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
            ModSpecContHtmlDesktopIconObj
            , 
            ,
            , ( )
            , ( )
              , active      , "active"          , 
              , icon        , "icon"            , 
          )

  //data.content.DESKTOP
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    ModSpecContHtmlDesktopObj
    , 
    , ( )
    , ( )
      , start_menu          , bool                              , false
      , toolbar             , bool                              , false
      , desktop_icon        , ModSpecContHtmlDesktopIconObj     , 
  )
          COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
            ModSpecContHtmlDesktopObj, , 
            , (
               desktop_icon         , "desktop_icon"      , ModSpecContHtmlDesktopIconObj
            )
            , ( )
              , start_menu          , "start_menu"        , 
              , toolbar             , "toolbar"           ,
          )
  
  //data.content.html.FORMAT
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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
          COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
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
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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
            COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
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
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    ModSpecContStyleSheetsObj
    , 
    , ( )
    , ( )
      , name              , std::string                 , ""
      , sheets            , coyot3::tools::CytStringSet , 
  )
            COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
              ModSpecContStyleSheetsObj, , 
              , (
                  sheets            ,"sheets"             , coyot3::tools::CytStringSet
              )
              , ( )
                , name              , "name"              , 
            )
  //data.content.STYLE_SHEET[]
  COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(ModSpecContStyleSheetsObj, 1000)
            
            COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DECLARATIONS(ModSpecContStyleSheetsObj)



  //data.content.JS
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    ModSpecContJavascriptObj
    , 
    , ( )
    , ( )
      , script              , std::string         , ""
      , init                , std::string         , ""
  )
            COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
              ModSpecContJavascriptObj, , , ( ), ( )
                , script          , "script"          , 
                , init            , "init"            , 
            )
  //data.content.JS
  COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(ModSpecContJavascriptObj, 1000)

            COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DECLARATIONS(ModSpecContJavascriptObj)

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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

            COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
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

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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

              COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
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

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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