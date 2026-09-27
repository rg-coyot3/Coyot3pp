#include "qsqlite_connector_example.hpp"

  COYOT3PP_MODEL_CLASS_DEFINITIONS(
    DatabaseEntityDAO
    , 
    , ( )
    , ( )
      , id                  , int64_t           , 0
      , name                , std::string       , ""
      , description         , std::string       , ""
      , metadata            , std::string       , ""
  )

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
      DatabaseEntityDAO
      , 
      , 
      , ( )
      , ( )
        , id                  , "id"                  ,
        , name                , "name"                ,
        , description         , "description"         ,
        , metadata            , "metadata"            ,
    )

// common-dao : end

// service-stop-dao : begin 

  COYOT3PP_MODEL_CLASS_DEFINITIONS(
    ServiceStopDAO
    , DatabaseEntityDAO
    , ( )
    , ( )
      , active              , bool                         , false
      , latitude            , double                       , 0.0
      , longitude           , double                       , 0.0
      , altitude            , double                       , 0.0
  )
    COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(ServiceStopDAO,)

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
    ServiceStopDAO
    , DatabaseEntityDAO
    ,
    , ( )
    , ( )
      , active              , "active"                            ,
      , latitude            , "latitude"                          ,
      , longitude           , "longitude"                         ,
      , altitude            , "altitude"                          ,
    )
    COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DEFINITIONS(ServiceStopDAO)


  CYT3MACRO_model_class_serializable_qsqlite_definitions(
      ServiceStopDAO
    , ( )
    , id                  , "id"                , "INTEGER PRIMARY KEY AUTOINCREMENT"   
    , name                , "name"              , "TEXT"                                
    , description         , "description"       , "TEXT"                                
    , active              , "active"            , "INTEGER"                             
    , latitude            , "latitude"          , "NUMERIC"                             
    , longitude           , "longitude"         , "NUMERIC"                             
    , altitude            , "altitude"          , "NUMERIC"                             
  )


  
  std::string ServiceStopDAO::to_string(){
    std::stringstream sstr;
    sstr << id() << "," << name() << "," << active();
    return sstr.str();
  }



