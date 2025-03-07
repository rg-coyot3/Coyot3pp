#include "qsqlite_connector_example.hpp"

  CYT3MACRO_model_class_definitions(
    DatabaseEntityDAO
    , 
    , ( )
    , ( )
      , id                  , int64_t           , 0
      , name                , std::string       , ""
      , description         , std::string       , ""
      , metadata            , std::string       , ""
  )

    CYT3MACRO_model_class_serializable_json_definitions(
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

  CYT3MACRO_model_class_definitions(
    ServiceStopDAO
    , DatabaseEntityDAO
    , ( )
    , ( )
      , active              , bool                         , false
      , latitude            , double                       , 0.0
      , longitude           , double                       , 0.0
      , altitude            , double                       , 0.0
  )
    CYT3MACRO_model_class_set_stack_definitions(ServiceStopDAO,)

    CYT3MACRO_model_class_serializable_json_definitions(
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
    CYT3MACRO_model_class_set_stack_serializable_json_definitions(ServiceStopDAO)


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



