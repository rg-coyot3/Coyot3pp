#include "sqlit3_macro_classes.hpp"



CYT3MACRO_model_class_definitions(
  SimpleClass
  ,
  , ( )
  , ( )
    , id      , int64_t         , 
    , name    , std::string     , 
    , surname , std::string     , 
    , age     , int             ,
    , height  , double          ,   
  )

CYT3MACRO_model_class_set_stack_definitions(SimpleClass,)


CYT3MACRO_model_class_serializable_sqlit3_definitions(
  SimpleClass
  , ( )
  , id          , "INTEGER PRIMARY KEY AUTOINCREMENT"     , "id"
  , name        , "TEXT"                                  , "name"
  , surname     , "TEXT"                                  , "surname"
  , age         , "INTEGER"                               , "age"
  , height      , "REAL"                                  , "height"
)


