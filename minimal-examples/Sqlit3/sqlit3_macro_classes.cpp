#include "sqlit3_macro_classes.hpp"



CYT3MACRO_model_class_definitions(
  SimpleClass
  ,
  , ( std::string to_string() const)
  , ( )
    , id      , int64_t         , 
    , name    , std::string     , 
    , surname , std::string     , 
    , age     , int             ,
    , height  , double          ,   
  )


  std::string SimpleClass::to_string() const{
    std::stringstream sstr;
    sstr << "id=" << id() << ";name=" << name() << ";"
      "surname=" << surname() << ";age=" << age() << ";height=" << height();
    return sstr.str();
  }
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


CYT3MACRO_model_class_serializable_sqlit3_autoinsert_definitions(SimpleClass)

  