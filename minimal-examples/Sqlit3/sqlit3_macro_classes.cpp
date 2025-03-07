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
  , id          , "id"        , "INTEGER PRIMARY KEY AUTOINCREMENT"     
  , name        , "name"      , "TEXT"                                  
  , surname     , "surname"   , "TEXT"                                  
  , age         , "age"       , "INTEGER"                               
  , height      , "height"    , "REAL"                                  
)



CYT3MACRO_model_class_serializable_sqlit3_autoinsert_definitions(SimpleClass)

  