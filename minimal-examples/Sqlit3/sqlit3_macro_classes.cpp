#include "sqlit3_macro_classes.hpp"



COYOT3PP_MODEL_CLASS_DEFINITIONS(
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
COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(SimpleClass,)




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

  