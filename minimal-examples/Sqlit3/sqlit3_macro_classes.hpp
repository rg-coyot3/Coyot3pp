#include <Coyot3pp/Sqlit3/Sqlit3Connector/Sqlit3Connector.hpp>


namespace csq = coyot3::ddbb::sqlite;

COYOT3PP_MODEL_CLASS_DECLARATIONS(
  SimpleClass
  ,
  , ( std::string to_string() const )
  , ( )
    , id      , int64_t         , 
    , name    , std::string     , 
    , surname , std::string     , 
    , age     , int             ,
    , height  , double          ,   
  )

COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(SimpleClass,)


CYT3MACRO_model_class_serializable_sqlit3_declarations(
  SimpleClass
  , ( )
  , id          , "id"          , "INTEGER PRIMARY KEY AUTOINCREMENT"     
  , name        , "name"        , "TEXT"                                  
  , surname     , "surname"     , "TEXT"                                  
  , age         , "age"         , "INTEGER"                               
  , height      , "height"      , "REAL"                                  
)


CYT3MACRO_model_class_serializable_sqlit3_autoinsert_declarations(SimpleClass)

  