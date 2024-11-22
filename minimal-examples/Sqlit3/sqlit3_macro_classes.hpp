#include <Coyot3pp/Sqlit3/Sqlit3Connector/Sqlit3Connector.hpp>


namespace csq = coyot3::ddbb::sqlite;

CYT3MACRO_model_class_declarations(
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

CYT3MACRO_model_class_set_stack_declarations(SimpleClass,)


CYT3MACRO_model_class_serializable_sqlit3_declarations(
  SimpleClass
  , ( )
  , id          , "INTEGER PRIMARY KEY AUTOINCREMENT"     , "id"
  , name        , "TEXT"                                  , "name"
  , surname     , "TEXT"                                  , "surname"
  , age         , "INTEGER"                               , "age"
  , height      , "REAL"                                  , "height"
)


CYT3MACRO_model_class_serializable_sqlit3_autoinsert_declarations(SimpleClass)



