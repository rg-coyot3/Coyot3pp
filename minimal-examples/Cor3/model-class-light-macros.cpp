#include <Coyot3pp/Cor3/Coyot3.hpp>





COYOT3PP_MODEL_CLASS_DECLARATIONS_AND_DEFINITIONS(
  UnaClase
  ,
  , ( )
  , ( )
  , uno       , int         , 1
  , dos       , std::string , "dos"
  , tres      , double      , 
)


COYOT3PP_MODEL_CLASS_DECLARATIONS_AND_DEFINITIONS(
  OtraClase
  ,
  , ( std::function<void()> callback )
  , ( )
  , cuatro      , float       , 
)


// COYOT3PP_MODEL_CLASS_DECLARATIONS_AND_DEFINITIONS(
//   MultipleHerencia
//   , (UnaClase, OtraClase)
//   , ( )
//   , ( )
//   , cinco , std::vector<int> , 
// )



COYOT3PP_ENUM_CLASS_DECLARATIONS_AND_DEFINITIONS(
  MyEnumClass
  , 
  , uno , 
  , dos , 2
)


// COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(MultipleHerencia,)


// COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(MultipleHerencia,)



int main(int argv, char** argc){

  // MultipleHerencia obj;
  // obj.uno() = 4;
  // obj.cuatro() = 2.3;
  // obj.cinco().push_back(5);

  // MultipleHerencia obj2(obj);
  // std::cout << obj2.model_structure();
  return 0;
}