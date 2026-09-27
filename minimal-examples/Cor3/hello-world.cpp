#include <Coyot3pp/Cor3/Coyot3.hpp>


#include <Coyot3pp/Cor3/ModuleBase.hpp>


    COYOT3PP_MODEL_CLASS_DECLARATIONS(
      Position
      , 
      , ( virtual std::string to_string() const)
      , ( )
        , latitude      , double      , 0.0
        , longitude     , double      , 0.0
        , altitude      , double      , 0.0
    )

    COYOT3PP_MODEL_CLASS_DEFINITIONS(
      Position
      , 
      , ( virtual std::string to_string() const)
      , ( )
        , latitude      , double      , 0.0
        , longitude     , double      , 0.0
        , altitude     , double      , 0.0
    )
    
    
    std::string Position::to_string() const{
      std::stringstream sstr;
      sstr << "lat=" << latitude() << ";lon=" << longitude() 
        << ";alt=" << altitude();
      return sstr.str();
    }

      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
        Position
        ,
        ,
        , ( )
        , ( )
          , latitude      , "latitude"      ,
          , longitude     , "longitude"     ,
          , altitude      , "altitude"      ,
      )

      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
        Position
        ,
        ,
        , ( )
        , ( )
          , latitude      , "latitude"      ,
          , longitude     , "longitude"     ,
          , altitude      , "altitude"      ,
      )

    COYOT3PP_MODEL_CLASS_DECLARATIONS(
      PosOrient
      , Position
      , ( virtual std::string to_string() const)
      , ( )
      , orientation   , double      , 0.0
    )
    
    COYOT3PP_MODEL_CLASS_DEFINITIONS(
      PosOrient
      , Position
      , ( virtual std::string to_string() const)
      , ( )
      , orientation   , double      , 0.0
    )


    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
      PosOrient
      ,Position
      , 
      , ( )
      , ( )
        , orientation   , "orientation"   ,
    )

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
      PosOrient
      ,Position
      , 
      , ( )
      , ( )
        , orientation   , "orientation"   ,
    )



COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(PosOrient,)
COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(PosOrient,)

COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DECLARATIONS(PosOrient)
COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DEFINITIONS(PosOrient)

COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(PosOrient,latitude)
COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(PosOrient,latitude)

COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DECLARATIONS(PosOrient,latitude)
COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DEFINITIONS(PosOrient,latitude)



std::string PosOrient::to_string() const{
      std::stringstream sstr;
      sstr << Position::to_string() << ";hea=" << orientation();
      return sstr.str();
    }
class MyClass : public coyot3::mod::ModuleBase{
  public:
    MyClass() : ModuleBase(){
      log_info("hola mundo");
    }
};

int main(int argc, char** argv){
  CLOG_INFO("hello world")
  
  Position pos(111.11,222.22,333.33);

  pos.latitude(pos.longitude());

  CLOG_INFO(" position : " << pos.to_string())

  PosOrient posor(pos);
  posor.orientation() = 444.44;

  CLOG_INFO(" position orientation " << posor.to_string())

  CLOG_INFO(" equals? pos == posor " << (pos == posor))
  CLOG_INFO(" equals? posor == pos " << (posor == pos))

  Position pos2;

  pos2 = posor;

  CLOG_INFO(" equals? pos2 == pos " << (pos2 == pos))

  PositionJsIO posjs(pos);

  pos2 = posjs;

  CLOG_INFO(" equals? pos2 == pos " << (pos == pos2))
  CLOG_INFO(" serialization : " << (PositionJsIO(pos2).to_json()))
  CLOG_INFO(" serialization : " << (PosOrientJsIO(posor)))

  PosOrientStack posstack;

  posstack.push_back(pos);
  posstack.push_front(posor);

  posstack.for_each([&](PosOrient& item){
    item.latitude()+=555.555;
    return true;
  });
  posstack.for_each([&](const PosOrient& item){
    CLOG_INFO(" - posstack item : " << item.to_string())
    return true;
  });
  PosOrientStack posstack2(posstack);
  posstack2.for_each([&](const PosOrient& item){
    CLOG_INFO(" - posstack item : " << item.to_string())
    return true;
  });

}


