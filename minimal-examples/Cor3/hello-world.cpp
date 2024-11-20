#include <Coyot3pp/Cor3/Coyot3.hpp>


#include <Coyot3pp/Cor3/ModuleBase.hpp>


    CYT3MACRO_model_class_declarations(
      Position
      , 
      , ( virtual std::string to_string() const)
      , ( )
        , latitude      , double      , 0.0
        , longitude     , double      , 0.0
        , altitude      , double      , 0.0
    )

    CYT3MACRO_model_class_definitions(
      Position
      , 
      , ( virtual std::string to_string() const)
      , ( )
        , latitude      , double      , 0.0
        , longitude     , double      , 0.0
        , altitude      , double      , 0.0
    )

    std::string Position::to_string() const{
      std::stringstream sstr;
      sstr << "lat=" << latitude() << ";lon=" << longitude() 
        << ";alt=" << altitude();
      return sstr.str();
    }

      CYT3MACRO_model_class_serializable_json_declarations(
        Position
        ,
        ,
        , ( )
        , ( )
          , latitude      , "latitude"      ,
          , longitude     , "longitude"     ,
          , altitude      , "altitude"      ,
      )

      CYT3MACRO_model_class_serializable_json_definitions(
        Position
        ,
        ,
        , ( )
        , ( )
          , latitude      , "latitude"      ,
          , longitude     , "longitude"     ,
          , altitude      , "altitude"      ,
      )

    CYT3MACRO_model_class_declarations(
      PosOrient
      , Position
      , ( virtual std::string to_string() const)
      , ( )
      , orientation   , double      , 0.0
    )
    
    CYT3MACRO_model_class_definitions(
      PosOrient
      , Position
      , ( virtual std::string to_string() const)
      , ( )
      , orientation   , double      , 0.0
    )


    CYT3MACRO_model_class_serializable_json_declarations(
      PosOrient
      ,Position
      , 
      , ( )
      , ( )
        , orientation   , "orientation"   ,
    )

    CYT3MACRO_model_class_serializable_json_definitions(
      PosOrient
      ,Position
      , 
      , ( )
      , ( )
        , orientation   , "orientation"   ,
    )



CYT3MACRO_model_class_set_stack_declarations(PosOrient,)
CYT3MACRO_model_class_set_stack_definitions(PosOrient,)

CYT3MACRO_model_class_set_stack_serializable_json_declarations(PosOrient)
CYT3MACRO_model_class_set_stack_serializable_json_definitions(PosOrient)

CYT3MACRO_model_class_set_mapped_declarations(PosOrient,latitude)
CYT3MACRO_model_class_set_mapped_definitions(PosOrient,latitude)

CYT3MACRO_model_class_set_mapped_serializable_json_declarations(PosOrient,latitude)
CYT3MACRO_model_class_set_mapped_serializable_json_definitions(PosOrient,latitude)



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


