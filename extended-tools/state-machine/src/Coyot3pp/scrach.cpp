#include <Coyot3pp/ExtendedTools/StateMachine/StateMachine.hpp>



class MyOwnerClass;


CYT3MACRO_boost_statechart_header(MyObject, StInitial)

CYT3MACRO_boost_statechart_implementation_declarations(MyObject, StInitial)


CYT3MACRO_boost_statechart_machine_declarations(
  MyObject 
  , StInitial
  , MyOwnerClass
  , ( 
        std::string onePropery
      , void        oneMethod()
  )
)

CYT3MACRO_boost_statechart_event_simple_declarations(EvMyEventType, int, "this is a simple description")
CYT3MACRO_boost_statechart_event_simple_definitions(EvMyEventType, int, "this is a simple description")



EvMyEventType::EvMyEventType( const int& parameter , const std::string& r) :reason(r), param(parameter){ EvMyEventType::custom_static_type_ptr("EvMyEventType"); } EvMyEventType::~EvMyEventType(){ if(6 <= coyot3::logger::SimpleLoggerClass::GlobalLogDebugLevel) { ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::DEBUG, coyot3::logger::Terminal::module_terminal_index); slc << "EvMyEventType"" : event destructor"; }; } const char* EvMyEventType::type(){ return custom_dynamic_type_ptr<char>(); }


CYT3MACRO_boost_statechart_event_list_declarations(
    oneEventType, int, "this is one event type example"
  , anotherEventType, bool, "this is another type of event type"
)


CYT3MACRO_boost_statechart_event_list_definitions(
    oneEventType , int , "this is one event type example"
    , anotherEventType, bool, "this is another type of event type"

)

