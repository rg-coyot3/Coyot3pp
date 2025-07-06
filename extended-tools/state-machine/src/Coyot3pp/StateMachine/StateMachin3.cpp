#include <Coyot3pp/StateMachin3/StateMachin3.hpp>


namespace coyot3::tools::statemachine{
  CYT3MACRO_model_class_definitions(
    StateMachineLogLine
    , 
    , ( 
        StateMachineLogLine(const std::string& d, int l = 3)

        , static constexpr  const int ERROR   = 0
        , static constexpr  const int WARN    = 1
        , static constexpr  const int TRANSITION = 2
        , static constexpr  const int INFO    = 3
        , static constexpr  const int DEBUG   = 4
    )
    , ( )
      , timestamp         , int64_t           , 
      , level             , int               , 2
      , description       , std::string       , 
  )

  
    CYT3MACRO_model_class_set_stack_definitions(StateMachineLogLine, 2000)

    CYT3MACRO_boost_statechart_event_simple_definitions(EvStateMachineTransitionConfirmation ,   , generic state machine confirmation event)
   
    
    StateMachineLogLine::StateMachineLogLine(const std::string& d, int l){
      timestamp(coyot3::tools::getCurrentTimestamp());
      description(d);
      level(l);
    }





}