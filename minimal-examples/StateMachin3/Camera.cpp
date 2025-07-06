#include "Camera.hpp"

//definition of events
CYT3MACRO_boost_statechart_event_list_definitions(
  EvConfig          ,       , start configuration
, EvShutterReleased ,       , shuttle released
, EvShutterHalf     ,       , shutter half
) 


//definitions of states
CYT3MACRO_boost_statechart_machine_definitions(
Camera
, NotShooting
, 
, ( )
, ( )
)



CYT3MACRO_boost_statechart_state_definitions(
NotShooting
, CameraStateMachine
, CameraIddle
, ( )
, ( )
  , EvShutterHalf         , Shooting
)


CYT3MACRO_boost_statechart_state_definitions(
CameraIddle
, NotShooting
, 
, ( )
, ( )
  , EvConfig      , Configuring
)

CYT3MACRO_boost_statechart_state_definitions(
Configuring
, NotShooting
, 
, ( )
, ( )
  , EvConfig      , CameraIddle
)

//definitions of the state machine
CYT3MACRO_boost_statechart_machine_definitions(
  Camera
  ,NotShooting
  ,
  , ( 
    std::string state() const,
    std::string state(const std::string& s)
  )
  , ( )
)



CYT3MACRO_model_class_definitions(
  CameraModel
  , 
  , ( )
  , ( )
  , state     , std::string , 
  )
//end of CYT3MACROS



//custom definitions of the methods for the states and state machine
std::string CameraStateMachine::state() const{
  return owner()->model.state();
}
std::string CameraStateMachine::state(const std::string& s){
  return owner()->model.state(s);
}


Camera::Camera()
:model(){

}
Camera::~Camera(){

}