#pragma once

#include <Coyot3pp/StateMachin3/StateMachin3.hpp>



struct Shooting;
struct CameraIddle;
struct Configuring;
class  Camera;

CYT3MACRO_boost_statechart_event_list_declarations(
    EvConfig          ,       , start configuration
  , EvShutterReleased ,       , shuttle released
  , EvShutterHalf     ,       , shutter half
) 

CYT3MACRO_boost_statechart_header(Camera, NotShooting)
CYT3MACRO_boost_statechart_implementation_declarations(Camera, NotShooting)

CYT3MACRO_boost_statechart_machine_declarations(
  Camera
  , NotShooting
  , 
  , ( )
  , ( )
)


CYT3MACRO_boost_statechart_state_declarations(
  NotShooting
  , CameraStateMachine
  , CameraIddle
  , ( )
  , ( )
    , EvShutterHalf         , Shooting
)



CYT3MACRO_boost_statechart_state_declarations(
  Configuring
  , NotShooting
  , 
  , ( )
  , ( )
    , EvConfig      , CameraIddle
)

CYT3MACRO_boost_statechart_state_declarations(
  CameraIddle
  , NotShooting
  , 
  , ( )
  , ( )
    , EvConfig      , Configuring
)

CYT3MACRO_model_class_declarations(
  CameraModel
  , 
  , ( )
  , ( )
  , state     , std::string , 
)


CYT3MACRO_boost_statechart_machine_declarations(
  Camera
  ,NotShooting
  ,
  , ( 
    std::string state() const,
    std::string state(const std::string& s)
  )
  , ( )
)


class Camera {
  public:
    Camera();
    virtual ~Camera();

    CameraModel model;

    

  protected:
    CYT3MACRO_boost_statechart_owner_class_declarations(Camera,csm,)
};


