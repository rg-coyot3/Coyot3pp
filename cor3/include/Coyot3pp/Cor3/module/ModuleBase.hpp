#pragma once


#include "../Coyot3.hpp"
#include "LoggerCapability.hpp"



namespace coyot3{
namespace mod{





  CYT3MACRO_enum_class_declarations(
    CytModuleState
    ,
      , CREATED 
      , INITIALIZING
      , INITIALIZED
      , STARTING
      , STARTED 
      , PAUSING
      , PAUSED 
      , STOPPING
      , STOPPED
      , ENDING
      , END_OF_LIFE
      , MAINTENANCE
      , MODULE_ERROR
  )

  
  
  class ModuleBase
  : public LoggerCapability{
    public:
      
      ModuleBase(
        const std::string& name = std::string("no-name"));
      virtual ~ModuleBase();

      virtual bool Init()                 ;
      virtual bool Start()                ;
      virtual bool Pause()                ;
      virtual bool Stop()                 ;
      virtual bool End(bool force = false);

              bool created();
              bool initialized();
              bool started();
              bool paused();
              bool stopped();
              bool ended();
              bool broken();

      ec::CytModuleState state() const;

      std::string        name() const;
      std::string        name(const std::string& nn);
    protected:

      void module_states_configuration(
        bool module_initializes,
        bool module_starts,
        bool module_pauses,
        bool module_stops,
        bool module_ends
      );
      
      ec::CytModuleState state(ec::CytModuleState state);

      
      virtual bool task_init();
      virtual bool task_start();
      virtual bool task_pause();
      virtual bool task_stop();
      virtual bool task_end(bool force = false);


      

    private:
      ec::CytModuleState state_;
      std::string        name_;

      bool check_state_for_init();
      bool check_state_for_start();
      bool check_state_for_pause();
      bool check_state_for_stop();
      bool check_state_for_end(bool force = true);

      bool _modconf_initializes;
      bool _modconf_starts;
      bool _modconf_pauses;
      bool _modconf_stops;
      bool _modconf_ends;
  };


}
}