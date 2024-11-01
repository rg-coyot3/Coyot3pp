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

      typedef std::function<bool()> ModuleTaskFunction;
      typedef std::vector<ModuleTaskFunction> ModuleTaskFunctionStack;

      ModuleBase(const std::string& name = std::string());

      virtual ~ModuleBase();

              bool Init();
              bool Start();
              bool Pause();
              bool Stop();
              bool End(bool force = false);

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

      ec::CytModuleState state(ec::CytModuleState state);

      bool conf_task_init_(ModuleTaskFunction f,bool prepend = false);
      bool conf_task_start_(ModuleTaskFunction f,bool prepend = false);
      bool conf_task_pause_(ModuleTaskFunction f,bool prepend = false);
      bool conf_task_stop_(ModuleTaskFunction f,bool prepend = false);
      bool conf_task_end_(ModuleTaskFunction f,bool prepend = false);

    private:
      ec::CytModuleState state_;
      std::string        name_;

      std::mutex         p_mtx_mod_transition;

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
        std::string _mod_configuration();

      bool _priv_init();
      bool _priv_start();
      bool _priv_pause();
      bool _priv_stop();
      bool _priv_end(bool force);


      bool tasks_init_();
      bool tasks_start_();
      bool tasks_pause_();
      bool tasks_stop_();
      bool tasks_end_();


      ModuleTaskFunctionStack   tasks_stack_init_;
      ModuleTaskFunctionStack   tasks_stack_start_;
      ModuleTaskFunctionStack   tasks_stack_pause_;
      ModuleTaskFunctionStack   tasks_stack_stop_;
      ModuleTaskFunctionStack   tasks_stack_end_;


      


  };


}
}