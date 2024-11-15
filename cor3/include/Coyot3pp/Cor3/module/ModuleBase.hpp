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
      typedef std::function<bool(ModuleBase* )> ModuleEventInvoker;
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


      std::string        module_configuration() const;

      /**
       * @brief set a callback to be invoked when the module enters in error 
       *  state
       * 
       * @param cb 
       */
      void               on_module_error_callback_set(ModuleEventInvoker cb);
      
    protected:

      ec::CytModuleState state(ec::CytModuleState state);

      
      bool add_task_init(ModuleTaskFunction f,bool prepend = false);
      bool add_task_start(ModuleTaskFunction f,bool prepend = false);
      bool add_task_pause(ModuleTaskFunction f,bool prepend = false);
      bool add_task_stop(ModuleTaskFunction f,bool prepend = false);
      bool add_task_end(ModuleTaskFunction f,bool prepend = false);

      bool signal_error();

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
        std::string _mod_configuration() const;

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


      ModuleEventInvoker        callback_on_module_error_;


      


  };


}
}