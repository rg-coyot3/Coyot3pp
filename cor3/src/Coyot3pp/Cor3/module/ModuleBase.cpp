#include <Coyot3pp/Cor3/module/ModuleBase.hpp>


namespace coyot3{
namespace mod{


  CYT3MACRO_enum_class_definitions(
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



  ModuleBase::ModuleBase(const std::string& name)
  :LoggerCapability(name)
  ,state_(ec::CytModuleState::MODULE_ERROR)
  ,name_(name)
  ,_modconf_initializes(true)
  ,_modconf_starts(true)
  ,_modconf_pauses(true)
  ,_modconf_stops(true)
  ,_modconf_ends(true)
  {
    log_debug(3,"here i am");
    state_ = ec::CytModuleState::CREATED;
  }

  ModuleBase::~ModuleBase(){
    if(_modconf_ends ==true && state_ != ec::CytModuleState::END_OF_LIFE){
      End(true);
    }
  }


  bool 
  ModuleBase::check_state_for_init(){
    if((state_ != ec::CytModuleState::CREATED) ){
      log_warn(o() << "state does not permit init. state=[" << state_ << "]."
      " needs : " << ec::CytModuleState::CREATED);
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_start(){
    if((state_ != ec::CytModuleState::INITIALIZED)
    && (state_ != ec::CytModuleState::PAUSED)){
      log_warn( o() << "state does not permit start. state=[" << state_ << "]."
      " needs : " << ec::CytModuleState::INITIALIZED << " or "
      << ec::CytModuleState::PAUSED);
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_pause(){
    if((state_ != ec::CytModuleState::STARTED)){
      log_warn(o() << "state does not permit pause. state=[" << state_ << "]."
      " needs : " << ec::CytModuleState::STARTED);
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_stop(){
    if((state_ != ec::CytModuleState::STARTED)
    &&((state_ != ec::CytModuleState::PAUSED))){
      log_warn(o() << "state does not permit stop. state=[" << state_ << "]."
      " needs : " << ec::CytModuleState::STARTED << " or "
      << ec::CytModuleState::PAUSED);
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_end(bool force){
    if(force == true)return true;
    if((state_ != ec::CytModuleState::STOPPED)){
      log_warn(o() << "state does not permit end. state=[" << state_ << "]."
      " needs : " << ec::CytModuleState::STOPPED);
      return false;
    }
    return true;
  }

  bool 
  ModuleBase::Init(){    
    if(check_state_for_init() == false)return false;
    bool r = true;
    state_ = ec::CytModuleState::INITIALIZING;
    if(_modconf_initializes == true){
      r = task_init();
    }
    if(r == true){
      state_ = ec::CytModuleState::INITIALIZED;
    }else{
      log_warn(o() << "init : error initializing");
      state_ = ec::CytModuleState::MODULE_ERROR;
    }
    return (state_ == ec::CytModuleState::INITIALIZED);
  }
  bool 
  ModuleBase::Start(){
    if(check_state_for_start() == false)return false;
    bool r = true;
    state_ = ec::CytModuleState::STARTING;
    if(_modconf_initializes == true){
      r = task_start();
    }
    if(r == true){
      state_ = ec::CytModuleState::STARTED;
    }else{
      log_warn(o() << "start : error starting");
      state_ = ec::CytModuleState::MODULE_ERROR;
    }
    return (state_ == ec::CytModuleState::STARTED);
  }
  bool 
  ModuleBase::Pause(){
    if(check_state_for_pause() == false)return false;
    bool r = true;
    state_ = ec::CytModuleState::PAUSING;
    if(_modconf_initializes == true){
      r = task_pause();
    }
    if(r == true){
      state_ = ec::CytModuleState::PAUSED;
    }else{
      log_warn(o() << "pause : error pausing");
      state_ = ec::CytModuleState::MODULE_ERROR;
    }
    return (state_ == ec::CytModuleState::PAUSED);
  }
  bool 
  ModuleBase::Stop(){
    if(check_state_for_stop() == false)return false;
    bool r = true;
    state_ = ec::CytModuleState::STOPPING;
    if(_modconf_initializes == true){
      r = task_stop();
    }
    if(r == true){
      state_ = ec::CytModuleState::STOPPED;
    }else{
      log_warn(o() << "stop : error stopping");
      state_ = ec::CytModuleState::MODULE_ERROR;
    }
    return (state_ == ec::CytModuleState::STOPPED);
  }

  bool
  ModuleBase::End(bool force){
    if(check_state_for_end(force) == false)return false;
    bool r = true;
    state_ = ec::CytModuleState::ENDING;
    if(_modconf_initializes == true){
      r = task_end(force);
    }
    if(r == true){
      state_ = ec::CytModuleState::END_OF_LIFE;
    }else{
      log_warn(o() << "end : error ending");
      state_ = ec::CytModuleState::MODULE_ERROR;
    }
    return (state_ == ec::CytModuleState::END_OF_LIFE);
  }



bool 
ModuleBase::created(){
  return state_ == ec::CytModuleState::CREATED;
}
bool 
ModuleBase::initialized(){
  return state_ == ec::CytModuleState::INITIALIZED;
}
bool 
ModuleBase::started(){
  return state_ == ec::CytModuleState::STARTED;
}
bool 
ModuleBase::paused(){
  return state_ == ec::CytModuleState::PAUSED;
}
bool 
ModuleBase::stopped(){
  return state_ == ec::CytModuleState::STOPPED;
}
bool 
ModuleBase::ended(){
  return state_ == ec::CytModuleState::END_OF_LIFE;
}
bool 
ModuleBase::broken(){
  return ((state_ == ec::CytModuleState::MODULE_ERROR) 
        ||(state_ == ec::CytModuleState::INTERNAL_ERROR)
        ||(state_ == ec::CytModuleState::UNKNOWN_OR_UNSET));  
}

std::string ModuleBase::name() const{return name_;}
std::string ModuleBase::name(const std::string& nn){
  name_ = nn;return instance_name(nn);
}

void 
ModuleBase::module_states_configuration(
        bool module_initializes,
        bool module_starts,
        bool module_pauses,
        bool module_stops,
        bool module_ends
){
  _modconf_initializes = module_initializes;
  _modconf_starts = module_starts;
  _modconf_pauses = module_pauses;
  _modconf_stops = module_stops;
  _modconf_ends = module_ends;
}


bool 
ModuleBase::task_init(){
  if(_modconf_initializes == true){
    log_err(o() << "module configures with initializer, but "
    "task-init is not defined!");
    return false;
  }
  return true;
}
bool 
ModuleBase::task_start(){
  if(_modconf_starts == true){
    log_err(o() << "module configures with starter, but "
    "task-start is not defined!");
    return false;
  }
  return true;
}
bool 
ModuleBase::task_pause(){
  if(_modconf_pauses == true){
    log_err(o() << "module configures with pauser, but "
    "task-pause is not defined!");
    return false;
  }
  return true;
}
bool 
ModuleBase::task_stop(){
  if(_modconf_stops == true){
    log_err(o() << "module configures with stopper, but "
    "task-stop is not defined!");
    return false;
  }
  return true;
}
bool 
ModuleBase::task_end(bool force){
  if(_modconf_ends == true){
    log_err(o() << "module configures with ender, but "
    "task-end is not defined!");
    return false;
  }
  return true;
}


}
}