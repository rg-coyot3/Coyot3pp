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
  ,_modconf_initializes(false)
  ,_modconf_starts(false)
  ,_modconf_pauses(false)
  ,_modconf_stops(false)
  ,_modconf_ends(false)
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
    
    bool cond01 = ( (_modconf_initializes == false) 
                    && (state_ == ec::CytModuleState::CREATED))
                  || (_modconf_initializes == true);
    bool cond02 = (   (_modconf_initializes == true)
                    && (state_ == ec::CytModuleState::INITIALIZED))
                  || (_modconf_initializes == false);
    bool cond03 = ( (_modconf_pauses == true)
                    && (state_ == ec::CytModuleState::PAUSED) )
                  || (_modconf_pauses == false);

    if(
      (cond01 == false) 
      || ((cond02 == false) && (state_ != ec::CytModuleState::PAUSED)) 
      || ((cond03 == false) && (state_ != ec::CytModuleState::INITIALIZED))
      ){
      log_warn( o() << "state does not permit start. state=[" << state_ << "]."
      " module states" << _mod_configuration() );
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_pause(){
    if((state_ != ec::CytModuleState::STARTED)){
      log_warn(o() << "state does not permit pause. state=[" << state_ << "]."
      " module states" << _mod_configuration() );
      return false;
    }
    return true;
  }
  bool 
  ModuleBase::check_state_for_stop(){
    bool cond01 = (_modconf_pauses == false) || ((_modconf_pauses == true) && (state_ == ec::CytModuleState::PAUSED));
    bool cond02 = (_modconf_starts == false) || ((_modconf_starts == true) && (state_ == ec::CytModuleState::STARTED));

    if((cond01 == false) && (cond02 == false)){
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
    bool cond01 =   (_modconf_stops == false) 
                  || ((_modconf_stops == true) && (state_ == ec::CytModuleState::STOPPED));
    if(cond01 == false){
      log_warn(o() << "state does not permit end. state=[" << state_ << "]."
      " module states" << _mod_configuration() );
      return false;
    }
    return true;
  }

  bool 
  ModuleBase::Init(){
    std::lock_guard<std::mutex> guard(p_mtx_mod_transition);
    return _priv_init();
  }
  bool 
  ModuleBase::Start(){
    std::lock_guard<std::mutex> guard(p_mtx_mod_transition);
    return _priv_start();
  }
  bool 
  ModuleBase::Pause(){
    std::lock_guard<std::mutex> guard(p_mtx_mod_transition);
    return _priv_pause();
  }
  bool 
  ModuleBase::Stop(){
    std::lock_guard<std::mutex> guard(p_mtx_mod_transition);
    return _priv_stop();
  }
  bool
  ModuleBase::End(bool force){
    std::lock_guard<std::mutex> guard(p_mtx_mod_transition);
    return _priv_end(force);
  }


  bool 
  ModuleBase::_priv_init(){
    if((state_ == ec::CytModuleState::INITIALIZED))return true;
    if(_modconf_initializes == false)return true;
    if(check_state_for_init() == false)return false;

    bool r;
    state_ = ec::CytModuleState::INITIALIZING;
    r = tasks_init_();
    if(r == true){
      state_ = ec::CytModuleState::INITIALIZED;
    }else{
      log_warn(o() << "init : error initializing");
      log_warn(o() << "stop : error stopping");
      if(signal_error() == false) return false;
      else state_ = ec::CytModuleState::INITIALIZED;
    }
    return (state_ == ec::CytModuleState::INITIALIZED);
  }
  bool 
  ModuleBase::_priv_start(){
    if((state_ == ec::CytModuleState::STARTED))return true;
    if(_modconf_starts == false)return true;
    if(check_state_for_start() == false)return false;

    bool r;
    state_ = ec::CytModuleState::STARTING;
    r = tasks_start_();
    if(r == true){
      state_ = ec::CytModuleState::STARTED;
    }else{
      log_warn(o() << "start : error starting");
      log_warn(o() << "stop : error stopping");
      if(signal_error() == false) return false;
      else state_ = ec::CytModuleState::STARTED;
    }
    return (state_ == ec::CytModuleState::STARTED);
  }
  bool 
  ModuleBase::_priv_pause(){
    if((state_ == ec::CytModuleState::PAUSED))return true;
    if(_modconf_pauses == false)return true;
    if(check_state_for_pause() == false)return false;
    bool r;
    
    state_ = ec::CytModuleState::PAUSING;
    r = tasks_pause_();
    if(r == true){
      state_ = ec::CytModuleState::PAUSED;
    }else{
      log_warn(o() << "pause : error pausing");
      if(signal_error() == false) return false;
      else state_ = ec::CytModuleState::PAUSED;
    }
    return (state_ == ec::CytModuleState::PAUSED);
  }
  bool 
  ModuleBase::_priv_stop(){
    if((state_ == ec::CytModuleState::STOPPED))return true;
    if(_modconf_stops == false)return true;
    if(check_state_for_stop() == false)return false;
    
    bool r = true;
    state_ = ec::CytModuleState::STOPPING;
    r = tasks_stop_();
    if(r == true){
      state_ = ec::CytModuleState::STOPPED;
    }else{
      log_warn(o() << "stop : error stopping");
      if(signal_error() == false) return false;
      else state_ = ec::CytModuleState::STOPPED;
    }
    return (state_ == ec::CytModuleState::STOPPED);
  }
  bool 
  ModuleBase::_priv_end(bool force){
    if((state_ == ec::CytModuleState::END_OF_LIFE))return true;
    if((check_state_for_end(force) == true) 
        && (_modconf_stops == true) 
        && (state_ != ec::CytModuleState::STOPPED))
    {
      log_info("end : mod stops but is not stopped. Invoking stop now");
      _priv_stop();
    }
    if(_modconf_ends == false)return true;
    if(check_state_for_end(force) == false)return false;

    bool r = true;
    state_ = ec::CytModuleState::ENDING;
    r = tasks_end_();
    if(r == true){
      state_ = ec::CytModuleState::END_OF_LIFE;
    }else{
      log_warn(o() << "end : error ending");
      if(signal_error() == false) return false;
      else state_ = ec::CytModuleState::END_OF_LIFE;
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
  ec::CytModuleState ModuleBase::state() const{
    return state_;
  }



bool 
ModuleBase::tasks_init_(){
  if(tasks_stack_init_.size() == 0)return true;
  bool ret = true;
  for(ModuleTaskFunction& func_ : tasks_stack_init_){
    ret &= func_();
  }
  return ret;
}
bool 
ModuleBase::tasks_start_(){
  if(tasks_stack_start_.size() == 0)return true;
  bool ret = true;
  for(ModuleTaskFunction& func_ : tasks_stack_start_){
    ret &= func_();
  }
  return ret;
}
bool 
ModuleBase::tasks_pause_(){
  if(tasks_stack_pause_.size() == 0)return true;
  bool ret = true;
  for(ModuleTaskFunction& func_ : tasks_stack_pause_){
    ret &= func_();
  }
  return ret;
}
bool 
ModuleBase::tasks_stop_(){
  if(tasks_stack_stop_.size() == 0)return true;
  bool ret = true;
  for(ModuleTaskFunction& func_ : tasks_stack_stop_){
    ret &= func_();
  }
  return ret;
}
bool 
ModuleBase::tasks_end_(){
  if(tasks_stack_end_.size() == 0)return true;
  bool ret = true;
  for(ModuleTaskFunction& func_ : tasks_stack_end_){
    ret &= func_();
  }
  return ret;
}

bool ModuleBase::add_task_init(ModuleTaskFunction f,bool prepend){
  _modconf_initializes = true;
  tasks_stack_init_.push_back(f);
  if((prepend == true) && (tasks_stack_init_.size() != 0)){
    std::rotate(tasks_stack_init_.rbegin(),
                tasks_stack_init_.rbegin()+1,
                tasks_stack_init_.rend());
  }
  return true;
}
bool ModuleBase::add_task_start(ModuleTaskFunction f,bool prepend){
  _modconf_starts = true;
  tasks_stack_start_.push_back(f);
  if((prepend == true) && (tasks_stack_start_.size() != 0)){
    std::rotate(tasks_stack_start_.rbegin(),
                tasks_stack_start_.rbegin()+1,
                tasks_stack_start_.rend());
  }
  return true;
}
bool ModuleBase::add_task_pause(ModuleTaskFunction f,bool prepend){
  _modconf_pauses = true;
  tasks_stack_pause_.push_back(f);
  if((prepend == true) && (tasks_stack_pause_.size() != 0)){
    std::rotate(tasks_stack_pause_.rbegin(),
                tasks_stack_pause_.rbegin()+1,
                tasks_stack_pause_.rend());
  }
  return true;
}
bool ModuleBase::add_task_stop(ModuleTaskFunction f,bool prepend){
  _modconf_stops = true;
  tasks_stack_stop_.push_back(f);
  if((prepend == true) && (tasks_stack_stop_.size() != 0)){
    std::rotate(tasks_stack_stop_.rbegin(),
                tasks_stack_stop_.rbegin()+1,
                tasks_stack_stop_.rend());
  }
  return true;
}
bool ModuleBase::add_task_end(ModuleTaskFunction f,bool prepend){
  _modconf_ends = true;
  tasks_stack_end_.push_back(f);
  if((prepend == true) && (tasks_stack_end_.size() != 0)){
    std::rotate(tasks_stack_end_.rbegin(),
                tasks_stack_end_.rbegin()+1,
                tasks_stack_end_.rend());
  }
  return true;
}

std::string 
ModuleBase::_mod_configuration() const{
  std::string ret;
  ret+="(";
  if(_modconf_initializes){
    ret+="I";
  }else{
    ret+="i";
  }
  if(_modconf_starts){
    ret+="S";
  }else
    ret+="s";
  if(_modconf_pauses){
    ret+="P";
  }else ret+="p";
  if(_modconf_stops){
    ret+="X";
  }else ret+="x";
  if(_modconf_ends){
    ret+="E";
  }else ret+="e";
  ret+=")";
  return ret;
}


bool ModuleBase::signal_error(){
  log_warn("signaled module error");
  ec::CytModuleState currentState = state();
  state(ec::CytModuleState::MODULE_ERROR);
  if(callback_on_module_error_){
    if(callback_on_module_error_(this) == true){
      log_warn("solved module error");
      state(currentState);
      return true;
    }else{
      log_warn("module error not solved");
    }
  }
  return false;
}

void ModuleBase::on_module_error_callback_set(ModuleEventInvoker cb){
  callback_on_module_error_ = cb;
}

ec::CytModuleState ModuleBase::state(ec::CytModuleState state){
  return state_ = state;
}

std::string ModuleBase::module_configuration() const{
  return _mod_configuration();
}

}
}