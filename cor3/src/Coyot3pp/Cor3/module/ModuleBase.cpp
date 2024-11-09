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
      state_ = ec::CytModuleState::MODULE_ERROR;
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
      state_ = ec::CytModuleState::MODULE_ERROR;
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
      state_ = ec::CytModuleState::MODULE_ERROR;
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
      state_ = ec::CytModuleState::MODULE_ERROR;
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
      Stop();
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

bool ModuleBase::conf_task_init_(ModuleTaskFunction f,bool prepend){
  _modconf_initializes = true;
  if((prepend == true) && (tasks_stack_init_.size() != 0)){
    tasks_stack_init_.insert(tasks_stack_init_.begin(),f);
  }else{
    tasks_stack_init_.push_back(f);
  }
  return true;
}
bool ModuleBase::conf_task_start_(ModuleTaskFunction f,bool prepend){
  _modconf_starts = true;
  
  if((prepend == true) && (tasks_stack_start_.size() != 0)){
    tasks_stack_start_.insert(tasks_stack_init_.begin(),f);
  }else{
    tasks_stack_start_.push_back(f);
  }
  return true;
}
bool ModuleBase::conf_task_pause_(ModuleTaskFunction f,bool prepend){
  _modconf_pauses = true;
  if((prepend == true) && (tasks_stack_pause_.size() != 0)){
    tasks_stack_pause_.insert(tasks_stack_init_.begin(),f);
  }else{
    tasks_stack_pause_.push_back(f);
  }
  return true;
}
bool ModuleBase::conf_task_stop_(ModuleTaskFunction f,bool prepend){
  _modconf_stops = true;
  if((prepend == true) && (tasks_stack_stop_.size() != 0)){
    tasks_stack_stop_.insert(tasks_stack_init_.begin(),f);
  }else{
    tasks_stack_stop_.push_back(f);
  }
  return true;
}
bool ModuleBase::conf_task_end_(ModuleTaskFunction f,bool prepend){
  _modconf_ends = true;
  if((prepend == true) && (tasks_stack_end_.size() != 0)){
    tasks_stack_end_.insert(tasks_stack_init_.begin(),f);
  }else{
    tasks_stack_end_.push_back(f);
  }
  return true;
}

std::string 
ModuleBase::_mod_configuration(){
  std::string ret;
  ret+="(";
  if(_modconf_initializes){
    ret+="I";
  }
  if(_modconf_starts){
    ret+="S";
  }
  if(_modconf_pauses){
    ret+="P";
  }
  if(_modconf_stops){
    ret+="X";
  }
  if(_modconf_ends){
    ret+="E";
  }
  ret+=")";
  return ret;
}




}
}