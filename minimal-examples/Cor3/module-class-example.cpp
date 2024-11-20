#include <Coyot3pp/Cor3/module/ModuleBase.hpp>

typedef coyot3::mod::LogLevel LogLevel;
class SimpleModuleExample : public coyot3::mod::ModuleBase{
  public:
    SimpleModuleExample(const std::string& name) : ModuleBase(name)
    , cth_(nullptr)
    , iterations_(0){
      class_name("simple-module-example");
      add_task_init(std::bind(&SimpleModuleExample::initialization,this));
      add_task_start(std::bind(&SimpleModuleExample::start,this));
      //add_task_pause(std::bind(&SimpleModuleExample::pause,this));
      add_task_stop(std::bind(&SimpleModuleExample::stop,this));
      add_task_end(std::bind(&SimpleModuleExample::endtask,this));
      modlog_verbosity(LogLevel::CLC_INFO);
    }
    ~SimpleModuleExample(){
      End(true);
    }

  protected:
    coyot3::tools::ControlThread* cth_;
    int                           iterations_;
      void mod_task(){
        log_info( o() << "iteration = " << iterations_++);
        if(iterations_> 15)signal_error();
      }

    bool initialization(){
      cth_ = new(std::nothrow) coyot3::tools::ControlThread(std::bind(&SimpleModuleExample::mod_task,this),"thread");
      if(!cth_){
        log_err("error initializing! module will be on error!");
        return false;
      }
      cth_->setInterval(499);
      return true;
    }
    bool start(){
      cth_->start();
      return true;
    }
    // bool pause(){
    //   cth_->stop();
    //   return true;
    // }
    bool stop(){
      cth_->stop();
      return true;
    }
    bool endtask(){
      cth_->stop();
      delete cth_;
      cth_ = nullptr;
      return true;
    }

};

bool on_module_error(coyot3::mod::ModuleBase* ptr){
  CLOG_INFO("received module error: assuming the error has been resolved."
  " module state = " << ptr->state() << "; config=" << ptr->module_configuration())
  return true;
}

int main(int argc, char** argv){
  SimpleModuleExample example("nameOfExample");
  example.on_module_error_callback_set(&on_module_error);
  int state = 0;
  while(state < 20){
    CLOG_INFO("iteration : " << state)
    switch(state){
      case 0:
        CLOG_INFO("initialize : " << example.Init() )
        CLOG_INFO("module state " << example.state())
        break;
      case 3:
        CLOG_INFO("start : " << example.Start())
        CLOG_INFO("module state " << example.state())
        break;
      case 6:
        CLOG_INFO("pausing : " << example.Pause())
        CLOG_INFO("module state " << example.state())
        break;
      case 10:
        CLOG_INFO("starting : " << example.Start())
        CLOG_INFO("module state " << example.state())
        break;
      case 15:
        CLOG_INFO("ending :" << example.End())
        CLOG_INFO("module state " << example.state())
        break;
      case 18:
        CLOG_INFO("stopping :" << example.Stop())
        CLOG_INFO("module state " << example.state())
        break;
    }
    state++;
    
    sleep(1);
  }

  CLOG_INFO("loop exit and ending")
  return 0;
}