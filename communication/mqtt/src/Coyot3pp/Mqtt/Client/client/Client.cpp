#include <Coyot3pp/Mqtt/Client/Client.hpp>



namespace coyot3{
namespace communication{
namespace mqtt{

  Client::Client(const std::string& name)
  :ModuleBase(name)
  ,config_()

  ,th_main_(nullptr)
  ,th_sec_(nullptr)
  
  {
    class_name("MqttClient");

    add_task_init(std::bind(&Client::init_,this));
    add_task_start(std::bind(&Client::start_,this));
    add_task_pause(std::bind(&Client::pause_,this));
    add_task_stop(std::bind(&Client::stop_,this));
    add_task_end(std::bind(&Client::end_,this));

  }


  Client::~Client(){
    log_warn("destroying instance. forcing end.");
    End(true);
  }


}
}
}