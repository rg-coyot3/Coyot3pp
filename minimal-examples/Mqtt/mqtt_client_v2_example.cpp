#include <Coyot3pp/Mqtt/Client/Client.hpp>



bool DoLoop = true;
bool on_message(const std::string &topic, const uint8_t *p, size_t s){
  std::string m(coyot3::tools::to_string(p,s));
  CLOG_INFO("message received at topic [" << topic << "] :message::" << m)
  if(m == "end"){
    CLOG_INFO("dropping loop flag")
    DoLoop = false;
  }
  return true;
}


int main(int argv, char** argc){
  
  coyot3::communication::mqtt::ClientConfiguration conf;
  coyot3::communication::mqtt::Client c("cliente");
  c.modlog_verbosity(9);

  conf.host_address("localhost");
  conf.host_port(5000);
  conf.qos(0);

  c.config(conf);

  int64_t regid;
  c.register_subscription("hello/subscription",0,on_message,regid);

  CLOG_INFO("initiating")
  c.Init();
  CLOG_INFO("starting")
  c.Start();

  while(DoLoop){
    sleep(1);
    c.publish("hello/publisher","this is a message");
    CLOG_INFO(" published.")
  }
  c.Stop();
  CLOG_INFO("end of example")

}