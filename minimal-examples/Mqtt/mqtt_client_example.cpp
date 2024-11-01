#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>


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

  coyot3::communication::mqtt::MqttGateway::OnMessageCallback c;
  coyot3::communication::mqtt::MqttGatewayConfigObject config;

  config.host_address("localhost");
  config.host_port(5000);
  config.qos(0);

  coyot3::communication::mqtt::MqttGateway client;

  client.set_configuration(config);


  client.subscribe_to_topic("hello/subscription", on_message);

  client.Init();
  client.Start();

  while(DoLoop){
    sleep(1);
    client.publish("hello/publisher",std::string("this is a message"));
  }

  client.Stop(); // invoked in any case when the instance is destroyed.
  CLOG_INFO("ending example.")

}