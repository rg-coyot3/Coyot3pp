#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>


bool DO_LOOP;

bool 
on_dyn_http(coyot3::services::websocket::WsgClient *client, 
            const std::string &request, 
            const coyot3::services::websocket::UriArguments &arguments, 
            std::string &response){

  CLOG_INFO("received request from [" << request << "]");
  for(auto a : arguments){
    CLOG_INFO("received argument : " << a.first << " = " << a.second);
    if(a.first == "end"){
      CLOG_WARN(" - found [end] argument. ending loop")
      DO_LOOP = false;
    }
  }
  
}


int main(int argv, char** argc){

  coyot3::services::websocket::WebsocketsServerGateway server;
  
  server.setName("one name");
  server.setServerPort(5001);
  server.addDynamicPath("/");
  server.setOnDynHttpCallback(on_dyn_http);
  server.asyncCommsTimeout(5000);
  server.setDefaultDoc404("/tmp/404.html");
  server.Init();
  server.Start();
  DO_LOOP = true;
  while(DO_LOOP){
    sleep(1);
    CLOG_INFO(" - pulse");
  }
  server.Stop();
  CLOG_INFO(" - bye")
}