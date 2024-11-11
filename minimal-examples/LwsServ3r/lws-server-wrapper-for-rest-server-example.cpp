#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>

//just for confort while developping with vscode
#ifndef LC3_EXAMPLE_HTML_CONTENT
  #define LC3_EXAMPLE_HTML_CONTENT ""
#endif

bool DO_LOOP;

bool 
on_dyn_http(coyot3::services::websocket::WsgClient *client, 
            const std::string &request, 
            const coyot3::services::websocket::UriArguments &arguments, 
            std::string &response){

  CLOG_INFO("received request from [" << request << "]");
  response="Hello! at [" + request + "] Arguments received: [";
  for(auto a : arguments){
    response+=a.first;
    response+="=";
    response+=a.second;
    response+=" ";
    CLOG_INFO("received argument : " << a.first << " = " << a.second);
    if(a.first == "end"){
      CLOG_WARN(" - found [end] argument. ending loop")
      DO_LOOP = false;
    }
  }
  response+="]";
  
  return true; // socket will not be on hold
  
}

bool websocket_connects(coyot3::services::websocket::WsgClient *client){
  CLOG_INFO("websocket connects")
  return true;
}
bool websocket_disconnects(coyot3::services::websocket::WsgClient *client){
  CLOG_INFO("websocket disconnects")
  return true;
}

//!IMPORTANT : all message strings sent from the clients MUST end with '\r\n'.
bool websocket_message(coyot3::services::websocket::WsgClient *client, const uint8_t *payload, size_t length){
  std::string message = coyot3::tools::to_string(payload,length);
  CLOG_INFO("received message = " << message);
  return true;
}
int main(int argv, char** argc){

  coyot3::services::websocket::WebsocketsServerGateway server;
  
  server.setName("one name");
  server.setServerPort(5001);
  
  CLOG_INFO("html root at :" << LC3_EXAMPLE_HTML_CONTENT)

  server.addDynamicPath("/restSync");
  server.setOnDynHttpCallback(on_dyn_http);
  server.setOnWSClientConnect(&websocket_connects);
  server.setOnWSClientDisconnects(&websocket_disconnects);
  server.setOnWSClientMessage(&websocket_message);
  server.asyncCommsTimeout(5000);
  server.set_debug_level(10);

    

  
  server.addStaticPath("/",LC3_EXAMPLE_HTML_CONTENT "/","index.html");
  server.setDefaultDoc404(LC3_EXAMPLE_HTML_CONTENT "/404.html");
  server.Init();
  server.Start();
  DO_LOOP = true;
  
  int iteration = 0;
  while(DO_LOOP){
    sleep(1);
    CLOG_INFO(" - pulse");
    // server.broadcastMessage(std::string("this is a message to be broadcasted : "
    //   "nth = ") + std::to_string(iteration++));
  }
  server.Stop();
  CLOG_INFO(" - bye")
}