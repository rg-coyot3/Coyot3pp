#include <Coyot3pp/R3st/Cli3nt/Client.h>


namespace rest = coyot3::communication::rest;





int main(int argc, char** argv){


  CLOG_INFO("creating simple http request")
  std::string res;
  bool        opres = rest::Client::SimpleUrlRequest(
                                "http://localhost:8080/get?arg1=1&arg2=dos",
                                res);
  CLOG_INFO(" simpleurlrequest : result was (" << (opres ? "true" : "false") 
  << ") : ")
  CLOG_INFO(res);
              
              
              opres = rest::Client::SimplePost(
                                "http://localhost:8080/post", 
                                std::string("uno:1"), 
                                res);
  CLOG_INFO(" simple-post-request : result was (" << (opres ? "true" : "false") 
  << ") : ")
  CLOG_INFO(res);

  Json::Value params;
  params["uno"] = 1;
  params["dos"] = "dos";
              opres = rest::Client::SimplePost(
                                "http://localhost:8080/post", 
                                params, 
                                res);
  CLOG_INFO(" simple-post-request : result was (" << (opres ? "true" : "false") 
  << ") : ")
  CLOG_INFO(res);
  return EXIT_SUCCESS;
}