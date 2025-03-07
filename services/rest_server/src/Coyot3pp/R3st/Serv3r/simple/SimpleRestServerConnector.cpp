#include <Coyot3pp/R3st/Serv3r/simple_rest_server/SimpleRestServerConnector.hpp>




namespace coyot3::communication::rest{


  SimpleRestApiServer::SimpleRestApiServer(const std::string& ownerName)
  : owner(ownerName)
  , config()
  , appPtr(nullptr)
  , raw_callbacks()
  , json_callbacks()
  , traces()
  , app_th_(nullptr)
  {
    CLOG_INFO("simple-rest-api-server : constructor")
  }
  SimpleRestApiServer::~SimpleRestApiServer(){
    CLOG_WARN("simple-rest-api-server : destructor")
    Stop();End();
  }


  bool 
  SimpleRestApiServer::register_post_route(const std::string& route, 
                                           RestPostApiCallback cb)
  {
    CLOG_INFO("simple-rest-server : add-raw-post-route : " << route)
    RestPostApiCallbackInfo cbi;
    cbi.method(route);
    cbi.callback = cb;
    return raw_callbacks.update(cbi,true);
  }
  bool 
  SimpleRestApiServer::register_post_route(const std::string& route, 
                                           RestJsonPostApiCallback cb)
  {
    RestPostJsonApiCallbackInfo cbi;
    CLOG_INFO("simple-rest-server : add-json-post-route : " << route)
    cbi.method(route);
    cbi.callback = cb;
    RestPostApiCallbackInfo cbraw;
    cbraw.method(route);
    cbraw.callback = std::bind(&SimpleRestApiServer::crow_callback_post_request_json_,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2,
                                std::placeholders::_3);
    return json_callbacks.update(cbi,true);
  }
  bool SimpleRestApiServer::set_config(const RestServerConnectorConfig& conf){
    config = conf;
    return true;
  }
  bool SimpleRestApiServer::Init(){
    CLOG_INFO("simple-rest-api-server : init : nothing to do")
    return true;
  }
  bool SimpleRestApiServer::Start(){
    CLOG_INFO("simple-rest-api-server : init : ")
    if(appPtr != nullptr){
      CLOG_WARN("simple-rest-api-server : start : the server seems to have been"
      " already started.")
      return false;
    }
    appPtr = new crow::SimpleApp();
    crow::SimpleApp& app = *appPtr;
    
    CROW_CATCHALL_ROUTE(app)
    ([&](const crow::request& req){

      switch(req.method){
        case crow::HTTPMethod::POST:
          return crow_callback_post_request_raw_(req);
          break;
      }
      std::stringstream sstr;
      sstr << "{ \"error\" : 400 , \"message\" : \"The current requested "
      "method [" << crow::method_strings[static_cast<int>(req.method)] << "] "
      "is not yet implemented in this crow's wrapper\"}";
      
      crow::response res;
      res.code = 400;
      res.body = sstr.str();
      return res;
    });
    app_th_ = new(std::nothrow) std::thread([&](){
      app.port(config.port()).multithreaded().run();
    });
    return true;
  }

  bool SimpleRestApiServer::Stop(){
    CLOG_INFO("simple-server-rest-api : stop : " << owner << " : ")
    appPtr->stop();
    app_th_->join();
    delete app_th_;
    delete appPtr;
    app_th_ = nullptr;
    appPtr = nullptr;

    return true;
  }
  bool SimpleRestApiServer::End(){
    return true;
  }


  crow::response 
  SimpleRestApiServer::crow_callback_post_request_raw_(const crow::request& req){
    crow::response res;
    std::stringstream sstr;
    if(raw_callbacks.is_member(req.url) == false){
      res.code = 400;
      sstr << "{ \"error\" : 400 , \"message\" : \"The current requested "
      "method [" << crow::method_strings[static_cast<int>(req.method)] << "("
      << req.url << ")] is not yet implemented in this crow's wrapper\"}";
      res.body = sstr.str();
      trace_activity(req.remote_ip_address,req.url,res.code,req.body,res.body);
      return res;
    }
    CLOG_DEBUG(5,"simple-server-rest-api : post-raw : [" << req.url << "]")
    res.body = raw_callbacks.get(req.url).callback(req.url,req.body,res.body);
    trace_activity(req.remote_ip_address,req.url,res.code,req.body,res.body);
    return res;
  }


  int 
  SimpleRestApiServer::crow_callback_post_request_json_(
    const std::string& method, 
    const std::string& req, 
    std::string& res
  ){
    std::stringstream sstr;
    if(json_callbacks.is_member(method) == false){
      sstr << "{ \"error\" : 400 , \"message\" : \"The current requested "
      "method [" << method << "("
      << method << ")] is not yet implemented in this crow's wrapper\"}";
      res = sstr.str();
      return 400;
    }
    Json::Value v;
    if(coyot3::tools::JsonParse(v,req) == false){
      sstr << "{ \"error\" : 400 , \"message\" : \"The current requested "
      "method [" << method << "("
      << method << ")] input parameter errors. impossible to parse input data [" 
      << req << "]\"}";
      res = sstr.str();
      return 400;
    }
    Json::Value resjson;
    int r = json_callbacks.get(method).callback(method,v,resjson);
    sstr << resjson;
    res = sstr.str();
    return r;
  }

  void 
  SimpleRestApiServer::trace_activity(const std::string& cli,
    const std::string& met, 
    int res, 
    const std::string& i, 
    const std::string& o
  ){
    ApiMethodTrace trace;
    trace.method(met);
    trace.ts(coyot3::tools::getCurrentTimestamp());
    trace.req(i);
    trace.res(o);
    trace.ret_code(res);
    if(traces.is_member(cli) == false){
      ApiMethodClientTrace t;
      t.client(cli);
      traces.insert(t);
    }
    traces.get(cli).update_stat(trace);
  }
}