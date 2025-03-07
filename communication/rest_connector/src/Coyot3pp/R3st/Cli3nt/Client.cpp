#include <Coyot3pp/R3st/Cli3nt/Client.h>



namespace coyot3{
namespace communication{
namespace rest{

  Client::Client(RestType type, const std::string& url)
  : params_()
  , content_type_()
  , timer_(nullptr)
  , timer_interval_(2000)
  , url_(url)
  , type_(type)
  {
    if(url_.size() == 0){type_ = RestType::UNSET;}
  }
  Client::~Client(){
    async_stop();
  }

  bool Client::set_target(const std::string& url){
    return ((url_ = url).size() != 0);
  }
  bool Client::param_del(const std::string& name){
    ParamsMap::iterator it;
    if((it = params_.find(name)) == params_.end()){
      return false;
    }
    params_.erase(it);
    return true;
  }

  bool Client::set_content_type(const std::string& name){
    content_type_ = name;
    return true;
  }

  bool Client::set_payload(const std::string& payload){
    payload_ = payload;
    set_content_type(MimeType::TEXT);
    return (payload_.size() != 0);
  }
  bool Client::set_payload(const Json::Value& payload){
    std::stringstream sstr;
    sstr << payload;
    set_content_type(MimeType::JSON);
    return (payload_.size() != 0);
  }

  bool Client::set_callback(Client::CallbackFunc callback){
    callback_ = callback;
    return true;
  }


  bool Client::async_start(int interval){
    if(timer_ != nullptr){
      CLOG_WARN("rest-client : start : client already launched")
      return false;
    }
    if(!callback_){
      CLOG_WARN("rest-client : start : the callback is not set!")
      return false;
    }
    if(interval <= 0){
      CLOG_WARN("rest-client : start : setting default rest timer to 2 seconds")
      timer_interval_ = 2000;
    }
    timer_ = new coyot3::tools::WorkerThread(
                std::bind(&Client::request_task_,this),
                "rest-client");
    timer_->setInterval(timer_interval_);
    timer_->start();
    return true;
  }

  bool Client::async_stop(){
    if(timer_ == nullptr){
      CLOG_WARN("rest-client : stop : client not started")
      return false;
    }
    CLOG_DEBUG(5,"rest-client : stop : stopping client");
    timer_->stop();
    delete timer_;
    timer_ = nullptr;
    return true;
  }


  void Client::request_task_(){

  }

  void Client::request_task_common_(bool& ok, int& code, std::string& result){
    curlpp::Easy                 request;
    std::stringstream            sstr;
    curlpp::options::WriteStream ws(&sstr);
    curlpp::options::Verbose     verbose(false);
    
    std::list<std::string>       h(headers_);
    try{
      request.setOpt(ws);
      request.setOpt(verbose);

      std::string url = url_;


      switch(type_){
        case RestType::GET:
          {
            bool firstparam=true;
            for(const std::pair<std::string,coyot3::tools::Value>& item : params_){
              if(firstparam==true){
                url += "?";
              }else{
                url += "&";
              }
              url+=item.first;
              url+=item.second.asString();
            }
            curlpp::options::Url u(url);
            request.setOpt(u);
          }
          
          break;
        case RestType::POST:
          {
            h.push_back(std::string("Content-Type: ") + content_type_);
            curlpp::options::PostFields       fields(this->payload_);
            curlpp::options::PostFieldSize    fieldsize(this->payload_.size());
            curlpp::options::Url u(url);
            request.setOpt(fields);
            request.setOpt(fieldsize);
            request.setOpt(u);
            
          }
        break;
        case RestType::FORM:
          {
            curlpp::Forms form;
            for(const std::pair<std::string,coyot3::tools::Value>& p : params_){
              form.push_back(new curlpp::FormParts::Content(p.first,p.second.asString()));
            }
            curlpp::options::HttpPost post(form);
            request.setOpt(post);
            curlpp::options::Url u(url);
            request.setOpt(u);
            
          }

      }
      request.perform();
      result = sstr.str();
      code = curlpp::infos::ResponseCode::get(request);
      ok = true;
    }catch(curlpp::RuntimeError& e){
      CLOG_WARN("rest-client : task : runtime-err : (" << e.what() << ")")
      ok = false;
    }catch(curlpp::LogicError& e){
      CLOG_WARN("rest-client : task : runtime-err : (" << e.what() << ")")
      ok = false;
    }

    
  }








  //statics ---------------------------

  /**
   * @brief 
   * 
   * @param url 
   * @param result 
   * @return true 
   * @return false 
   */
  bool 
  Client::SimpleUrlRequest(
                  const std::string& url, 
                  std::string& result){
    bool res = true;            
    curlpp::options::Url          u(url);
    curlpp::Easy                  request;
    std::stringstream             ss;
    curlpp::options::WriteStream  ws(&ss);
    try{
      CLOG_DEBUG(7, "coyot3-http-request : simple-url-request : creating "
      "request to (" << url<< ")")
      request.setOpt(u);
      request.setOpt(ws);
      request.perform();
      result = ss.str();
    }catch(curlpp::RuntimeError& e){
      CLOG_WARN("coyot3-http-request : simple-url-request : exception raised "
      "while making request : runtime-err(" << e.what() << ")")
      res = false;
    }catch(curlpp::LogicError& e){
      CLOG_WARN("coyot3-http-request : simple-url-request : exception raised "
      "while making request : logic-err(" << e.what() << ")")
      res = false;
    }
    
    return res;

  }

  bool
  Client::SimplePost(
    const std::string& url,
    const std::string& payload,
          std::string& result
  ){
    curlpp::Easy request;
    bool res = true;
    std::list<std::string> headers;
    std::stringstream             ss;

    
    try{
      curlpp::Forms formParts;
      formParts.push_back(new curlpp::FormParts::Content("name1","value1"));
      formParts.push_back(new curlpp::FormParts::Content("name2","2"));
      formParts.push_back(new curlpp::FormParts::Content("name3","true"));
      request.setOpt(new curlpp::options::HttpPost(formParts));
      
      request.setOpt(new curlpp::options::Url(url));
      request.setOpt(new curlpp::options::Verbose(true));
      // headers.push_back("Content-Type: application/string");
      // request.setOpt(new curlpp::options::HttpHeader(headers));
      // request.setOpt(new curlpp::options::PostFields(payload));
      // request.setOpt(new curlpp::options::PostFieldSize(payload.size() + 1));
      request.setOpt(new curlpp::options::WriteStream(&ss));
      request.perform();
    }catch(curlpp::RuntimeError& e){
      CLOG_WARN("coyot3-http-request : simple-post : exception raised "
      "while making request : runtime-err(" << e.what() << ")")
      res = false;
    }catch(curlpp::LogicError& e){
      CLOG_WARN("coyot3-http-request : simple-post : exception raised "
      "while making request : logic-err(" << e.what() << ")")
      res = false;
    }

    result = ss.str();
    return res;
  }


  bool
  Client::SimplePost(
    const std::string& url,
    const Json::Value& payload,
          std::string& result
  ){
    curlpp::Easy request;
    bool res = true;
    std::list<std::string> headers;
    std::stringstream             ss;

    
    std::stringstream iss;
    iss << payload;
    try{
      curlpp::Forms formParts;
      request.setOpt(new curlpp::options::HttpPost(formParts));
      request.setOpt(new curlpp::options::Url(url));
      request.setOpt(new curlpp::options::Verbose(true));
      headers.push_back("Content-Type: application/json");
      request.setOpt(new curlpp::options::HttpHeader(headers));
      request.setOpt(new curlpp::options::PostFields(iss.str()));
      request.setOpt(new curlpp::options::PostFieldSize(iss.str().size()));
      request.setOpt(new curlpp::options::WriteStream(&ss));
      request.perform();
    }catch(curlpp::RuntimeError& e){
      CLOG_WARN("coyot3-http-request : simple-post : exception raised "
      "while making request : runtime-err(" << e.what() << ")")
      res = false;
    }catch(curlpp::LogicError& e){
      CLOG_WARN("coyot3-http-request : simple-post : exception raised "
      "while making request : logic-err(" << e.what() << ")")
      res = false;
    }

    result = ss.str();
    return res;
  }


}
}
}