#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Infos.hpp>
#include <curlpp/Exception.hpp>


namespace coyot3{
namespace communication{
namespace rest{

  
  class Client{
    public:
      enum class RestType{
        UNSET,
        GET,
        POST,
        FORM
      };
      struct MimeType{
        static constexpr const char* TEXT    ="text/plain";
        static constexpr const char* BINDATA ="application/octect-stream";
        static constexpr const char* AAC     ="audio/aac";
        static constexpr const char* JSON    ="application/json";
        static constexpr const char* BZ      ="application/x-bzip";
        static constexpr const char* BZ2     ="application/x-bzip2";
        static constexpr const char* GZ      ="application/gzip";
        static constexpr const char* CSV     ="text/csv";
        static constexpr const char* ABW     ="application/x-abiword";
      };

      typedef std::map<std::string,coyot3::tools::Value> ParamsMap;
      typedef std::function<void(bool, int, const std::string&)> CallbackFunc;
    // statics : begin
      static bool SimpleUrlRequest(const std::string& url, std::string& result);
      static bool SimplePost(const std::string& url, const std::string& payload, std::string& result);
      static bool SimplePost(const std::string& url, const Json::Value& payload, std::string& result);
    // statics : end

      Client(RestType type = RestType::GET,const std::string& url = std::string());
      virtual ~Client();

      bool      set_target(const std::string& url);
      bool      set_content_type(const std::string& name);
      bool      set_payload(const std::string& payload);
      bool      set_payload(const Json::Value& payload);
      bool      set_callback(CallbackFunc callback);

      /**
       * @brief 
       * 
       * @param interval 
       * @return true 
       * @return false 
       */
      bool      async_start(int interval = 0);
      bool      async_stop();
      

      template<typename T>
      bool param_set(const std::string& name, const T& val){
        if(params_.find(name) == params_.end()){
          params_.insert(std::make_pair(name,val));
        }else{
          params_[name] = val;
        }
        return true;
      }

      bool param_del(const std::string& name);

    protected:
      std::list<std::string>            headers_;
      ParamsMap                         params_;
      std::string                       content_type_;
      coyot3::tools::WorkerThread*      timer_;
      int64_t                           timer_interval_;
      std::string                       url_;
      CallbackFunc                      callback_;
      RestType                          type_;
      std::string                       payload_;
      
      void                              request_task_();

      void                              request_task_common_(bool& ok, 
                                                          int& code, 
                                                          std::string& result);
    private:


  };

}
}
}