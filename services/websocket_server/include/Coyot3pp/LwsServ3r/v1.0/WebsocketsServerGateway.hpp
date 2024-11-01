#pragma once
/**
 * Websockets Gateway
 *  is a wrapper to easily create a websockets server with http server 
 *  capabilities.
 * 
 * @author : Ricardo GONZALEZ-ALMEIDA <ricardogonalm@ltlab.net>
 * 
 * LICENSE : GPLv2
 * 
 * */



#include <iostream>
#include <string>
#include <unistd.h>
#include <libwebsockets.h>
#include <assert.h>
#include <thread>
#include <vector>
#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <functional>
#include <map>
#include <mutex>

#include "WsgConnectionStructure.hpp"
#include "WsgClient.hpp"
#include "WsgExchangePacket.hpp"


#define CYTWSG_CONFIG_DEFAULT_COMMSCONTROL_TIMEOUT  10000
#define CYTWSG_CONFIG_DEFAULT_LWS_HTTPTIMEOUT_SECS  20
#define CYTWSG_CONFIG_DEFAULT_COMMSCONTROL_INTERVAL 750

#define CYTWSG_DEVELOPPEMENT false
namespace coyot3{
namespace services{
namespace websocket{

  
  typedef std::map<std::string,std::string> UriArguments;
  
  int callback_dynamic_http_route(
    struct lws *wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
  );
  int callback_minimal_ws(
    struct lws *wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
  );





// GET / HTTP/1.1
// Host: localhost:6005
// User-Agent: curl/7.58.0
// Accept: */*

// HTTP/1.1 200 OK
// content-security-policy: default-src 'none'; img-src 'self' data: ; script-src 'self'; font-src 'self'; style-src 'self'; connect-src 'self' ws: wss:; frame-ancestors 'none'; base-uri 'none';form-action 'self';
// x-content-type-options: nosniff
// x-xss-protection: 1; mode=block
// x-frame-options: deny
// referrer-policy: no-referrer
// content-type: text/html
// connection: close


// HTTP/1.1 200 OK
// content-type: text/html
// connection: close
#define _WSG_CHUNK_MAX_LENGTH 59392



class WebsocketsServerGateway {

  public :

    struct ServerHeaders{
      
      
      struct CSP{

      };

      ServerHeaders();
      ServerHeaders(const ServerHeaders& o);
      virtual ~ServerHeaders();
      ServerHeaders& operator=(const ServerHeaders& o);

      bool          setHeader(const std::string& header,const std::string& content);
      std::string   getHeader(const std::string& header) const;
      std::string   stringify() const;
      bool          setCsp(const std::string& policy,const std::string& rule);
      std::string   getCsp(const std::string& policy);
      size_t        size();

      bool   prepareDefaultCsp();
      bool   prepareDefaultHeaders();
      size_t generateLwsServerHeaders();
      bool   areGenerated();
      size_t cleanHeaders();
      struct lws_protocol_vhost_options* getHeadersPtr();


      typedef std::map<std::string,std::string> StrstrMap;

      StrstrMap headersMap;
      StrstrMap cspMap;
      struct lws_protocol_vhost_options* headers;
    };



    
    static const char* CallbackReasonToString(int reason);
    
    WebsocketsServerGateway();
    virtual ~WebsocketsServerGateway();

    

    virtual bool Init();
    virtual bool Start();
    virtual bool Stop();

    std::string     name();
    bool            setName(const std::string& n);
    
    bool            serverPrepareDefaultHeaders();
    bool            serverSetHeader(const std::string& header,const std::string& value);
    bool            serverSetCspRule(const std::string& policy,const std::string& value);
    bool            serverDefaultSecurePolicyHeaders();
    bool            serverDefaultSecurePolicyHeaders(bool activate);
    /**
     * 
     * */
    bool            addStaticPath(const std::string& baseUrl
                                 ,const std::string& basePath
                                 ,const std::string& defaultFile = std::string("404.html"));
    bool            addDynamicPath(const std::string& baseUrl);
    bool            addSimpleWebsocketsServer(const std::string& baseUrl);
    bool            setServerPort(int p);
    bool            setDefaultDoc404(const std::string& p);

    bool            setOnDynHttpCallback(std::function<bool(WsgClient*,const std::string&,const UriArguments&, std::string&)> callback);
    bool            setOnWSClientConnect(std::function<bool(WsgClient*)> callback);
    bool            setOnWSClientDisconnects(std::function<bool(WsgClient*)> callback);
    bool            setOnWSClientMessage(std::function<bool(WsgClient*,const uint8_t*,size_t)> callback);

    int             broadcastMessage(const std::string& message);
    int             broadcastMessage(const Json::Value& message);
    
    bool            setConfigMaxChunkSize(size_t s);
    bool            setSSLCertKeyPath(const std::string& certPath,const std::string& keyPath);

    bool            activateSendForHoldingConnection(WsgClient* client);
    size_t          appendToResponseContent(WsgClient* client,const std::string& data);
    size_t          appendToResponseContent(WsgClient* client,const Json::Value& data);
    std::string     getClientPeerInfo(WsgClient* client);
    std::string     getConnectedClientsPeerInfo();


    void            set_debug_level(int level);

    int64_t         asyncCommsTimeout();
    int64_t         asyncCommsTimeout(int64_t timeoutMsecs);

  protected : 

    size_t          _appendToResponseContent(WsgClient* client,const std::string& data);
    char*                   _name;


    volatile bool                   _initialized;
      coyot3::tools::ControlThread* _server_thread;
      void                              _server_loop();
      volatile bool                     _server_active;

      coyot3::tools::ControlThread* _async_comms_control;
      int64_t                           _async_comms_control_timeout;
      int64_t                           _async_comms_control_interval;
      void                              _async_comms_control_pulse();

      



    struct lws_context_creation_info       _info;
    lws_context*                           _context;
    std::vector<struct lws_http_mount*>    _mounts_collection;
    std::vector<struct lws_protocols*>     _protocols_collection;
    const struct lws_protocols**           _pprotocols_ptrs;

    ServerHeaders                          headersManager;
    bool                                   headersDefaultSecurePolicy;


    
    int                     _server_port;
    std::string             _404_path;
      std::string             _ssl_cert_path;
      std::string             _ssl_key_path;

    virtual   bool _initialize_info();
    virtual   bool _initialize_lws_context();


    void           _clean_mounts();
    void           _clean_prots();
    bool           _push_mount(struct lws_http_mount* m);


    /**
     * @brief 
     * @return      true  : if the response is prepared. it will be commanded to the lws to respond inmediatly.
     *              false : if it is not prepared. it is expected for the user to prepare the response.
     */
    std::function<bool(WsgClient*,const std::string&,const UriArguments&,std::string&)>   callback_ondynhttpreq;
    
    
    std::function<bool(WsgClient*,const uint8_t*,size_t)> callback_onwsmessage;
    std::function<bool(WsgClient*)>                       callback_onwsconnects;
    std::function<bool(WsgClient*)>                       callback_onwsdisconnects;
    std::function<bool(WsgClient*,const std::string&)>    callback_on_error;

    bool dummy_callback_on_dyn_http(WsgClient* client,const std::string& originPath,const UriArguments&,std::string& response);
    bool dummy_callback_on_ws_message(WsgClient* client,const uint8_t* buffer,size_t length);
    bool dummy_callback_on_ws_connects(WsgClient* client);
    bool dummy_callback_on_ws_disconnects(WsgClient* client);
    bool dummy_callback_on_error(WsgClient* client,const std::string& what);

    public:
    friend int callback_dynamic_http_route(struct ::lws               *wsi,
                                          enum ::lws_callback_reasons reason,
                                          void*                       user,
                                          void*                       in,
                                          size_t                      len);
    int    callback_dyn_http(struct ::lws*                wsi,
                              enum ::lws_callback_reasons reason,
                              void*                       in,
                              size_t                      len,
                              WsgConnectionStructure*           dpr);

      typedef std::map<int64_t,WsgClient*>  TCClMap;
      typedef std::pair<int64_t,WsgClient*> TCClPair;
      
    TCClMap     _connected_clients;
      std::mutex  _connected_clients_mtx;
    uint64_t    _connection_struct_id_index;
      WsgClient*  add_client(WsgConnectionStructure* ref,struct ::lws* wsi);
      WsgClient*  get_client(uint64_t id);
      WsgClient*  get_client(WsgClient* client);

      bool        del_client(uint64_t id);
      bool        del_client(WsgClient* client);
      

    bool add_minimal_ws_callback();

    static bool HasUriArguments(struct ::lws* wsi);
    static size_t ExtraitUriArguments(struct ::lws* wsi,UriArguments& destination);

    
    void set_debug_level_mcallbacks_(int level);
    void set_debug_level_mmisc_(int level);
    void set_debug_level_mstatics_(int level);
  
  private:

    size_t            _config_chunk_max_size;


};



}
}//eons wrappers
}//eons coyot3