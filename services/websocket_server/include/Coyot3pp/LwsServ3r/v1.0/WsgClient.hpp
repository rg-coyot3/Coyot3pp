#pragma once



#include <string>
#include <functional>
#include <jsoncpp/json/value.h>
#include <libwebsockets.h>

#include <Coyot3pp/Cor3/Coyot3.hpp>
#include "WsgConnectionStructure.hpp"
#include "WsgExchangePacket.hpp"
namespace coyot3{
namespace services{
namespace websocket{

class WebsocketsServerGateway;

#define CYTWSGCLIENT_CONFIG_DEFAULT_STANDBYTIME 61000
/**
 * @brief prototype class to give to the user some interfaces to manage the 
 *        the connection.
 * 
 * */
class WsgClient{
    friend class WebsocketsServerGateway;
  public:
    static int64_t            DefaultStandbyTimeout;
    WsgClient(const WsgClient& o);
    WsgClient(WsgConnectionStructure* source);
    virtual     ~WsgClient();
    WsgClient& operator=(const WsgClient& o);

    bool                      getState();
    bool                      disconnect();
    int64_t                   getConnectionID();
    bool                      send(const std::string& s);
    bool                      send(const Json::Value& s);
    bool                      send(const WsgExchangePacket& ep);
    
    std::string               getPeerInfo();
    
    uint8_t*                  getOutputBuffer();
    size_t                    getOutputBufferSize();
    size_t                    getOutputBufferIndex();
    bool                      outputBufferIndexInc(size_t i);
    bool                      outputBufferClean();

    WebsocketsServerGateway*  getGatewayPtr();
    std::string               getPath();
    size_t                    setResponseContent(const std::string& res);
    size_t                    appendToResponseContent(const std::string& res);
    void                      resetOutputBufferIndex();
    bool                      standBy(int64_t timeout=0);
    bool                      standByTimeout();
    int64_t                   getStandbyTime();
    bool                      hasPendingOps();
    bool                      sendPreparedData();

  protected:
    WsgConnectionStructure*   _source;
    int64_t                   _creation_ts;
    int64_t                   _standby_start_ts;
    int64_t                   _standby_time;
    bool                      _has_pending_ops;
    
    
    WsgClient();

  private:
  


};



}
}//eons wrappers
}//eons coyot3