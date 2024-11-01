#include <Coyot3pp/LwsServ3r/v1.0/WsgClient.hpp>

namespace coyot3{
namespace services{
namespace websocket{

int64_t WsgClient::DefaultStandbyTimeout = CYTWSGCLIENT_CONFIG_DEFAULT_STANDBYTIME;

WsgClient::WsgClient(WsgConnectionStructure* source)
: _source(source)
,_creation_ts(0)
,_standby_start_ts(0)
,_standby_time(CYTWSGCLIENT_CONFIG_DEFAULT_STANDBYTIME)
,_has_pending_ops(false)
{
  _creation_ts = coyot3::tools::getCurrentTimestamp();
}
WsgClient::~WsgClient()
{
  
}
WsgClient::WsgClient()
{
  CLOG_ERROR("CYTWSGCLIENT : protected constructor : not available");
}
WsgClient::WsgClient(const WsgClient& o)
{
  *this = o;
}
WsgClient& WsgClient::operator=(const WsgClient& o)
{
  _source = o._source;
  _creation_ts = o._creation_ts;
  _standby_start_ts = o._standby_start_ts;
  _standby_time = o._standby_time;
  _has_pending_ops = o._has_pending_ops;

  return *this;
}


size_t WsgClient::setResponseContent(const std::string& res)
{
  _source->response = res;
  return _source->response.size();
}
size_t WsgClient::appendToResponseContent(const std::string& res)
{
  if(res.size() == 0)
  {

    return _source->response.size();
  }
  _source->response+=res;
  _source->response_total_length = _source->response.size();
  return _source->response.size();
}
void WsgClient::resetOutputBufferIndex()
{
  _source->response_current_index = 0;
}

bool        WsgClient::sendPreparedData()
{
  if(_source->wsi == nullptr)
  {
    CLOG_WARN("CYTWSGCLIENT : send-prepared-data : error. Current client structure has no WSI ptr");
    return false;
  }
  int res = lws_callback_on_writable(_source->wsi);
  if( res == 0 )
  {
    CLOG_WARN("CYTWSGCLIENT : send-prepared-data : warn , result is zero");
  }else{
    CLOG_DEBUG(3,"CYTWSGCLIENT : send-prepared-data : warn , result is [" << res << "]");
  }
  return true;
}

bool        WsgClient::getState()
{
  return false;
}
bool        WsgClient::disconnect()
{
  return false;
}
int64_t     WsgClient::getConnectionID()
{
  int64_t res;
  if(_source == nullptr)
  {
    return -1;
  }else{
    return _source->ref_id;
  }
  
  
}

bool        WsgClient::send(const std::string& s)
{
  if(!_source)
  {
    CLOG_ERROR("WgsClient : send : no client structure is set!");
    return false;
  }
  if(!_source->wsi)
  {
    CLOG_ERROR("WsgClient : " << _source->ref_id << " : send : no LWS associated structure!");
  }
  if(_source->ws_output_stream)
  {
    CLOG_WARN("WgsClient : " << _source->ref_id << " : send : os is not null");
    return false;
  }
  _source->ws_output_stream = new(std::nothrow) uint8_t[s.size()+LWS_PRE+2];
  if(!_source->ws_output_stream)
  {
    CLOG_ERROR("WsgClient : " << _source->ref_id << " :  error allocating output stream");
    return false;
  }

  _source->ws_output_stream_index = 0;
  memcpy(&(_source->ws_output_stream[LWS_PRE]),s.c_str(),s.size());
  _source->ws_output_stream[LWS_PRE+s.size()]   = (uint8_t)'\r';
  _source->ws_output_stream[LWS_PRE+s.size()+1] = (uint8_t)'\n';
  _source->ws_output_stream_size = s.size()+2;
  lws_callback_on_writable(_source->wsi);
  return true;
}

bool        WsgClient::send(const Json::Value& s)
{
  std::stringstream ss;
  ss << s;
  return send(ss.str());
}

bool WsgClient::send(const WsgExchangePacket& ep)
{
  return send(ep.toJson());
}





std::string WsgClient::getPeerInfo()
{
  return (_source == nullptr?std::string():_source->source_peer);
}

uint8_t* WsgClient::getOutputBuffer(){return _source->ws_output_stream;}
size_t   WsgClient::getOutputBufferSize(){return _source->ws_output_stream_size;}
size_t   WsgClient::getOutputBufferIndex(){return _source->ws_output_stream_index;}
bool     WsgClient::outputBufferIndexInc(size_t i)
{
  if(_source->ws_output_stream_index+i>=_source->ws_output_stream_size)
  {
    _source->ws_output_stream_index = _source->ws_output_stream_size;
    return false;
  }
  _source->ws_output_stream_index+=i;
  return true;
}
bool     WsgClient::outputBufferClean()
{
  if(!_source->ws_output_stream)
  {
    return false;
  }
  delete _source->ws_output_stream;
  _source->ws_output_stream = nullptr;
  _source->ws_output_stream_index = 0;
  _source->ws_output_stream_size = 0;
  return true;
}
WebsocketsServerGateway* WsgClient::getGatewayPtr()
{
  return _source->gateway_instance;
}

std::string WsgClient::getPath()
{
  return _source->path;
}
bool WsgClient::standBy(int64_t timeout)
{
  if(timeout <= 0)
  {
    _standby_time = CYTWSGCLIENT_CONFIG_DEFAULT_STANDBYTIME;
  }else{
    _standby_time = timeout;
  }
  _standby_start_ts = coyot3::tools::getCurrentTimestamp();
  _has_pending_ops = true;
  return _has_pending_ops;
}
bool WsgClient::standByTimeout()
{
  int64_t now = coyot3::tools::getCurrentTimestamp();
  return (((now - _standby_start_ts) > _standby_time) && _has_pending_ops);
}
bool WsgClient::hasPendingOps()
{
  return _has_pending_ops;
}


}
}//eons wrappers
}//eons coyot3