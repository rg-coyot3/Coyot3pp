#include <Coyot3pp/Mqtt/Client/Client.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{
  std::string Client::mosq_rc_stringify(int rc){
    std::string ret;
    switch(rc){
      case MOSQ_ERR_INVAL: return "MOSQ_ERR_INVAL : the input parameters were invalid.";break;
      case MOSQ_ERR_NOMEM: return "MOSQ_ERR_NOMEM : an out of memory condition occurred.";break;
      case MOSQ_ERR_NO_CONN: return "MOSQ_ERR_NO_CONN : the client isn't connected to a broker.";break;
      case MOSQ_ERR_MALFORMED_UTF8: return "MOSQ_ERR_MALFORMED_UTF8 : the topic is not valid UTF-8";break;
      case MOSQ_ERR_OVERSIZE_PACKET: return "MOSQ_ERR_OVERSIZE_PACKET : the resulting packet would be larger than supported by the broker";break;
      case MOSQ_ERR_PROTOCOL: return "MOSQ_ERR_PROTOCOL : protocol error comunicating with the broker"; break;
      case MOSQ_ERR_PAYLOAD_SIZE: return "MOSQ_ERR_PAYLOAD_SIZE : payload length is too large"; break;
      case MOSQ_ERR_QOS_NOT_SUPPORTED: return "MOSQ_ERR_QOS_NOT_SUPPORTED : qos not supported by the broker"; break;
      default:
        return "mosq-error-unknown-error-code";
    }
  }
}
}
}