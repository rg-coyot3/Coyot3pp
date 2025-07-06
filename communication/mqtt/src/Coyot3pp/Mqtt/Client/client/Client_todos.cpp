#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace ct = coyot3::tools;
namespace coyot3::communication::mqtt{


  void
  Client::on_mosq_client_unsubscribe(int message_id){
    log_warn(o() << "on-mosq-client-unsubscribe : received unsubscription "
    "confirmation with id [" << message_id << "]");
  }

  void
  Client::on_mosq_client_logs(int level, const char* msg){
    log_info( o() << "on-mosq-logs : level [" << level << "] : msg = (" 
    << std::string(msg) << ")");
  }

}