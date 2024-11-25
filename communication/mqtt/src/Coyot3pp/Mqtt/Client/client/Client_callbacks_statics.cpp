#include <Coyot3pp/Mqtt/Client/Client.hpp>



namespace coyot3{
namespace communication{
namespace mqtt{

int  mosq_on_tls_certs_password_callback(char *buff, 
                              int size, 
                              int rwflag, 
                              void *userdata){
  
}

void mosq_on_client_connects(struct mosquitto* client,
                              void* userdata,
                              int result){
  static_cast<Client*>(userdata)->on_mosq_client_connects(result);                                
}

void mosq_on_client_disconnect(struct mosquitto* c,
                              void* userdata, 
                              int rc){
  static_cast<Client*>(userdata)->on_mosq_client_disconnects(rc);
}

void mosq_on_subscribe(struct mosquitto* c, 
                              void* userdata, 
                              int mid, 
                              int qos_count, 
                              const int* granted_qos){
  static_cast<Client*>(userdata)->on_mosq_client_subscribe(mid,
    qos_count,
    granted_qos);
}

void mosq_on_unsubscribe(struct mosquitto* c,
                              void* userdata,
                              int mid){
  static_cast<Client*>(userdata)->on_mosq_client_unsubscribe(mid);
}

void mosq_on_message_received(struct mosquitto* mosq,
                              void* userdata,
                              const struct mosquitto_message* message){
  static_cast<Client*>(userdata)->on_mosq_client_message(message);
}

void mosq_on_log(struct mosquitto* client,
                              void* userdata,
                              int level,
                              const char* msg){
  static_cast<Client*>(userdata)->on_mosq_client_logs(level,msg);
}

void mosq_on_publish(struct mosquitto* client,
                              void* userdata,
                              int mid){
  static_cast<Client*>(userdata)->on_mosq_client_published(mid);

}

int mosq_on_tls_certs_password_callback(char* buff, 
                              int size,
                              int rwflag,
                              void* userdata){
  return static_cast<Client*>(userdata)->on_mosq_key_file_passphrase_request(
                              buff,
                              size,
                              rwflag);
}



}
}
}
