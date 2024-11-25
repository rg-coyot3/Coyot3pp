#pragma once



#include <mosquitto.h>
#include <openssl/err.h>
#include <openssl/ssl.h>

#include <Coyot3pp/Cor3/ModuleBase.hpp>
#include "config/ClientConfiguration.hpp"
#include "message/Message.hpp"
#include "datamodel/ClientDataModel.hpp"




namespace coyot3{
namespace communication{
namespace mqtt{



int  mosq_on_tls_certs_password_callback(char *buff, 
                              int size, 
                              int rwflag, 
                              void *userdata);

void mosq_on_client_connects(struct mosquitto* client,
                              void* userdata,
                              int resutl);
void mosq_on_client_disconnects(struct mosquitto* c,
                              void* userdata, 
                              int rc);
void mosq_on_subscribe(struct mosquitto* c, 
                              void* userdata, 
                              int mid, 
                              int qos_count, 
                              const int* granted_qos);
void mosq_on_unsubscribe(struct mosquitto* c,
                              void* userdata,
                              int mid);
void mosq_on_message_received(struct mosquitto* mosq,
                              void* userdata,
                              const struct mosquitto_message* message);
void mosq_on_log(struct mosquitto* client,
                              void* userdata,
                              int level,
                              const char* msg);
void mosq_on_publish(struct mosquitto* client,
                              void* userdata,
                              int mid);


// misc-helpers
std::string mosquitto_error_codes_descriptions(int rc);
std::string mqtt_on_connect_return_code_version_3(int rc);
std::string mqtt_on_connect_return_code_version_5(int rc);


//mqtt client
  class Client : public coyot3::mod::ModuleBase{

    friend int  mosq_on_tls_certs_password_callback(char *buff, 
                                  int size, 
                                  int rwflag, 
                                  void *userdata);
    
    friend void mosq_on_client_connects(struct mosquitto* client,
                                  void* userdata,
                                  int resutl);
    friend void mosq_on_client_disconnects(struct mosquitto* c,
                                  void* userdata, 
                                  int rc);
    friend void mosq_on_subscribe(struct mosquitto* c, 
                                  void* userdata, 
                                  int mid, 
                                  int qos_count, 
                                  const int* granted_qos);
    friend void mosq_on_unsubscribe(struct mosquitto* c,
                                  void* userdata,
                                  int mid);
    friend void mosq_on_message_received(struct mosquitto* mosq,
                                  void* userdata,
                                  const struct mosquitto_message* message);
    friend void mosq_on_log(struct mosquitto* client,
                                  void* userdata,
                                  int level,
                                  const char* msg);
    friend void mosq_on_publish(struct mosquitto* client,
                                  void* userdata,
                                  int mid);
    
    public:
      typedef coyot3::tools::ControlThread ControlThread;
        typedef MqttClientCallbackOnEventSimple CallbackOnEventSimple;
        typedef MqttClientCallbacksOnEventSimple CallbacksOnEventSimple;
        typedef MqttClientCallbacksOnEventSimpleIter CallbacksOnEventSimpleIter;
        typedef MqttClientCallbacksOnEventSimplePair CallbacksOnEventSimplePair;


      Client(const std::string& name);
      virtual ~Client();

      ClientConfiguration& config(const ClientConfiguration& configuration);
      const ClientConfiguration& config() const;


      bool register_subscription(const std::string& topic,
                                    int mqttQos,
                                    int64_t& id,
                                    MqttClientOnMessageCallback cb);
      bool register_publisher(const std::string& topic, int mqttQos,int64_t& id);
      
      int  on_connection_callback_add(CallbackOnEventSimple cb);
      int  on_disconnection_callback_add(CallbackOnEventSimple cb);
      int  on_error_callback_set();

      std::string client_id() const;

    protected:

      bool init_();
      bool start_();
      bool pause_();
      bool stop_();
      bool end_();

      ClientConfiguration config_;

      

      void on_mosq_client_connects(int result);
        
      void on_mosq_client_disconnects(int reason); //to-do
      void on_mosq_client_published(int message_id);
      void on_mosq_client_unsubscribe(int message_id);
      void on_mosq_client_subscribe(int message_id,int qos_count,const int* granted_qos);
      void on_mosq_client_message(const struct mosquitto_message* message);
      void on_mosq_client_logs(int level,const char* msg);
      int  on_mosq_key_file_passphrase_request(char* buff, int size,int rwflag);

      MessageMappedSet message_stack_prim_;
      MessageMappedSet message_stack_sec_;
    private:
      
      struct mosquitto*   client_;
      std::mutex          client_tx_mtx_; ///!< control to send only one comm.


      ClientDataModel     model;

      bool full_reset_flag_;  ///!< making a full reset

      ControlThread* th_main_;          ///!< main thread controller
        int64_t      th_main_interval_; ///!< interval for the task controller pulse
        void    task_controller();      ///!< main controller task

      ControlThread* th_sec_;



      bool connect_to_broker_();
      bool disconnect_from_broker_();

      bool full_reset_connection_();


      bool prepare_subscriptions_();
        bool make_subscriptions_();

      
    public:

      static constexpr const int64_t CONTROLLER_INTERVAL_NORMAL = 1000;
      static constexpr const int64_t CONTROLLER_INTERVAL_INTENSIVE = 250;

  };


}
}
}