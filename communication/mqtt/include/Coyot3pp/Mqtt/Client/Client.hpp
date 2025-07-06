#pragma once



#include <mosquitto.h>
#include <openssl/err.h>
#include <openssl/ssl.h>

#include <Coyot3pp/Cor3/ModuleBase.hpp>
#include "config/ClientConfiguration.hpp"
#include "message/Message.hpp"
#include "datamodel/ClientDataModel.hpp"




namespace coyot3::communication::mqtt{




int  mosq_on_tls_certs_password_callback_v3(char *buff, 
                              int size, 
                              int rwflag, 
                              void *userdata);


void mosq_on_client_connects_v3(struct mosquitto* client,
                              void* userdata,
                              int resutl);
void mosq_on_client_disconnects_v3(struct mosquitto* c,
                              void* userdata, 
                              int rc);
void mosq_on_subscribe_v3(struct mosquitto* c, 

                              void* userdata, 
                              int mid, 
                              int qos_count, 
                              const int* granted_qos);

void mosq_on_unsubscribe_v3(struct mosquitto* c,
                              void* userdata,
                              int mid);
void mosq_on_message_received_v3(struct mosquitto* mosq,
                              void* userdata,
                              const struct mosquitto_message* message);
void mosq_on_log_v3(struct mosquitto* client,
                              void* userdata,
                              int level,
                              const char* msg);
void mosq_on_publish_v3(struct mosquitto* client,
                              void* userdata,
                              int mid);


// misc-helpers
std::string mosquitto_error_codes_descriptions(int rc);
std::string mqtt_on_connect_return_code_version_3(int rc);
std::string mqtt_on_connect_return_code_version_5(int rc);


//mqtt client
  class Client : public coyot3::mod::ModuleBase{


    friend int  mosq_on_tls_certs_password_callback_v3(char *buff, 
                                  int size, 
                                  int rwflag, 
                                  void *userdata);
    

    friend void mosq_on_client_connects_v3(struct mosquitto* client,
                                  void* userdata,
                                  int resutl);
    friend void mosq_on_client_disconnects_v3(struct mosquitto* c,
                                  void* userdata, 
                                  int rc);
    friend void mosq_on_subscribe_v3(struct mosquitto* c, 
                                  void* userdata, 
                                  int mid, 
                                  int qos_count, 
                                  const int* granted_qos);

    friend void mosq_on_unsubscribe_v3(struct mosquitto* c,
                                  void* userdata,
                                  int mid);
    friend void mosq_on_message_received_v3(struct mosquitto* mosq,
                                  void* userdata,
                                  const struct mosquitto_message* message);
    friend void mosq_on_log_v3(struct mosquitto* client,
                                  void* userdata,
                                  int level,
                                  const char* msg);
    friend void mosq_on_publish_v3(struct mosquitto* client,
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
                                    MqttClientOnMessageCallback cb);

      /**
       * @brief registers a subscription and 
       * 
       * @param topic 
       * @param mqttQos 
       * @param id 
       * @param cb 
       * @return true 
       * @return false 
       */
      bool register_subscription(const std::string& topic,
                                    int mqttQos,
                                    MqttClientOnMessageCallback cb,
                                    int64_t& id);
      bool register_publisher(const std::string& topic, int mqttQos,int64_t& id);

      


      /**
       * @brief lowest level public publication exposed method.
       * 
       * @param topic 
       * @param payload 
       * @param length 
       * @param priority 
       * @return true : publication taken in count OK.
       * @return false : error publishing. (mainly because the client is not connected.)
       */

      
      bool publish(const std::string& topic, 
                  const uint8_t* payload,
                  std::size_t length,
                  int qos = 0);

      template<typename T>
      bool publish(const std::string& topic,
                  const T& payload,
                  int qos = 0){
        std::stringstream sstr;
        sstr << payload;
        return publish(topic, (uint8_t*)sstr.str().c_str(), sstr.str().size(), qos);
      }
      
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
      /** to republish because it was important but at the moment of the 
       *  republish it was not connected, and to store messages when forcing 
       *  reconnections.
       */
      MessageStack     message_stack_repub_; 
      void             backup_stack_prim_to_repub_();
      
      bool             full_reset_connection(); // mutex locks
    private:
      
      struct mosquitto*   client_;
      std::mutex          client_tx_mtx_; ///!< control to send only one comm.
      std::mutex          client_models_mtx_;


      ClientDataModel     model;

      bool full_reset_flag_;  ///!< making a full reset

      ControlThread* th_main_;          ///!< main thread controller
        int64_t      th_main_interval_; ///!< interval for the task controller pulse
        void    task_controller();      ///!< main controller task

      ControlThread* th_sec_;


      bool connect_to_broker_();
      bool disconnect_from_broker_();


      bool full_reset_connection_(); // no mutex locks



      bool prepare_subscriptions_();
        bool make_subscriptions_();
        bool all_subscriptions_are_done_();

      
      bool publish_(const std::string& topic, 
                  const uint8_t* payload,
                  std::size_t length,
                  int qos);
      /**
       * @brief simple mqtt publish 
       * 
       * @param t topic
       * @param p payload ptr
       * @param s payload size
       * @param q mqtt qos (by default = 0)
       * @param mid mosquitto message id to control sent packets.
       * @return true : published ok.
       * @return false : not published because of any error.
       */
      bool publish_payload_v3_(
              const std::string& t, 
              const uint8_t* p, 
              int s, 
              int q,
              int* mid);

      
    public:

      static constexpr const int64_t CONTROLLER_INTERVAL_NORMAL = 1000;
      static constexpr const int64_t CONTROLLER_INTERVAL_INTENSIVE = 250;
      static std::string mosq_rc_stringify(int rc);

  };


}