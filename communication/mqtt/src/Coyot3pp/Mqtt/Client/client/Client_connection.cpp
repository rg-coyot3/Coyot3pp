#include <Coyot3pp/Mqtt/Client/Client.hpp>

namespace ct = coyot3::tools;

namespace coyot3{
namespace communication{
namespace mqtt{
  
  int Client::on_mosq_key_file_passphrase_request(char* buff,
                                                  int size, 
                                                  int rwflag){
                                    
    if(config_.certificates_passphrase().size() 
        < static_cast<unsigned int>(size)){
      strncpy(buff, 
          config_.certificates_passphrase().c_str(),
          buff[config_.certificates_passphrase().size()]);
      buff[config_.certificates_passphrase().size()] = '\0';
    }else{
      strncpy(buff, config_.certificates_passphrase().c_str(),size);
    }
    int l = strlen(buff);
    log_info(o() << "on-mosq-key-file-passphrase-req- : passphrase set ["
    << config_.certificates_passphrase().size() << "," 
    << config_.certificates_passphrase() << "][ret=" << l << "]");
    return l;
  }
  
  int MOSQUITTO_LIB_INITIATED = false;

  bool Client::connect_to_broker_(){
    
    if(config_.debug_mode() == true){
      log_warn("client in debug mode.");
      return true;
    }
    backup_stack_prim_to_repub_(); // in case of reconnection, backup important messages. 
    if(MOSQUITTO_LIB_INITIATED == false){
      log_debug(3,"connect-to-broker- : initializing mosquitto lib");
      if(mosquitto_lib_init() != MOSQ_ERR_SUCCESS){
        log_err("ERROR INITIALIZING MOSQUITTO LIB INIT");
        return false;
      }
      MOSQUITTO_LIB_INITIATED = true;
    }

    if(config_.client_id().size() == 0){
      
      model.mosquitto_id() = ct::generate_string_aphanumeric(10);
      log_info(o() << "connect-to-broker- : generating random id session : [" 
        << model.mosquitto_id() << "]");
    }else{
      model.mosquitto_id() = config_.client_id();
      if(config_.clean_session() == true){
        model.mosquitto_id()+="_";
        model.mosquitto_id()+=ct::generate_string_aphanumeric(5);
        log_info(o() << "connect-to-broker- : client id set to [" << model.mosquitto_id() 
          << "]");
      }
    }

    client_ = mosquitto_new(
        model.mosquitto_id().c_str(),
        config_.clean_session(),
        static_cast<void*>(this));
    if(client_ == nullptr){
      log_err("connect-to-broker- : error creating mosquitto instance");
      return false;
    }else{
      log_debug(5,"connect-to-broker- : mosquitto instance generated.");
    }
    int rc;
    if((config_.user().size() != 0) && config_.password().size() != 0){
      rc = mosquitto_username_pw_set(
            client_,
            config_.user().c_str(),
            config_.password().c_str());
      switch(rc){
        case MOSQ_ERR_SUCCESS:
          log_debug(3,o() << "connect-to-broker- : user and password set : " << config_.user() 
                          << "@" << config_.password());
          break;
        case MOSQ_ERR_INVAL:
          log_warn("connect-to-broker- : error invalid parameters setting "
            "user password!");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
          break;
        case MOSQ_ERR_ERRNO:
          log_warn("connect-to-broker- : eror NO MEMORY!");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
          break;
        default:
          log_warn("connect-to-broker : error UNKNOWN CODE!");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
      }
    }

    if(
      (config_.certificates_ca().size() != 0)
    &&(config_.certificates_cert().size() != 0)
    &&(config_.certificates_key().size() != 0)
    ){
      if(config_.tls_version().size() == 0){
        log_warn("connect-to-broker- : error setting certificates! TLS VERSION MUST BE SET!");
        return false;
      }
      rc = mosquitto_tls_set(
        client_,
        config_.certificates_ca().c_str(),
        nullptr,
        config_.certificates_cert().c_str(),
        config_.certificates_key().c_str(),
        mosq_on_tls_certs_password_callback_v3
      );
      switch(rc){
        case MOSQ_ERR_SUCCESS:
          log_debug(3,"connect-to-broker- : certificates set correctly");
          break;
        case MOSQ_ERR_INVAL:
          log_warn(o() << "connect-to-broker- : error ! INVALID PARAMETERS "
            "setting certificates : " << config_.certificates_ca() << "; " <<
            config_.certificates_cert() << "; " << config_.certificates_key()
            << "; " << config_.tls_version());
            mosquitto_destroy(client_);client_=nullptr;
          return false;
        case MOSQ_ERR_NOMEM:
          log_warn(o() << "connect-to-broker- : error ! NO MEMORY "
            "setting certificates : " << config_.certificates_ca() << "; " <<
            config_.certificates_cert() << "; " << config_.certificates_key()
            << "; " << config_.tls_version());
            mosquitto_destroy(client_);client_=nullptr;
          return false;
        default:
          log_warn(o() << "connect-to-broker- : error UNKNOWN ERROR "
          "setting certificates : " << config_.certificates_ca() << "; " <<
            config_.certificates_cert() << "; " << config_.certificates_key()
            << "; " << config_.tls_version());
          mosquitto_destroy(client_);client_=nullptr;
      }
      rc = mosquitto_tls_opts_set(
            client_,
            SSL_VERIFY_PEER,
            config_.tls_version().c_str(),
            config_.ciphers().c_str());

      switch(rc){
        case MOSQ_ERR_SUCCESS:
          log_debug(3,"connect-to-broker- : tls options set OK");
          break;
        case MOSQ_ERR_INVAL:
          log_warn("connect-to-broker- : error : INVALID PARAMS setting"
          " tls options");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
        case MOSQ_ERR_NOMEM:
          log_warn("connect-to-broker- : error : NO MEMORY setting"
          " tls options");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
        default:
          log_warn("connect-to-broker- : error : UNKNOWN ERROR setting "
          "tls options");
          mosquitto_destroy(client_);client_=nullptr;
          return false;
      }
    }
    
    log_debug(7,"connect-to-broker- : setting callbacks");
    mosquitto_connect_callback_set(client_,mosq_on_client_connects_v3);
    mosquitto_disconnect_callback_set(client_,mosq_on_client_disconnects_v3);
    mosquitto_message_callback_set(client_,mosq_on_message_received_v3);
    mosquitto_log_callback_set(client_,mosq_on_log_v3);
    mosquitto_subscribe_callback_set(client_,mosq_on_subscribe_v3);
    mosquitto_unsubscribe_callback_set(client_,mosq_on_unsubscribe_v3);
    mosquitto_publish_callback_set(client_,mosq_on_publish_v3);
    log_debug(5,"connect-to-broker- : launching mosquitto-loop-");
    rc = mosquitto_loop_start(client_);

    switch(rc){
      case MOSQ_ERR_SUCCESS:
        log_info("connect-to-broker- : mosquitto-loop launched");
        break;
      case MOSQ_ERR_INVAL:
        log_warn("connect-to-broker- : error INVALID PARAMS launching loop");
        mosquitto_destroy(client_);client_ = nullptr; return false;
        return false;
      case MOSQ_ERR_NOT_SUPPORTED:
        log_warn("connect-to-broker- : error THREAD SUPPORT NOT AVAILABLE"
        " launching loop");
        mosquitto_destroy(client_);client_ = nullptr; return false;
      default:
        log_warn("connect-to-broker- : error UNKNOWN launching loop!");
        mosquitto_destroy(client_);client_ = nullptr; return false;
    }
    log_debug(5, "connect-to-broker- : mosquitto client configured.");

    model.set_state(ec::MosqClientState::CONNECTING);
    
    bool retry = false;
    do{
      if(retry == true){
        log_warn("connect-to-broker- : retrying connection : forcing wait "
        ".25ms");
        usleep(250000);
      }
      int keepaliveparam = (config_.timeout() < 20 ? config_.timeout() : 20);
      rc = mosquitto_connect_async(client_,
            config_.host_address().c_str(),
            config_.host_port(),
            keepaliveparam);
      switch (rc)
      {
      case MOSQ_ERR_SUCCESS:
        log_info("connect-to-broker- : client connecting to broker");
        retry = false;
        break;
      case MOSQ_ERR_INVAL:
        log_err("connect-to-broker- : error INVALID PARAMETERS connecting to "
        "broker");
        mosquitto_destroy(client_);client_ = nullptr; return false;
        model.set_state(ec::MosqClientState::ERROR);
        return false;
      default:
        log_warn(o() << "connect-to-broker- : error CODE(" << rc << 
        ") connecting");
        retry = true;
      }
    }while(retry == true);
    log_info(o() << "connect-to-broker- : client connecting to broker" 
    << config_.host_address() << ":" << config_.host_port());
    return true;
  }



  // ON CONNECTION
  void Client::on_mosq_client_connects(int rc){
    log_debug(3, o() << "on-client-connects- : code [" 
      << mosquitto_error_codes_descriptions(rc) << "]");
    bool invokeSubscriptions = false;
    switch (rc)
    {
    case MOSQ_ERR_SUCCESS:
      log_info(o() << "on-client-connects- : connected to [" 
      << config_.host_address() <<":" << config_.host_port() << "]");
      model.set_state(ec::MosqClientState::CONNECTED);
      if(model.ts_first_connection() == 0)
        model.ts_first_connection(model.ts_last_transition());
      model.ts_last_connection(model.ts_last_transition());
      invokeSubscriptions = true;
      break;
    default:
      log_warn( o() << "on-client-connects- : connection error [" 
      << mosquitto_error_codes_descriptions(rc) << "]");
      model.state(ec::MosqClientState::DISCONNECTED);
      model.ts_last_transition(ct::get_current_timestamp());
      return;
    }
    

    // TO-DO
    // if(full_reset_connection_ == true){
    //   model.subscriptions_pending()
    // }else{
    //   //
    // }

    if((model.callbacks_on_connection().size() != 0) 
    && (invokeSubscriptions == true)){
      log_debug(3,"on-client-connects- : invoking on-connection callbacks");
      for(CallbacksOnEventSimplePair p : model.callbacks_on_connection()){
        log_debug(5, o() << "on-client-connects- : invoking callback id [" 
        << p.first << "]");
        p.second(rc == MOSQ_ERR_SUCCESS);
      }
    }
    
    prepare_subscriptions_();
  }
  




  bool
  Client::disconnect_from_broker_(){

    if(client_ == nullptr){
      log_debug(5,"disconnect-from-broker- : mosquitto client not "
      "instantiated");
      return false;
    }
    log_debug(5,"disconnect-from-broker- : disconnecting from broker and "
    "forcing loop stop");
    mosquitto_disconnect(client_);
    mosquitto_loop_stop(client_,true);
    mosquitto_destroy(client_);
    client_ = nullptr;
    log_info("disconnect-from-broker- : done");
    model.state(ec::MosqClientState::DISCONNECTED);
    return true;
  }







  void
  Client::on_mosq_client_disconnects(int reason){
    model.state(ec::MosqClientState::DISCONNECTED);
    if(reason != 0){
      log_warn("on-mosq-client-disconnect- : unexpected disconnection");
    }else{
      log_debug(3,"on-mosq-client-disconnect- : disconnected");
    }
    if(model.callbacks_on_disconnection().size() == 0)return;
    log_debug(3,"on-mosq-client-disconnect- : invoking callbacks");
    for(MqttClientCallbacksOnEventSimplePair c : model.callbacks_on_disconnection()){
      c.second(reason == 0);
    }
  }
}
}
}