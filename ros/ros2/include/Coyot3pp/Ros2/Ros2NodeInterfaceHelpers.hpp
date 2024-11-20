#pragma once


#include "../tools/cyt_dev_tools.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>


// MACROS : SUBSCRIBERS : BEGIN

/**
 * @brief declares
 *  std::string                   <var-name>_rostopic initialized with <rostopiv_default> string
 *  <var-type>::SharedPtr         <var-name>_subscription 
 *  void <var-name>_msg_callback  
 *  <var-name>_buffer
 *  <var-name>_buffer_mtx
 *  <var-name>_buffer_last_rx
 *  <var-name>_buffer_min_gather_interval
 *  <var-type> <var-name>() 
 *  bool       <var-name>_config(topic, min-activity-interval)
*/
#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEC(VAR_NAME,MSG_TYPE,ROSTOPIC_DEFAULT,MIN_INTERVAL)\
    std::string  VAR_NAME##_rostopic = ROSTOPIC_DEFAULT;\
    rclcpp::Subscription<MSG_TYPE>::SharedPtr VAR_NAME##_subscription;\
    void                 VAR_NAME##_msg_callback(MSG_TYPE::ConstSharedPtr i##VAR_NAME);\
      MSG_TYPE           VAR_NAME##_buffer;\
      mutable std::mutex VAR_NAME##_buffer_mtx;\
      int64_t            VAR_NAME##_buffer_last_rx = 0;\
      int64_t            VAR_NAME##_buffer_min_gather_interval = MIN_INTERVAL;\
      MSG_TYPE           VAR_NAME() const;\
      bool               VAR_NAME##_config(const std::string& topic, int64_t min_act_interval = -1);\
      std::string        VAR_NAME##_config_print();\
      int64_t            VAR_NAME##_last_rx() const;



#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_SUBCALLBACK_003(CLASS_OWNER,VAR_NAME,MSG_TYPE)\
  void CLASS_OWNER::VAR_NAME##_msg_callback(MSG_TYPE::ConstSharedPtr i##VAR_NAME)\
  {\
    CLOG_DEBUG(7, #CLASS_OWNER " : callback_msg_" #VAR_NAME " : received message of type " #MSG_TYPE);\
    int64_t thisMoment = milla::connapp::getCurrentTimestamp();\
    if((thisMoment - VAR_NAME##_buffer_last_rx) < VAR_NAME##_buffer_min_gather_interval){return;}\
    VAR_NAME##_buffer_last_rx = thisMoment;\
    {\
      std::lock_guard guard(VAR_NAME##_buffer_mtx);\
      VAR_NAME##_buffer = *i##VAR_NAME;\
    }\
  }\
  MSG_TYPE CLASS_OWNER::VAR_NAME() const{\
    MSG_TYPE buffer;\
    {\
      std::lock_guard guard(VAR_NAME##_buffer_mtx);\
      buffer = VAR_NAME##_buffer;\
    }\
    return buffer;\
  }\
  bool CLASS_OWNER::VAR_NAME##_config(const std::string& topic, int64_t min_act_interval){\
    VAR_NAME##_rostopic = topic;\
    VAR_NAME##_buffer_min_gather_interval = (min_act_interval<0?VAR_NAME##_buffer_min_gather_interval:min_act_interval);\
    return ((!topic.empty()) && (VAR_NAME##_buffer_min_gather_interval>=0));\
  }\
  int64_t CLASS_OWNER::VAR_NAME##_last_rx() const{\
   return VAR_NAME##_buffer_last_rx;}\
  std::string CLASS_OWNER::VAR_NAME##_config_print(){\
    std::stringstream sstr;\
    sstr << #VAR_NAME ":topic=" << VAR_NAME##_rostopic << ",min-rx-ival=" << VAR_NAME##_buffer_min_gather_interval;\
    return sstr.str();\
  }

#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_SUBCALLBACK_004(CLASS_OWNER,VAR_NAME,MSG_TYPE, CALLBACK_FUNC)\
  void CLASS_OWNER::VAR_NAME##_msg_callback(MSG_TYPE::ConstSharedPtr i##VAR_NAME)\
  {\
    CLOG_DEBUG(7, #CLASS_OWNER " : callback_msg_" #VAR_NAME " : received message of type " #MSG_TYPE);\
    int64_t thisMoment = milla::connapp::getCurrentTimestamp();\
    if((thisMoment - VAR_NAME##_buffer_last_rx) < VAR_NAME##_buffer_min_gather_interval){return;}\
    VAR_NAME##_buffer_last_rx = thisMoment;\
    {\
      std::lock_guard guard(VAR_NAME##_buffer_mtx);\
      VAR_NAME##_buffer = *i##VAR_NAME;\
    }\
    CALLBACK_FUNC ;\
  }\
  MSG_TYPE CLASS_OWNER::VAR_NAME() const{\
    MSG_TYPE buffer;\
    {\
      std::lock_guard guard(VAR_NAME##_buffer_mtx);\
      buffer = VAR_NAME##_buffer;\
    }\
    return buffer;\
  }\
  bool CLASS_OWNER::VAR_NAME##_config(const std::string& topic, int64_t min_act_interval){\
    VAR_NAME##_rostopic = topic;\
    VAR_NAME##_buffer_min_gather_interval = (min_act_interval<0?VAR_NAME##_buffer_min_gather_interval:min_act_interval);\
    return ((!topic.empty()) && (VAR_NAME##_buffer_min_gather_interval>=0));\
  }\
  int64_t CLASS_OWNER::VAR_NAME##_last_rx() const{\
   return VAR_NAME##_buffer_last_rx;}\
  std::string CLASS_OWNER::VAR_NAME##_config_print(){\
    std::stringstream sstr;\
    sstr << #VAR_NAME ":topic=" << VAR_NAME##_rostopic << ",min-rx-ival=" << VAR_NAME##_buffer_min_gather_interval;\
    return sstr.str();\
  }


#define cyt3macro_helper_rostopicsub_get_macro(_1,_2,_3,_4, NAME, ...) NAME


/**
 * @brief to be used at the .cpp, defines the functions for a declared ros2-subscription.
 * @param _1 : REQUIRED : name of the class where the subscription is declared
 * @param _2 : REQUIRED : name of the subscription
 * @param _3 : REQUIRED : ros-message-type
 * @param _4 : OPTIONAL : callback that will be invoked... recommended type void()
 */
#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_SUBCALLBACK(...) cyt3macro_helper_rostopicsub_get_macro(__VA_ARGS__, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_SUBCALLBACK_004, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_SUBCALLBACK_003 ) (__VA_ARGS__)





#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION(CLASS_OWNER,VAR_NAME,MSG_TYPE)\
  {\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME);\
    VAR_NAME##_subscription = create_subscription<MSG_TYPE>(VAR_NAME##_rostopic,\
                                                10,\
                                                std::bind(&CLASS_OWNER::VAR_NAME##_msg_callback,\
                                                this,\
                                                std::placeholders::_1));\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "::callback_msg_" #VAR_NAME ": DONE");\
  }
  
#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_EXT(CLASS_OWNER,VAR_NAME,MSG_TYPE,ROS_NODE_PTR)\
  {\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME);\
    VAR_NAME##_subscription = ROS_NODE_PTR->create_subscription<MSG_TYPE>(VAR_NAME##_rostopic,\
                                                10,\
                                                std::bind(&CLASS_OWNER::VAR_NAME##_msg_callback,\
                                                this,\
                                                std::placeholders::_1));\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "::callback_msg_" #VAR_NAME ": DONE");\
  }

#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_MAKEREG(...) \
  cyt3macro_helper_rostopicsub_get_macro(__VA_ARGS__, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_EXT, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION)(__VA_ARGS__)






#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_TRANSIENT(CLASS_OWNER,VAR_NAME,MSG_TYPE)\
  {\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME);\
    VAR_NAME##_subscription = create_subscription<MSG_TYPE>(VAR_NAME##_rostopic,\
                                                rclcpp::QoS{1}.transient_local(),\
                                                std::bind(&CLASS_OWNER::VAR_NAME##_msg_callback,this,std::placeholders::_1));\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME ": DONE");\
  }

#define CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_TRANSIENT_EXT(CLASS_OWNER,VAR_NAME,MSG_TYPE,ROS_NODE_PTR)\
  {\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME);\
    VAR_NAME##_subscription = ROS_NODE_PTR->create_subscription<MSG_TYPE>(VAR_NAME##_rostopic,\
                                                rclcpp::QoS{1}.transient_local(),\
                                                std::bind(&CLASS_OWNER::VAR_NAME##_msg_callback,this,std::placeholders::_1));\
    CLOG_INFO("ros2-node-interface-base : macro-define-subscription : <" #MSG_TYPE ">" #VAR_NAME " subscribing to [" << VAR_NAME##_rostopic << "] for callback to " #CLASS_OWNER "callback_msg_" #VAR_NAME ": DONE");\
  }

#define CYT3MACRO_ROS2_rostopic_subscriber_def_makereg_transient(...) \
  cyt3macro_helper_rostopicsub_get_macro(__VA_ARGS__, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_TRANSIENT_EXT, CYT3MACRO_ROS2_ROSTOPIC_SUBSCRIBER_DEF_REGISTRATION_TRANSIENT)(__VA_ARGS__)



// MACROS : SUBSCRIBERS : END

// MACROS : PUBLISHERS : BEGIN


/**
 * @brief : creates
 * std::string                                <var_name>_rostopic
 * rclcpp::Publisher<msg_type>::SharedPtr     <var_name>_publisher
 * bool                                       <var_name>_publish(const <msg_type>& p<var_name>) 
 * std::mutex                                 <var_name>_publisher_mtx
 * 
*/
#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_DEC(VAR_NAME,MSG_TYPE,ROSTOPIC_DEFAULT)\
    std::string VAR_NAME##_rostopic = ROSTOPIC_DEFAULT;\
    rclcpp::Publisher<MSG_TYPE>::SharedPtr VAR_NAME##_publisher;\
    bool VAR_NAME##_publish(const MSG_TYPE & p##VAR_NAME);\
    std::mutex VAR_NAME##_publisher_mtx;\
    int64_t VAR_NAME##_min_publish_interval = 0;\
    int64_t VAR_NAME##_last_publication_ts = 0;\
    bool VAR_NAME##_config(const std::string& topic, int64_t min_act_interval = -1);

#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_DEF_REGISTRATION(VAR_NAME,MSG_TYPE)\
    {\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising");\
      VAR_NAME##_publisher = create_publisher<MSG_TYPE>(VAR_NAME##_rostopic,10);\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising : DONE");\
    }

#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_DEF_REGISTRATION_EXT(VAR_NAME,MSG_TYPE,ROS_NODE_PTR)\
    {\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising");\
      VAR_NAME##_publisher = ROS_NODE_PTR->create_publisher<MSG_TYPE>(VAR_NAME##_rostopic,10);\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising : DONE");\
    }

#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_DEF_REGISTRATION_FOR_TRANSIENT(VAR_NAME,MSG_TYPE)\
    {\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising");\
      rclcpp::QoS _qos =rclcpp::QoS(1);\
      _qos.transient_local();\
      VAR_NAME##_publisher = create_publisher<MSG_TYPE>(VAR_NAME##_rostopic,_qos);\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising : DONE");\
    }

#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_DEF_REGISTRATION_FOR_TRANSIENT_EXT(VAR_NAME,MSG_TYPE,ROS_NODE_PTR)\
    {\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising");\
      rclcpp::QoS _qos =rclcpp::QoS(1);\
      _qos.transient_local();\
      VAR_NAME##_publisher = ROS_NODE_PTR->create_publisher<MSG_TYPE>(VAR_NAME##_rostopic,_qos);\
      CLOG_INFO("ros2-node-interface-base : macro : rostopic-publisher-def : <" #MSG_TYPE ">" #VAR_NAME " : advertising : DONE");\
    }
// #define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_FUNCDEF(CLASS_OWNER,VAR_NAME,MSG_TYPE) 
//     bool CLASS_OWNER::VAR_NAME##_publish(const MSG_TYPE & p##VAR_NAME ){ 
//       CLOG_DEBUG(6, #CLASS_OWNER " : publisher- publishing <" #MSG_TYPE "> : doing"); 
//       std::lock_guard<std::mutex> guard(VAR_NAME##_publisher_mtx); 
//       VAR_NAME##_publisher->publish(p##VAR_NAME); 
//       CLOG_DEBUG(5, #CLASS_OWNER " : publisher- publishing <" #MSG_TYPE "> : DONE"); 
//     }
#define CYT3MACRO_ROS2_ROSTOPIC_PUBLISHER_FUNCDEF(CLASS_OWNER,VAR_NAME,MSG_TYPE)\
    bool CLASS_OWNER::VAR_NAME##_publish(const MSG_TYPE & p##VAR_NAME ){\
      CLOG_DEBUG(7, #CLASS_OWNER " : publisher- publishing <" #MSG_TYPE "> : doing");\
      std::lock_guard<std::mutex> guard(VAR_NAME##_publisher_mtx);\
      int64_t thisMoment = milla::connapp::get_current_timestamp();\
      if((thisMoment - VAR_NAME##_last_publication_ts) < VAR_NAME##_min_publish_interval)return false;\
      VAR_NAME##_last_publication_ts = thisMoment;\
      VAR_NAME##_publisher->publish(p##VAR_NAME);\
      CLOG_DEBUG(8, #CLASS_OWNER " : publisher- publishing <" #MSG_TYPE "> : DONE");\
      return true;\
    }\
    bool CLASS_OWNER::VAR_NAME##_config(const std::string& topic, int64_t min_act_interval){\
      VAR_NAME##_rostopic = topic;\
      if(min_act_interval >= 0){\
        VAR_NAME##_min_publish_interval = min_act_interval;\
      }\
      return ((!topic.empty()) && (min_act_interval >= 0));\
    }
// MACROS : PUBLISHERS : END


// MACROS : SERVICE : BEGIN
#define CYT3MACRO_ROS2_SERVICE_DECLARATION(V_NAME,V_TYPE)\
  std::string             V_NAME##_service_rosroute ;\
  rclcpp::Service<V_TYPE> V_NAME##_service ;\
  void V_NAME##_service_callback(const V_TYPE::Request::SharedPtr req, const V_TYPE::Response::SharedPtr res); 


#define CYT3MACRO_ROS2_SERVICE_DEF_REGISTRATION(V_CLASS,V_NAME,V_TYPE)\
  CLOG_INFO(#V_CLASS " :: cyt3macro-ros2-service-def-registration : "\
  "registering service [" #V_NAME "] of type [" #V_TYPE "] at route "\
  << V_NAME##_service_rosroute << "to invoke [" #V_CLASS\
  "::" #V_NAME "_service_callback()]");\
  V_NAME##_service = create_service<V_TYPE>(\
    V_NAME##_service_rosroute\
    ,std::bind(& V_CLASS::V_NAME##_service_callback\
              , this\
              , std::placeholders::_1\
              , std::placeholders::_2));

#define CYT3MACRO_ROS2_SERVICE_DEF_REGISTRATION_EXT(V_CLASS,V_NAME,V_TYPE,V_ROSNODEPTR)\
  CLOG_INFO(#V_CLASS " :: cyt3macro-ros2-service-def-registration : "\
  "registering service [" #V_NAME "] of type [" #V_TYPE "] at route "\
  << V_NAME##_service_rosroute << "to invoke [" #V_CLASS\
  "::" #V_NAME "_service_callback()]");\
  V_NAME##_service = V_ROSNODEPTR->create_service<V_TYPE>(\
    V_NAME##_service_rosroute\
    ,std::bind(& V_CLASS::V_NAME##_service_callback\
              , this\
              , std::placeholders::_1\
              , std::placeholders::_2));

// MACROS : SERVICE : END


// MACROS : CLIENT : BEGIN

/**
 * @brief : needs 2 params: client-name and service-type. Declares
 *  std::string                 <client-name>_client_rosroute
 * <v_type>::Client::SharedPtr  <client-name>_client
 * bool                         <client-name>_client_sender(<service-type>::Request::SharedPtr req = *<service-type>::Request() )
 * bool                         <client-name>_client_on_response(const <service-type>::Response::SharedPtr res)
 *     NOTA :                   <client-name>_client_on_response MUST BE DEFINED BY THE USER
 */  
#define CYT3MACRO_ROS2_CLIENT_DECLARATION(V_NAME,V_TYPE,V_DEFAULT_ROUTE)\
  std::string                           V_NAME##_client_rosroute = V_DEFAULT_ROUTE;\
  rclcpp::Client<V_TYPE>::SharedPtr     V_NAME##_client;\
  bool                                  V_NAME##_client_sender(\
                                          const V_TYPE::Request::SharedPtr& req\
                                          = std::make_shared<V_TYPE::Request>() );\
  bool                                  V_NAME##_client_on_response(const V_TYPE::Response::SharedPtr res);

#define CYT3MACRO_ROS2_CLIENT_DEF_REGISTRATION(V_CLASS,V_NAME,V_TYPE)\
  CLOG_INFO(#V_CLASS " :: cyt3macro-ros2-client-def-registration : registering "\
  "client [" #V_NAME "] of type [" #V_TYPE "] at route" << V_NAME##_client_rosroute);\
  V_NAME##_client = create_client<V_TYPE>(V_NAME##_client_rosroute); 

#define CYT3MACRO_ROS2_CLIENT_DEF_REGISTRATION_EXT(V_CLASS,V_NAME,V_TYPE,V_ROSNODEPTR)\
  CLOG_INFO(#V_CLASS " :: cyt3macro-ros2-client-def-registration : registering "\
  "client [" #V_NAME "] of type [" #V_TYPE "] at route" << V_NAME##_client_rosroute);\
  V_NAME##_client = V_ROSNODEPTR->create_client<V_TYPE>(V_NAME##_client_rosroute); 


#define CYT3MACRO_ROS2_CLIENT_DEF_CALLBACK_HEAD(V_CLASS,V_NAME,V_TYPE)\
  bool V_CLASS::V_NAME##_client_on_response(const V_TYPE::Response::SharedPtr res){\
  CLOG_INFO(#V_CLASS " :  client_" #V_NAME "_on_response(const " #V_TYPE "::Response::SharedPtr)");
  


/**
 * @brief 
 */
#define CYT3MACRO_ROS2_CLIENT_DEF_SENDER(V_CLASS,V_NAME,V_TYPE)\
  bool V_CLASS::V_NAME##_client_sender(const V_TYPE::Request::SharedPtr& req){\
    CLOG_INFO(#V_CLASS " : client-" #V_NAME "-sender : sending message");\
    if(V_NAME##_client->service_is_ready() == false){\
      CLOG_WARN(#V_CLASS " : client-" #V_NAME "-sender : service is NOT ready!");\
      return false;\
    }\
    V_NAME##_client->async_send_request(req\
      ,[&](rclcpp::Client<V_TYPE>::SharedFuture future){\
        CLOG_INFO(#V_CLASS " : client-" #V_NAME "-sender : received response : invoking callback");\
        V_NAME##_client_on_response(future.get());\
      });\
    CLOG_INFO(#V_CLASS " : client-" #V_NAME "-sender : sending message [ OK ]");\
    return true;\
  }

// MACROS : CLIENT : END

namespace milla{
namespace connapp{
namespace rostools{

/** XmlRpc::XmlRpcValue is a very exigent class... it has not implemented many overloads 
 * 
 * if you need to recover from an int or uint, you will need to use an intermediary bufer for the destination
 */
#define ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(VAR_TYPE)\
        bool getParamFromXmlRpcSource( ::XmlRpc::XmlRpcValue source, const ::std::string& member_name, VAR_TYPE & destination);

ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(int);
ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(double);
ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(::std::string);
ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(bool);
//ROSGADGET_ROSXMLRPCPARAM_LOAD_DEC(XmlRpc::XmlRpcValue);


#define ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(VAR_TYPE_DESTINATION)\
        bool get_param_from_xmlrpc(::XmlRpc::XmlRpcValue source, const ::std::string& member_name,VAR_TYPE_DESTINATION & destination);

ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(int8_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(int16_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(int32_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(int64_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(uint8_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(uint16_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(uint32_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(uint64_t);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(float);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(double);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(bool);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(::std::string);
ROSGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEC(::XmlRpc::XmlRpcValue);




template <typename T>
bool obtain_yaml_nested_param(XmlRpc::XmlRpcValue source,const std::string& member_name,T& destination)
{
    std::string splitted;
    //CLOG_INFO("ROSDEV TOOL : start search " << member_name);
    //draw_xmlrpcpp(source);
    if(member_name[0] == '/')
    {
        return obtain_yaml_nested_param(source,member_name.substr(1,member_name.size()-2),destination);
    }
    size_t pos = member_name.find_first_of("/");
    CLOG_INFO("ROSDEV TOOL : get yaml nested param : searching [" << member_name << "]");
    if(pos == std::string::npos)
    {
        CLOG_INFO("ROSDEV TOOL : get yaml nested param : invoking specific param from xmlrpc for [" << member_name << "]")
        return get_param_from_xmlrpc(source,member_name,destination);
    }
    std::string current = member_name.substr(0,pos);
    splitted = member_name.substr(pos+1);
    if(!source.hasMember(current))
    {
        CLOG_ERROR("ROSDEV TOOL : get yaml nested param : input[" << member_name << "] not found member [" << current << "]")
        return false;
    }
    XmlRpc::XmlRpcValue& buffer = source[current.c_str()];
    CLOG_INFO("ROSDEV TOOL : yaml nested : recursive for [" << splitted << "]");
    bool result = obtain_yaml_nested_param(buffer,splitted,destination); 
    if(!result)
    {
        CLOG_ERROR("ROSDEV TOOL : get yaml nested param : input[" << member_name <<"] gave no result");
    }
    return result;
}


}
}
}