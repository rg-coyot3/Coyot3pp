


#include <milla_connect_tools/ros2_node_interface/Ros2NodeInterfaceHelpers.hpp>
#include <milla_connect_tools/tools/cyt_simple_logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace milla{


#if CY_LOGGER_WRAPPER_MODEL == CY_LOGGER_WRAPPER_FOR_ROS2

  namespace connapp{
  namespace logger{
    
    SimpleLoggerClass::~SimpleLoggerClass()
    {

      switch(t)
      {
        case SimpleLoggerClass::LogLineType::INFO:
          RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),slcstream.str());
          break;
        case SimpleLoggerClass::LogLineType::WARNING:
          RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),slcstream.str());
          break;
        case SimpleLoggerClass::LogLineType::ERROR:
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),slcstream.str());
          break;
        case SimpleLoggerClass::LogLineType::DEBUG:
          RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
            __SIMPLELOGGER_COLOR_CYAN_ 
            + slcstream.str() 
            + __SIMPLELOGGER_STDOUTCOLORS_RESET_);
          break;
      }
    }


  }// eons logger
  }
#endif

namespace connapp{
namespace rostools{

#define CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(VAR_TYPE) \
        bool getParamFromXmlRpcSource(XmlRpc::XmlRpcValue source,const ::std::string& member_name, VAR_TYPE & destination)\
        {\
            bool hasParam; \
            hasParam = source.hasMember( member_name.c_str() ); \
            if(!hasParam)\
            {\
                CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [" \
                    #VAR_TYPE "] :: ERROR loading param [ " << member_name << " ]"); \
                return false;  \
            } \
            VAR_TYPE & buffer = source[ member_name.c_str() ]; \
            try{ \
            destination = buffer;\
            }catch(const XmlRpc::XmlRpcException& err){\
              CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [" \
                #VAR_TYPE "] :: catched exception loading param [" << member_name \
                << "] : (" << err.getMessage() << ")");\
                return false;\
            }catch(...){\
              CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [" \
                #VAR_TYPE "] :: catched exception loading param [" << member_name \
                << "] : (unhandled except)");\
              return false;\
            }\
            CLOG_DEBUG(7,"CYTGadget::getParamFromXmlRpcSource :: [" #VAR_TYPE \
                "] :: loaded parameter [" << member_name.c_str() << "] correctly, with content [" << \
                destination << "]"); \
            return true; \
        }

CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(int);
CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(double);
CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(::std::string)
CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(bool);
//CYTGADGET_ROSXMLRPCPARAM_LOAD_DEF(XmlRpc::XmlRpcValue);




bool getParamFromXmlRpcSource(XmlRpc::XmlRpcValue source,const ::std::string& member_name, XmlRpc::XmlRpcValue& destination)
{
    bool hasParam;
    hasParam = source.hasMember( member_name.c_str() );
    if(!hasParam)
    {
        CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [XmlRpcValue] :: ERROR loading param [ " << member_name.c_str() << " ]");
        return false; 
    }
    try{
    XmlRpc::XmlRpcValue& buffer = source[member_name.c_str()];
    destination = buffer;
    }catch(XmlRpc::XmlRpcException e)
    {
      CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [XmlRpcValue] :: Error "
        "obtaining param [ " << member_name << "]:: exception raised : ("
        << e.getMessage() << ")");
      return false;
    }catch(...)
    {
      CLOG_ERROR("CYTGadget::getParamFromXmlRpcSource :: [XmlRpcValue] :: Error "
        "obtaining param [ " << member_name << "]:: exception raised : (unhandled except)");
        return false;
    }
    
    CLOG_DEBUG(6,"CYTGadget::getParamFromXmlRpcSource :: [XmlRpcValue] :: loaded parameter [" << member_name.c_str() << "] correctly, with content [" <<
        destination << "]");
    return true;
}

#define CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(VAR_TYPE_SOURCE,VAR_TYPE_DESTINATION) \
    bool get_param_from_xmlrpc(XmlRpc::XmlRpcValue source, \
                                        const std::string& member_name, \
                                        VAR_TYPE_DESTINATION & destination) \
    {                                                               \
        VAR_TYPE_SOURCE buffer;                                     \
        bool result;                                                \
        CLOG_DEBUG(6,"CYTDEVTool : param from xmlrpc : " #VAR_TYPE_SOURCE " : " #VAR_TYPE_DESTINATION " : searching [" << member_name <<"]"); \
        result = getParamFromXmlRpcSource(source,member_name,buffer);    \
        if(result == true)                                            \
        {                                                             \
            try{                                                      \
              destination = static_cast<VAR_TYPE_DESTINATION>(buffer);\
            }catch(const XmlRpc::XmlRpcException& err){\
              CLOG_ERROR("CYTDEVTool : param from xmlrpc : " #VAR_TYPE_SOURCE \
                " : " #VAR_TYPE_DESTINATION " : xmlrpc exception raised! : " \
                << err.getMessage());\
              return false;\
            }catch(...){\
              CLOG_ERROR("CYTDEVTool : param from xmlrpc : " #VAR_TYPE_SOURCE \
                " : " #VAR_TYPE_DESTINATION " : unhandled exception raised!");\
              return false; \
            }                  \
        }                                                             \
        return result;                                                \
    }

CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,int8_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,int16_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,int32_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,int64_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,uint8_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,uint16_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,uint32_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(int,uint64_t);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(double,float);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(double,double);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(bool,bool);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(::std::string,::std::string);
CYTGADGET_GET_PARAM_FROM_YAML_REQUIRED_DEF(::XmlRpc::XmlRpcValue,::XmlRpc::XmlRpcValue);






}
}
}