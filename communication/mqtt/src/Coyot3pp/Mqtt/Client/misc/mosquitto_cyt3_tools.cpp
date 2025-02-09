#include <Coyot3pp/Mqtt/Client/Client.hpp>


namespace coyot3{
namespace communication{
namespace mqtt{

  std::string mosquitto_error_codes_descriptions(int rc){
    switch(rc){
      case MOSQ_ERR_AUTH_CONTINUE: return "(-4)MOSQ_ERR_AUTH_CONTINUE";break;
      case MOSQ_ERR_NO_SUBSCRIBERS: return "(-3)MOSQ_ERR_NO_SUBSCRIBERS";break;
      case MOSQ_ERR_SUB_EXISTS: return "(-2)MOSQ_ERR_SUB_EXISTS";break;
      case MOSQ_ERR_CONN_PENDING: return "(-1)MOSQ_ERR_CONN_PENDING";break;
      case MOSQ_ERR_SUCCESS: return "(0)MOSQ_ERR_SUCCESS";break;
      case MOSQ_ERR_NOMEM: return "(1)MOSQ_ERR_NOMEM";break;
      case MOSQ_ERR_PROTOCOL: return "(2)MOSQ_ERR_PROTOCOL";break;
      case MOSQ_ERR_INVAL: return "(3)MOSQ_ERR_INVAL";break;
      case MOSQ_ERR_NO_CONN: return "(4)MOSQ_ERR_NO_CONN";break;
      case MOSQ_ERR_CONN_REFUSED: return "(5)MOSQ_ERR_CONN_REFUSED";break;
      case MOSQ_ERR_NOT_FOUND: return "(6)MOSQ_ERR_NOT_FOUND";break;
      case MOSQ_ERR_CONN_LOST: return "(7)MOSQ_ERR_CONN_LOST";break;
      case MOSQ_ERR_TLS: return "(8)MOSQ_ERR_TLS";break;
      case MOSQ_ERR_PAYLOAD_SIZE: return "(9)MOSQ_ERR_PAYLOAD_SIZE";break;
      case MOSQ_ERR_NOT_SUPPORTED: return "(10)MOSQ_ERR_NOT_SUPPORTED";break;
      case MOSQ_ERR_AUTH: return "(11)MOSQ_ERR_AUTH";break;
      case MOSQ_ERR_ACL_DENIED: return "(12)MOSQ_ERR_ACL_DENIED";break;
      case MOSQ_ERR_UNKNOWN: return "(13)MOSQ_ERR_UNKNOWN";break;
      case MOSQ_ERR_ERRNO: return "(14)MOSQ_ERR_ERRNO";break;
      case MOSQ_ERR_EAI: return "(15)MOSQ_ERR_EAI";break;
      case MOSQ_ERR_PROXY: return "(16)MOSQ_ERR_PROXY";break;
      case MOSQ_ERR_PLUGIN_DEFER: return "(17)MOSQ_ERR_PLUGIN_DEFER";break;
      case MOSQ_ERR_MALFORMED_UTF8: return "(18)MOSQ_ERR_MALFORMED_UTF8";break;
      case MOSQ_ERR_KEEPALIVE: return "(19)MOSQ_ERR_KEEPALIVE";break;
      case MOSQ_ERR_LOOKUP: return "(20)MOSQ_ERR_LOOKUP";break;
      case MOSQ_ERR_MALFORMED_PACKET: return "(21)MOSQ_ERR_MALFORMED_PACKET";break;
      case MOSQ_ERR_DUPLICATE_PROPERTY: return "(22)MOSQ_ERR_DUPLICATE_PROPERTY";break;
      case MOSQ_ERR_TLS_HANDSHAKE: return "(23)MOSQ_ERR_TLS_HANDSHAKE";break;
      case MOSQ_ERR_QOS_NOT_SUPPORTED: return "(24)MOSQ_ERR_QOS_NOT_SUPPORTED";break;
      case MOSQ_ERR_OVERSIZE_PACKET: return "(25)MOSQ_ERR_OVERSIZE_PACKET";break;
      case MOSQ_ERR_OCSP: return "(26)MOSQ_ERR_OCSP";break;
      default:
        return "unknown-error-code?" + std::to_string(rc);
    }
  }

  std::string mqtt_on_connect_return_code_version_3(int rc){
    switch(rc){
      case 0 : return "0x00 Connection Accepted"; break;
      case 1 : return "0x01 Connection Refused, unacceptable protocol version : The Server does not support the level of the MQTT protocol requested by the Client"; break;
      case 2 : return "0x02 Connection Refused, identifier rejected : The Client identifier is correct UTF-8 but not allowed by the Server"; break;
      case 3 : return "0x03 Connection Refused, Server unavailable : The Network Connection has been made but the MQTT service is unavailable"; break;
      case 4 : return "0x04 Connection Refused, bad user name or password : The data in the user name or password is malformed"; break;
      case 5 : return "0x05 Connection Refused, not authorized : The Client is not authorized to connect"; break;
      default: 
        if(rc >= 6 && rc <=255){
          return std::to_string(rc) + "?6-255 : Reserved for future use";
        }else{
          return std::to_string(rc) + "? UNKNOWN!";
        }
    }
  }

  std::string mqtt_on_connect_return_code_version_5(int rc){
    switch(rc){
      case 0   : return "000 : 0x00 : Success : The Connection is accepted.";break;
      case 128 : return "128 : 0x80 : Unspecified error : The Server does not wish to reveal the reason for the failure, or none of the other Reason Codes apply.";break;
      case 129 : return "129 : 0x81 : Malformed Packet : Data within the CONNECT packet could not be correctly parsed.";break;
      case 130 : return "130 : 0x82 : Protocol Error : Data in the CONNECT packet does not conform to this specification.";break;
      case 131 : return "131 : 0x83 : Implementation specific error : The CONNECT is valid but is not accepted by this Server.";break;
      case 132 : return "132 : 0x84 : Unsupported Protocol Version : The Server does not support the version of the MQTT protocol requested by the Client.";break;
      case 133 : return "133 : 0x85 : Client Identifier not valid : The Client Identifier is a valid string but is not allowed by the Server.";break;
      case 134 : return "134 : 0x86 : Bad User Name or Password : The Server does not accept the User Name or Password specified by the Client";break;
      case 135 : return "135 : 0x87 : Not authorized : The Client is not authorized to connect.";break;
      case 136 : return "136 : 0x88 : Server unavailable : The MQTT Server is not available.";break;
      case 137 : return "137 : 0x89 : Server busy : The Server is busy. Try again later.";break;
      case 138 : return "138 : 0x8A : Banned : This Client has been banned by administrative action. Contact the server administrator.";break;
      case 140 : return "140 : 0x8C : Bad authentication method : The authentication method is not supported or does not match the authentication method currently in use.";break;
      case 144 : return "144 : 0x90 : Topic Name invalid : The Will Topic Name is not malformed, but is not accepted by this Server.";break;
      case 149 : return "149 : 0x95 : Packet too large : The CONNECT packet exceeded the maximum permissible size.";break;
      case 151 : return "151 : 0x97 : Quota exceeded : An implementation or administrative imposed limit has been exceeded.";break;
      case 153 : return "153 : 0x99 : Payload format invalid : The Will Payload does not match the specified Payload Format Indicator.";break;
      case 154 : return "154 : 0x9A : Retain not supported : The Server does not support retained messages, and Will Retain was set to 1.";break;
      case 155 : return "155 : 0x9B : QoS not supported : The Server does not support the QoS set in Will QoS.";break;
      case 156 : return "156 : 0x9C : Use another server : The Client should temporarily use another server.";break;
      case 157 : return "157 : 0x9D : Server moved : The Client should permanently use another server.";break;
      case 159 : return "159 : 0x9F : Connection rate exceeded : The connection rate limit has been exceeded.    ";break;  
      default:
        return std::to_string(rc) + "? unknown-error-code ?";
    }
  }





}
}
}