#pragma once

#include <string>
#include <libwebsockets.h>


namespace coyot3{
namespace services{
namespace websocket{

  
class WebsocketsServerGateway;

struct WsgConnectionStructure{
  std::string                                   path;
  WebsocketsServerGateway*                      gateway_instance;

  size_t                                        response_total_length;
  size_t                                        response_current_index;

  std::string                                   response;
  std::string                                   source_peer;
  std::string                                   mount_point;
  
  std::basic_string<uint8_t>                    ws_input_stream;
  int64_t                                       ws_message_last_rx_ts;

  uint8_t*                                      ws_output_stream;
  size_t                                        ws_output_stream_size;
  size_t                                        ws_output_stream_index;

  int64_t                                       connection_ts;
  
  int                                           ref_id;
  lws*                                          wsi;
};


}
}//eons wrappers
}//eons coyot3