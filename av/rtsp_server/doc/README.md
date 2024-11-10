# H264RtspServer

A simple way to create an h264 rtsp server and forget.


## INTRODUCTION and blablah

The need is very clear: I need to be able to create a ultra-compatible RTSP server with minimal latency, and be able to manage the content independently.

It is very difficult (at least for me) to keep in mind all what is needed to be able to create an RTSP server using `gstream`. In fact, using ANY server. I did not even yet tried using Qt to do this. So I really needed to create my component that I could reuse any time. We need to take in count that one server can handle many streams, and each streams will have its image source and its clients.

I started using it from a ROS1 source, and it was not until much time later that I learned that it was possible to include an OpenCV source. Honestly I had a lot of headaches trying to understand the gstream components... until I realized of one thing: **There is no shortcut when dealing with this, because gstreamer is an universe by itself.**

So the best way to deal with this was to create a component that made the work to be able to create the most-possible-compatible stream, and be able to deal with the content in other modules.

When dealing with this kind of service, I needed to be able to easily configure, where the configuration was to inform about the frame rate, and the dimensions of the final image.

## H264RtspServer component.

When creating an RTSP server it is needed to deal with the following steps:

1. Configure the image sources
2. Instantiate the rtsp server
3. Configure each stream and create at the rtsp server
4. Link each created stream to an image source
5. Ensure to have `Init` and `Start` the sources.
6. `Init` and `Start` the rtsp server.


```cpp

  #include <Coyot3pp/H264RtspServer/H264RtspServer.hpp>

  int main(int argc,char** argv){
    coyot3::av::image::ImageContentDerivedClass source;
    //configure the source and start them
    source.Init();
    source.Start();

    // 2. instantiation of the rtsp server
    coyot3::av::rtsp::RtspH264Server server("name-of-server");
    coyot3::av::rtsp::RtspServerConfig serverConf;
    serverConf.server_port(8554);
    
    server.set_configuration(serverConf);
    
    // 3. configuration of the streams
    coyot3::av::rtsp::RtspStreamConfig streamConf;
    streamConf.stream_name("name-of-stream");
    streamConf.path("/path_of_stream");
    streamConf.is_active(true);
    streamConf.show_debug_preview(false); //for the moment I have a bug with true
    streamConf.width(800);streamConf.height(600);
    streamConf.update_frequece(15);
    streamConf.gst_pipeline_chain().push_back(
      coyot3::av::rtsp::RtspH264Server::pipeline_preset_ull //preset for ultra-low-latency
    );
    server.stream_create(streamConf);
    server.stream_get_by_name("name-of-stream")->image_source_set(source);
    
    server.Init();
    server.Start();
  }

```

And the source will be available at `rtsp://server-uri:8554/path_of_stream`
