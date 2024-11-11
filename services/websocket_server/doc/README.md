# LwsServ3r

## INTRODUCTION *and blah*

The best way to communicate with low-level components and give them a GUI is to give it an HTML server. Create a webapp generally needs low level of resources for the server because the main expense is to serve the content "models" (html, css and javascript) while the renderization of those models is done at the client machine.

For the applications I use to develop is necessary to be able easily create a web interface and be able to update the content at the client in real time.

That I created a wrapper for `libwebsockets` to ease me the tasks of:

* Being able to create a web application server at a specific port.
* Easily configure the path for the *static data*, meaning for all the html, css and javascript files of the application.
* Creating routes for *dynamic paths*, meaning, being able to directly reference a c++ callback function when the client makes a REST-GET request. *I may include REST POST requests later*.
* Being able to manage websocket connections.

## HOW TO USE IT

1. Create the server and specify the server port.
2. Set the default page to reference when the client tries to access a non existent route.
3. Create the default `/` static route.
4. Create dynamic paths for REST-GET requests and be able to dynamically generate the content of the response, define the main callback for these requests, and define the timeout for asynchronous response communications.
5. Specify the callbacks for incoming websockets communications.

```cpp
// 1.
coyot3::services::websocket::WebsocketsServerGateway server;
server.setName("anyName");
server.setServerPort(5001);

// 2.
server.setDefaultDoc404("/404.html"); // it will redirect the client to that page

// 3.
server.addStaticPath("/","/path/to/the/content/www/","index.html");

// 4.
server.addDynamicPath("/restSync");


// typedef std::map<std::string,std::string> UriArguments;
//
// bool callback_function(coyot3::services::websocket::WsgClient* client,
//                        const std::string& request,
//                        const coyot3::services::websocket::UriArguments& arguments,
//                        std::string& response);
//
server.setOnDynHttpCallback(callback_function);
server.asyncCommsTimeout(5000);

```


