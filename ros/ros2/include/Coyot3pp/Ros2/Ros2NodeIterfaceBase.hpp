#pragma once

#include <string>
#include <thread>
#include "../tools/cyt_dev_tools.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executor_options.hpp>
#include "Ros2NodeInterfaceHelpers.hpp"

namespace milla{
namespace connapp{
namespace ros2{



class Ros2NodeInterfaceBase : public rclcpp::Node{

  protected:

    static bool                                      ros_initialized;
    static bool                                      ros_spinning;
    static std::string                               ros_node_name;
    static rclcpp::executors::MultiThreadedExecutor* ros_multithreaded_executor;
    static std::thread*                              ros_multithreaded_executor_thread;
    //static ros::NodeHandle* main_ros_node_handle;
    //static ros::NodeHandle* absolute_ros_node_handle;

  public:

    Ros2NodeInterfaceBase(const std::string& node_name);
    virtual ~Ros2NodeInterfaceBase();


    static bool IsRosSpinning();

    
    enum class State{
      CREATED = 0,
      INITIALIZED = 1,
      LAUNCHED = 2,
      STOPPED = 3,
      ERROR = 4
    };

    static const char*            StateToString(State s);
    static bool                   RosInitialize(int argc, char* argv[], const std::string& rosNodeName = "", bool customSigInt = false);
    
    static bool                   MultithreadedExecutorCreate(int number_of_threads = 1);
    static bool                   MultithreadedExecutorAddNode(std::shared_ptr<rclcpp::Node> node);
    static bool                   MultithreadedExecutorSpin();
    static bool                   MultithreadedExecutorShutdown();

    static std_msgs::msg::Header  createHeaderPacket(int32_t seq = 0);
    static std_msgs::msg::Header  createHeaderPacket(const std::string& frameId);
    
    static rclcpp::Time           get_ros_time();
    
    static bool                   GetActiveNodesList(std::vector<std::string>& nodelist);
    static bool                   node_is_active(const std::string& nodeName);

    /**
     * @brief template obtained from the milla-doc. It creates a 
     */
    template <typename T>
    bool client_service_invoke_void(
      const typename rclcpp::Client<T>::SharedPtr client
      ,const typename T::Request::SharedPtr& req = std::make_shared<typename T::Request>())
    {
      CLOG_INFO("ros2-node-interface-base : client-service-invoke-void : "
      "making request [    ]");

      if (!client->service_is_ready()) {
        CLOG_WARN("ros2-node-interface-base : client-service-invoke-void : "
        "service is not available!");
        return false;
      }

      client->async_send_request(req
      , [this](typename rclcpp::Client<T>::SharedFuture result) {
        CLOG_INFO("ros2-node-interface-base : client-service-invoke-void : "
          "response received from server. code:" << result.get()->status.code
          << "; message:" << result.get()->status.message);
      });
      CLOG_INFO("ros2-node-interface-base : client-service-invoke-void : making"
      " request [ OK ]");
      return true;

    }



    bool hasThread();
    bool hasThread(bool state);
    bool thisModuleSpinsRos() const;
    bool thisModuleSpinsRos(bool state);


    /**
     * @brief : to be invoked as late as possible in the highest layer
    */
    virtual bool Init();

    /**
     * @brief : if its multithreaded launch, it will add it to the multithreaded.
     *          ideally to be invoked as late as possible at the highest layer.
    */
    virtual bool Start();

    virtual bool Stop();

    protected:

      bool startRosThread();
      bool stopRosThread();

      std::string  _node_name;
      std::thread* _rosThread               = nullptr;
      bool         _hasThread               = false;
      bool         _doLoop                  = false;
      bool         _doRosSpin               = false;
      bool         _rosThreadInitialized    = false;
      std::shared_ptr<Ros2NodeInterfaceBase> _base_ptr;

      bool         launchRosSpinningThread();
      bool         stopRosSpinningThread();

      State        _state;
    public:

      template<typename T>
      static bool getRosParam(const std::string& key, T& param)
      {
        return false;
      }

      template<typename T>
      static bool getRosNestedParam(const std::string& keySequence, T& param)
      {
        return false;
      }

      size_t    deleteSubscribers();
      size_t    deletePublishers();


      static    bool makeRosSpin(rclcpp::Node::SharedPtr node);
      static    bool stopRosSpin();


      State     state() const;
    
    protected:
      std::map<std::string,rclcpp::SubscriptionBase::SharedPtr>    subscribers_map_;
      std::map<std::string,rclcpp::PublisherBase::SharedPtr>       publishers_map_;
      std::map<std::string,rclcpp::ServiceBase::SharedPtr>         service_servers_map_;
      std::map<std::string,rclcpp::ClientBase::SharedPtr>          service_clients_map_;

    




};

}
}
}