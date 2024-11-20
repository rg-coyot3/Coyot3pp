#include <milla_connect_tools/ros2_node_interface/Ros2NodeInterfaceBase.h>



namespace milla{
namespace connapp{
namespace ros2{

  bool                                      
  Ros2NodeInterfaceBase::ros_initialized  = false;

  bool                                      
  Ros2NodeInterfaceBase::ros_spinning     = false;

  std::string                               
  Ros2NodeInterfaceBase::ros_node_name    = "";

  rclcpp::executors::MultiThreadedExecutor* 
  Ros2NodeInterfaceBase::ros_multithreaded_executor = nullptr;

  std::thread*
  Ros2NodeInterfaceBase::ros_multithreaded_executor_thread = nullptr;

  const char* Ros2NodeInterfaceBase::StateToString(State s){
    switch(s){
      case Ros2NodeInterfaceBase::State::CREATED: return "CREATED";break;
      case Ros2NodeInterfaceBase::State::INITIALIZED: return "INITIALIZED";break;
      case Ros2NodeInterfaceBase::State::LAUNCHED: return "LAUNCHED";break;
      case Ros2NodeInterfaceBase::State::STOPPED: return "STOPPED";break;
      case Ros2NodeInterfaceBase::State::ERROR: return "ERROR";break;
      default : 
        return "err_state_enum_error";
    }
  }


  
  bool Ros2NodeInterfaceBase::RosInitialize(int argc, char* argv[], const std::string& rosNodeName, bool customSigInt)
  {
    if(Ros2NodeInterfaceBase::ros_initialized == true){
      CLOG_WARN("ros2-node-interface-base : STATIC : ros-initialize : ROS2 has "
      "already been initialized. Ignoring this invokation");
      return false;
    }
    rclcpp::init(argc,argv);
    Ros2NodeInterfaceBase::ros_initialized = true;
    CLOG_INFO("ros2-node-interface-base : ros-initialize : initialize DONE");
    return true;
  }

  std_msgs::msg::Header
  Ros2NodeInterfaceBase::createHeaderPacket(int32_t seq){
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(static_cast<uint32_t>(milla::connapp::getCurrentTimestamp()));
    if(seq){
      h.frame_id = std::to_string(seq);
    }
    return h;
  }

  std_msgs::msg::Header
  Ros2NodeInterfaceBase::createHeaderPacket(const std::string& frameId){
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(static_cast<uint32_t>(milla::connapp::getCurrentTimestamp()));
    h.frame_id = frameId;
    return h;
  }

  rclcpp::Time Ros2NodeInterfaceBase::get_ros_time(){
    return rclcpp::Clock{RCL_ROS_TIME}.now();
  }


  bool Ros2NodeInterfaceBase::GetActiveNodesList(std::vector<std::string>& nodelist){
    CLOG_WARN("ros2-node-interface-base : get-active-nodes-list : NOT YET IMPLEMENTED")
    return false;
  }

  bool Ros2NodeInterfaceBase::node_is_active(const std::string& nodeName){
    CLOG_WARN("ros2-node-interface-base : node-is-active : to be implemented")
    return false;
  }


  bool Ros2NodeInterfaceBase::MultithreadedExecutorCreate(int number_of_threads){
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor != nullptr){
      CLOG_WARN("ros2-node-interface-base : statics : create-multithreaded-executor : "
      "the multithreaded executor is already created. Ignoring this action")
      return true;
    }
    CLOG_INFO("ros2-node-interface-base : statics : create-multithreaded-executor : "
    "creating multithreaded executor holding [" << number_of_threads << "] thread(s)")
    ros_multithreaded_executor = 
      new(std::nothrow) rclcpp::executors::MultiThreadedExecutor(
        rclcpp::ExecutorOptions()
      ,number_of_threads);
    if(ros_multithreaded_executor == nullptr){
      CLOG_ERROR("ros2-node-interface-base : statics : create-multithreaded-executor :"
      "ERROR (FATAL) ALLOCATING MTH-EXECUTOR. NO MEMORY?")
      return false;
    }
    CLOG_INFO("ros2-node-interface-base : statics : create-multithreaded-executor : "
    "creating multithreaded executor holding [" << number_of_threads << "] thread(s) : DONE OK");
    return true;
  }

  bool 
  Ros2NodeInterfaceBase::MultithreadedExecutorAddNode(std::shared_ptr<rclcpp::Node> node){
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor == nullptr){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-add-node"
      " : multithreaded executor is not created");
      return false;
    }
    if(Ros2NodeInterfaceBase::ros_spinning){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-add-node"
      " : multithreaded executor already initialized");
      return false;
    }
    Ros2NodeInterfaceBase::ros_multithreaded_executor->add_node(node);
    CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-add-node"
    " : added node [" << node->get_name() << "]");
    return true;
  }

  bool 
  Ros2NodeInterfaceBase::MultithreadedExecutorSpin(){
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor == nullptr){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-spin"
      " : multithreaded executor is not created");
      return false;
    }
    if(Ros2NodeInterfaceBase::ros_spinning){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-spin"
      " : multithreaded executor already initialized");
      return false;
    }
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor_thread != nullptr){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-thread"
      " : is already created! MUST NOT be initialized");
    }
    CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-spin"
      " : Creating multithreaded-spinner thread");
    Ros2NodeInterfaceBase::ros_multithreaded_executor_thread = 
      new std::thread(std::bind(
        &rclcpp::executors::MultiThreadedExecutor::spin
        ,Ros2NodeInterfaceBase::ros_multithreaded_executor));
    // detaching has no use for these nodes.
    // CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-spin"
    //   " : Creating multithreaded-spinner thread : DONE : detaching");
    // Ros2NodeInterfaceBase::ros_multithreaded_executor_thread->detach();
    // CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-spin"
    //   " : Creating multithreaded-spinner thread : DONE : detaching : DONE");
    return true;
  } 

  bool
  Ros2NodeInterfaceBase::MultithreadedExecutorShutdown(){
    if(Ros2NodeInterfaceBase::ros_spinning == false){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-shutdown : "
      "seems not to be spinning");
    }
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor_thread == nullptr){
      CLOG_WARN("ros2-node-interface-base : statics : multithreaded-executor-shutdown : "
      "seems not to have multithreaded spinner");
    }
    CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-shutdown");
    rclcpp::shutdown();
    CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-shutdown : DONE");
    if(Ros2NodeInterfaceBase::ros_multithreaded_executor_thread != nullptr){
      CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-shutdown : "
      "with multithreaded spinner, waiting and removing...");
      Ros2NodeInterfaceBase::ros_multithreaded_executor_thread->join();
      delete Ros2NodeInterfaceBase::ros_multithreaded_executor_thread;
      Ros2NodeInterfaceBase::ros_multithreaded_executor_thread = nullptr;
      CLOG_INFO("ros2-node-interface-base : statics : multithreaded-executor-shutdown : "
      "with multithreaded spinner, waiting and removing... DONE");
    }
    return true;
  }

  bool Ros2NodeInterfaceBase::IsRosSpinning(){
    return ros_spinning;
  }
  /**
   * @brief constructor
  */
  Ros2NodeInterfaceBase::Ros2NodeInterfaceBase(const std::string& node_name)
  :Node(node_name)
  ,_node_name(node_name)
  {
    _state = State::CREATED;
    CLOG_INFO("ros-node-interface-base : constructor : (" << _node_name << ")");
  }

  Ros2NodeInterfaceBase::~Ros2NodeInterfaceBase(){
    CLOG_WARN("ros-node-interface-base : destructor : (" << _node_name << ")");
  }

  bool Ros2NodeInterfaceBase::hasThread(){
    return _hasThread;
  }
  bool Ros2NodeInterfaceBase::hasThread(bool state){
    CLOG_INFO("ros2-node-interface-base : has-thread : setting [" << (state == true?"true":"false") << "]");
    return (_hasThread = state);
  }

  bool Ros2NodeInterfaceBase::thisModuleSpinsRos() const{
    return _doRosSpin;
  }
  bool Ros2NodeInterfaceBase::thisModuleSpinsRos(bool state){
    return (_doRosSpin = state);
  }

  bool Ros2NodeInterfaceBase::startRosThread(){
    CLOG_WARN("ros2-node-interface-base : start-ros-thread : not-implemented");
    return false;
  }
  bool Ros2NodeInterfaceBase::stopRosThread(){
    CLOG_WARN("ros2-node-interface-base : stop-ros-thread : not-implemented");
    return false;
  }


  bool Ros2NodeInterfaceBase::Init(){
    CLOG_INFO("ros2-node-interface-base : init : begin");
    if(_state != State::CREATED){
      CLOG_WARN("ros2-node-interface-base : init : node state is NOT [" 
      << State::CREATED << "] : current-state(" << _state << ")");
      return false;
    }
    if(hasThread() == true){
      CLOG_INFO("ros2-node-interface-base : init : this node will run in other "
      "thread.");
      if(Ros2NodeInterfaceBase::MultithreadedExecutorCreate() == false){
        CLOG_ERROR("ros2-node-interface-base : init : ERROR initializing"
        " multithreaded executor. Initialization will not continue");
        _state = State::ERROR;
        return false;
      }else{
        CLOG_INFO("ros2-node-interface-base : init : multithreaded-executor created");
      }
      _base_ptr = std::shared_ptr<Ros2NodeInterfaceBase>(this);
      if(Ros2NodeInterfaceBase::MultithreadedExecutorAddNode(_base_ptr) == false)
      {
        CLOG_ERROR("ros2-node-interface-base : init : ERROR : "
        "impossible to add this node");
        _state = State::ERROR;
        return false;
      }else{
        CLOG_INFO("ros2-node-interface-base : init : added node to multithreaded-executor");
      }
    }else{
      CLOG_INFO("ros2-node-interface-base : init : node does not run under an additional thread");
    }
    _state = State::INITIALIZED;
    CLOG_INFO("ros2-node-interface-base : init : DONE [" << _state << "]");
    return true;
  }

  bool Ros2NodeInterfaceBase::Start(){
    if(_state != State::INITIALIZED)
    {
      CLOG_WARN("ros2-node-interface-base : start : node state is NOT [" 
      << State::INITIALIZED << "] : current-state(" << _state << ")");
      return false;
    }

    if(hasThread()){
      CLOG_INFO("ros2-node-interface-base : start : launching threaded")
      if(Ros2NodeInterfaceBase::MultithreadedExecutorSpin() == false)
      {
        if(Ros2NodeInterfaceBase::IsRosSpinning() == false){
          CLOG_ERROR("ros2-node-interface-base : start : multithreded : multithreaded-executor "
          "impossible to start node!");
          _state = State::ERROR;
          return false;
        }
        CLOG_INFO("ros2-node-interface-base : start : multithreded : multithreaded-executor "
        "is already spinning...");
      }
      CLOG_INFO("ros2-node-interface-base : start : multithreded : multithreaded-executor "
        "NOW SPINNING");
    }else{
      CLOG_INFO("ros2-node-interface-base : start : this node does not spins ros base");
    }
    _state = State::CREATED;
    CLOG_INFO("ros2-node-interface-base : start : DONE");
    return true;
  }

  bool Ros2NodeInterfaceBase::Stop(){
    CLOG_WARN("ros2-node-interafce-base : stop : NOT YET IMPLEMENTED");
    return false;
  }
  
  Ros2NodeInterfaceBase::State Ros2NodeInterfaceBase::state() const{
    return _state;
  }


  bool Ros2NodeInterfaceBase::makeRosSpin(rclcpp::Node::SharedPtr node){
    rclcpp::spin(node);
    return true;
  }

  

}
}
}

std::ostream& operator<<(std::ostream& o,milla::connapp::ros2::Ros2NodeInterfaceBase::State s)
{
  return (o << milla::connapp::ros2::Ros2NodeInterfaceBase::StateToString(s));
}
