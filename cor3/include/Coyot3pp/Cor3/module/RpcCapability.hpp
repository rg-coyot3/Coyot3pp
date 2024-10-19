#pragma once

#include "../Coyot3.hpp"
#include <optional>

namespace coyot3{
namespace mod{

  class RpcCapability;

  typedef std::function<
        bool(const coyot3::tools::Value&, 
                    coyot3::tools::Value&)> RpcInvokableMethod;
  
  typedef int64_t rpc_index_t;

  CYT3MACRO_model_class_declarations(
    RpcInvokableStruct
    ,
    , ( )
    , ( )
      , active          , bool                , true
      , description     , std::string         , 
      , method          , RpcInvokableMethod  , 
  )

  CYT3MACRO_model_class_declarations(
    RpcSubmoduleStruct
    ,
    , ( )
    , ( )
      , id              , rpc_index_t     , -1
      , active          , bool            , true
      , description     , std::string     , 
      , instance        , RpcCapability*  , nullptr
  )

  CYT3MACRO_model_class_set_mapped_declarations(RpcSubmoduleStruct,id)
  

  class RpcCapability{
    public:
      typedef coyot3::tools::Value      Value;

      typedef std::map<std::string,RpcInvokableStruct>          RpcInvokablesMap;
      typedef std::map<std::string,RpcSubmoduleStructMappedSet> RpcSubmodulesMap;
      

      virtual ~RpcCapability();

      std::string rpc_instance_name() const;
      int64_t     rpc_instance_id() const;
      std::string rpc_class() const;
      std::string rpc_module_description() const;
      


      /**
       * @brief adds a submodule.
       * 
       * @param modClass submodule name
       * @param modPtr   pointer to the submodule
       * @return true    added ok
       * @return false   not added.
       */
      bool  add_rpc_submodule(  RpcCapability* modPtr,
                                const std::string& modClass = std::string());

      /**
       * @brief invokes a method from the inherited class. 
       * 
       * @param method : strings containing the method, or route to the 
       *                  submodule. the submodules route is separated by '.', 
       *                  and when having arrays... '[]'
       * @param params : ct::Value containing the input parameters.
       * @param result : ct::Value returning the results.
       * @return true  : the method was found. results are defined.
       * @return std::optional<Value> the result of the operation or empty 
       *                 optional on error or instance not found.
       */
      std::optional<Value>  rpc_invoke(const std::string& method, 
                   const Value& params);

      Value rpc_capabilities() const;
      Value rpc_capabilities_ext() const;

      bool  rpc_capability_activation(const std::string& c) const;
      bool  rpc_capability_activation(const std::string& c, bool act);

      static void  DebugLevel(int l);

    protected:

      RpcCapability();
      RpcCapability(const std::string& name = std::string("not defined"), 
                    const std::string& description = std::string("no desc..."));

      std::string  rpc_instance_name(const std::string& n);
      int64_t      rpc_instance_id(int64_t i);
      std::string  rpc_class(const std::string& c);
      std::string  rpc_module_description(const std::string& d);

      bool rpc_method_register(
                  const std::string& method,
                  RpcInvokableMethod callback, 
                  const std::string& method_description = std::string());

      

    private:
      std::string       mod_instance_name_;
      int64_t           mod_instance_id_;
      std::string       mod_class_;
      std::string       mod_description_;
      RpcInvokablesMap  invokables_;
      RpcSubmodulesMap  submodules_;

  };

}
}