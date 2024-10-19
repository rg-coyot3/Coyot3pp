#include <Coyot3pp/Cor3/module/RpcCapability.hpp>


namespace ct = coyot3::tools;

namespace coyot3{
namespace mod{

  void RpcCapability::DebugLevel(int l){
    CLOG_DEBUG_LEVEL_SET(l)
  }
  RpcCapability::RpcCapability(){

  }

  RpcCapability::~RpcCapability(){

  }


  std::string 
  RpcCapability::rpc_instance_name() const{
    return mod_instance_name_;
  }
  std::string
  RpcCapability::rpc_instance_name(const std::string& n){
    return (mod_instance_name_ = n);
  }

  
  int64_t
  RpcCapability::rpc_instance_id() const{ return mod_instance_id_;}
  int64_t
  RpcCapability::rpc_instance_id(int64_t i){return (mod_instance_id_ = i);}

  
  std::string
  RpcCapability::rpc_class()const{return mod_class_;}
  std::string
  RpcCapability::rpc_class(const std::string& c){return (mod_class_ = c);}
  
  
  std::string
  RpcCapability::rpc_module_description() const{
    return mod_description_;
  }
  std::string
  RpcCapability::rpc_module_description(const std::string& n){
    return (mod_description_ = n);
  }

  bool
  RpcCapability::add_rpc_submodule(
          RpcCapability* modPtr, 
          const std::string& modClass)
  {
    std::string m;
    m = (modClass.size() != 0?modClass:modPtr->rpc_instance_name());
    RpcSubmodulesMap::iterator it;
    it = submodules_.find(m);
    if(it != submodules_.end()){
      RpcSubmoduleStructMappedSet submodInstances;
      CLOG_DEBUG(6,"rpc-capability : add-rpc-submodule : new class [" 
      << m << "]")
      submodules_.insert(std::make_pair(m,submodInstances));
      it = submodules_.find(m);
    }
    RpcSubmoduleStructMappedSet& ms = it->second;
    
    if(ms.is_member(modPtr->rpc_instance_id())){
      CLOG_WARN("rpc-capability : add-rpc-submodule : class [" << m << "] "
      "already contains instance id [" << modPtr->rpc_instance_id() << "]")
      return false;
    }
    if(modPtr->rpc_instance_id() == -1){
      modPtr->rpc_instance_id(ms.size());
      while(ms.is_member(modPtr->rpc_instance_id())){
        modPtr->rpc_instance_id(modPtr->rpc_instance_id()+1);
      }
      CLOG_DEBUG(7,"rpc-capability : add-rpc-submodule : class [" << m << "] "
      "adding instance id[" << modPtr->rpc_instance_id() << "] : auto-assigned")
    }
    RpcSubmoduleStruct submod;
    submod.id(modPtr->rpc_instance_id());
    submod.active(true);
    submod.instance(modPtr);
    submod.id(modPtr->rpc_instance_id());
    if(!ms.insert(submod)){
      CLOG_WARN("rpc-capability : add-rpc-submodule : class [" << m << "] "
      "adding instance id[" << modPtr->rpc_instance_id() << "] : ID EXISTS! "
      "INSTANCE NOT ADDED")
      return false;
    }
    CLOG_DEBUG(7,"rpc-capability : add-rpc-submodule : class [" << m << "] "
      "adding instance id[" << modPtr->rpc_instance_id() << "]")
    return true;
  }

  RpcCapability::Value
  RpcCapability::rpc_capabilities() const{
    RpcInvokablesMap::const_iterator it;
    RpcSubmodulesMap::const_iterator iu;
    Value caps;
    caps["class"] = mod_class_;

    caps["description"] = mod_description_;
    for(it = invokables_.begin();it != invokables_.end();++it){
      Value item;
      item["method"]       = it->first;
      item["description"]  = it->second.description();
      item["active"]       = it->second.active();
      caps["methods"].append(item);
    }

    
    if(submodules_.size() == 0){return caps;} // no submod installed
    else{
      Value item;
      for(auto sm : submodules_){
        if(sm.second.size() == 0)continue;
        //submodule class capabilities description.
        item[sm.first] = sm.second[0].instance()->rpc_capabilities();
        sm.second.for_each([&](const RpcSubmoduleStruct& m){
          //mention ids
          item[sm.first]["instances"].append(m.id());
          return true;
        });
      }
           
      
    }
    return caps;
  }

  RpcCapability::Value
  RpcCapability::rpc_capabilities_ext() const{
    Value caps;
    caps = rpc_capabilities();
    caps["name"]  = mod_instance_name_;
    caps["id"]    = mod_instance_id_;
    return caps;
  }

  std::optional<RpcCapability::Value> 
  RpcCapability::rpc_invoke(const std::string& method, const Value& params){
    std::size_t posp,posc;
    posp = method.find_first_of(".");
    posc = method.find_first_of("[");
    rpc_index_t idx;
    if(
         (posp != std::string::npos)
      || (posc != std::string::npos)
    ){
      std::string modclass;
      //subclass[index].
      modclass = method.substr(0,std::min(posc,posp));
      if(posc < posp){
        std::string indexstr = 
                      method.substr(posc+1,
                                    method.find_first_of("]") - posc - 1);
        
        if(ct::is_number(indexstr) == true){
          try{
            idx = static_cast<rpc_index_t>(std::stoll(indexstr));
            CLOG_DEBUG(7, "rpc-capability : rpc-invoke : from argument [" 
            << method << "] parsed index [" << idx << "]")
          }catch(...){
            CLOG_WARN("rpc-capability : rpc-invoke : from argument [" << method
            << "] indexation [" << indexstr << "] parse error")
            return {};
          }
        }else{
          CLOG_WARN("rpc-capability : rpc-invoke : from argument [" << method 
          << "] submodule [" << modclass << "] : module indexation error. "
          "Index must be a number")
          return {};
        }

      }
      //find instance.
    }

    if(posc != std::string::npos){
      std::string subclass = method.substr(0,posc);

      RpcSubmodulesMap::iterator itsc = submodules_.find(subclass);
      if(itsc == submodules_.end()){
        CLOG_WARN("rpc-capability : rpc-invoke : from argument [" << method 
        << "] submodule [" << subclass << "] not found.")
        return {};
      }
    }



    
    return {};
  }

}
}
