#pragma once

#include "ModuleBase.hpp"

namespace coyot3{
namespace mod{

  class ModulesController : public ModuleBase{
    public:
      typedef std::map<std::string,ModuleBase*> ModulesMap;

      ModulesController(const std::string& name);
      virtual ~ModulesController();



    protected:

      bool task_init();
      bool task_start();
      bool task_pause();
      bool task_stop();
      bool task_end();


      ModulesMap  mods;



    private:



  };


}
}
