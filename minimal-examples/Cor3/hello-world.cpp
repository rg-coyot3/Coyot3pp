#include <Coyot3pp/Cor3/Coyot3.hpp>


#include <Coyot3pp/Cor3/ModuleBase.hpp>


class MyClass : public coyot3::mod::ModuleBase{
  public:
    MyClass() : ModuleBase(){
      log_info("hola mundo");
    }
};

int main(int argc, char** argv){
  CLOG_INFO("hello world")

  MyClass instance;
  
}