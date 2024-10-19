#include <Coyot3pp/Cor3/Coyot3.hpp>



int main(int argv, char** argc){
  CLOG_INFO("hello world")


  coyot3::tools::Value val;

  val = 1;
  CLOG_INFO("val = " << val.str())
  val = "hola";
  CLOG_INFO("val = " << val.str())
  val.append(1);
  val.append("dos");
  val.append(3.3);
  CLOG_INFO("val = " << val.str())

  coyot3::tools::Value sec;
  sec = val;

  CLOG_INFO("sec = " << sec.str())
  CLOG_INFO("sec[2] << " << sec[2].str())

  CLOG_INFO("sec[2] << " << sec.str())
  coyot3::tools::Value& ter = sec[2];
  ter = 5.5;
  CLOG_INFO("sec[2] << " << sec.str())

  val["uno"] = 1;
  val["dos"] = "dos";
  val["tres"] = sec; 

  CLOG_INFO("val = " << val.str())
  val["uno"] = 1.2345;
  CLOG_INFO("val = " << val.str())


  CLOG_INFO("asjson = " << val.to_json())

  coyot3::tools::Value cua;
  cua = val.to_json();

  CLOG_INFO("cua = " << cua.str())
  CLOG_INFO("cua = " << cua.to_json())

  std::vector<int> enteros{1,2,3,4,5};

  coyot3::tools::Value cinq;
  cinq = enteros;
  CLOG_INFO("cinq= " << cinq.str())


  std::map<std::string, std::string> mapa;
  mapa.insert(std::make_pair("uno","one"));
  mapa.insert(std::make_pair("dos","two"));
  mapa.insert(std::make_pair("tres","three"));
  coyot3::tools::Value sei;
  sei = mapa;
  CLOG_INFO("sei = " << sei.str())

}