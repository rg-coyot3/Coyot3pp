#include <Coyot3pp/Mqtt/Gateway/AcrpDataPacket.hpp>



namespace coyot3{
namespace communication{
namespace mqtt{


const char* AcrpPacket::JsField::timestampOrigToken = "timestampOrigToken";
const char* AcrpPacket::JsField::timestampGen       = "timestampGen";
const char* AcrpPacket::JsField::numRetry           = "numRetry";
const char* AcrpPacket::JsField::payload            = "payload";
const char* AcrpPacket::JsField::tokenSec           = "tokenSec";

AcrpPacket::AcrpPacket()
:JsonSerializablePacketBase()
,timestampOrigToken(0)
,timestampGen(0)
,numRetry(0)
,payload(Json::Value())
,tokenSec(0)
,isValid(false)
,lastInternalActivity(0)
{
  timestampOrigToken = timestampGen = lastInternalActivity = coyot3::tools::getCurrentTimestamp();
  
}

AcrpPacket::AcrpPacket(const AcrpPacket& o)
{
  *this = o;
}
AcrpPacket::AcrpPacket(const Json::Value& s)
:JsonSerializablePacketBase()
,timestampOrigToken(0)
,timestampGen(0)
,numRetry(0)
,payload(s)
,tokenSec(0)
,isValid(true)
,acrpPacketTreated(false)
{
  lastInternalActivity = 
    timestampOrigToken = 
      timestampGen = coyot3::tools::getCurrentTimestamp();
}

AcrpPacket::~AcrpPacket(){}

AcrpPacket& AcrpPacket::operator=(const AcrpPacket& o)
{
  timestampOrigToken = o.timestampOrigToken;
  timestampGen       = o.timestampGen;
  numRetry           = o.numRetry;
  payload            = o.payload;
  tokenSec           = o.tokenSec;

  isValid            = o.isValid;
  acrpPacketTreated  = o.acrpPacketTreated;
  lastInternalActivity = o.lastInternalActivity;

  
  return *this;
}


Json::Value AcrpPacket::to_json() const
{
  Json::Value packet;
  
  packet[JsField::timestampOrigToken] = static_cast<Json::LargestInt>(timestampOrigToken);
  packet[JsField::timestampGen]       = static_cast<Json::LargestInt>(timestampGen);
  packet[JsField::numRetry]           = static_cast<Json::Int>(numRetry);
  packet[JsField::payload]            = payload;

  return packet;
}




bool AcrpPacket::from_json(const Json::Value& source)
{
  if( !source.isMember(JsField::timestampOrigToken)
    ||!source.isMember(JsField::timestampGen)
    ||!source.isMember(JsField::numRetry)
    ||!source.isMember(JsField::payload))
  {
    CLOG_DEBUG(6,"ACRP PACKET : from_json : packet source does not contain all "
      "needed members.");
    return false;
  }
  try{
    timestampOrigToken = source[JsField::timestampOrigToken].asLargestInt();
    timestampGen       = source[JsField::timestampGen].asLargestInt();
    numRetry           = source[JsField::numRetry].asInt();
    payload            = source[JsField::payload];
  }
  catch(Json::Exception& e)
  {
    CLOG_ERROR("ACRP PACKET : from json (json) : error parsing");
    return false;
  }
  return true;
}

bool AcrpPacket::acknowledge(const AcrpPacket& o)
{
  timestampOrigToken = o.timestampOrigToken;
  timestampGen       = o.timestampGen;
  numRetry           = o.numRetry;
  payload            = static_cast<Json::LargestInt>(coyot3::tools::getCurrentTimestamp());
  tokenSec           = o.tokenSec;
  return true;
}



AcrpPacket& AcrpPacket::operator++()
{
  lastInternalActivity = timestampGen = coyot3::tools::getCurrentTimestamp();

  numRetry++;
  return *this;
}

int AcrpPacket::updateIteration()
{
  lastInternalActivity = timestampGen = coyot3::tools::getCurrentTimestamp();
  numRetry++;
  return numRetry;
}
int AcrpPacket::getIteration()
{
  return numRetry;
}

const Json::Value&  AcrpPacket::getPayload() 
{
  return payload;
}
std::string AcrpPacket::getPayloadStringified() const{
  std::stringstream s;
  s << payload;
  return s.str();
}
bool AcrpPacket::setPayload(const Json::Value& p)
{
  payload = p;
  return true;
}
bool AcrpPacket::setPayloadZero()
{
  payload = Json::Value();
  return true;
}

bool AcrpPacket::isTreated()
{
  return acrpPacketTreated;
}

bool AcrpPacket::jobDone()
{
  return (acrpPacketTreated = true);
}

int64_t AcrpPacket::getLastInternalActivityTs()
{
  return lastInternalActivity;
}
void    AcrpPacket::setLastInternalActivityTs()
{
  lastInternalActivity = coyot3::tools::getCurrentTimestamp();
}

/////////
///////// ACRPPACKET : END
/////////


///////// 
///////// ACKNOWLEDGEMENT GENERATION : BEGIN
/////////



AcrpPacketAcknowledgement::AcrpPacketAcknowledgement(const AcrpPacket& source)
:AcrpPacket()
{
  timestampOrigToken  = source.timestampOrigToken;
  timestampGen        = source.timestampGen;
  tokenSec            = source.tokenSec;
  numRetry            = source.numRetry;
  isValid             = true;
  payload             = static_cast<Json::LargestInt>(coyot3::tools::getCurrentTimestamp());
}

AcrpPacketAcknowledgement::~AcrpPacketAcknowledgement(){}



///////// 
///////// ACKNOWLEDGEMENT GENERATION : END.
///////// 


///////// 
///////// DYNAMIC OPTIMIZATION PACKET : BEGIN.
///////// 
AcrpPacketDynamicOptimization::AcrpPacketDynamicOptimization()
:AcrpPacket()
{
  timestampGen = coyot3::tools::getCurrentTimestamp();
  timestampOrigToken = 0;
  payload             = static_cast<Json::LargestInt>(coyot3::tools::getCurrentTimestamp());

}

AcrpPacketDynamicOptimization::~AcrpPacketDynamicOptimization(){}
///////// 
///////// DYNAMIC OPTIMIZATION PACKET : BEGIN.
///////// 


/////////
///////// ACRP MANAGEMENT STACK : BEGIN
/////////


const char* AcrpPacketManagementStack::PacketStateToString(PacketState s)
{
  switch(s)
  {
    case PacketState::STACK_ERROR:return "STACK_ERROR";break;
    case PacketState::NOT_MANAGED:return "NOT_MANAGED";break;
    case PacketState::TREATING:return "TREATING";break;
    case PacketState::TREATED:return "TREATED";break;
    default:
      return "acrp-packet-value-error";
  }
}
std::ostream& operator<<(std::ostream& i,const AcrpPacketManagementStack::PacketState& p)
{
  return (i << AcrpPacketManagementStack::PacketStateToString(p));  
}
    



AcrpPacketManagementStack::AcrpPacketManagementStack()
: stack()
, manager(nullptr)
{

}
AcrpPacketManagementStack::~AcrpPacketManagementStack()
{
  // delete current stack;
  StackType::iterator it;
  std::lock_guard<std::mutex> guard(stack_mtx);
  
  for(it = stack.begin();it != stack.end();++it)
  {
    CLOG_DEBUG(5,"ACRP PACKET MANAGEMENT STACK : destructor : deleting item with "
      "token : [" << it->second->timestampOrigToken << "]");
    delete it->second;
  }
}


AcrpPacket* AcrpPacketManagementStack::find(int64_t token,int64_t token_sec) 
{
  
  std::lock_guard<std::mutex> guard(stack_mtx);
  return _find(token,token_sec);
}

AcrpPacket* AcrpPacketManagementStack::_find(int64_t token,int64_t token_sec)
{
  StackType::iterator it;
  it = stack.find(token);
  if(it == stack.end())
  {
    return nullptr;
  }
  token_sec = 0;
  return (it->second);
}

AcrpPacketManagementStack::PacketState AcrpPacketManagementStack::evaluate(int64_t token,int64_t tokenSec)
{
  AcrpPacket* packet;
  std::lock_guard<std::mutex> guard(stack_mtx);
  packet= _find(token,tokenSec);

  if(packet == nullptr)
  {
    return PacketState::NOT_MANAGED;
  }
  if(packet->isTreated())
  {
    return PacketState::TREATED;
  }else{
    return PacketState::TREATING;
  }
}

AcrpPacketManagementStack::PacketState AcrpPacketManagementStack::evaluate(const AcrpPacket& packet)
{
  return evaluate(packet.timestampOrigToken,packet.tokenSec);
}

bool AcrpPacketManagementStack::packetJobDone(int64_t token,int64_t token_sec)
{
  AcrpPacket* p;
  std::lock_guard<std::mutex> guard(stack_mtx);
  p = _find(token,token_sec);
  if(!p)
  {
    return false;
  }
  p->jobDone();
  return true;
}

bool AcrpPacketManagementStack::packetJobDone(const AcrpPacket& packet)
{
  return packetJobDone(packet.timestampOrigToken,packet.tokenSec);
}





AcrpPacket* AcrpPacketManagementStack::registerPacket(const AcrpPacket& packet)
{

  AcrpPacket* p;
  std::lock_guard<std::mutex> guard(stack_mtx);

  p = _find(packet.timestampOrigToken,packet.tokenSec);
  if(p != nullptr)
  {
    //packet already exists with primary keys pair
    return nullptr;  
  }
  p = new(std::nothrow) AcrpPacket(packet);
  if(!p)
  {
    CLOG_ERROR("ACRP PACKET MANAGEMENT STACK : register packet : error "
      "creating copy of packet [" << packet.timestampOrigToken << "]");
    return nullptr;
  }
  CLOG_INFO("*** to-delete - acrp-stack : registered packet [" << packet.timestampOrigToken << "==?" << p->timestampOrigToken << "]");
  stack.insert(std::make_pair(packet.timestampOrigToken,p));
  return p;
}


bool AcrpPacketManagementStack::removeFromStack(int64_t token,int64_t token_sec)
{
  StackType::iterator it;
  std::lock_guard<std::mutex> guard(stack_mtx);
  it = stack.find(token);
  if(it == stack.end())
  {
    //not found
    return false;
  }
  AcrpPacket* p = it->second;
  stack.erase(it);
  if(it->second == nullptr)
  {
    CLOG_ERROR("ACRP PACKET MANAGEMENT STACK : remove from stack : error "
      "removing from stack token [" << token << "] : pointer to packet "
      "container info is null!");

    return false;
  }
  delete p;
  return true;
}
bool AcrpPacketManagementStack::removeFromStack(const AcrpPacket& packet)
{
  return removeFromStack(packet.timestampOrigToken,packet.tokenSec);
}

bool AcrpPacketManagementStack::set_user_pointer(void* managerptr)
{
  return ((manager = managerptr) != nullptr);
}

size_t AcrpPacketManagementStack::forEach(std::function<bool(AcrpPacket*)> func)
{
  StackType::iterator it;
  size_t res = 0;
  std::lock_guard<std::mutex> guard(stack_mtx);
  for(it = stack.begin();it != stack.end();++it)
  {
    if(func(it->second) == true)
    {
      ++res;
    }
  }
  return res;
}



/////////
///////// ACRP MANAGEMENT STACK : END
/////////
}
}//eons wrappers
}//epns coyot3