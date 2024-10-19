#include <Coyot3pp/Mqtt/Gateway/AcrpTopicManagementInfoStack.hpp>

namespace coyot3{
namespace communication{
namespace mqtt{


AcrpTopicManagementInfoStack::AcrpTopicManagementInfoStack(std::string n)
:name(n)
{
  CLOG_INFO("ACRP TOPIC MANAGEMENT INFO STACK : constructor");
}
AcrpTopicManagementInfoStack::~AcrpTopicManagementInfoStack()
{
  CLOG_WARN("ACRP TOPIC MANAGEMENT INFO STACK : destructor : deleting stack");
  while(pop() == true); //delete stack
}




bool AcrpTopicManagementInfoStack::addPublisher(
    const std::string& topicBase
  ,const std::string& topicReq
  ,const std::string& topicRes
)
{
  AcrpTopicManagementInfo* newAtmi;

  newAtmi = new(std::nothrow) AcrpTopicManagementInfo(
     topicBase
    ,topicReq
    ,topicRes
  );
  if(!newAtmi)
  {
    CLOG_ERROR("ATMI STACK : addPublisher : FATAL ERROR ALLOCATING ACRP TOPIC "
      "INFO FOR TOPIC [" << topicBase << "]");
    return false;
  }
  newAtmi->setRole(AcrpTopicManagementInfo::Role::EMITTER);
  
  if(!insert(newAtmi))
  {
    CLOG_WARN("ATMI STACK : add publisher - no callbackop : error storing atmi "
      "at stack.");
    return false;
  }
  CLOG_INFO("ATMI STACK : add publisher : added ACRP topic management info for "
    "[" << topicBase << "] - no callback on-operation-done");
  return true;
}


bool AcrpTopicManagementInfoStack::addPublisher(
    const std::string& topicBase
  ,const std::string& topicReq
  ,const std::string& topicRes
  ,AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType onPublishOpDone
)
{
  AcrpTopicManagementInfo* newAtmi;
  newAtmi = new(std::nothrow) AcrpTopicManagementInfo(
     topicBase
    ,topicReq
    ,topicRes
  );
  if(!newAtmi)
  {
    CLOG_ERROR("ATMI STACK : addPublisher : FATAL ERROR ALLOCATING ACRP TOPIC "
      "INFO FOR TOPIC [" << topicBase << "]");    
    return false;
  }
  newAtmi->setRole(AcrpTopicManagementInfo::Role::EMITTER);
  newAtmi->updateOnPublishedCallback(onPublishOpDone);
  if(!insert(newAtmi))
  {
    delete newAtmi;
    CLOG_WARN("ATMI STACK : addPublisher-with-callback : error storing ATMI "
      "for [" << topicBase << "]");
    return false;
  }
  CLOG_INFO("ATMI STACK : add publisher : added ACRP topic management info for "
    "[" << topicBase << "] - active callback on-operation-done");
  return true;
}

bool AcrpTopicManagementInfoStack::addSubscriber(
    const std::string& topicBase
  ,const std::string& topicReq
  ,const std::string& topicRes
  ,AcrpTopicManagementInfo::OnSubscriberDataCallbackType onDataCallback
  ,AcrpTopicManagementInfo::OnSubscriberParsedDataCallbackType onParsedDataCallback
)
{
  AcrpTopicManagementInfo* newAtmi;
  bool res;
  newAtmi = new(std::nothrow) AcrpTopicManagementInfo(
    topicBase
    ,onDataCallback
    ,onParsedDataCallback
    ,topicReq
    ,topicRes
  );
  if(!newAtmi)
  {
    CLOG_ERROR("ATMI STACK : addSubscriber : FATAL ERROR ALLOCATING ACRP TOPIC"
      "INFO FOR TOPIC [" << topicBase << "]");
    return false;
  }
  newAtmi->setRole(AcrpTopicManagementInfo::Role::SUBSCRIBER);
  res = insert(newAtmi);
  if(res== true){
    CLOG_INFO("ATMI STACK : addSubscriber : added ACRP topic management info for "
    "[" << topicBase << "]");
  }else{
    CLOG_WARN("ATMI STACK : addSubscriber : error adding acrp topic management "
      "info for [" << topicBase << "]");
    delete newAtmi;
  }
  return res;
}




AcrpTopicManagementInfo* AcrpTopicManagementInfoStack::getByBase(const std::string& t)
{
  AcrpTopicManagementInfo* ptr;
  AtmiTopicBook::iterator it;
  it = atmisByTopicBase.find(t);
  if(it == atmisByTopicBase.end())
  {
    //not found
    return nullptr;
  }
  ptr = it->second;
  return ptr;
}

AcrpTopicManagementInfo* AcrpTopicManagementInfoStack::getByReq(const std::string& t)
{
  AcrpTopicManagementInfo* ptr;
  AtmiTopicBook::iterator it;
  it = atmisByTopicReq.find(t);
  if(it == atmisByTopicReq.end())
  {
    //not found
    return nullptr;
  }
  ptr = it->second;
  return ptr;
}

AcrpTopicManagementInfo* AcrpTopicManagementInfoStack::getByRes(const std::string& t)
{
  AcrpTopicManagementInfo* ptr;
  AtmiTopicBook::iterator it;
  it = atmisByTopicRes.find(t);
  if(it == atmisByTopicRes.end())
  {
    //not found
    return nullptr;
  }
  ptr = it->second;
  return ptr;
}



bool AcrpTopicManagementInfoStack::remove(const std::string& topic_base)
{
  AtmiTopicBook::iterator    it;
  AcrpTopicManagementInfo*   ptr;
  AtmiTopicBook::iterator    it2;
  std::string                t;
  it = atmisByTopicBase.find(topic_base);
  if(it == atmisByTopicBase.end())
  {
    //not found
    return false;
  }
  ptr = it->second;
  //found... deleting
  CLOG_INFO("ATMI STACK : unindexing and deleting from base [" << topic_base << "]");
  atmisByTopicReq.erase(atmisByTopicReq.find(ptr->getTopicCrpReq()));
  atmisByTopicReq.erase(atmisByTopicRes.find(ptr->getTopicCrpRes()));
  delete ptr;
  atmisByTopicBase.erase(it);
  return true;
}

size_t AcrpTopicManagementInfoStack::size()
{
  return atmisByTopicBase.size();
}


bool AcrpTopicManagementInfoStack::insert(AcrpTopicManagementInfo* atmi)
{
  AtmiTopicBook::iterator it;
  std::lock_guard<std::mutex> guard(stack_mtx);
  it = atmisByTopicBase.find(atmi->getTopicBase());
  if(it != atmisByTopicBase.end())
  {
    CLOG_WARN("ATMI STACK : insert (ptr) : topic already managed! [" 
      << atmi->getTopicBase() << "]");
    return false;
  }
  CLOG_INFO("ATMI STACK : insert (ptr) : indexing to stack, topic [" << atmi->getTopicBase() << "]");
  atmisByTopicBase.insert(std::make_pair(atmi->getTopicBase(),atmi));
  atmisByTopicReq.insert(std::make_pair(atmi->getTopicCrpReq(),atmi));
  atmisByTopicRes.insert(std::make_pair(atmi->getTopicCrpRes(),atmi));
  return true;
}


bool AcrpTopicManagementInfoStack::pop()
{
  std::lock_guard<std::mutex> guard(stack_mtx);
  if(atmisByTopicBase.size() == 0)
  {
    return false;
  }
  CLOG_WARN("ATMI STACK : pop : removing element");
  AtmiTopicBook::iterator i;
  AcrpTopicManagementInfo* ptr;
  ptr = atmisByTopicBase.begin()->second;
  atmisByTopicBase.erase(atmisByTopicBase.begin());
  atmisByTopicReq.erase(atmisByTopicReq.begin());
  atmisByTopicRes.erase(atmisByTopicRes.begin());
  delete ptr;
  CLOG_DEBUG(3,"ATMI STACK : pop : removing element : DONE");
  return true;

}

size_t AcrpTopicManagementInfoStack::forEach(std::function<bool(AcrpTopicManagementInfo*)> callback)
{
  AtmiTopicBook::iterator it;
  size_t res = 0;
  CLOG_DEBUG(4,"ATMI STACK : [" << name << "] : for-each - init");
  std::lock_guard<std::mutex> guard(stack_mtx);
  for(it = atmisByTopicBase.begin();it != atmisByTopicBase.end();++it)
  {
    AcrpTopicManagementInfo* atmiPtr;
    atmiPtr = it->second;
    if(callback(atmiPtr) == true)
    {
      ++res;
      CLOG_DEBUG(5, "ATMI STACK : [" << name << "] : for-each : managing ATMI "
      "topic base [" << atmiPtr->getTopicBase() << "] : result OK");
    }else{
      CLOG_DEBUG(4, "ATMI STACK : [" << name << "] : for-each : managing ATMI "
      "topic base [" << atmiPtr->getTopicBase() << "] : result KO! (soft-warn)");
    }
  }
  return res;
}

}
}//eons wrappers
}//eons coyot3