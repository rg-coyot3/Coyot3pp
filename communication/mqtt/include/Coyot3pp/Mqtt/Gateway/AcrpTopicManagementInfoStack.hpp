#pragma once

#include <map>
#include <mutex>
#include <Coyot3pp/Cor3/Coyot3.hpp>

#include "AcrpTopicManagementInfo.hpp"

namespace coyot3{
namespace communication{
namespace mqtt{

/**
 * @brief : the MqttManagerBase class is going to search for the ATMI indexing
 *          by the strings of the three different topics. This class is created
 *          to ease the integration while developing the MqttManagerBase
 * */
class AcrpTopicManagementInfoStack{
  public:
    typedef std::map<std::string,AcrpTopicManagementInfo*> AtmiTopicBook;


    

    AcrpTopicManagementInfoStack(std::string n = std::string("no-name"));
    virtual ~AcrpTopicManagementInfoStack();

    bool addPublisher(
       const std::string& topicBase
      ,const std::string& postfixReq
      ,const std::string& postfixRes
    );
    bool addPublisher(
       const std::string& topicBase
      ,const std::string& postfixReq
      ,const std::string& postfixRes
      ,AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType onPublishOpDone
    );

    bool addSubscriber(
       const std::string& topicBase
      ,const std::string& postfixReq
      ,const std::string& postfixRes
      ,AcrpTopicManagementInfo::OnSubscriberDataCallbackType onDataCallback
      ,AcrpTopicManagementInfo::OnSubscriberParsedDataCallbackType onParsedDataCallback
    );


    AcrpTopicManagementInfo* getByBase(const std::string& t);
    AcrpTopicManagementInfo* getByReq(const std::string& t);
    AcrpTopicManagementInfo* getByRes(const std::string& t);

    bool remove(const std::string& topic_base);

    size_t size();

    /**
     * @brief : foreach wrapper
     */
    size_t forEach(std::function<bool(AcrpTopicManagementInfo*)>);
  protected:
    std::mutex    stack_mtx;

    AtmiTopicBook atmisByTopicBase;
    AtmiTopicBook atmisByTopicReq;
    AtmiTopicBook atmisByTopicRes;

    std::string name;

    
    bool insert(AcrpTopicManagementInfo* atmi);
    bool pop();



};

}//eons mqtt
}//eons wrappers
}//eons coyot3