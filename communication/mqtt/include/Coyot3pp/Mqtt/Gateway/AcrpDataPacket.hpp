#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <Coyot3pp/Cor3/JsonSerializablePacketBase.hpp>

#include <mutex>

namespace coyot3{
namespace communication{
namespace mqtt{
/**
 * @brief : declare the ACRP packet that will be used for the exchanges
 * 
 * */

class AcrpPacket : public coyot3::tools::JsonSerializablePacketBase{
  public:


    struct JsField{
      static const char* timestampOrigToken;
      static const char* timestampGen;
      static const char* numRetry;
      static const char* payload;
      static const char* tokenSec;
    };
    //properties for the serialization.
    int64_t     timestampOrigToken;
    int64_t     timestampGen;
    int         numRetry;
    Json::Value payload;
    int64_t     tokenSec;

    AcrpPacket();                     // base constructor
    AcrpPacket(const AcrpPacket& o);  // copy constructor
    AcrpPacket(const Json::Value& s); // generation from a payload
    template<typename T>

    AcrpPacket(const T& s)
    :JsonSerializablePacketBase()
    ,timestampOrigToken(0)
    ,timestampGen(0)
    ,numRetry(0)
    ,payload()
    ,tokenSec(0)
    ,isValid(true)
    ,acrpPacketTreated(false)
    {
      lastInternalActivity = 
        timestampOrigToken = 
          timestampGen = coyot3::tools::getCurrentTimestamp();
      payload = Json::Value(s);
    }
    virtual ~AcrpPacket();

    AcrpPacket& operator=(const AcrpPacket& o); //copy operator
    
    bool sameToken(const AcrpPacket& o);

    /**
     * @brief acknowledge : updates the packet to be an acknowledge it.
     *  - all same but payload = local timestamp.
     * */
    bool acknowledge(const AcrpPacket& o);
    


    virtual   Json::Value  to_json() const                      override;

    virtual   bool         from_json(const Json::Value& source) override;

    /**
     * @brief updates the timestampGen and numRetry
     * */
    AcrpPacket&         operator++();
    int                 updateIteration();  //returns current iteration number.
    int                 getIteration();
    
    const Json::Value&         getPayload();
    std::string                getPayloadStringified() const;
    
    bool                setPayload(const Json::Value& p);
    
    template<typename T>
    bool                setPayload(const T& p)
    {
      payload = Json::Value(p);
      return true;
    }


    template<typename T>
    AcrpPacket&          operator<<(const T& p)
    {
      payload = Json::Value(p);
      return *this;
    }

    bool                setPayloadZero();
    bool                isTreated();
    bool                jobDone();           /** << marks a packet as treated */
    int64_t             getLastInternalActivityTs(); /** << in msecs from epoch*/
    void                setLastInternalActivityTs(); /** << marks as last activity done related to 'this' packet */
  
  protected:

      bool        isValid;
      bool        acrpPacketTreated;
      /**
       * @brief : for publishers, will be used as the last moment there's a publication
       *          for subscribers, the last time it was obtained an ACK
       * */
      int64_t        lastInternalActivity;

  private:



};


/**
 * @brief : wrapper to code easily an ACK
 * */
class AcrpPacketAcknowledgement : public AcrpPacket{
  public:
    AcrpPacketAcknowledgement(const AcrpPacket& source);
    virtual ~AcrpPacketAcknowledgement();

  protected:
    
};

class AcrpPacketDynamicOptimization : public AcrpPacket{
  public:
    AcrpPacketDynamicOptimization();
    virtual ~AcrpPacketDynamicOptimization();
};

/**
 * @brief : class to contain a list of packets, classified by its token
 *          note: for the moment tokenSec is not used
 * */
class AcrpPacketManagementStack {
  public:
  //classification by timestampOrigToken
    typedef std::map<std::int64_t, AcrpPacket*> StackType;



    enum class PacketState{
      STACK_ERROR = 0,                //identify one error in the management chain
      NOT_MANAGED = 1,                // the packet is no
      TREATING = 2,                   // managed but not treated (using : Role Emitter :: ACK received; Role Receiver :: Packet already treated);
      TREATED = 3                     // managed and already treated
    };
    static const char* PacketStateToString(PacketState s);

    AcrpPacketManagementStack();
    virtual ~AcrpPacketManagementStack();

    /**
     * @brief : returns the pointer of an existing packet. nullptr if the packet
     *          token is not found;(mutually exclusive)
     */
    AcrpPacket* find(int64_t token,int64_t token_sec = 0);
    
    PacketState evaluate(int64_t token,int64_t token_sec = 0);
    PacketState evaluate(const AcrpPacket& packet);

    /**
     * @brief : marks a job done for certain packet (emitter: packet received 
     *          by the endpoint; subscriber: packet already treated by the
     *          user class).(mutually exclusive)
     * 
     * @return : false if packet is not found.
     * */
    bool        packetJobDone(int64_t token, int64_t token_sec = 0);
    bool        packetJobDone(const AcrpPacket& packet);

    /**
     * @brief : makes a copy of the packet and includes it in the stack. (mutually exclusive)
     * @return : pointer to the packet copy if operation-success. nullptr on error.
     * */
    AcrpPacket*    registerPacket(const AcrpPacket& packet);
    
    /**
     * @brief : finds a packet in the stack and removes it. (mutually exclusive)
     * @return: true on success; false on packet not found
     * */
    bool        removeFromStack(int64_t token,int64_t token_sec = 0);
    bool        removeFromStack(const AcrpPacket& packet);

    /**
     * @brief : user class pointer
     * @return: true if the pointer is not null.
     * */
    bool        set_user_pointer(void* managerptr);

    /**
     * @brief for-each wrapper - (mutually exclusive)
     * */
    size_t      forEach(std::function<bool(AcrpPacket* acrpPtr)> func);
  protected:
    StackType stack;
    void* manager;

    std::mutex stack_mtx;
    /**
     * @brief : internal "find by token" - non mutually exclusive
     * */
    AcrpPacket* _find(int64_t tp, int64_t ts);

};


} // namespace
} // eons 
} // namespace 

std::ostream& operator<<(std::ostream& i,const coyot3::communication::mqtt::AcrpPacketManagementStack::PacketState& p);
