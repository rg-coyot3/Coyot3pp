#include <string>

class CMqttMessage
{
public:
  void gents_now();
  void activity_ts_now();
  bool payload(const uint8_t *p, std::size_t s);
  bool payload(const std::string &p);
  CMqttMessage(const uint8_t *p, std::size_t s);
  CMqttMessage(const std::string &p);
  std::string to_string();
  typedef int64_t id_t;
  typedef int64_t creation_time_t;
  typedef int64_t last_activity_t;
  typedef std::string topic_t;
  typedef std::vector<uint8_t> payload_t;
  typedef ec::CMessagePriority priority_t;
  typedef int retries_t;
  id_t id() const;
  id_t &id();
  id_t id(const id_t &v);
  creation_time_t creation_time() const;
  creation_time_t &creation_time();
  creation_time_t creation_time(const creation_time_t &v);
  last_activity_t last_activity() const;
  last_activity_t &last_activity();
  last_activity_t last_activity(const last_activity_t &v);
  topic_t topic() const;
  topic_t &topic();
  topic_t topic(const topic_t &v);
  payload_t payload() const;
  payload_t &payload();
  payload_t payload(const payload_t &v);
  priority_t priority() const;
  priority_t &priority();
  priority_t priority(const priority_t &v);
  retries_t retries() const;
  retries_t &retries();
  retries_t retries(const retries_t &v);
  CMqttMessage &operator=(const CMqttMessage &o);
  bool operator==(const CMqttMessage &o) const;
  bool operator!=(const CMqttMessage &o) const;
  CMqttMessage();
  CMqttMessage(const CMqttMessage &o);
  CMqttMessage(const int64_t &p_id, const int64_t &p_creation_time, const int64_t &p_last_activity, const std::string &p_topic, const std::vector<uint8_t> &p_payload, const ec::CMessagePriority &p_priority, const int &p_retries);
  virtual ~CMqttMessage();
  static std::string get_model_template();

protected:
  id_t id_ = 0;
  creation_time_t creation_time_ = 0;
  last_activity_t last_activity_ = 0;
  topic_t topic_;
  payload_t payload_;
  priority_t priority_ = ec::CMessagePriority::DEFAULT;
  retries_t retries_ = 0;
};
