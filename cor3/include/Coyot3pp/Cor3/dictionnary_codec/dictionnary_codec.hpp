#pragma once


/* This file contains the declarations of the following classes

  - Meaning
  - MeaningsSet

  - ConceptItem : ConceptItemsSet
  - ConceptEvaluation : ConceptEvaluationSet
  - DictionnarySection
  - EvalResult
  - EvaluationGroupCalculationResult
  - EvaluationGroupsResult
  - EvaluationGroup

  - StatesDictionnary
  
*/
#include <chrono>
#include "../Coyot3.hpp"

#include <string>
#include <time.h>

#include <ostream>
#include <jsoncpp/json/json.h>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <fstream>
#include <thread>
#include <functional>
#include <chrono>
#include <list>
#include <vector>
#include <mutex>
#include <stdexcept>


#define CYTOOLS_DICT_EVAL_NOT_INITIALIZED             "evaluation-unknown"
#define CYTOOLS_DICT_EVALRESULT_NOT_INITIALIZED       "evaluation-result-not-initialized"

namespace coyot3{
namespace tools{


  void SetDebugLevel(int l);

    enum class StateEvaluation{
      NOT_APPLICABLE = 0,
      NORMAL = 1,
      WARNING = 2,
      ERROR  = 3,
      DEBUG = 4,
      err_UNKNOWN = -1,
    };

    const char* StateEvaluationToString(StateEvaluation s);
    StateEvaluation StateEvaluationFromString(const std::string& s);


struct Meaning{
  
    struct JsField{
      static const char* value;
      static const char* description;
      static const char* evaluation;
    };
    
    Meaning();
    Meaning(const Meaning& o);
    Meaning(int64_t val,StateEvaluation sev, const std::string& eva = std::string());

    virtual ~Meaning();

    int64_t     value() const;
    int64_t     value(int64_t v);
    
    std::string description() const;
    std::string description(const std::string& d);
    
    StateEvaluation evaluation() const;
    StateEvaluation evaluation(StateEvaluation e);

    bool      operator==(const Meaning& o) const;
    Meaning&  operator= (const Meaning& o);


    bool            fromJson(const Json::Value& s);
    Json::Value     toJson(bool numericStateEval = true) const;
    std::string     toString() const;

  

    int64_t                           value_;
    std::string                       description_;
    StateEvaluation                   evaluation_;

};

struct MeaningsSet{
    typedef std::map<int64_t,Meaning>                   MeaningSetValueMap;
    typedef std::map<int64_t,Meaning>::iterator         MeaningSetValueMapIterator;
    typedef std::map<int64_t,Meaning>::const_iterator   MeaningSetValueMapConstIterator;
    
    MeaningsSet(bool is_array = true);
    MeaningsSet(const MeaningsSet& o);
    virtual ~MeaningsSet();

    MeaningsSet&  operator=(const MeaningsSet& o);

    bool          fromJson(const Json::Value& s);
    Json::Value   toJson() const;
    std::string   toString() const;

    size_t        push(const Meaning& m);
    size_t        push(const MeaningsSet& m);
    bool          get(int64_t i,Meaning& m) const;
    void          clear();
    size_t        remove(int64_t i);
    size_t        remove(const Meaning& m);
    size_t        remove(const MeaningsSet& s);

  protected:
    MeaningSetValueMap  m_meaning_set_;

    
    bool                m_as_array_; //if true, serialized as an array... else, serialized as an object with stringified values.

  };

  struct ConceptItem{ 
      typedef std::list<MeaningsSet> MeaningsSetList;
      
      struct JsField{
        static const char* name;
        static const char* id;
        static const char* description;
        static const char* meta;
        static const char* bit_start;
        static const char* length;
        static const char* check;
        static const char* evaluation;
        
      };

      ConceptItem();
      ConceptItem(const ConceptItem& o);
      virtual ~ConceptItem();
      
      ConceptItem&      operator=(const ConceptItem& o);

      bool    decode(int64_t value,Meaning& m) const;
      bool    pushMeaning(StateEvaluation s
                          ,int64_t v
                          ,const std::string& content = std::string());

      Json::Value toJson() const;
      std::string toString() const;
      bool        fromJson(const Json::Value& js);

      std::string       name_;
      std::string       id_;
      std::string       description_;
      std::string       meta_;
      int               bit_start_;
      int               length_;
      bool              check_;

      MeaningsSet       meanings_;
};


//--- concept-evaluation - end
class DictionnarySection;

struct ConceptItemsSet{
  friend class DictionnarySection;

    typedef std::map<std::string,ConceptItem>                 concept_item_set_c;
    typedef std::map<std::string,ConceptItem>::iterator       concept_item_set_iterator;
    typedef std::map<std::string,ConceptItem>::const_iterator concept_item_set_const_iterator;

    ConceptItemsSet();
    ConceptItemsSet(const ConceptItemsSet& o);
    virtual ~ConceptItemsSet();

    ConceptItemsSet& operator=(const ConceptItemsSet& o);
    
    
    Json::Value toJson() const;
    bool        fromJson(const Json::Value& source);
    std::string toString() const;
    std::string getKeys() const;
    size_t      size() const;

    bool        get(const std::string& id,ConceptItem& destination) const;
    concept_item_set_c citems_;

};




//--- concept-evaluation - begin 
struct ConceptEvaluation{

    struct JsField{
      static const char* id;
      static const char* name;
      static const char* ev;
      static const char* desc;
    };

    ConceptEvaluation();
    ConceptEvaluation(const ConceptEvaluation& o);
    ConceptEvaluation(const std::string& i,const std::string& n);
    ConceptEvaluation(const std::string& i,const std::string& n,StateEvaluation e,const std::string& d);
    ~ConceptEvaluation();

    ConceptEvaluation& operator=(const ConceptEvaluation& o);

    bool        set(const std::string& i,const std::string& n);
    bool        set(const Meaning& m);

    Json::Value toJson(bool numericStateEval = false)const;
    bool        fromJson(const Json::Value& js);
    std::string toString()const;

    std::string     id_;
    std::string     name_;
    StateEvaluation ev_;
    std::string     desc_;
};


/**
 * @brief ConceptEvaluationSet : the result of the evaluation of the 
 *  ConceptItemSet included at a DictionnarySection
 * 
 */
struct ConceptEvaluationSet{
  
    typedef std::map<std::string,ConceptEvaluation> concept_eval_set_c;
    typedef std::map<std::string,ConceptEvaluation>::iterator concept_eval_set_iterator;
    typedef std::map<std::string,ConceptEvaluation>::const_iterator concept_eval_set_const_iterator;

    typedef std::pair<std::string,ConceptEvaluation> concept_eval_set_pair_c;

    ConceptEvaluationSet();
    ConceptEvaluationSet(const ConceptEvaluationSet& o);
    virtual ~ConceptEvaluationSet();

    ConceptEvaluationSet& operator=(const ConceptEvaluationSet& o);

    Json::Value toJson(bool numericStateEval = false) const;
    bool        fromJson(const Json::Value& js);
    std::string getKeys() const;
    size_t      size() const;
    
    size_t push(const ConceptEvaluation& o);
    size_t push(const ConceptEvaluationSet& o);
    size_t push(const std::string& id,const std::string& name);

    bool   update(const std::string& id,StateEvaluation s,const std::string& desc = std::string());
    bool   update(const ConceptItem& i,int64_t sourceValue);
    size_t update(const ConceptItemsSet& i,int64_t sourceValue);

    bool    get(const std::string& id,ConceptEvaluation& eval) const;

    concept_eval_set_c cevalset_;

};

struct DictionnarySection{

    struct JsField {
      static const char* id;
      static const char* name;
      static const char* description;
      static const char* items;
    };
    DictionnarySection();
    DictionnarySection(const DictionnarySection& o);
    virtual ~DictionnarySection();

    DictionnarySection& operator=(const DictionnarySection& o);

    Json::Value toJson()const;
    bool        fromJson(const Json::Value& js);
    std::string toString() const;

    ConceptEvaluationSet evaluate(int64_t input) const;
    
    ConceptEvaluation    evaluateId(const std::string& id,int64_t value) const;

    bool get(const std::string& id,ConceptItem& destination) const;
 
    std::string             id_;
    std::string             name_;
    std::string             description_;
    ConceptItemsSet         ciset_;    
};


typedef std::map<std::string,ConceptEvaluationSet> CevalSetMap;
typedef std::map<std::string,ConceptEvaluationSet>::iterator CevalSetMapIterator;
typedef std::map<std::string,ConceptEvaluationSet>::const_iterator CevalSetMapConstIterator;



/**
 * @brief Evaluation of a evaluation group.
 * 
 */
struct EvalResult{
  struct JsField{
    static const char* content;
    static const char* state;
  };
  EvalResult();
  EvalResult(const EvalResult& o);
  ~EvalResult();
  EvalResult& operator=(const EvalResult& o);
  Json::Value toJson(bool numericStateEval = false)const;
  bool        fromJson(const Json::Value& js);
  std::string toString()const;

  std::string content;
  StateEvaluation state;
};


/**
 * @brief Complete information for the evaluation for a calculation result
 * 
 */
struct EvaluationGroupCalculationResult{
  struct JsField{
      static const char* id;
      static const char* name;
      static const char* timestamp;
      static const char* description;
      static const char* result;
      static const char* items;
  };
  
  EvaluationGroupCalculationResult();
  EvaluationGroupCalculationResult(const EvaluationGroupCalculationResult& o);
  virtual ~EvaluationGroupCalculationResult();

  EvaluationGroupCalculationResult& operator=(const EvaluationGroupCalculationResult& o);

  Json::Value toJson(bool numericStateEval = false) const;
  bool        fromJson(const Json::Value& js);

  size_t      push(const ConceptEvaluation& c); // to-do;
  size_t      calculate(); //to-do

  std::string id;
  std::string name;
  int64_t     timestamp;
  std::string description;
  EvalResult  result;
  std::vector<ConceptEvaluation> items;

};



struct EvaluationGroupsResult{
  typedef std::map<std::string,EvaluationGroupCalculationResult> EgcrSet;

  EvaluationGroupsResult();
  EvaluationGroupsResult(const EvaluationGroupsResult& o);
  virtual ~EvaluationGroupsResult();

  EvaluationGroupsResult& operator=(const EvaluationGroupsResult& o);

  Json::Value toJson(bool numericStateEval = false)const;
  bool        fromJson(const Json::Value& js);

  size_t      push(const EvaluationGroupCalculationResult& c);

  void clear();


  EgcrSet evg_results;

};

struct EvaluationGroup{
  struct JsField{
    static const char* id;
    static const char* name;
    static const char* description;

    static const char* evaluations;
    static const char* all_ok;
    static const char* all_down;
    static const char* warnings;

    static const char* items;
  };

  EvaluationGroup();
  EvaluationGroup(const EvaluationGroup& o);
  virtual ~EvaluationGroup();

  EvaluationGroup& operator=(const EvaluationGroup& o);

  Json::Value toJson() const;
  bool        fromJson(const Json::Value& js);
  std::string toString() const;

  bool calculate(const CevalSetMap& source,EvaluationGroupCalculationResult& result) const;

  std::string               id;
  std::string               name;
  std::string               description;

  EvalResult                meaning_all_ok;
  EvalResult                meaning_all_down;
  EvalResult                meaning_warnings;

  std::vector<std::string>  identifiers;

};





struct StatesDictionnary{

  


  struct JsField{
    static const char* title;
    static const char* evaluation_groups;
    static const char* sections;
  };

  StatesDictionnary();
  StatesDictionnary(const StatesDictionnary& o);
  virtual ~StatesDictionnary();
  
  StatesDictionnary& operator=(const StatesDictionnary& o);

  std::string                              title;
  std::map<std::string,EvaluationGroup>    evaluatedGroups;
  std::map<std::string,DictionnarySection> sections;

  Json::Value dictionnaryJsonize() const;
  bool        dictionnaryFromJson(const Json::Value& js);

  bool make_report_group_indexed(int64_t value,const std::string& sectionId, EvaluationGroupsResult& destination);
  bool make_report_section(int64_t value,const std::string& sectionId,ConceptEvaluationSet& cevalset);
  bool make_report_sections_all(int64_t value,CevalSetMap& cevalset);

  std::string toStringLittle();

};



}
}



std::ostream& operator << (std::ostream& o, coyot3::tools::StateEvaluation s);