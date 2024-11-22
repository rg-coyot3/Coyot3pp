#pragma once

#include <math.h>
#include <string>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <list>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>




#include "cyt_swiss_knife_tools.hpp"
#include "cyt_macro_model_class.hpp"
#include "cyt_macros_model_class_serializable_json.hpp"
#include "cyt_simple_logger.hpp"
#include "../JsonSerializablePacketBase.hpp"

#include <math.h>
#include <string>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <list>

namespace coyot3{
namespace tools{
namespace geo{
  
  struct LlhCoord{
    double latitude;
    double longitude;
    double altitude;
    bool   is_data;

    LlhCoord(bool isData = true);
    LlhCoord(const LlhCoord& o);
    ~LlhCoord();

    LlhCoord& operator=(const LlhCoord& o);

    Json::Value toJsonVectorLonLat() const;
  };
  typedef std::list<LlhCoord> LlhCoordsList;

  struct EnuCoord {
    double e;
    double n;
    double u;

    EnuCoord();
    EnuCoord(const EnuCoord& o);
    EnuCoord(double c_e,double c_n,double c_u);

    EnuCoord& operator=(const EnuCoord& o);
    bool      operator==(const EnuCoord& o);
  };
  typedef std::list<EnuCoord> EnuCoordsList;

  struct EcefCoord{
    double x;
    double y;
    double z;
  };
  typedef std::list<EcefCoord> EcefCoordsList;

  struct GeoJson{
    enum class Type{
      FeatureCollection,

      err_UNKNOWN_GEOJSON_TYPE
    };

    static const char* TypeToString(Type s);
    static Type        TypeFromString(const std::string& s);


    enum class FeatureType{
      Feature,


      err_UNKNOWN_FEATURE_TYPE

    };

    static const char* FeatureTypeToString(FeatureType s);
    static FeatureType FeatureTypeFromString(const std::string& s);
    
    enum class FeatureGeometryType{
      Point,
      MultiPoint,
      LineString,
      MultiLineString,
      Polygon,
      MultiPolygon,
      err_NOT_RECOGNIZED_OR_NOT_IMPLEMENTED
    };
    static const char*  FeatureGeometryTypeToString(FeatureGeometryType s);
    static FeatureGeometryType FeatureGeometryTypeFromString(const std::string& s);
         
    struct Crs{
      struct JsField{
        static const char* type;
        static const char* properties;
        static const char* properties_name;
        static const char* properties_local_path_is_valid;

        static const char* type_default;
        static const char* properties_name_default;
        static const bool  properties_local_path_is_valid_default;
      };

      std::string type;
      std::string properties_name;
      bool        properties_path_valid;

      Crs();
      Crs(const Crs& o);
      ~Crs();

      Crs& operator=(const Crs& o);

      Json::Value toJson() const;

    };
    struct Feature{
      struct JsField {
        static const char* type;
        static const char* name;
        static const char* properties;
        static const char* geometry;
        static const char* geometry_type;
        static const char* geometry_coordinates;
      };

      Feature(GeoJson::FeatureGeometryType t, const std::string& name = std::string());
      Feature(const Feature& o);
      virtual ~Feature();
      Feature& operator=(const Feature& o);
      
      virtual Json::Value toJson() const;

      FeatureType type;
      std::string name;
      FeatureGeometryType geometry_type;

      LlhCoordsList coordinates;

      std::map<std::string,Json::Value> properties;

      size_t pushLlhCoord(const LlhCoord& c);
      size_t pushDelimiter();
      
      Feature&   addProperty(const std::string& key,const Json::Value& val);

    };

    static Feature CreateFeature(GeoJson::FeatureGeometryType type, const std::string& name = std::string());

    struct JsField{

      static const char* type;
      static const char* name;
      static const char* crs;
      static const char* features;

    };

    GeoJson(const std::string& n = std::string());
    GeoJson(const GeoJson& o);
    virtual ~GeoJson();

    GeoJson& operator=(const GeoJson& o);

    Type             type;
    std::string             name;
    Crs                     crs;
    std::vector<Feature>    features;

    size_t appendFeature(const Feature& f);
    void   clearFeatures();
    Json::Value toJson() const;

  };





  // a basic wrapper for geographic-lib

/*
# Projector type
# Also refer to https://github.com/autowarefoundation/autoware.universe/tree/main/map/map_projection_loader/README.md
string LOCAL = "local"
string LOCAL_CARTESIAN_UTM = "LocalCartesianUTM"
string MGRS = "MGRS"
string TRANSVERSE_MERCATOR = "TransverseMercator"
string projector_type

# Vertical datum
# Also refer to https://github.com/autowarefoundation/autoware.universe/tree/main/map/map_projection_loader/README.md
string WGS84 = "WGS84"
string EGM2008 = "EGM2008"
string vertical_datum

# Used for MGRS map
string mgrs_grid

# Used for some map projection types
# altitude may not be in ellipsoid height
geographic_msgs/GeoPoint map_origin


*/


  CYT3MACRO_model_class_declarations(
    GeoPoint
    , 
    , ( 
        std::string to_string() const
      , bool        proximal(const GeoPoint& other, double tolerance = -1.)
    )
    , ( )
    , latitude      , double    , 0.0
    , longitude     , double    , 0.0
    , altitude      , double    , 0.0
    , heading       , double    , 0.0
    , dist_tolerance, double    , 1.0
  )
    CYT3MACRO_model_class_set_stack_declarations(GeoPoint,)

  CYT3MACRO_model_class_declarations(
    ProjectionPoint
    , 
    , ( std::string to_string() const)
    , ( )
    , x , double , 0.
    , y , double , 0.
    , z , double , 0.
  )
    CYT3MACRO_model_class_set_stack_declarations(ProjectionPoint, )


  CYT3MACRO_model_class_declarations(
    ProjectorInformation
    , 
    , ( 
          static const std::string LOCAL
        , static const std::string CUSTOM_ENU
        , static const std::string LOCAL_CARTESIAN_UTM
        , static const std::string MGRS
        , static const std::string TRANSVERSE_MERCATOR
        , static const std::string WGS84
        , static const std::string EGM2008
        , std::string  to_string() const
        )
    , ( )
    , projector_type        , std::string   ,       ""
    , vertical_datum        , std::string   ,       ""
    , mgrs_grid             , std::string   ,       ""
    , map_origin            , GeoPoint      , 
  )

  class ProjectorInterface{
    public:
      

      ProjectorInterface();
      virtual ~ProjectorInterface();


      virtual bool initialize() = 0;
      virtual bool initialize(double lat, double lon, double alt) = 0;

              bool is_initialized();

      virtual bool forward(double i1, double i2, double i3, double& o1, double& o2, double& o3);
      virtual bool reverse(double i1, double i2, double i3, double& o1, double& o2, double& o3);

      virtual bool forward(const GeoPoint& in, ProjectionPoint& out);
      virtual bool reverse(const ProjectionPoint& in, GeoPoint& out);
      virtual bool forward(const GeoPointStack& in, ProjectionPointStack& out);
      virtual bool reverse(const ProjectionPointStack& in, GeoPointStack& out);

      ProjectorInformation& info();
      const ProjectorInformation& info() const;

    protected:
      bool                  is_initialized_;
      ProjectorInformation  proj_info_;
  };

    
  ProjectorInterface* create_projector(const ProjectorInformation& inf, bool init = true);
  



  double deg2rad(double rad);
  double rad2deg(double deg);
  double llh_distance(double lat0,double lon0,double lat1,double lon1);



  struct S_Mat_0{
    double slat;
    double clat;
    double slon;
    double clon;
    S_Mat_0();
    S_Mat_0(const S_Mat_0& o);
    S_Mat_0(double lat,double lon);
    ~S_Mat_0();
    S_Mat_0& operator=(const S_Mat_0& o);    
  };

  

  struct S_Mat_ht{

    double c0_l0;
    double c1_l0;
    double c2_l0;
    double c0_l1;
    double c1_l1;
    double c2_l1;
    double c0_l2;
    double c1_l2;
    double c2_l2;


    S_Mat_ht();
    S_Mat_ht(const S_Mat_0& s);
    S_Mat_ht(const S_Mat_ht& o);
    ~S_Mat_ht();
    S_Mat_ht& operator=(const S_Mat_ht& o);

  };

  struct RefCoords{
    double x;
    double y;
    double z;


  };

  // bool InitializeReference(double latitude,double longitude);
  // bool enu_2_ecef(double e,double n,double u,double& x,double& y,double &z);
  // bool enu_2_ecef_ref(double e,double n,double u,double& x,double& y,double &z);
  // bool ecef_2_enu(double x,double y,double z, double& e,double& n, double& u);
  // bool ecef_2_llh(double x,double y,double z,double& lat,double& lon, double& he);
  // bool ecef_2_llh_ref(double x,double y,double z,double& lat,double& lon, double& he);
  // bool llh_2_ecef(double lat,double lon,double he,double& x, double& y, double& z);

  class ProjectorLlhEnu : public ProjectorInterface{
    public:

    ProjectorLlhEnu();
    ProjectorLlhEnu(double latitude,double longitude);
    virtual ~ProjectorLlhEnu();

    bool setllh0(double lat0,double lon0);

    virtual bool initialize() override;
    virtual bool initialize(double lat, double lon, double alt) override;
    virtual bool forward(double i1, double i2, double i3, double& o1, double& o2, double& o3) override;
    virtual bool reverse(double i1, double i2, double i3, double& o1, double& o2, double& o3) override;



    bool enu_2_ecef(double e,double n,double u,double& x,double& y,double &z);
    bool enu_2_ecef_ref(double e,double n,double u,double& x,double& y,double &z);
    static bool ecef_2_enu(double x,double y,double z, double& e,double& n, double& u);
    static bool ecef_2_llh(double x,double y,double z,double& lat,double& lon, double& he);
    bool ecef_2_llh_ref(double x,double y,double z,double& lat,double& lon, double& he);
    static bool llh_2_ecef(double lat,double lon,double he,double& x, double& y, double& z);

    size_t enu_2_llh(const EnuCoordsList& input,LlhCoordsList& output);
    size_t enu_2_llh(double e,double n,double u,double& lat,double& lon, double &he);
    GeoJson::Feature enu_2_geojson_feature_linestring(const EnuCoordsList& input);
    
    //------------------------------
    double    lat_0;
    double    lon_0;
    S_Mat_0   mat_s;
    S_Mat_ht  ht;
    RefCoords ecefRef;

  };




  class ProjectorLlhMgrs : public ProjectorInterface{
    public:

      struct Defaults{
        static int  coordinates_precision;
        static int  str_size_zone;
        static bool mgrs_rev_centerp;
      };



      struct Point{
        double x;
        double y;
        double z;
      };

      ProjectorLlhMgrs();
      /**
       * @brief constructor with reference initializer
       * @param 
       */
      ProjectorLlhMgrs(double lat, double lon, double alt);
      ProjectorLlhMgrs(const ProjectorLlhMgrs& o);
      virtual ~ProjectorLlhMgrs();

      virtual bool initialize() override;
      virtual bool initialize(double lat,double lon, double alt) override;


      virtual bool forward(double lat, double lon, double alt, double& x, double& y, double& z) override;
      virtual bool reverse(double x, double y, double z, double& lat, double& lon, double& alt) override;
      virtual bool reverse(const ProjectionPointStack& in, GeoPointStack& out) override;
      virtual bool forward(const GeoPointStack& in, ProjectionPointStack& out) override;
      virtual bool reverse(const ProjectionPoint& in, GeoPoint& out) override;
      virtual bool forward(const GeoPoint& in, ProjectionPoint& out) override;


    protected:
      double initial_lat_;
      double initial_lon_;
      double initial_alt_;


      int   coordinates_precision_  = 7;
      int   str_size_zone_          = 5;
      bool  mgrs_rev_centerp_       = false;

      int           init_zone_;
      bool          init_northp_;
      double        init_utmups_x_;
      double        init_utmups_y_;
      double        init_utmups_z_;

      double        init_utmups_rev_x_;
      double        init_utmups_rev_y_;
      double        init_utmups_rev_z_;
      int           init_utmups_rev_crpr_;

      std::string   init_mgrs_code_;

      float         north_;
      std::string   north_src_str_;

      float         east_;
      std::string   east_src_str_;

      
      




  };












}//ens geo
}//ens tools
}//ens coyot3