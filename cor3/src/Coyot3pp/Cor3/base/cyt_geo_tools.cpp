#include <Coyot3pp/Cor3/base/cyt_geo_tools.hpp>



namespace coyot3{
namespace tools{
namespace geo{


const double RAD_TO_DEG = 180.0 /M_PI;
const double DEG_TO_RAD = M_PI / 180.0;
const double HALF_PI = M_PI/2.0;
const double EPSILON10 = pow(10,-6);
const double EPSILON12 = pow(10,-12);

const double EQUATOR_RADIUS = 6378137.0;
const double FLATTENING = 1.0 / 298.257223563;
const double SQUARED_ECCENTRICITY = ((2.0 * FLATTENING) - (FLATTENING * FLATTENING));
const double SQUARE_ROOT_ONE_MINUS_ECCENTRICITY = 0.996647189335;
const double POLE_RADIUS = (EQUATOR_RADIUS * SQUARE_ROOT_ONE_MINUS_ECCENTRICITY);

const double C00 = (double)(1.0);
const double C02 = (double)(0.25);
const double C04 = (double)(0.046875);
const double C06 = (double)(0.01953125);
const double C08 = (double)(0.01068115234375);
const double C22 = (double)(0.75);
const double C44 = (double)(0.46875);
const double C46 = (double)(0.01302083333333333333);
const double C48 = (double)(0.00712076822916666666);
const double C66 = (double)(0.36458333333333333333);
const double C68 = (double)(0.00569661458333333333);
const double C88 = (double)(0.3076171875);
const double MAXNUMBER = pow(2.0,53.0);

const double R0 = (C00 - SQUARED_ECCENTRICITY * 
                 (C02 + SQUARED_ECCENTRICITY * 
                 (C04 + SQUARED_ECCENTRICITY * 
                 (C06 + SQUARED_ECCENTRICITY * C08))));
const double R1 = (SQUARED_ECCENTRICITY * 
                 (C22 - SQUARED_ECCENTRICITY * 
                 (C04 + SQUARED_ECCENTRICITY * 
                 (C06 + SQUARED_ECCENTRICITY * C08))));
const double R2T = (SQUARED_ECCENTRICITY * 
                 SQUARED_ECCENTRICITY);
const double R2 = (R2T * (C44 - SQUARED_ECCENTRICITY *
                 (C46 + SQUARED_ECCENTRICITY * C48)));
const double R3T = (R2T * SQUARED_ECCENTRICITY);
const double R3 = (R3T * (C66 - SQUARED_ECCENTRICITY * C68));
const double R4 = (R3T * SQUARED_ECCENTRICITY * C88);

const double GRS_a = 6378137;
const double GRS_f = 1.0/298.257222101;
const double GRS_b = GRS_a*(1.0-GRS_f);
const double GRS_e = sqrt((pow(GRS_a,2) - pow(GRS_b,2)) / pow(GRS_a,2));
const double GRS_esq = pow(GRS_e,2); // Square of Eccentricity
const double GRS_eps = (GRS_esq / (1.0-GRS_esq));




double lat_0 = 0.0;
double lon_0 = 0.0;

S_Mat_0::S_Mat_0()
:slat(0.0)
,clat(0.0)
,slon(0.0)
,clon(0.0)
{

}
S_Mat_0::S_Mat_0(const S_Mat_0& o)
{
  *this = o;
}
S_Mat_0::S_Mat_0(double lat, double lon)
{
  slat = sin(DEG_TO_RAD*lat);
  clat = cos(DEG_TO_RAD*lat);
  slon = sin(DEG_TO_RAD*lon);
  clon = cos(DEG_TO_RAD*lon);
}
S_Mat_0::~S_Mat_0(){}
S_Mat_0& S_Mat_0::operator=(const S_Mat_0& o)
{
  slat = o.slat;
  clat = o.clat;
  slon = o.slon;
  clon = o.clon;
  return *this;
}



S_Mat_ht::S_Mat_ht()
:c0_l0(0.0)
,c1_l0(0.0)
,c2_l0(0.0)
,c0_l1(0.0)
,c1_l1(0.0)
,c2_l1(0.0)
,c0_l2(0.0)
,c1_l2(0.0)
,c2_l2(0.0)
{

}
S_Mat_ht::S_Mat_ht(const S_Mat_0& s)
{
  c0_l0 = -s.slon;
  c1_l0 = -s.slat * s.clon;
  c2_l0 =  s.clat * s.clon;

  c0_l1 =  s.clon;
  c1_l1 = -s.slon * s.slat;
  c2_l1 =  s.clat * s.slon;
  
  c0_l2 =  0.0;
  c1_l2 =  s.clat;
  c2_l2 =  s.slat;
}
S_Mat_ht::S_Mat_ht(const S_Mat_ht& o)
{
  *this = o;
}
S_Mat_ht::~S_Mat_ht()
{

}
S_Mat_ht& S_Mat_ht::operator=(const S_Mat_ht& o)
{
  c0_l0 = o.c0_l0;
  c1_l0 = o.c1_l0;
  c2_l0 = o.c2_l0;
  c0_l1 = o.c0_l1;
  c1_l1 = o.c1_l1;
  c2_l1 = o.c2_l1;
  c0_l2 = o.c0_l2;
  c1_l2 = o.c1_l2;
  c2_l2 = o.c2_l2;
  return *this;
}




// S_Mat_0   mat_s;
// S_Mat_ht  ht;
// RefCoords ecefRef;


// bool InitializeReference(double latitude,double longitude)
// {
//   lat_0=latitude;
//   lon_0=longitude;
//   mat_s = S_Mat_0(latitude,longitude);
//   ht    = S_Mat_ht(mat_s);
//   llh_2_ecef(latitude,longitude,0,ecefRef.x,ecefRef.y,ecefRef.z);
//   CLOG_INFO("coyot3 : tools-geo-initialize : initializing with (lat:" 
//     << latitude << ",lon:" <<longitude << ") => [" << ecefRef.x << "," << ecefRef.y << "]");  
//   return true;
// }

// bool enu_2_ecef(double e,double n,double u,double& x,double& y,double &z)
// {
//   /*
// return {
//              x : (this.ht.c0_l0*e + this.ht.c1_l0*n + this.ht.c2_l0*u) + this.geo_reference.ecefh[0]
//             ,y : (this.ht.c0_l1*e + this.ht.c1_l1*n + this.ht.c2_l1*u) + this.geo_reference.ecefh[1]
//             ,z : this.ht.c0_l2*e + this.ht.c1_l2*n + this.ht.c2_l2*u   + this.geo_reference.ecefh[2]
//         }

//   */
//   x = (ht.c0_l0*e + ht.c1_l0*n + ht.c2_l0*u);
//   y = (ht.c0_l1*e + ht.c1_l1*n + ht.c2_l1*u);
//   z =  ht.c0_l2*e + ht.c1_l2*n + ht.c2_l2*u ;

//   return true;
// }

// bool enu_2_ecef_ref(double e,double n,double u,double& x,double& y,double &z)
// {
//   x = (ht.c0_l0*e + ht.c1_l0*n + ht.c2_l0*u) + ecefRef.x;
//   y = (ht.c0_l1*e + ht.c1_l1*n + ht.c2_l1*u) + ecefRef.y;
//   z = (ht.c0_l2*e + ht.c1_l2*n + ht.c2_l2*u) + ecefRef.z;

//   return true;
// }

// bool ecef_2_llh(double x,double y,double z,double& lat,double& lon, double& he)
// {
//   double p = sqrt(pow(x,2.0) + pow(y,2.0));
  
//   double q = atan2((z * GRS_a), (p * GRS_b));

//   double sin_q = sin(q);
//   double cos_q = cos(q);
  
//   // double sin_q_3 = sin(q) * sin(q) * sin(q);
//   // double cos_q_3 = cos(q) * cos(q) * cos(q);
//   double sin_q_3 = pow(sin(q),3);
//   double cos_q_3 = pow(cos(q),3);
//   double phi = atan2(
//           (z + GRS_eps * GRS_b * sin_q_3),
//           (p - GRS_esq * GRS_a * cos_q_3)
//   );
  
//   double v = GRS_a /sqrt(1.0 - GRS_esq * pow(sin(phi),2));
//   double height = (p / cos(phi)) - v;

//   lat = RAD_TO_DEG * phi;
//   lon = RAD_TO_DEG * atan2(y,x);
//   he  = height;
//   CLOG_DEBUG(6,"coyot3 : tools : ecef-2-geo-ref : (" <<lat << ";" << lon << ";" << he << ")");
//   CLOG_DEBUG(6,"coyot3 : tools-geo-initialize : REFERENCES with => [" << ecefRef.x << "," << ecefRef.y << "]");  
//   return true;
// }

// bool ecef_2_llh_ref(double x,double y,double z,double& lat,double& lon, double& he)
// {

//   //x+=ecefRef.x;
//   //y+=ecefRef.y;
//   //z+=ecefRef.z;

//   return ecef_2_llh(x + ecefRef.x, y+ ecefRef.y,z+ ecefRef.z,lat,lon,he);
  
// }



// bool llh_2_ecef(double lat,double lon,double he,double& x, double& y, double& z)
// {
//   double n = GRS_a / sqrt(1.0 - pow(GRS_e,2.0) * pow(sin(DEG_TO_RAD*lat),2.0));

//   x = (n + he) * cos(DEG_TO_RAD*lat) * cos(DEG_TO_RAD*lon);
//   y = (n + he) * cos(DEG_TO_RAD*lat) * sin(DEG_TO_RAD*lon);
//   z = (n * (1.0 - pow(GRS_e,2.0)) + he) * sin(DEG_TO_RAD*lat);

//   return true;
// }



// bool ecef_2_enu(double x,double y,double z, double& e,double& n, double& u)
// {
//   return false;
// }




LlhCoord::LlhCoord(bool isData)
:latitude(0.0)
,longitude(0.0)
,altitude(0.0)
,is_data(isData){

}
LlhCoord::LlhCoord(const LlhCoord& o){
  *this = o;
}
LlhCoord::~LlhCoord(){
  
}
LlhCoord& LlhCoord::operator=(const LlhCoord& o){
  latitude  = o.latitude;
  longitude = o.longitude;
  altitude  = o.altitude;
  is_data = o.is_data;

  return *this;
}


Json::Value LlhCoord::toJsonVectorLonLat() const{
  Json::Value js(Json::arrayValue);

  js.append(longitude);
  js.append(latitude);

  return js;
}

EnuCoord::EnuCoord():e(0.0),n(0.0),u(0.0){}
EnuCoord::EnuCoord(const EnuCoord& o){*this = o;}
EnuCoord::EnuCoord(double c_e,double c_n,double c_u)
:e(e)
,n(n)
,u(u)
{

}
EnuCoord& EnuCoord::operator=(const EnuCoord& o){
  e=o.e;n=o.n;u=o.u;return *this;
}
bool EnuCoord::operator==(const EnuCoord& o){ return ((e == o.e) && (n == o.n) && (u == o.u));}


const char* GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType s)
{
  switch(s)
  {
    case FeatureGeometryType::Point: return "Point";break;
    case FeatureGeometryType::MultiPoint: return "MultiPoint";break;
    case FeatureGeometryType::LineString: return "LineString";break;
    case FeatureGeometryType::MultiLineString: return "MultiLineString";break;
    case FeatureGeometryType::Polygon: return "Polygon";break;
    case FeatureGeometryType::MultiPolygon: return "MultiPolygon";break;
    case FeatureGeometryType::err_NOT_RECOGNIZED_OR_NOT_IMPLEMENTED: 
    default:
      return "err_NOT_RECOGNIZED_OR_NOT_IMPLEMENTED";
  }
}
GeoJson::FeatureGeometryType GeoJson::FeatureGeometryTypeFromString(const std::string& s)
{
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::Point)))return GeoJson::FeatureGeometryType::Point;
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::MultiPoint)))return GeoJson::FeatureGeometryType::MultiPoint;
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::LineString)))return GeoJson::FeatureGeometryType::LineString;
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::MultiLineString)))return GeoJson::FeatureGeometryType::MultiLineString;
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::Polygon)))return GeoJson::FeatureGeometryType::Polygon;
  if(!s.compare(GeoJson::FeatureGeometryTypeToString(GeoJson::FeatureGeometryType::MultiPolygon)))return GeoJson::FeatureGeometryType::MultiPolygon;
  return GeoJson::FeatureGeometryType::err_NOT_RECOGNIZED_OR_NOT_IMPLEMENTED;
}



const char* GeoJson::Crs::JsField::type = "type";
const char* GeoJson::Crs::JsField::properties = "properties";
const char* GeoJson::Crs::JsField::properties_name = "name";
const char* GeoJson::Crs::JsField::properties_local_path_is_valid = "local_path_is_valid";

const char* GeoJson::Crs::JsField::type_default = "name";
const char* GeoJson::Crs::JsField::properties_name_default = "urn:ogc:def:crs:OGC:1.3:CRS84";
const bool  GeoJson::Crs::JsField::properties_local_path_is_valid_default = true;

GeoJson::Crs::Crs()
{
  type = JsField::type_default;
  properties_name = JsField::properties_name_default;
  properties_path_valid = JsField::properties_local_path_is_valid_default;
}
GeoJson::Crs::Crs(const GeoJson::Crs& o)
{
  *this = o;
}
GeoJson::Crs::~Crs(){

}

GeoJson::Crs& GeoJson::Crs::operator=(const GeoJson::Crs& o)
{
  type = o.type;
  properties_name = o.properties_name;
  properties_path_valid = o.properties_path_valid;
  return *this;
}


Json::Value GeoJson::Crs::toJson() const{
  Json::Value js;

  js[JsField::type] = type;
  js[JsField::properties][JsField::properties_name] = properties_name;
  js[JsField::properties][JsField::properties_local_path_is_valid] = properties_path_valid;

  return js;
}

const char* GeoJson::Feature::JsField::type = "type";
const char* GeoJson::Feature::JsField::name = "name";
const char* GeoJson::Feature::JsField::properties = "properties";
const char* GeoJson::Feature::JsField::geometry = "geometry";
const char* GeoJson::Feature::JsField::geometry_type = "type";
const char* GeoJson::Feature::JsField::geometry_coordinates = "coordinates";

GeoJson::Feature::Feature(GeoJson::FeatureGeometryType t,const std::string& featureName)
:type(FeatureType::Feature)
,name(featureName)
,geometry_type(t)
,coordinates()
,properties()
{

}
GeoJson::Feature::Feature(const GeoJson::Feature& o)
{
  //CLOG_INFO(" --- to delete : creating instance of feature geometry type [" << FeatureGeometryTypeToString(o.geometry_type) << "]");
  *this = o;
  //CLOG_INFO(" --- to delete : creating instance of feature geometry type resulted in [" << FeatureGeometryTypeToString(geometry_type) << "]");
}
GeoJson::Feature::~Feature(){

}



GeoJson::Feature& GeoJson::Feature::operator=(const GeoJson::Feature& o)
{
  type = o.type;
  name = o.name;
  geometry_type = o.geometry_type;
  coordinates = o.coordinates;
  properties = o.properties;

  return *this;
}

Json::Value GeoJson::Feature::toJson() const{
  Json::Value js;

  js[JsField::type] = FeatureTypeToString(type);
  js[JsField::name] = name;
  js[JsField::properties] = Json::Value(Json::objectValue);
  for(const std::pair<std::string,Json::Value>& p : properties)
  {
    js[JsField::properties][p.first] = p.second;
  }
  js[JsField::geometry][JsField::geometry_type] = GeoJson::FeatureGeometryTypeToString(geometry_type);
  
  Json::Value jscoord;
  switch(geometry_type)
  {
    case GeoJson::FeatureGeometryType::Point:
      CLOG_INFO(" --- --- feature : point");
      jscoord = coordinates.front().toJsonVectorLonLat();
      break;
    case GeoJson::FeatureGeometryType::LineString:
    case GeoJson::FeatureGeometryType::Polygon:
    case GeoJson::FeatureGeometryType::MultiPoint:
      {
        //CLOG_INFO(" --- to-delete --- feature : linestring polygon multipoint");
        jscoord = Json::Value(Json::arrayValue);
        for(const LlhCoord& c : coordinates)
        {
          if(c.is_data)
          {
            //CLOG_INFO(" --- to-delete --- feature : adding ((" << c.toJsonVectorLonLat());
            jscoord.append(c.toJsonVectorLonLat());
          }
        }
      }
      break;
    case GeoJson::FeatureGeometryType::MultiLineString:
    case GeoJson::FeatureGeometryType::MultiPolygon:
      {
        jscoord = Json::Value(Json::arrayValue);
        Json::Value buffer = Json::Value(Json::arrayValue);
        for(const LlhCoord& c : coordinates)
        {
          if(c.is_data)
          {
            buffer.append(c.toJsonVectorLonLat());
          }else{
            jscoord.append(buffer);
            buffer.clear();
            buffer = Json::Value(Json::arrayValue);
          }
        }
        if(buffer.size())
        {
          jscoord.append(buffer);
        }
      }
      break;
  }
  js[JsField::geometry][JsField::geometry_coordinates] = jscoord;
  return js;
}

size_t GeoJson::Feature::pushLlhCoord(const LlhCoord& c)
{
  coordinates.push_back(c);
  return static_cast<size_t>(1);
}
size_t GeoJson::Feature::pushDelimiter()
{
  LlhCoord coord(false);
  coordinates.push_back(coord);
  return static_cast<size_t>(1);
}

GeoJson::Feature& GeoJson::Feature::addProperty(const std::string& key,const Json::Value& val)
{
  if(properties.find(key) != properties.end())
  {
    properties.find(key)->second = val;
    
  }else{
    properties.insert(std::make_pair(key,val));
  }
  return *this;
}



GeoJson::Feature GeoJson::CreateFeature(GeoJson::FeatureGeometryType t, const std::string& name)
{
  return Feature(t,name);
}










const char* GeoJson::JsField::name     = "name";
const char* GeoJson::JsField::type     = "type";
const char* GeoJson::JsField::crs      = "crs";
const char* GeoJson::JsField::features = "features";


const char* GeoJson::TypeToString(GeoJson::Type s)
{
  switch(s)
  {
    case Type::FeatureCollection: return "FeatureCollection";break;
    case Type::err_UNKNOWN_GEOJSON_TYPE : 
    default : 
      return "err_UNKNOWN_GEOJSON_TYPE";
  }
}

GeoJson::Type GeoJson::TypeFromString(const std::string& s)
{
  if(!s.compare(GeoJson::TypeToString(GeoJson::Type::FeatureCollection)))return Type::FeatureCollection;
  return Type::err_UNKNOWN_GEOJSON_TYPE;
}


const char* GeoJson::FeatureTypeToString(FeatureType s)
{
  switch(s){
    case FeatureType::Feature: return "Feature";break;
    case FeatureType::err_UNKNOWN_FEATURE_TYPE:
    default:
      return "err_UNKNOWN_FEATURE_TYPE";
  }
}
GeoJson::FeatureType GeoJson::FeatureTypeFromString(const std::string& s)
{
  if(!s.compare(GeoJson::FeatureTypeToString(GeoJson::FeatureType::Feature)))return GeoJson::FeatureType::Feature;
  return FeatureType::err_UNKNOWN_FEATURE_TYPE;
}









GeoJson::GeoJson(const std::string& n)
:type(GeoJson::Type::FeatureCollection)
,name(n)
,crs()
,features()
{

}
GeoJson::GeoJson(const GeoJson& o)
{
  *this = o;
}
GeoJson::~GeoJson()
{

}

GeoJson& GeoJson::operator=(const GeoJson& o)
{
  type      = o.type;
  name      = o.name;
  crs       = o.crs;
  features  = o.features;

  return *this;
}


size_t GeoJson::appendFeature(const Feature& f)
{
  features.push_back(f);
  return static_cast<size_t>(1);
}

Json::Value GeoJson::toJson() const{
  Json::Value js;
  
  js[JsField::type] = TypeToString(type);
  js[JsField::name] = name;
  js[JsField::crs]  = crs.toJson();
  js[JsField::features] = Json::Value(Json::arrayValue);

  for(const Feature& f: features)
  {
    js[JsField::features].append(f.toJson());
  }

  return js;
}































ProjectorLlhEnu::ProjectorLlhEnu()
:lat_0(0.0)
,lon_0(0.0)
,mat_s()
,ht()
,ecefRef()
{

}

ProjectorLlhEnu::ProjectorLlhEnu(double latitude,double longitude)
:lat_0(latitude)
,lon_0(longitude)
,mat_s()
,ht()
,ecefRef()
{
  initialize();
}

ProjectorLlhEnu::~ProjectorLlhEnu(){

}
bool ProjectorLlhEnu::setllh0(double lat0,double lon0)
{
  lat_0 = lat0;
  lon_0 = lon0;
  return true;
}



bool ProjectorLlhEnu::initialize()
{
  CLOG_INFO("CYTOOLS : geo : bridge-llh-enu : initializing for (lat:" 
    << this->lat_0 
    <<",lon: " 
    << this->lon_0 << ")")
  this->mat_s = S_Mat_0(this->lat_0,this->lon_0);
  this->ht = S_Mat_ht(this->mat_s);
  llh_2_ecef(this->lat_0,this->lon_0,0,this->ecefRef.x,this->ecefRef.y,this->ecefRef.z);
  CLOG_INFO("coyot3 : tools : geo : bridge-llh-enu : initialize : "
    "lat:" << lat_0 << ",lon:" << lon_0 << ")"
    " => [x:" << ecefRef.x << ", y:" << ecefRef.y << ", z:" << ecefRef.z << "]");
  is_initialized_ = true;
  return true;
}

bool ProjectorLlhEnu::initialize(double lat, double lon, double alt){
  lat_0 = lat;
  lon_0 = lon;
  return initialize();
  return true;
}

bool ProjectorLlhEnu::enu_2_ecef(double e,double n,double u,double& x,double& y,double &z)
{
  /*
return {
    x : (this.ht.c0_l0*e + this.ht.c1_l0*n + this.ht.c2_l0*u) + this.geo_reference.ecefh[0]
   ,y : (this.ht.c0_l1*e + this.ht.c1_l1*n + this.ht.c2_l1*u) + this.geo_reference.ecefh[1]
   ,z : (this.ht.c0_l2*e + this.ht.c1_l2*n + this.ht.c2_l2*u) + this.geo_reference.ecefh[2]
        }
  */
    x = (this->ht.c0_l0*e + this->ht.c1_l0*n + this->ht.c2_l0*u);
    y = (this->ht.c0_l1*e + this->ht.c1_l1*n + this->ht.c2_l1*u);
    z =  this->ht.c0_l2*e + this->ht.c1_l2*n + this->ht.c2_l2*u ;

    return true;
}


bool ProjectorLlhEnu::enu_2_ecef_ref(double e,double n,double u,double& x,double& y,double &z)
{
  x = (this->ht.c0_l0*e + this->ht.c1_l0*n + this->ht.c2_l0*u) + this->ecefRef.x;
  y = (this->ht.c0_l1*e + this->ht.c1_l1*n + this->ht.c2_l1*u) + this->ecefRef.y;
  z = (this->ht.c0_l2*e + this->ht.c1_l2*n + this->ht.c2_l2*u) + this->ecefRef.z;

  return true;
}

bool ProjectorLlhEnu::ecef_2_llh(double x,double y,double z,double& lat,double& lon, double& he)
{
  double p = sqrt(pow(x,2.0) + pow(y,2.0));
  
  double q = atan2((z * GRS_a), (p * GRS_b));

  double sin_q = sin(q);
  double cos_q = cos(q);
  
  // double sin_q_3 = sin(q) * sin(q) * sin(q);
  // double cos_q_3 = cos(q) * cos(q) * cos(q);
  double sin_q_3 = pow(sin(q),3);
  double cos_q_3 = pow(cos(q),3);
  double phi = atan2(
          (z + GRS_eps * GRS_b * sin_q_3),
          (p - GRS_esq * GRS_a * cos_q_3)
  );
  
  double v = GRS_a /sqrt(1.0 - GRS_esq * pow(sin(phi),2));
  double height = (p / cos(phi)) - v;

  lat = RAD_TO_DEG * phi;
  lon = RAD_TO_DEG * atan2(y,x);
  he  = height;
  CLOG_DEBUG(6,"coyot3 : tools : ecef-2-geo-ref : (" <<lat << ";" << lon << ";" << he << ")");
  //CLOG_DEBUG(6,"coyot3 : tools-geo-initialize : REFERENCES with => [" << ecefRef.x << "," << ecefRef.y << "]");  
  return true;
}

bool ProjectorLlhEnu::ecef_2_llh_ref(double x,double y,double z,double& lat,double& lon, double& he)
{
  //x+=ecefRef.x;
  //y+=ecefRef.y;
  //z+=ecefRef.z;

  return ecef_2_llh(x + ecefRef.x, y+ ecefRef.y,z+ ecefRef.z,lat,lon,he); 
}

bool ProjectorLlhEnu::llh_2_ecef(double lat,double lon,double he,double& x, double& y, double& z)
{
  double n = GRS_a / sqrt(1.0 - pow(GRS_e,2.0) * pow(sin(DEG_TO_RAD*lat),2.0));

  x = (n + he) * cos(DEG_TO_RAD*lat) * cos(DEG_TO_RAD*lon);
  y = (n + he) * cos(DEG_TO_RAD*lat) * sin(DEG_TO_RAD*lon);
  z = (n * (1.0 - pow(GRS_e,2.0)) + he) * sin(DEG_TO_RAD*lat);

  return true;
}
bool ProjectorLlhEnu::ecef_2_enu(double x,double y,double z, double& e,double& n, double& u)
{
  //js : Math.fmod = function (a,b) { return Number((a - (Math.floor(a / b) * b)).toPrecision(8)); };
  return false;
}



size_t ProjectorLlhEnu::enu_2_llh(const EnuCoordsList& input,LlhCoordsList& output)
{
  output.clear();
  size_t sz = 0;
  for(const EnuCoord& enu : input)
  {
    EcefCoord ecef;
    this->enu_2_ecef_ref(enu.e,enu.n,enu.u,ecef.x,ecef.y,ecef.z);
    LlhCoord llh;
    this->ecef_2_llh(ecef.x,ecef.y,ecef.z,llh.latitude,llh.longitude,llh.altitude);
    output.push_back(llh);
    ++sz;
  }
  return sz;
}
size_t ProjectorLlhEnu::enu_2_llh(double e,double n,double u,double& lat,double& lon, double &he)
{

  double x,y,z;
  this->enu_2_ecef_ref(e,n,u,x,y,z);
  this->ecef_2_llh(x,y,z,lat,lon,he);
  return 1;
}

bool ProjectorLlhEnu::forward(double i1, double i2, double i3, double& o1, double& o2, double& o3){
  double e1,e2,e3;
  enu_2_ecef(i1,i2,i3,e1,e2,e3);
  ecef_2_enu(e1,e2,e3,o1,o2,o3);
  return true;
}

bool ProjectorLlhEnu::reverse(double i1, double i2, double i3, double& o1, double& o2, double& o3){
  if(!is_initialized())return false;
  double e1,e2,e3;
  enu_2_ecef_ref(i1,i2,i3,e1,e2,e3);
  ecef_2_llh(e1,e2,e3,o1,o2,o3);
  return true; 
}

GeoJson::Feature ProjectorLlhEnu::enu_2_geojson_feature_linestring(const EnuCoordsList& input)
{
  GeoJson::Feature feature(GeoJson::FeatureGeometryType::LineString);
  for(const EnuCoord& enu : input)
  {
    EcefCoord ecef;
    
    this->enu_2_ecef_ref(enu.e,enu.n,enu.u,ecef.x,ecef.y,ecef.z);
    LlhCoord llh;
    this->ecef_2_llh(ecef.x,ecef.y,ecef.z,llh.latitude,llh.longitude,llh.altitude);
    feature.pushLlhCoord(llh);
  }
  return feature;
  
}

double deg2rad(double deg) {
  return (deg * M_PI / 180.0);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180.0 / M_PI);
}

double llh_distance(double lat0,double lon0,double lat1,double lon1)
{
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat0);
  lon1r = deg2rad(lon0);
  lat2r = deg2rad(lat1);
  lon2r = deg2rad(lon1);
  u = sin((lat2r - lat1r)/2.0);
  v = sin((lon2r - lon1r)/2.0);
  return (2.0 * EARTH_RADIUS_KM * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)))*1000.0;

}


// bridge llh < == > 6dof
  
  int   ProjectorLlhMgrs::Defaults::coordinates_precision = 0;
  int   ProjectorLlhMgrs::Defaults::str_size_zone         = 7;
  bool  ProjectorLlhMgrs::Defaults::mgrs_rev_centerp      = false;

  ProjectorLlhMgrs::ProjectorLlhMgrs()
  : ProjectorInterface()
  {
    coordinates_precision_ = Defaults::coordinates_precision;
    str_size_zone_         = Defaults::str_size_zone;
    mgrs_rev_centerp_      = Defaults::mgrs_rev_centerp;
  }

  ProjectorLlhMgrs::ProjectorLlhMgrs(double lat, double lon, double alt)
  : ProjectorInterface()
  {

  }
  ProjectorLlhMgrs::~ProjectorLlhMgrs()
  {

  }

  bool ProjectorLlhMgrs::initialize(double lat,double lon, double alt){
    CLOG_INFO("br-llh-2-6dof : initialize : (lat:" << lat << ",lon:" << lon 
      << ",alt:" << alt << ")")
    proj_info_.map_origin().latitude(lat);
    proj_info_.map_origin().longitude(lon);
    proj_info_.map_origin().altitude(alt);
    return initialize();
  }
  bool ProjectorLlhMgrs::initialize(){
    initial_lat_ = proj_info_.map_origin().latitude();
    initial_lon_ = proj_info_.map_origin().longitude();
    initial_alt_ = proj_info_.map_origin().altitude();
    is_initialized_ = false;
    try{
      CLOG_INFO("projector-llj-mgrs : initialize : forwarding to utmups ")
      
      GeographicLib::UTMUPS::Forward(initial_lat_,
                                      initial_lon_,
                                      init_zone_,
                                      init_northp_, 
                                      init_utmups_x_, 
                                      init_utmups_y_);
      CLOG_INFO("projector-llj-mgrs : initialize : forwarding to mgrs ")
      GeographicLib::MGRS::Forward(init_zone_, 
                                      init_northp_, 
                                      init_utmups_x_, 
                                      init_utmups_y_, 
                                      initial_lat_, 
                                      coordinates_precision_, 
                                      init_mgrs_code_);
      CLOG_INFO("projector-llj-mgrs : initialize : obtained [" << init_mgrs_code_ << "] ")

      GeographicLib::MGRS::Reverse(init_mgrs_code_,
                                      init_zone_, 
                                      init_northp_,
                                      init_utmups_rev_x_, 
                                      init_utmups_rev_y_, 
                                      init_utmups_rev_crpr_,false);

    }catch(const GeographicLib::GeographicErr& err){
      CLOG_WARN("projector-llh-mgrs : initialize : error initializing (" 
        << initial_lat_ << ", " << initial_lon_ << ", " << initial_alt_ << ") : except.geographic(" << err.what() << ")")
      return false;
    }
    is_initialized_ = true;
    return true;
  }

  bool ProjectorLlhMgrs::forward(double lat, double lon, double alt, double& x, double& y, double& z)
  {
    int         zone;
    bool        is_north;
    int precision = 7;
    std::string mgrs_code;
    CLOG_INFO("** to-delete : input: lat=" << lat << ", lon=" << lon << ", alt=" << alt)
    try{
      GeographicLib::UTMUPS::Forward(lat,
                                     lon,
                                     zone, 
                                     is_north, 
                                     x, 
                                     y);
      GeographicLib::MGRS::Forward(zone,
                                   is_north,
                                   x,
                                   y,
                                   precision,
                                   mgrs_code);
      std::stringstream iss(mgrs_code);
    }catch(const GeographicLib::GeographicErr& err){
      CLOG_WARN("projector-llh-mgrs : forward : error projecting (" 
        << lat << ", " << lon << ") : except.geographic(" << err.what() << ")")
      return false;
    }

    try{
      CLOG_INFO("** to-delete : forward : working with [" << mgrs_code 
        << "] - precision = [" << precision << "]"
        << " - " << (is_north==true?"NORTH":"SOUTH") );
      std::string east = mgrs_code.substr(5,precision);
      std::string north= mgrs_code.substr(precision + 5, precision);
      x = std::stof(east) * pow (10, 5 - precision);
      y = std::stof(north)* pow (10, 5 - precision);
    }catch(const std::invalid_argument& e){
      CLOG_WARN("projector-llh-mgrs : forward : error parsing mgrs-string : "
      "exception.inval-arg(" << e.what() << ")")
      return false;
    }catch(const std::out_of_range& e){
      CLOG_WARN("projector-llh-mgrs : forward : error parsing mgrs-string : "
      "exception.out-of-range(" << e.what() << ")")
      return false;
    }catch(...){
      CLOG_WARN("projector-llh-mgrs : forward : error parsing mgrs-string : "
      "exception.UNKNOWN( ... )")
      return false;
    }
    
    z = alt;
    return true;
  }

  bool ProjectorLlhMgrs::reverse(double x, double y, double z, double& lat, double& lon, double& alt){
    double ux_,uy_;
    if(is_initialized() == false)return false;
    
    try{
      int precision = 15;
      //CLOG_INFO("*** to delete : precision al inicio :" << precision)
      // GeographicLib::MGRS::Reverse(init_mgrs_code_,
      //                                 init_zone_, 
      //                                 init_northp_,
      //                                 init_utmups_rev_x_, 
      //                                 init_utmups_rev_y_, 
      //                                 precision,
      //                                 false);
      //CLOG_INFO("*** to delete : precision despues   :" << precision)
      //CLOG_INFO("to-delete: antes : x,=" << init_utmups_x_ << "\ty.=" << init_utmups_y_)
      //CLOG_INFO("to-delete: antes : x0=" << init_utmups_rev_x_ << "\ty0=" << init_utmups_rev_y_)
      //CLOG_INFO("to-delete: antes : x =" << x << "\t;y=" << y << "\t;z=" << z)

      //CLOG_INFO("to-delete : fmod(x, pow(10, precision)) = " << fmod(x, pow(10, 5 - 1)))
      //CLOG_INFO("to-delete : fmod(y, pow(10, precision)) = " << fmod(y, pow(10, 5 - 1)))                                      
      ux_ = init_utmups_rev_x_ + fmod(x, pow(10, 5 - init_utmups_rev_crpr_));
      uy_ = init_utmups_rev_y_ + fmod(y, pow(10, 5 - init_utmups_rev_crpr_));

      //CLOG_INFO("to-delete : input [x]=")
      
      //CLOG_INFO("to-delete: fmod(x, pow(10, 5 - coordinates_precision_))" << fmod(x, pow(10, 5 - precision)));
      //CLOG_INFO("to-delete: fmod(y, pow(10, 5 - coordinates_precision_))" << fmod(y, pow(10, 5 - precision)));
      
      //CLOG_INFO("to-delete: antes  : lat=" << lat << ";lon=" << lon << ";alt=" << alt)
      GeographicLib::UTMUPS::Reverse( init_zone_, 
                                      init_northp_, 
                                      ux_, 
                                      uy_, 
                                      lat,
                                      lon);
      //CLOG_INFO("to-delete: desp  : lat=" << lat << ";lon=" << lon << ";alt=" << alt)                                      
      

    }catch(const GeographicLib::GeographicErr& err){
      CLOG_ERROR("projector-llh-mgrs : reverse : error reversing from (" << x 
      << ", " << y << ", " << z << ") : exception.geographiclib(" << err.what() 
      << ")")
      return false;
    }
    alt = init_utmups_rev_z_;
    return true;
  }

  bool 
  ProjectorLlhMgrs::reverse(const ProjectionPointStack& in, GeoPointStack& out){
    out.clear();
    std::size_t doneok = in.forEach([&](const ProjectionPoint& input){
      GeoPoint p;
      bool opres = reverse(
        input.x(),input.y(),input.z(),p.latitude(),p.longitude(),p.altitude()
      );
      out.push_back(p);
      return opres;
    });
    return (doneok == in.size());
  }
  
  bool 
  ProjectorLlhMgrs::forward(const GeoPointStack& in, ProjectionPointStack& out){
    out.clear();
    std::size_t doneok = in.forEach([&](const GeoPoint& input){
      ProjectionPoint p;
      bool opres = reverse(
        input.latitude(),input.longitude(),input.altitude()
        ,p.x(),p.y(),p.z()
      );
      out.push_back(p);
      return opres;
    });
    return (doneok == in.size());
  }
bool ProjectorLlhMgrs::reverse(const ProjectionPoint& in, GeoPoint& out){
  return reverse(in.x(),in.y(),in.z(), out.latitude(),out.longitude(),out.altitude());
}
bool ProjectorLlhMgrs::forward(const GeoPoint& in, ProjectionPoint& out){
  return forward(in.latitude(),in.longitude(),in.altitude(),out.x(),out.y(),out.z());
}






  // interface
  CYT3MACRO_model_class_definitions(
    GeoPoint
    , 
    , ( 
        std::string to_string() const
      , bool        proximal(const GeoPoint& other, double tolerance)
    )
    , ( )
    , latitude      , double    , 0.0
    , longitude     , double    , 0.0
    , altitude      , double    , 0.0
    , heading       , double    , 0.0
    , dist_tolerance, double    , 1.0
  )
  std::string GeoPoint::to_string() const{
    std::stringstream sstr;
    sstr << "[lat=" << latitude() << ";lon=" << longitude() << ";alt=" << altitude() << ";hea=" << heading() << ";tol=" << dist_tolerance() << "]";
    return sstr.str();
  }
  bool GeoPoint::proximal(const GeoPoint& other, double tolerance){
    double t;
    if(tolerance == -1.){
      tolerance = dist_tolerance();
    }else{
      t = tolerance;
    }
    CLOG_INFO("** to-delete : proximal, distance = " << distance_m_earth_input_degrees(latitude(),longitude(),other.latitude(),other.longitude()));
    if(distance_m_earth_input_degrees(
      latitude(),longitude(),
      other.latitude(),other.longitude()) 
      > t){
        return false;
      }
    return true;
  }
  CYT3MACRO_model_class_set_stack_definitions(GeoPoint,)
  CYT3MACRO_model_class_definitions(
    ProjectionPoint
    , 
    , ( std::string to_string() const)
    , ( )
    , x , double , 0.
    , y , double , 0.
    , z , double , 0.
  )

  std::string ProjectionPoint::to_string() const{
    std::stringstream sst;
    sst << "x=" << x() << ",y=" << y() << ",z=" << z();
    return sst.str();
  }

  CYT3MACRO_model_class_set_stack_definitions(ProjectionPoint, )





  CYT3MACRO_model_class_definitions(
    ProjectorInformation
    , 
    , ( 
          static const std::string LOCAL
        , static const std::string LOCAL_CARTESIAN_UTM
        , static const std::string MGRS
        , static const std::string TRANSVERSE_MERCATOR
        , static const std::string WGS84
        , static const std::string EGM2008
        , std::string to_string() const
    )
    , ( )
    , projector_type        , std::string   ,       ""
    , vertical_datum        , std::string   ,       ""
    , mgrs_grid             , std::string   ,       ""
    , map_origin            , GeoPoint      , 
  )

  const std::string ProjectorInformation::LOCAL = "local";
  const std::string ProjectorInformation::CUSTOM_ENU = "enu";
  const std::string ProjectorInformation::LOCAL_CARTESIAN_UTM = "LocalCartesianUTM";
  const std::string ProjectorInformation::MGRS = "MGRS";
  const std::string ProjectorInformation::TRANSVERSE_MERCATOR = "TransverseMercator";
  const std::string ProjectorInformation::WGS84 = "WGS84";
  const std::string ProjectorInformation::EGM2008 = "EGM2008";
  std::string ProjectorInformation::to_string() const{
    std::stringstream sst;
    sst << "type=" << projector_type() << ";vertical_datum=" << vertical_datum() 
    << ";mgrs_grid=" << mgrs_grid() << ";map-origin=" << map_origin().to_string();
    return sst.str();
  }


  ProjectorInterface::ProjectorInterface()
  :is_initialized_(false)
  ,proj_info_(){

  }
  ProjectorInterface::~ProjectorInterface(){

  }


  
  
  
  
  
  

  bool ProjectorInterface::is_initialized(){return is_initialized_;}

  bool ProjectorInterface::forward(double i1, double i2, double i3, double& o1, double& o2, double& o3){
    CLOG_WARN("projector-interface: forward : not-implemented")
    return false;
  }
  bool ProjectorInterface::reverse(double i1, double i2, double i3, double& o1, double& o2, double& o3){
    CLOG_WARN("projector-interface: reverse : not-implemented")
    return false;
  }
  bool ProjectorInterface::forward(const GeoPointStack& in, ProjectionPointStack& out){
    CLOG_WARN("projector-interface : forward : not implemented")
    return false;
  }
  bool ProjectorInterface::reverse(const ProjectionPointStack& in, GeoPointStack& out){
    CLOG_WARN("projector-interface : reverse : not implemented")
    return false;
  }
  bool ProjectorInterface::forward(const GeoPoint& in, ProjectionPoint& out){
    CLOG_WARN("projector-interface : forward : not implemented")
    return false;
  }
  bool ProjectorInterface::reverse(const ProjectionPoint& in, GeoPoint& out){
    CLOG_WARN("projector-interface : reverse : not implemented")
    return false;
  }
  ProjectorInformation& ProjectorInterface::info(){return proj_info_;}
  const ProjectorInformation& ProjectorInterface::info() const{return proj_info_;}
  

  ProjectorInterface* create_projector(const ProjectorInformation& inf, bool init){
    ProjectorInterface* np = nullptr;
    CLOG_INFO("milla::connapp::geo : create-projector : init(" 
      << (init==true?"true":"false") << ")[" << inf.to_string() << "]")
    if(inf.projector_type() == ProjectorInformation::MGRS){
      CLOG_INFO("milla::connapp::geo : create-projector : creating llt-mgrs "
      "projector")
      np = new(std::nothrow) ProjectorLlhMgrs();
    }else if(inf.projector_type() == ProjectorInformation::CUSTOM_ENU){
      CLOG_INFO("milla::connapp::geo : create-projector : creating llt-enu "
      "projector")
      np = new(std::nothrow) ProjectorLlhEnu();
    }else{
      CLOG_WARN("coyot3:tools:geo : create-projector : projector information "
      "does not give a correct initializer : [" << inf.to_string() << "]")
    }
    if(!np){
      CLOG_ERROR("milla::connapp::geo : create-projector : FATAL! IMPOSSIBLE "
      "TO CREATE PROJECTOR! NO MEM?")
      return nullptr;
    }
    CLOG_INFO("milla::connapp::geo : create-projector : setting information (" 
    << inf.to_string() << ")")
    np->info() = inf;
    if(init == true){
      if(np->initialize()){
        CLOG_INFO("milla::connapp::geo : create-projector : initialized ok"); 
      }else{
        CLOG_WARN("milla::connapp::geo : create-projector : initialization "
        "error! destroying instance");
        delete np;
        np = nullptr; 
      }
    }
    return np;
  }

}
}
}