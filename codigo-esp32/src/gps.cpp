#include "gps.h"

// ====== Estado interno (escopo de arquivo) ======
static TinyGPSPlus      s_gps;
static HardwareSerial*  s_port = nullptr;

static double s_target_lat_deg = NAN;
static double s_target_lon_deg = NAN;

static double p0_lat_deg = NAN;
static double p0_lon_deg = NAN;

static double p1_lat_deg = NAN;
static double p1_lon_deg = NAN;

static double s_inner_radius_m = 8.0;
static double s_outer_radius_m = 12.0;
static bool   s_inside_radius  = false;  // estado da histerese

static uint32_t s_report_interval_ms = 1000;
static uint32_t s_last_report_ms     = 0;

// ====== Constantes ======
static constexpr double kEarthRadius_m = 6371000.0;

// ====== Helpers internos ======
static bool target_is_valid(void) {
  return isfinite(s_target_lat_deg) && isfinite(s_target_lon_deg);
}

// ====== API ======

void gps_begin(HardwareSerial* port, uint32_t baud, int rxPin, int txPin) {
  s_port = port;
  if (s_port) {
    s_port->begin(baud, SERIAL_8N1, rxPin, txPin);
  }
  // reset de histerese
  s_inside_radius = false;
}

void gps_feed(void) {
  if (!s_port) return;
  while (s_port->available()) {
    s_gps.encode(s_port->read());
  }
}

bool gps_save_p0(){
  return true;
  return gps_current_location(&p0_lat_deg,&p0_lon_deg);
}

bool gps_save_p1(){
  return true;
  return gps_current_location(&p1_lat_deg,&p1_lon_deg);
}

void gps_set_report_interval_ms(uint32_t ms) { s_report_interval_ms = ms; }
uint32_t gps_get_report_interval_ms(void) { return s_report_interval_ms; }
uint32_t gps_get_last_report_ms(void) { return s_last_report_ms; }

bool gps_has_data(uint32_t minChars) {
  return s_gps.charsProcessed() >= minChars;
}

bool gps_has_fix(void) {
  return s_gps.location.isValid();
}

bool gps_current_location(double* lat_deg, double* lon_deg) {
  if (!gps_has_data(10)) Serial.println("gps no DATA");
  if (!s_gps.location.isValid()) Serial.println("gps no valid");
  if (!lat_deg || !lon_deg) return false;
  if (!s_gps.location.isValid()) return false;
  *lat_deg = s_gps.location.lat();
  *lon_deg = s_gps.location.lng();
  Serial.print("Lat:");
  Serial.print(*lat_deg,10);
  Serial.print(" Lon:");
  Serial.println(*lon_deg,10);
  return true;
}

int gps_satellites(void) {
  if (s_gps.satellites.isValid()) {
    return (int)s_gps.satellites.value();
  }
  return -1;
}

double gps_hdop(void) {
  if (s_gps.hdop.isValid()) {
    // TinyGPS++ fornece o HDOP em centésimos (inteiro).
    // Ex.: 95 => 0.95
    return s_gps.hdop.value() / 100.0;
  }
  return -1.0;
}


void gps_set_target(double lat_deg, double lon_deg,
                    double inner_radius_m, double outer_radius_m) {
  s_target_lat_deg = lat_deg;
  s_target_lon_deg = lon_deg;
  s_inner_radius_m = inner_radius_m;
  s_outer_radius_m = outer_radius_m;
  s_inside_radius  = false; // reavalia ao próximo cálculo
}

bool gps_distance_bearing_to_target(double* distance_m, double* bearing_deg) {
  if (!distance_m || !bearing_deg) return false;
  if (!gps_has_fix() || !target_is_valid()) return false;

  double lat, lon;
  if (!gps_current_location(&lat, &lon)) return false;

  *distance_m  = gps_haversine_m(lat, lon, s_target_lat_deg, s_target_lon_deg);
  *bearing_deg = gps_bearing_deg(lat, lon, s_target_lat_deg, s_target_lon_deg);
  return true;
}

bool gps_evaluate() {
  p1_lat_deg =  p0_lat_deg;
  p1_lon_deg =  p0_lon_deg;
  gps_save_p0();

  GpsNavCommand navCmd = gps_calculate_route();

  if(navCmd.distance_m > 2) return false;
  return true;
}

struct Point{
  double x;
  double y;
};


Point MercantorPoint(double lat, double lon){
  double x = kEarthRadius_m * (lon-s_target_lon_deg) * M_PI / 180.0;

  double lat_rad = (lat-s_target_lat_deg) * M_PI / 180.0;
  float y = kEarthRadius_m * log(tan(lat_rad) + (1.0 / cos(lat_rad)));
  
  return Point{x=x,y=y};
}

double DistanceBetween(Point a, Point b){
  return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

double calculateAngle(Point p1, Point p2, Point p3) {
     // Create vectors v1 = p1 - p2, v2 = p3 - p2
    double v1x = p1.x - p2.x;
    double v1y = p1.y - p2.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;

    // Compute dot product and magnitudes
    double dot = v1x * v2x + v1y * v2y;
    double mag1 = sqrt(v1x * v1x + v1y * v1y);
    double mag2 = sqrt(v2x * v2x + v2y * v2y);

    // Debug: print intermediate values
    Serial.println("---- Debug Info ----");
    Serial.print("v1: ("); Serial.print(v1x); Serial.print(", "); Serial.print(v1y); Serial.println(")");
    Serial.print("v2: ("); Serial.print(v2x); Serial.print(", "); Serial.print(v2y); Serial.println(")");
    Serial.print("dot = "); Serial.println(dot, 6);
    Serial.print("mag1 = "); Serial.println(mag1, 6);
    Serial.print("mag2 = "); Serial.println(mag2, 6);

    // Avoid division by zero
    if (mag1 == 0.0 || mag2 == 0.0) {
        Serial.println("Error: Two points are identical, cannot define an angle.");
        return NAN;
    }

    // Compute cosine of the angle
    double cos_theta = dot / (mag1 * mag2);

    // Clamp to avoid domain errors
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    Serial.print("cos(theta) = "); Serial.println(cos_theta, 6);

    // Compute angle
    double angle_rad = acos(cos_theta);
    double angle_deg = angle_rad * (180.0 / M_PI);

    Serial.print("angle (deg) = "); Serial.println(angle_deg, 6);
    Serial.println("---------------------");

    return angle_deg;
}                                                                                                              

GpsNavCommand gps_calculate_route(){
  GpsNavCommand navCmd;
  // Point point0 = MercantorPoint(p0_lat_deg,p0_lon_deg);
  // Point point1 = MercantorPoint(p1_lat_deg,p1_lon_deg);
  // Point pointRef = MercantorPoint(s_target_lat_deg,s_target_lon_deg);
  Point point0 = {-1,-1};
  Point point1 = {0,-1};
  Point pointRef = {0,0};
  Serial.print("P0: ");
  Serial.print(point0.x);
  Serial.print(" ");
  Serial.print(point0.y);
  Serial.print(" P1: ");
  Serial.print(point1.x);
  Serial.print(" ");
  Serial.println(point1.y);
  navCmd.bearing_to_goal_deg = 180-calculateAngle(point0,point1,pointRef); 
  navCmd.distance_m = DistanceBetween(point1,pointRef); 

  return navCmd;
}