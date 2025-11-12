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

static double distance_from_goal = NAN;
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

double get_p0_lat(){
  return p0_lat_deg;
}

double get_p0_lon(){
  return p0_lon_deg;
}

double get_p1_lat(){
  return p1_lat_deg;
}

double get_p1_lon(){
  return p1_lon_deg;
}

double get_distance_from_goal(){
  return distance_from_goal;
}

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

// --- Define Your Averaging Parameters ---

// The number of points to average (as you requested)
const int AVG_POINTS_COUNT = 5; 

// The number of seconds to wait BETWEEN each point
const int AVG_DELAY_SECONDS = 3; 

/*
 * --------------------------------------------------------------------
 * New Helper Function
 * --------------------------------------------------------------------
 * This function takes all the measurements.
 * It returns 'true' only if ALL 5 points are read successfully.
 * If any single point fails, it stops immediately and returns 'false'.
 */
bool getAverageGpsLocation(double* outLat, double* outLon, int numPoints=10, int delaySeconds=1) {
    double totalLat = 0.0;
    double totalLon = 0.0;
    
    for (int i = 0; i < numPoints; i++) {
        if (i > 0) {
            // Convert seconds to milliseconds for the delay
            delay(delaySeconds * 1000); 
        }

        // --- 2. Try to get the current location ---
        double currentLat, currentLon;
        if (!gps_current_location(&currentLat, &currentLon)) {
            
            return false;
        }

        // --- 4. Add the successful point to the total ---
        totalLat += currentLat;
        totalLon += currentLon;
    }

    // --- 5. If we get here, all 5 points were successful ---
    // Calculate the average and store it in the output variables
    *outLat = totalLat / numPoints;
    *outLon = totalLon / numPoints;

    return true;
}


bool gps_save_p0(){
  return getAverageGpsLocation(&p0_lat_deg,&p0_lon_deg);
}

bool gps_save_p1(){
  return getAverageGpsLocation(&p1_lat_deg,&p1_lon_deg);
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
  if (!s_gps.location.isValid() || gps_satellites() > 4 || gps_hdop() > 3) return false;
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
  return 999;
}


void gps_set_target(double lat_deg, double lon_deg,
                    double inner_radius_m, double outer_radius_m) {
  s_target_lat_deg = lat_deg;
  s_target_lon_deg = lon_deg;
  s_inner_radius_m = inner_radius_m;
  s_outer_radius_m = outer_radius_m;
  s_inside_radius  = false; // reavalia ao próximo cálculo
}

// bool gps_distance_bearing_to_target(double* distance_m, double* bearing_deg) {
//   if (!distance_m || !bearing_deg) return false;
//   if (!gps_has_fix() || !target_is_valid()) return false;

//   double lat, lon;
//   if (!gps_current_location(&lat, &lon)) return false;

//   *distance_m  = gps_haversine_m(lat, lon, s_target_lat_deg, s_target_lon_deg);
//   *bearing_deg = gps_bearing_deg(lat, lon, s_target_lat_deg, s_target_lon_deg);
//   return true;
// }

const double R = 6371000.0; // Earth radius in meters

double deg2rad(double deg) {
    return deg * PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / PI;
}


double bearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = deg2rad(lat1);
    double phi2 = deg2rad(lat2);
    double deltaLambda = deg2rad(lon2 - lon1);

    double x = sin(deltaLambda) * cos(phi2);
    double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
    double theta = atan2(x, y);

    double brng = fmod(rad2deg(theta) + 360.0, 360.0);
    return brng;
}

double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = deg2rad(lat1);
    double phi2 = deg2rad(lat2);
    double dphi = phi2 - phi1;
    double dlambda = deg2rad(lon2 - lon1);

    double a = pow(sin(dphi / 2), 2) +
               cos(phi1) * cos(phi2) * pow(sin(dlambda / 2), 2);
    double c = 2 * asin(sqrt(a));
    return R * c;
}    

bool gps_evaluate() {
  double lat, lon; 
  getAverageGpsLocation(&lat, &lon);
  distance_from_goal = haversine_distance( lat, lon, s_target_lat_deg,s_target_lon_deg);
  if(distance_from_goal > 5) return false;
  return true;
}

                                                                                                 

void p1_to_p0(){
  p0_lat_deg=p1_lat_deg;
  p0_lon_deg=p1_lon_deg;
}

GpsNavCommand gps_calculate_route(){
    GpsNavCommand navCmd;
    double bearing12 = bearing(p0_lat_deg, p0_lon_deg, p1_lat_deg, p1_lon_deg);
    double bearing23 = bearing( p1_lat_deg, p1_lon_deg, s_target_lat_deg,s_target_lon_deg);
    navCmd.distance_m = haversine_distance( p1_lat_deg, p1_lon_deg, s_target_lat_deg,s_target_lon_deg);

    double delta = fmod(bearing23 - bearing12 + 540.0, 360.0) - 180.0;

    int turnDirection =0;
    Serial.print("Turn Direction: ");
    if (delta > 0){
      Serial.println("Direita");
      turnDirection = 1;
    }
    else if (delta < 0){
      Serial.println("Esquerda");
      turnDirection = -1;
    }

    navCmd.bearing_to_goal_deg = fabs(delta) * turnDirection;
    Serial.print("Bearing to Goal: ");
    Serial.println(navCmd.bearing_to_goal_deg);
    return navCmd;
}

void gps_loop_3_second(){
  double lastTime = millis();
  while ( lastTime + 3000> millis()){
    gps_feed();
  }
}
