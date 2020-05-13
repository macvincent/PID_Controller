#include <mutex>
#include <utility>
#include <condition_variable>
std::mutex mtx;
std::condition_variable cv;
using std::string;

bool updated = false;
bool reset = false;
double temp_error = 0.0f;
double error = 0.0f;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void update_error(){
  std::unique_lock<std::mutex> lck(mtx);
  while (!updated)cv.wait(lck);
  temp_error = error;
  error = 0;
  lck.unlock();
  reset = true;
  updated = false;
}