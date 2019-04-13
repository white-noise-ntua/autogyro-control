#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Kalman.h>


// Moments of Inertia
const float m = 75e-3;
const float r = 0.35;
const float Mass = 0.33;
const float mu = (Mass * m) / (Mass + m);
const float R = 0.05;
const float H = 0.3;
const float h = 0.15;
const float rho = h * m / (Mass + m);
const float rho_h = h * Mass / (Mass + m);
const float Ix = 1 / 4 * m * r * r + 1 / 12 * Mass * (3 * R * R + H * H) + Mass * rho_h * rho_h + m * rho * rho;
const float Iy = Ix;
const float Iz = 1 / 2 * m * r * r;

const int N_CLUSTERS = 20;
const float FLOAT_MAX = 3.4028235E+38;

// Fins configurations
const float F_ITERS = 1;
const float FINS_OFFSET[3] = {90, 90, 90};
const float FINS_MIN[3] = {60, 60, 60};
const float FINS_MAX[3] = {120, 120, 120};
const int finPins[3] = {9, 10, 11};
Servo fins[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Sampling period
const float SAMPLING_PERIOD = 0.001;

// IMU Structs
imu::Vector<3> euler;
imu::Vector<3> gyroscope;
imu::Vector<3> acceleration;

// Kalman Filters
Kalman kalman_phi(0.01, 0.01, 1, 1, 1);
Kalman kalman_theta(0.01, 0.01, 1, 1, 1);
Kalman kalman_psi(0.01, 0.01, 1, 1, 1);

// Complementary Gains (must add to 1)
const float COMP_GAIN[2] = { 0.5, 0.5 };

// Moments (in Nm)
float M[3];

// Fin Positions (in degrees) 
float finPos[3];


// Struct to hold coordinates to our reference frame
typedef struct coords_t {
  float phi, theta, psi;
  float phi_dot, theta_dot, psi_dot;
  
} coordinates;

coordinates coords;
coordinates prev_coords;

// Transform coordinates to our reference frame
void transform_coords(bool raw, bool complementary) {
  
  coords.phi = deg_to_rad(-euler.z());
  coords.theta = deg_to_rad(-euler.y());
  coords.psi = deg_to_rad(- wrap_angle(euler.x()));

  if (raw) {
    coords.phi_dot = gyroscope.x();
    coords.theta_dot = gyroscope.y();
    coords.psi_dot = gyroscope.z();  
  }
  else {
    coords.phi_dot = kalman_phi.filter(gyroscope.x(), acceleration.x() * SAMPLING_PERIOD);
    coords.theta_dot = kalman_theta.filter(gyroscope.y(), acceleration.y() * SAMPLING_PERIOD);
    coords.psi_dot = kalman_psi.filter(gyroscope.z(), acceleration.z() * SAMPLING_PERIOD); 
  }

  if (complementary) {
    float phi_dot_e = (coords.phi - prev_coords.phi) / SAMPLING_PERIOD;
    float theta_dot_e = (coords.theta - prev_coords.theta) / SAMPLING_PERIOD;
    float psi_dot_e = (coords.psi - prev_coords.psi) / SAMPLING_PERIOD; 
    coords.phi_dot = COMP_GAIN[0] * coords.phi_dot + COMP_GAIN[1] * phi_dot_e;
    coords.theta_dot = COMP_GAIN[0] * coords.theta_dot + COMP_GAIN[1] * theta_dot_e;
    coords.psi_dot = COMP_GAIN[0] * coords.psi_dot + COMP_GAIN[1] * psi_dot_e;
    
  }
  
}

float kmeans_angles [20][3] = {
  {2.0,23.0,21.0},
  {-1.0,23.0,-24.0},
  {0.0,-26.0,-2.0},
  {-23.0,1.0,25.0},
  {26.0,28.0,-27.0},
  {-8.0,-20.0,-26.0},
  {-27.0,-26.0,27.0},
  {-23.0,-2.0,-21.0},
  {26.0,-25.0,-27.0},
  {-28.0,26.0,26.0},
  {-25.0,-21.0,-5.0},
  {20.0,8.0,27.0},
  {-26.0,28.0,-26.0},
  {24.0,3.0,-26.0},
  {25.0,-26.0,26.0},
  {-2.0,-24.0,26.0},
  {26.0,0.0,2.0},
  {27.0,-28.0,-1.0},
  {-24.0,25.0,0.0},
  {20.0,25.0,5.0},
};

float kmeans_centers [20][3] = {
  {0.005427,-0.0003853,0.004247},
  {4.376e-05,-0.01109,-0.0002065},
  {-0.003787,0.005622,-0.00251},
  {0.009661,0.005554,0.0001668},
  {-0.006841,-0.01274,0.002382},
  {-0.004152,-0.001481,-0.005},
  {0.007556,0.01232,-0.002369},
  {0.003087,-0.004368,-0.004284},
  {-0.01399,-0.0004159,-0.002382},
  {0.01445,3.785e-05,0.002193},
  {0.003243,0.003703,-0.004545},
  {-0.000757,0.004353,0.005},
  {0.007174,-0.01254,-0.002208},
  {-0.009585,-0.006635,8.499e-05},
  {-0.006751,0.01225,0.002394},
  {0.0008499,0.01168,-4.239e-05},
  {-0.006702,0.0004477,0.002569},
  {-0.01112,0.006299,-4.736e-05},
  {0.009879,-0.005729,5.583e-06},
  {-0.001446,-0.004624,0.004568},
};

float W[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

float b[3] = {0, 0, 0};


// Closest Point to K-Means Centers
void closest_point(float *x, float* y, const int N) {
  float minimum = FLOAT_MAX;
  int argmin = 0;
  float tmp;
  for (int i = 0; i < N_CLUSTERS; i++) {
    tmp = dist_sq(x, kmeans_centers[i], 3);
    if (tmp < minimum) {
      minimum = tmp;
      argmin = i;
    }
    
  }
  for (int i = 0; i < N; i++) y[i] = kmeans_angles[argmin][i];
  

}


// Convert rad to degrees
float rad_to_deg(float rad) {
  return 180.0 / PI * rad;
}

// Convert degrees to rad
float deg_to_rad(float deg) {
  return PI / 180 * deg;
}

// Dot product of two-element vectors
float dot(float *x, float*y, const int N) {
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += x[i] * y[i];
  }
  return sum;
}

// Returns the squared distance between two points
float dist_sq(float *x, float *y, const int N) {
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += (x[i] - y[i]) * (x[i] - y[i]);
  }
  return sum;
}

// Vector addition routine
void add(float *x, float *y, float *result, const int N) {
  for (int i = 0; i < N; i++) result[i] = x[i] + y[i];
}

// Multiply an NxN matrix A with a vector x and store result in y
void matvecmul(float A[3][3], float *x, float *y, const int N) {
  for (int i = 0; i < N; i++) {
    y[i] = dot(A[i], x, N);
  }
  
}

// Return a linear combination y = Wx + b
void lincomb(float *x, float *y, const int N) {
  float *temp;
  matvecmul(W, x, temp, N);
  add(temp, b, y, N);
}

// Wraps angles for yaw
float wrap_angle(float x) {
  if (x >= 180) {
    return x - 360;  
  } else {
    return x;
  }
  
}


// Turn fins to a position
void turn_fins(float *thetas, const int N, const int iters, bool degs) {
   float us[N];

   for (int i = 0; i < N; i++) {
    if (degs) {
      us[i] = thetas[i] / (1.0 * iters);
    } else {
      us[i] = rad_to_deg(thetas[i]) / (1.0 * iters); 
    }
    
   }

   for (int i = 0; i < iters; i++) {
    for (int j = 0; j < N; j++) {
      fins[j].write(constrain(us[j] + FINS_OFFSET[j], FINS_MIN[j], FINS_MAX[j]));  
    }
   }
  
    
}

/* YAW CONTROL */
/* PITCH AND ROLL CONTROL */

// Controller parameters

// Gains
const float KMx[4] = {     1.7286,    0.0000,    0.1099,    1.7944 };
const float KMy[4] = {     0.1099,    1.7944,   -1.7286,    0.0000 };
const float KMz[2] = { 1  ,  0 };


void calculate_moments(float *moments) {
  // Create the state vectors
  float x1[4];
  float x2[2];
  x1[0] = coords.phi;
  x1[1] = coords.phi_dot;
  x1[2] = coords.theta;
  x1[3] = coords.theta_dot;
  x2[0] = coords.psi;
  x2[1] = coords.psi_dot;
  // u = - kx
  moments[0] = - Ix * dot(KMx, x1, 4); 
  moments[1] = - Ix * dot(KMy, x1, 4);
  moments[2] = -dot(KMz, x2, 2);
  
}


void get_measurements() {
  // Get raw measurements from BNO055
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Transform to our reference frame w/no filtering
  transform_coords(true, false); // Change argument to false to use Kalman Filters
}

// Log measurements
void logging() {
  Serial.print(" Phi / Pitch: "); 
  Serial.print(coords.phi);
  Serial.print(F(" "));
  Serial.print(coords.phi_dot);

  Serial.print(" | Theta / Roll: "); 
  Serial.print(coords.theta);
  Serial.print(F(" "));
  Serial.print(coords.theta_dot);
 
  Serial.print(" | Psi / Yaw: "); 
  Serial.print(coords.psi);
  Serial.print(F(" "));
  Serial.print(coords.psi_dot);
  Serial.print(F(" M: "));
  Serial.print(M[0], 5);
  Serial.print(F(" "));
  Serial.print(M[1], 5);
  Serial.print(F(" "));
  Serial.print(M[2], 5);
  Serial.print(F(""));
  Serial.print(F(" F: "));
  Serial.print(finPos[0]);
  Serial.print(F(" "));
  Serial.print(finPos[1]);
  Serial.print(F(" "));
  Serial.print(finPos[2]);
  Serial.println(F(""));
 
}

/* MAIN CONTROL ROUTINES */
void control() {
  // Main controller routine

  // Get measurements from IMU and filter them
  get_measurements();
  calculate_moments(M);

  // Log measurements to output
  logging();

  // Get fin position
  closest_point(M, finPos, 3);

  // Turn fins to desired position
  turn_fins(finPos, 3, F_ITERS, false);

  prev_coords = coords;
}


void setup() {
  for (int i = 0; i < 3; i++) {
    fins[i].attach(finPins[i]);   
  }

  Serial.begin(115200);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

}

void loop() {
  control();
  delay(SAMPLING_PERIOD * 1000);
}
