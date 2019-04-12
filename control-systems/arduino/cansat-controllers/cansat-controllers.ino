#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>


const int N_CLUSTERS = 20;
const float FLOAT_MAX = 3.4028235E+38;
const float F_ITERS = 1;
const float FINS_OFFSET[3] = {90, 90, 90};
const float FINS_MIN[3] = {60, 60, 60};
const float FINS_MAX[3] = {120, 120, 120};
const int finPins[3] = {9, 10, 11};
Servo fins[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;
imu::Vector<3> gyroscope;

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

// TODO CHANGE SERVOS



/* Closest Point to K-Means Centers */
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

void lincomb(float *x, float *y, const int N) {
  float *temp;
  matvecmul(W, x, temp, N);
  add(temp, b, y, N);
}

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

// PD Controller Parameters
const float K_p_yaw = 0.00324;
const float K_d_yaw = 0.0028;

float calculate_z_moment(float psi, float psi_dot, float &moment) {
  moment = K_p_yaw * wrap_angle(psi) + K_d_yaw * psi_dot;
}


/* PITCH AND ROLL CONTROL */
// Controller parameters
const float KMx[4] = { -0.1099,    -1.7944,   1.7286,    -0.0000 };
const float KMy[4] = { -1.7286,    -0.0000,    -0.1099,    +1.7944 };

void calculate_xy_moments(float phi, float phi_dot, float theta, float theta_dot, float *moments) {
  float x[6];
  x[0] = phi;
  x[1] = phi_dot;
  x[2] = theta;
  x[3] = theta_dot;
  moments[0] = dot(KMx, x, 4); 
  moments[1] = dot(KMy, x, 4);
}


float Mz;
float Mxy[2];
float M[3];
float finPos[3];

/* MAIN CONTROL ROUTINES */
void control() {
  // Main controller routine
  // Calculate Moments from controller

  // Initialize Sensor
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


  // Debug Statements
  Serial.print(F("Orientation: "));
  Serial.print(euler.x());
  Serial.print(F(" "));
  Serial.print(euler.y());
  Serial.print(F(" "));
  Serial.print(euler.z());
  Serial.print(F(""));

  Serial.print(F(" Gyros: "));
  Serial.print(gyroscope.x());
  Serial.print(F(" "));
  Serial.print(gyroscope.y());
  Serial.print(F(" "));
  Serial.print(gyroscope.z());
  Serial.print(F(""));
  calculate_z_moment(euler.x(), gyroscope.x(), Mz);
  calculate_xy_moments(euler.y(), gyroscope.y(), euler.z(), gyroscope.z(), Mxy);
  
  
  M[0] = Mxy[0];
  M[1] = Mxy[1];
  M[2] = Mz;

  Serial.print(F(" M: "));
  Serial.print(M[0]);
  Serial.print(F(" "));
  Serial.print(M[1]);
  Serial.print(F(" "));
  Serial.print(M[2]);
  Serial.print(F(""));


  // Get fin position
  closest_point(M, finPos, 3);

  Serial.print(F(" F: "));
  Serial.print(finPos[0]);
  Serial.print(F(" "));
  Serial.print(finPos[1]);
  Serial.print(F(" "));
  Serial.print(finPos[2]);
  Serial.println(F(""));


  turn_fins(finPos, 3, F_ITERS, false);
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
  delay(500);
}
