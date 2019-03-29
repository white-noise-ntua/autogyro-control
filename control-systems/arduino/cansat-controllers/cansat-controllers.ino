#include <Servo.h>

const int N_CLUSTERS = 20;
const float FLOAT_MAX = 3.4028235E+38;
const float F_ITERS = 10;
const int finPins[3] = {9, 10, 11};
Servo fins[3];

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
void closest_point(float *x, float* y) {
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
  y = kmeans_angles[argmin];
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



// Turn fins to a position
void turn_fins(float *thetas, const int N, const int iters) {
   float us[N];

   for (int i = 0; i < N; i++) {
    us[i] = rad_to_deg(thetas[i]) / (1.0 * iters);
   }

   for (int i = 0; i < iters; i++) {
    for (int j = 0; j < N; j++) {
      fins[j].write(us[j]);  
    }
   }
  
    
}

/* YAW CONTROL */

// PD Controller Parameters
const float K_p_yaw = 0.00324;
const float K_d_yaw = 0.0028;

float calculate_z_moment(float &psi, float &psi_dot, float &moment) {
  moment = K_p_yaw * psi + K_d_yaw * psi_dot;
}


/* PITCH AND ROLL CONTROL */
// Controller parameters
const float KMx[4] = { -0.1099,    -1.7944,   1.7286,    -0.0000 };
const float KMy[4] = { -1.7286,    -0.0000,    -0.1099,    +1.7944 };

void calculate_xy_moments(float &phi, float &phi_dot, float &theta, float &theta_dot, float *moments) {
  float x[6];
  x[0] = phi;
  x[1] = phi_dot;
  x[2] = theta;
  x[3] = theta_dot;
  moments[0] = dot(KMx, x, 4); 
  moments[1] = dot(KMy, x, 4);
}

float ttmp = 0;
float Mz;
float Mxy[2];
float M[3];
float finPos[3];

/* MAIN CONTROL ROUTINES */
void control() {
  // Main controller routine
  // Calculate Moments from controller
  // TODO CHANGE
  calculate_z_moment(ttmp, ttmp, Mz);
  calculate_xy_moments(ttmp, ttmp, ttmp, ttmp, Mxy);
  M[0] = Mxy[0];
  M[1] = Mxy[1];
  M[2] = Mz;

  // Get fin position
  closest_point(M, finPos);

  turn_fins(finPos, 3, F_ITERS);
}


void setup() {
  for (int i = 0; i < 3; i++) {
    fins[i].attach(finPins[i]);   
  }


}

void loop() {
  // put your main code here, to run repeatedly:

}
