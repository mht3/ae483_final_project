#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"


// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Setpoint
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;

// Measurements
// Lighhouse positions hijcked from `lighthouse_position_est.c`
static float lighthouse_x = 0.0f;
static float lighthouse_y = 0.0f;
static float lighthouse_z = 0.0f;

static float x_meas = 0.0f;
static float y_meas = 0.0f;
static float z_meas = 0.0f;

static float a_z = 0.0f;

// Constants
static float g = 9.81f;
static float dt = 0.002f;
// static float o_z_eq = 0.1f;

// MEASUREMENT ERROR
// -------------------------------------------------
static float x_meas_err = 0.0f;
static float y_meas_err = 0.0f;
static float z_meas_err = 0.0f;

// Parameters
static bool use_observer = false;
static bool reset_observer = false;

void ae483UpdateWithLighthouse(float x, float y, float z) 
{
  lighthouse_x = x;
  lighthouse_y = y;
  lighthouse_z = z;
}

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Everything in here runs at 500 Hz

    // Desired position
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Measurements
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    a_z = g * sensors->acc.z;
    x_meas = lighthouse_x;
    y_meas = lighthouse_y;
    z_meas = lighthouse_z;

    if (reset_observer) {
      o_x = 0.0f;
      o_y = 0.0f;
      o_z = 0.0f;
      psi = 0.0f;
      theta = 0.0f;
      phi = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      reset_observer = false;
    }

    // State estimates
    if (use_observer) {
    
      // Compute each element of:
      // 
      //   C x + D u - y
      // 
      // Get measurements (do not change)
      
      // X, Y, and Z measurements are the ground truth of the lighthouse's crossing beams method.
      // Sensor error
      x_meas_err = o_x - x_meas;
      y_meas_err = o_y - y_meas;
      z_meas_err = o_z - z_meas;

      // Update Estimates
      o_x += dt * (v_x - 17.758181912487f * x_meas_err);
      o_y += dt * (v_y - 17.722312157029f * y_meas_err);
      o_z += dt * (v_z - 21.430118991736f * z_meas_err);
      
      psi += dt * (w_z);

      theta += dt * (w_y - 34.333333333333f * x_meas_err);
      phi += dt * (w_x - (-50.75f  * y_meas_err));
      v_x += dt * (g * theta - 120.12095686293f * x_meas_err);
      v_y += dt * (-g * phi - 135.91517409559f * y_meas_err);
      v_z += dt * (a_z - g - 93.5f * z_meas_err);

    } else {
      o_x = state->position.x;
      o_y = state->position.y;
      o_z = state->position.z;
      psi = radians(state->attitude.yaw);
      theta = - radians(state->attitude.pitch);
      phi = radians(state->attitude.roll);
      w_x = radians(sensors->gyro.x);
      w_y = radians(sensors->gyro.y);
      w_z = radians(sensors->gyro.z);
      v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
      v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
      v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);
    }

    if (setpoint->mode.z == modeDisable) {
      // If there is no desired position, then all
      // motor power commands should be zero

      powerSet(0, 0, 0, 0);

    } else {
      // Otherwise, motor power commands should be
      // chosen by the controller



      tau_x = 0.00118322f * (o_y - o_y_des) -0.00499485f * phi + 0.00141594f * v_y -0.00089651f * w_x;
      tau_y = -0.00173205f * (o_x - o_x_des) -0.00623766f * theta -0.00189279f * v_x -0.00099107f * w_y;
      tau_z = -0.00160000f * psi -0.00175942f * w_z;
      f_z = -0.06324555f * (o_z - o_z_des) -0.07349171f * v_z + 0.35217900f;

      m_1 = limitUint16( -3496531.0f * tau_x -3496531.0f * tau_y -41109810.7f * tau_z + 122378.6f * f_z );
      m_2 = limitUint16( -3496531.0f * tau_x + 3496531.0f * tau_y + 41109810.7f * tau_z + 122378.6f * f_z );
      m_3 = limitUint16( 3496531.0f * tau_x + 3496531.0f * tau_y -41109810.7f * tau_z + 122378.6f * f_z );
      m_4 = limitUint16( 3496531.0f * tau_x -3496531.0f * tau_y + 41109810.7f * tau_z + 122378.6f * f_z );
      // Apply motor power commands
      powerSet(m_1, m_2, m_3, m_4);
    }
  }
}

LOG_GROUP_START(ae483log)
LOG_ADD(LOG_FLOAT,          o_x,                    &o_x)
LOG_ADD(LOG_FLOAT,          o_y,                    &o_y)
LOG_ADD(LOG_FLOAT,          o_z,                    &o_z)
LOG_ADD(LOG_FLOAT,          psi,                    &psi)
LOG_ADD(LOG_FLOAT,          theta,                  &theta)
LOG_ADD(LOG_FLOAT,          phi,                    &phi)
LOG_ADD(LOG_FLOAT,          v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,          v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,          v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,          w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,          w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,          w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,          o_x_des,                &o_x_des)
LOG_ADD(LOG_FLOAT,          o_y_des,                &o_y_des)
LOG_ADD(LOG_FLOAT,          o_z_des,                &o_z_des)
LOG_ADD(LOG_FLOAT,          tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,          tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,          tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,          f_z,                    &f_z)
LOG_ADD(LOG_UINT16,         m_1,                    &m_1)
LOG_ADD(LOG_UINT16,         m_2,                    &m_2)
LOG_ADD(LOG_UINT16,         m_3,                    &m_3)
LOG_ADD(LOG_UINT16,         m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,          a_z,                    &a_z)
LOG_ADD(LOG_FLOAT,          x_meas,                 &x_meas)
LOG_ADD(LOG_FLOAT,          y_meas,                 &y_meas)
LOG_ADD(LOG_FLOAT,          z_meas,                 &z_meas)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483par)