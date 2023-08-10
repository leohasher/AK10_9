#ifndef _MOTOR_H
#define _MOTOR_H

  #include "Arduino.h"
  #include <SPI.h>
  #include "mcp2515_can.h"
  #include "AK10_9.h"


  #define P_MIN -12.5f
  #define P_MAX 12.5f
  #define V_MIN -46.57f
  #define V_MAX 46.57f
  #define KP_MIN 0.0f
  #define KP_MAX 500.0f
  #define KD_MIN 0.0f
  #define KD_MAX 5.0f
  #define T_MIN -54.0f
  #define T_MAX 54.0f
  #define SPI_CS_PIN_SEND 9


  class AK10_9
  {
    public:
      float p_in; //desired position
      float v_in; //desired velocity
      float t_in; //desired torque
      float kp_in; //desired proportional
      float kd_in; //desired derivative

      AK10_9(unsigned int canId);
      bool begin();
 
      void setZero();
      void sendCommand();
      bool getReply();

      unsigned int getCanId();
      float getPosition();
      float getVelocity();
      float getTorque();

    private:
      unsigned int _canId; //Motor ID
      mcp2515_can CAN;
    
      float _p_out; //position
      float _v_out; //velocity
      float _t_out; //torque

      unsigned char _len;
      unsigned char _buf[8];

      void enterMotorMode();
      void exitMotorMode();
      unsigned int floatToUint(float float_val, float min_val, float max_val, unsigned int size);
      float uintToFloat(unsigned int int_val, float min_val, float max_val, unsigned int size);
  };

#endif