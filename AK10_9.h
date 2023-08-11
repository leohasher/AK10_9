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

      /*************************************************************************
      ** Descriptions:    Desired parameters to send to motor
      *************************************************************************/
      float p_in;  // desired position
      float v_in;  // desired velocity
      float t_in;  // desired torque
      float kp_in; // desired proportional
      float kd_in; // desired derivative

      /*************************************************************************
      ** Construtor name: AK10_9
      ** Descriptions:    Initialize instance of a Motor by passing its CAN ID.
      **                  CAN ID can be set using R-Link. (current motor is set
      **                  to 0x01)
      *************************************************************************/
      AK10_9(unsigned int canId);

      /*************************************************************************
      ** Function name:   begin
      ** Descriptions:    Initialize CAN-BUS and enter motor mode. Returns true
      **                  when connection successful.
      *************************************************************************/
      bool begin();
 
      /*************************************************************************
      ** Function name:   setZero
      ** Descriptions:    Set current motor position to zero.
      **                          
      *************************************************************************/
      void setZero();

      /*************************************************************************
      ** Function name:   sendCommand
      ** Descriptions:    Send contents of buffer to motor.
      **                  Set p_in, v_in, t_in, kp_in, kd_in beforehand
      *************************************************************************/
      void sendCommand();

      /*************************************************************************
      ** Function name:   sendCommand
      ** Descriptions:    Get values from motor. contents can be checked using.
      **                  the get functions. Returns true if CAN recieved 
      **                  message and false otherwise.
      *************************************************************************/
      bool getReply();

      /*************************************************************************
      ** Function name:   getCanId
      ** Descriptions:    Get CAN ID of message recieved using getReply
      *************************************************************************/
      unsigned int getCanId();

      /*************************************************************************
      ** Function name:   getPosition
      ** Descriptions:    Get motor position from message recieved using
      **                  getReply. [units: rad]
      *************************************************************************/
      float getPosition();

      /*************************************************************************
      ** Function name:   getVelocity
      ** Descriptions:    get motor velocity from message recieved using
      **                  getReply. [Units: rad/s]
      *************************************************************************/
      float getVelocity();

      /*************************************************************************
      ** Function name:   getTorque
      ** Descriptions:    get motor torque from message recieved using
      **                  getReply. [Units: unknown (amps maybe?)]
      *************************************************************************/
      float getTorque();

    private:
      unsigned int _canId; // Motor ID
      mcp2515_can CAN; // CAN BUS
    
      float _p_out; // output position
      float _v_out; // output velocity
      float _t_out; // output torque

      unsigned char _len;
      unsigned char _buf[8]; // buffer for storing messages to be sent

      //may make these public later:
      void enterMotorMode(); // enter motor mode 
      void exitMotorMode(); // exit motor mode (may be needed when using multiple motors?)

      unsigned int floatToUint(float float_val, float min_val, float max_val, unsigned int size);
      float uintToFloat(unsigned int int_val, float min_val, float max_val, unsigned int size);
  };

#endif