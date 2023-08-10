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

// For Arduino MCP2515 Hat:
// SPI_CS Pin: D9
#define SPI_CS_PIN_SEND 9
// mcp2515_can CAN;

unsigned int _canId; //Motor ID
unsigned char _len = 0;
unsigned char _buf[8];

float p_in = 0.; //position
float v_in = 0.; //velocity
float t_in = 0.; //torque
float kp_in = 0.; //proportional
float kd_in = 0.; //derivative

float _p_out; //position
float _v_out; //velocity
float _t_out; //torque

AK10_9::AK10_9(unsigned int canId) : CAN(SPI_CS_PIN_SEND){
  _canId = canId;
}

bool AK10_9::begin(){
  while (CAN.begin(CAN_1000KBPS) != 0){
      Serial.println("CAN-BUS initiliased error!");
      delay(1000);
    }
  AK10_9::enterMotorMode();
  return true;
}

void AK10_9::setZero(){
  _buf[0] = 0xFF;
  _buf[1] = 0xFF;
  _buf[2] = 0xFF;
  _buf[3] = 0xFF;
  _buf[4] = 0xFF;
  _buf[5] = 0xFF;
  _buf[6] = 0xFF;
  _buf[7] = 0xFE;
  CAN.MCP_CAN::sendMsgBuf(_canId, 0, 8, _buf);
}

void AK10_9::sendCommand(){
  /// CAN Reply Packet Structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and + 30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  //limit data to bounds
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float t_des = constrain(t_in, T_MIN, T_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);

  //convert to uint
  unsigned int p_int = floatToUint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = floatToUint(v_des, V_MIN, V_MAX, 12);
  unsigned int t_int = floatToUint(t_des, T_MIN, T_MAX, 12);
  unsigned int kp_int = floatToUint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = floatToUint(kd, KD_MIN, KD_MAX, 12);

  //construct CAN packet
  _buf[0] = p_int >> 8;
  _buf[1] = p_int & 0xFF;
  _buf[2] = v_int >> 4;
  _buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  _buf[4] = kp_int & 0xFF;
  _buf[5] = kd_int >> 4;
  _buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  _buf[7] = t_int & 0xFF;
  
  CAN.MCP_CAN::sendMsgBuf(_canId, 0, 8, _buf);
}

bool AK10_9::getReply(){
  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  if (!CAN.checkReceive()) return false;
  CAN.readMsgBuf(&_len, _buf);
  
  //unpack uints
  unsigned int id = _buf[0];
  unsigned int p_int = (_buf[1] << 8) | _buf[2];
  unsigned int v_int = (_buf[3] << 4) | _buf[4] >> 4;
  unsigned int t_int = ((_buf[4] & 0xF) << 8) | _buf[5];

  //convert to float
  _p_out = uintToFloat(p_int, P_MIN, P_MAX, 16);
  _v_out = uintToFloat(v_int, V_MIN, V_MAX, 12);
  _t_out = uintToFloat(t_int, T_MIN, T_MAX, 12);
  return true;
}

unsigned int AK10_9::getCanId(){
  return _canId;
}

float AK10_9::getPosition(){
  return _p_out;
}

float AK10_9::getVelocity(){
  return _v_out;
}

float AK10_9::getTorque(){
  return _t_out;
}

void AK10_9::enterMotorMode(){
  _buf[0] = 0xFF;
  _buf[1] = 0xFF;
  _buf[2] = 0xFF;
  _buf[3] = 0xFF;
  _buf[4] = 0xFF;
  _buf[5] = 0xFF;
  _buf[6] = 0xFF;
  _buf[7] = 0xFC;
  CAN.MCP_CAN::sendMsgBuf(_canId, 0, 8, _buf);
}

void AK10_9::exitMotorMode(){
  _buf[0] = 0xFF;
  _buf[1] = 0xFF;
  _buf[2] = 0xFF;
  _buf[3] = 0xFF;
  _buf[4] = 0xFF;
  _buf[5] = 0xFF;
  _buf[6] = 0xFF;
  _buf[7] = 0xFD;
  CAN.MCP_CAN::sendMsgBuf(_canId, 0, 8, _buf);
}

unsigned int AK10_9::floatToUint(float float_val, float min_val, float max_val, unsigned int size){
  return (unsigned int)((float_val-min_val)/(max_val-min_val)*(float)(0xFFFF >> (16 - size))); 
}

float AK10_9::uintToFloat(unsigned int int_val, float min_val, float max_val, unsigned int size){
  return ((float)int_val)*(max_val-min_val)/(float)(0xFFFF >> (16 - size))+min_val;
}