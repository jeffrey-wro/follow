#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "I2C.h"


enum ADDRESS{
	DC = 1,
	SERVO = 2
};
enum DC_CHANNEL{
	DC_1 = 0,
	DC_2 = 1,
};
enum SERVO_CHANNEL{
	SERVO_1 = 1,
	SERVO_2 = 2,
	SERVO_3 = 3,
	SERVO_4 = 4,
	SERVO_5 = 5,
	SERVO_6 = 6,
	CR_SERVO_1 = 7,
	CR_SERVO_2 = 8
};

class Motor_Controller {
private:
	MyRio_I2c* i2c;

public:

	Motor_Controller(MyRio_I2c* i2c);

	void waitFor (unsigned int secs);

	//controller function
    void controllerEnable(int address);
    void controllerReset(int address);
    int readBatteryVoltage(int address);
    int readFirmware(int address);

	//DC motor functions
    void setMotorSpeed(int address, uint8_t channel, long Mspeed);
    void setMotorSpeeds(int address, long Mspeed1, long Mspeed2);
    void setMotorPower(int address, uint8_t channel, uint8_t power);
    void setMotorPowers(int address, uint8_t power1, uint8_t power2);
    void setMotorDegree(int address, uint8_t channel, long speed, long degrees);
    void setMotorDegrees(int address, long speed1, long degrees1, long speed2, long degrees2);
    void setMotorTarget(int address, uint8_t channel, long speed, long target);
    void setMotorTargets(int address, long speed1, long target1, long speed2, long target2);
    void setMotorSpeedPID(int address, int P, int I, int D);
    void setMotorTargetPID(int address, int P, int I, int D);
    void setMotorInvert(int address, uint8_t channel, uint8_t invert);

    long readEncoderCount(int address, uint8_t channel);
    long readEncoderDegrees(int address, uint8_t channel);
    void resetEncoder(int address, uint8_t channel);
    void resetEncoders(int address);

    int readMotorBusy(int address, uint8_t channel);
    int readMotorCurrent(int address, uint8_t channel);

	//Servo motor functions
    void setServoSpeed(int address, uint8_t channel, uint8_t servospeed);
    void setServoSpeeds(int address, uint8_t servospeed1, uint8_t servospeed2, uint8_t servospeed3, uint8_t servospeed4, uint8_t servospeed5, uint8_t servospeed6);
    void setServoPosition(int address, uint8_t channel, uint8_t servoposition);
    void setServoPositions(int address, uint8_t servoposition1,uint8_t servoposition2,uint8_t servoposition3,uint8_t servoposition4,uint8_t servoposition5,uint8_t servoposition6);
    void setCRServoState(int address, uint8_t channel, uint8_t servospeed);

    uint8_t readServoPosition(int address, uint8_t channel);


    /********************************************

     Functions yet to implement


	//Controller functions
     void WDT_STOP(int address);
     void setExpID(int newID);
     int readExpID();













     */
};

#endif
