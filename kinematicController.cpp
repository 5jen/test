#include "KinematicController.h"

KinematicController::KinematicController(RegulatedMotor* leftMotor, RegulatedMotor* rightMotor,
		int leftMotorDirection, int rightMotorDirection,
		unsigned int wheelDistance, unsigned int wheelDiameter, unsigned int encoderCPR){
	this->leftMotor = leftMotor;
	this->rightMotor = rightMotor;
	this->leftMotorDirection = leftMotorDirection;
	this->rightMotorDirection = rightMotorDirection;
	this->wheelDistance = wheelDistance;
	this->wheelDiameter = wheelDiameter;
	this->encoderCPR = encoderCPR;
}

void KinematicController::setAcceleration(
	unsigned int forwardAcceleration, unsigned int ccwAcceleration, 
	unsigned int forwardDeceleration, unsigned int ccwDeceleration){
	
	this->forwardAcceleration = mmToTick(forwardAcceleration);
	this->ccwAcceleration = degToTick(ccwAcceleration);
	this->forwardDeceleration = mmToTick(forwardDeceleration);
	this->ccwDeceleration = degToTick(ccwDeceleration);

	atomicForwardAcceleration = (long)forwardAcceleration * sampleTime / 1000;
	atomicForwardDeceleration = (long)forwardDeceleration * sampleTime / 1000;
	atomicCCWAcceleration = (long)ccwAcceleration * sampleTime / 1000;
	atomicCCWDeceleration = (long)ccwDeceleration * sampleTime / 1000;
}


boolean KinematicController::run(){
	unsigned long currentTime = millis();
	if (currentTime - lastRunTime < sampleTime){
		return false;
	}

	long forwardOutput = 0;
	long ccwOutput = 0;

	if (state == KINEMATIC_VELOCITY){
		if (lastForwardVelocity == targetForwardVelocity) {
			forwardOutput = targetForwardVelocity;
			standby = true;
		}
		else {
			forwardOutput = speedRamp(lastForwardVelocity, targetForwardVelocity, atomicForwardAcceleration, atomicForwardDeceleration);
			standby = false;
		}

		if (lastCCWVelocity == targetCCWVelocity) {
			ccwOutput = targetCCWVelocity;
			standby = true;
		}
		else {
			ccwOutput = speedRamp(lastCCWVelocity, targetCCWVelocity, atomicCCWAcceleration, atomicCCWDeceleration);
			standby = false;
		}

		leftMotor->setSpeed(calculateLeftWheelSpeed(forwardOutput, ccwOutput));
		rightMotor->setSpeed(calculateRightWheelSpeed(forwardOutput, ccwOutput));
		lastForwardVelocity = forwardOutput;
		lastCCWVelocity = ccwOutput;
	} else if (state = KINEMATIC_OFF) {
		standby = true;
		lastForwardVelocity = 0;
		lastCCWVelocity = 0;
	}



	lastRunTime = currentTime;

	return true;
}

void KinematicController::goVelocity(int forwardVelocity, int ccwVelocity){
	state = KINEMATIC_VELOCITY;
	targetForwardVelocity = mmToTick(forwardVelocity);
	targetCCWVelocity = degToTick(ccwVelocity);
}

void KinematicController::goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed){
	state = KINEMATIC_POSITION;
	_goPosition(forwardDistance, ccwAngle, forwardSpeed, ccwSpeed);
}

void KinematicController::brake(){
	state = KINEMATIC_OFF;
	leftMotor->setState(MOTORSTATE_BRAKE);
	rightMotor->setState(MOTORSTATE_BRAKE);
}

void KinematicController::coast(){
	state = KINEMATIC_OFF;
	leftMotor->setState(MOTORSTATE_COAST);
	rightMotor->setState(MOTORSTATE_COAST);
}

void KinematicController::_goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed){
	originTime = millis();
	originForwardTick = calculateForwardTick();
	originCCWTick = calculateCCWTick();
	targetForwardTick = mmToTick(forwardDistance);
	targetCCWTick = degToTick(ccwAngle);
	targetForwardVelocity = forwardSpeed;
	targetCCWVelocity = ccwSpeed;
}


long KinematicController::mmToTick(long mm){
	return (mm*encoderCPR)/(3.142*wheelDiameter);
}

long KinematicController::degToTick(long deg){
	return mmToTick(deg*wheelDistance*3.142/180)/2;
}

long KinematicController::calculateLeftWheelSpeed(long forwardVelocity, long ccwVelocity){
	return (forwardVelocity - ccwVelocity)*leftMotorDirection;
}

long KinematicController::calculateRightWheelSpeed(long forwardVelocity, long ccwVelocity){
	return (forwardVelocity + ccwVelocity)*rightMotorDirection;
}

long KinematicController::calculateForwardTick(){
	return (leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection)/2;
}

long KinematicController::calculateCCWTick(){
	return (-leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection);
}

long KinematicController::speedRamp(long last, long target,long up, long down){
	long ret;
    if (last >= 0){
        if (target > last){
            ret = last + up;
            if (ret > target) ret = target;
        } else {
            ret = last - down;
            if (ret < target) ret = target;
        }
    } else { // last < 0
        if (target > last){
            ret = last + down;
            if (ret > target) ret = target;
        } else {
            ret = last - up;
            if (ret < target) ret = target;
        }
    }
    return ret;
}

boolean KinematicController::isStandby(){
	return standby;
}

long KinematicController::getOdometryForward(){
	return calculateForwardTick()* (3.142*wheelDiameter) / encoderCPR;
}

long KinematicController::getOdometryCCW(){
	return (calculateCCWTick() * (3.142*wheelDiameter) / encoderCPR)/(wheelDistance*3.142/180);
}