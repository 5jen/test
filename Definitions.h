#define SAFE_DISTANCE 23
const int trigger_pin = 22;
const int echo_pin =  23;
const int front_ir_pin = A3;
//const int rear_ir_pin ;

int lasthb = 0;
int lastRightTurn = 0;
int lastEncoderSample = 0;
int lastPing = 0;
int lastir = 0;

long l, r, reference_l = 0;

unsigned int frontWallDistance;
unsigned int frontIrValue;
unsigned int rearIrValue;
unsigned int sideWallDistance;
const int OpenDistance = 50;
const int threshHold = 230;

enum dirveState{goStraight, turnLeft_90, turnRight_90, turnToOpenArea, };

bool facingCliff, nearFrontWall, rightIsOpen, atCliff, getCurrentPos = false;

NewPing sonar(trigger_pin, echo_pin);
MotorControllerMaster c;