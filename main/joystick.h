#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1

typedef struct {
    int x;
    int y;
  } Position;

typedef struct {
    int left;
    int right;
  } MotorValues;

Position getPosition() {
    Position pos;
    pos.x = analogRead(JOYSTICK_X_PIN)-512;
    pos.y = analogRead(JOYSTICK_Y_PIN)-512;
    return pos;
  }

MotorValues getMotorValues() {
    Position pos = getPosition();
    MotorValues values;
    values.left=((pos.y-pos.x)/8)+128;
    values.right=((pos.y+pos.x)/8)+128;
    return values;
  }

