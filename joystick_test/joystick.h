#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1

typedef struct {
    float x;
    float y;
  } Position;

typedef struct {
    float left;
    float right;
  } MotorValues;

Position getPosition() {
    Position pos;
    pos.x = analogRead(JOYSTICK_X_PIN)/676.0;
    pos.y = analogRead(JOYSTICK_Y_PIN)/676.0;
    return pos;
  }

MotorValues getMotorValues() {
    Position pos = getPosition();
    MotorValues values;
    values.left=pos.y*255;
    values.right=pos.y*255;
    return values;
  }


