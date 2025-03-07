#define MOTLeft_1_ENB 9
#define MOTLeft_1_IN3 38
#define MOTLeft_1_IN4 39
#define MOTLeft_2_ENB 6
#define MOTLeft_2_IN3 26
#define MOTLeft_2_IN4 27
#define MOTLeft_3_ENA 7
#define MOTLeft_3_IN1 30
#define MOTLeft_3_IN2 31

//RIGH motors
#define MOTRight_1_ENA 8
#define MOTRight_1_IN1 34
#define MOTRight_1_IN2 35
#define MOTRight_2_ENA 5
#define MOTRight_2_IN1 22
#define MOTRight_2_IN2 23
#define MOTRight_3_ENB 10
#define MOTRight_3_IN3 42
#define MOTRight_3_IN4 43

int SpeedTest = 150;

class Motor
{
  private:
     int pwmPin;
     int in1Pin;
     int in2Pin;
     int speed;
  public:
    //Constructor default sin uso
    Motor() {}
    //Constructor
    Motor(int pwmPin, int in1Pin, int in2Pin)
    {
      this->pwmPin = pwmPin;
      this->in1Pin = in1Pin;
      this->in2Pin = in2Pin;
      this->speed = 0;
      init();
     }
    //Métodos
    void init()
    {
      //Definimos Pines como salida pwm e in
      pinMode(pwmPin, OUTPUT);
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
       // Asegurarse que el motor esté detenido al iniciar
      stop();
      //Para agregar más rutinas de inicialización
    }

    void forward() 
    {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, speed);
    }

    void backward()
    {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, speed);
    }
    
    void stop()
    {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);
      this->speed = 0;
    }
    
    //Método para asignar velocidad
    void setSpeed(int newSpeed)
    {
      // Aseguramos que la velocidad esté en el rango 0-255
      this->speed = constrain(newSpeed, 0, 255);
      analogWrite(pwmPin, this->speed);
    }

    // Método para leer la velocidad
    int getSpeed()
    {
      return this->speed;
    }
};

//MOTORS
//FRONT AXLE
  //front axle left
Motor motorL_1 (MOTLeft_1_ENB, MOTLeft_1_IN3, MOTLeft_1_IN4);
  //front axle rigth
Motor motorR_1 (MOTRight_1_ENA, MOTRight_1_IN1, MOTRight_1_IN2);
//MIDDLE AXLE
  //Middle axle Left
Motor motorL_2 (MOTLeft_2_ENB, MOTLeft_2_IN3, MOTLeft_2_IN4);
  //Middle axle right
Motor motorR_2 (MOTRight_2_ENA, MOTRight_2_IN1, MOTRight_2_IN2);
//REAR AXLE
  //Rear axle left
Motor motorL_3 (MOTLeft_3_ENA, MOTLeft_3_IN1, MOTLeft_3_IN2);
  //Rear axle right
Motor motorR_3 (MOTRight_3_ENB, MOTRight_3_IN3, MOTRight_3_IN4);


void setup() 
{
  //Código de Inicialización SETUP
  Serial.begin(9600);
    //Serial.println("Motores inicializados");
}

void loop() 
{
  // Código Loop
  if (Serial.available() > 0) {
    char comando = Serial.read();
    // Controlamos el motor en función del comando recibido
    if (comando == 'f') {
      controlarMotores(SpeedTest, true);
    } else if (comando == 'b') {
        controlarMotores(SpeedTest, false);
    } else if (comando == 's') {
        detenerMotores();
        SpeedTest = 0;
    } else if (comando == '+') {  // '+' para aumentar la velocidad
        ajustarVelocidad(15);
    } else if (comando == '-') {  // '-' para disminuir la velocidad
        ajustarVelocidad(-15);
    }
  }
}

void controlarMotores(int speed, bool forward) {
  if (forward) {
      motorL_1.setSpeed(speed);
      motorL_1.forward();
      motorL_2.setSpeed(speed);
      motorL_2.forward();
      motorL_3.setSpeed(speed);
      motorL_3.forward();
      motorR_1.setSpeed(speed);
      motorR_1.forward();
      motorR_2.setSpeed(speed);
      motorR_2.forward();
      motorR_3.setSpeed(speed);
      motorR_3.forward();
  } else {
      motorL_1.setSpeed(speed);
      motorL_1.backward();
      motorL_2.setSpeed(speed);
      motorL_2.backward();
      motorL_3.setSpeed(speed);
      motorL_3.backward();

      motorR_1.setSpeed(speed);
      motorR_1.backward();
      motorR_2.setSpeed(speed);
      motorR_2.backward();
      motorR_3.setSpeed(speed);
      motorR_3.backward();
  }
}

void detenerMotores() {
  motorL_1.stop();
  motorL_2.stop();
  motorL_3.stop();
  motorR_1.stop();
  motorR_2.stop();
  motorR_3.stop();
}

void ajustarVelocidad(int ajuste) {
  SpeedTest = constrain(SpeedTest + ajuste, 0, 255);
  motorL_1.setSpeed(SpeedTest);
  motorL_2.setSpeed(SpeedTest);
  motorL_3.setSpeed(SpeedTest);
  motorR_1.setSpeed(SpeedTest);
  motorR_2.setSpeed(SpeedTest);
  motorR_3.setSpeed(SpeedTest);
}

