/**
 * Control de Movimientos Básicos - Robot Rocker-Bogie
 * 
 * Este código implementa el control básico de movimientos para un robot tipo Rocker-Bogie
 * diseñado para entornos agrícolas. Incluye control de traslación y rotación con protecciones
 * básicas y diagnóstico.
 * 
 * Características del hardware:
 * - Arduino MEGA R3
 * - 6 Motores TT (3-6V DC, relación 1:48)
 * - 3 Controladores L298N
 * 
 * @author: [Edgardo Tomas Martinez]
 * @version: 1.0
 * @date: Ultimo cambio octubre 2024
 */

// Definiciones de pines para motores izquierdos en base a configuración
// Analizada y versión final.
#define MOTLeft_1_ENB 9    // Motor Frontal Izquierdo
#define MOTLeft_1_IN3 38
#define MOTLeft_1_IN4 39
#define MOTLeft_2_ENB 6    // Motor Medio Izquierdo
#define MOTLeft_2_IN3 26
#define MOTLeft_2_IN4 27
#define MOTLeft_3_ENA 7    // Motor Trasero Izquierdo
#define MOTLeft_3_IN1 30
#define MOTLeft_3_IN2 31

// Definiciones de pines para motores derechos
#define MOTRight_1_ENA 8   // Motor Frontal Derecho
#define MOTRight_1_IN1 34
#define MOTRight_1_IN2 35
#define MOTRight_2_ENA 5   // Motor Medio Derecho
#define MOTRight_2_IN1 22
#define MOTRight_2_IN2 23
#define MOTRight_3_ENB 10  // Motor Trasero Derecho
#define MOTRight_3_IN3 42
#define MOTRight_3_IN4 43

// Constantes de operación
#define MIN_SPEED 0      // Velocidad mínima (~3V)
#define MAX_SPEED 255      // Velocidad máxima (~6V)
#define SPEED_STEP 1      // Incremento de velocidad
#define TURN_INNER_RATIO 0.4  // Reducción de velocidad para rueda interior en giros
#define TURN_OUTER_RATIO 0.8  // Reducción de velocidad para rueda exterior en giros
#define DIRECTION_CHANGE_DELAY 200  // Retardo en ms para cambios de dirección
int i; //Variable para uso en pruebas de baterías 18650

/**
 * Clase Motor: Control individual de cada motor DC
 * Maneja la configuración, dirección y velocidad de un motor.
 */
class Motor {
  private:
    int pwmPin;     // Pin PWM para control de velocidad
    int in1Pin;     // Pin de control de dirección 1
    int in2Pin;     // Pin de control de dirección 2
    int speed;      // Velocidad actual (0-255)
    bool isRunning; // Estado del motor
    unsigned long lastDirectionChange; // Tiempo del último cambio de dirección

  public:
    // Constructor por defecto
    Motor() : pwmPin(0), in1Pin(0), in2Pin(0), speed(0), isRunning(false), lastDirectionChange(0) {}
    
    /**
     * Constructor con parámetros
     * @param pwmPin Pin PWM para control de velocidad
     * @param in1Pin Primer pin de control de dirección
     * @param in2Pin Segundo pin de control de dirección
     */
    Motor(int pwmPin, int in1Pin, int in2Pin)
        : pwmPin(pwmPin), in1Pin(in1Pin), in2Pin(in2Pin), 
          speed(0), isRunning(false), lastDirectionChange(0) {
        init();
    }

    /**
     * Inicializa los pines del motor
     */
    void init() {
        pinMode(pwmPin, OUTPUT);
        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);
        stop();
    }

    /**
     * Mueve el motor hacia adelante
     */
    void forward() {
        if (canChangeDirection()) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
            analogWrite(pwmPin, speed);
            isRunning = true;
            lastDirectionChange = millis();
        }
    }

    /**
     * Mueve el motor hacia atrás
     */
    void backward() {
        if (canChangeDirection()) {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
            analogWrite(pwmPin, speed);
            isRunning = true;
            lastDirectionChange = millis();
        }
    }
    
    /**
     * Detiene el motor
     */
    void stop() {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, 0);
        this->speed = 0;
        isRunning = false;
    }
    
    /**
     * Establece la velocidad del motor
     * @param newSpeed Nueva velocidad (0-255)
     */
    void setSpeed(int newSpeed) {
        // Aseguramos que la velocidad esté en el rango válido
        this->speed = constrain(newSpeed, 0, MAX_SPEED);
        if (isRunning) {
            analogWrite(pwmPin, this->speed);
        }
    }

    /**
     * Verifica si es seguro cambiar la dirección
     */
    bool canChangeDirection() {
        return (millis() - lastDirectionChange) >= DIRECTION_CHANGE_DELAY;
    }

    // Getters
    int getSpeed() { return speed; }
    bool isActive() { return isRunning; }
};

/**
 * Clase RockerBogieControl: Gestión del movimiento del robot
 * Coordina los seis motores para realizar los movimientos básicos.
 */
class RockerBogieControl {
  private:
    Motor& frontLeft;    // Motor frontal izquierdo
    Motor& frontRight;   // Motor frontal derecho
    Motor& middleLeft;   // Motor medio izquierdo
    Motor& middleRight;  // Motor medio derecho
    Motor& rearLeft;     // Motor trasero izquierdo
    Motor& rearRight;    // Motor trasero derecho
    int currentSpeed;    // Velocidad actual del sistema

    /**
     * Establece la velocidad de todos los motores
     * @param speed Velocidad deseada
     */
    void setAllSpeeds(int speed) {
        frontLeft.setSpeed(speed);
        frontRight.setSpeed(speed);
        middleLeft.setSpeed(speed);
        middleRight.setSpeed(speed);
        rearLeft.setSpeed(speed);
        rearRight.setSpeed(speed);
    }

    /**
     * Establece velocidades diferentes para cada lado
     * @param leftSpeed Velocidad lado izquierdo
     * @param rightSpeed Velocidad lado derecho
     */
    void setSideSpeeds(int leftSpeed, int rightSpeed) {
        frontLeft.setSpeed(leftSpeed);
        middleLeft.setSpeed(leftSpeed);
        rearLeft.setSpeed(leftSpeed);
        
        frontRight.setSpeed(rightSpeed);
        middleRight.setSpeed(rightSpeed);
        rearRight.setSpeed(rightSpeed);
    }

  public:
    /**
     * Constructor
     * Inicializa el control con referencias a todos los motores
     */
    RockerBogieControl(
        Motor& fl, Motor& fr, Motor& ml, Motor& mr, Motor& rl, Motor& rr)
        : frontLeft(fl), frontRight(fr), 
          middleLeft(ml), middleRight(mr), 
          rearLeft(rl), rearRight(rr), 
          currentSpeed(0) {}

    /**
     * Movimiento hacia adelante
     * @param speed Velocidad deseada
     */
    void moveForward(int speed) {
        currentSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
        setAllSpeeds(currentSpeed);
        
        frontLeft.forward();
        frontRight.forward();
        middleLeft.forward();
        middleRight.forward();
        rearLeft.forward();
        rearRight.forward();
    }

    /**
     * Movimiento hacia atrás
     * @param speed Velocidad deseada
     */
    void moveBackward(int speed) {
        currentSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
        setAllSpeeds(currentSpeed);
        
        frontLeft.backward();
        frontRight.backward();
        middleLeft.backward();
        middleRight.backward();
        rearLeft.backward();
        rearRight.backward();
    }

    /**
     * Giro sobre eje central hacia la izquierda
     * @param speed Velocidad base del giro
     */
    void turnLeft(int speed) {
        int adjustedSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
        setSideSpeeds(adjustedSpeed * TURN_INNER_RATIO, 
                     adjustedSpeed * TURN_OUTER_RATIO);
        
        frontLeft.backward();
        middleLeft.backward();
        rearLeft.backward();
        
        frontRight.forward();
        middleRight.forward();
        rearRight.forward();
    }

    /**
     * Giro sobre eje central hacia la derecha
     * @param speed Velocidad base del giro
     */
    void turnRight(int speed) {
        int adjustedSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
        setSideSpeeds(adjustedSpeed * TURN_OUTER_RATIO, 
                     adjustedSpeed * TURN_INNER_RATIO);
        
        frontLeft.forward();
        middleLeft.forward();
        rearLeft.forward();
        
        frontRight.backward();
        middleRight.backward();
        rearRight.backward();
    }

    /**
     * Detiene todos los motores
     */
    void stop() {
        frontLeft.stop();
        frontRight.stop();
        middleLeft.stop();
        middleRight.stop();
        rearLeft.stop();
        rearRight.stop();
        currentSpeed = 0;
    }

    /**
     * Ajusta la velocidad actual
     * @param adjustment Valor de ajuste (+/-)
     */
    void adjustSpeed(int adjustment) {
        currentSpeed = constrain(currentSpeed + adjustment, MIN_SPEED, MAX_SPEED);
        if(frontLeft.isActive()) {
            setAllSpeeds(currentSpeed);
        }
    }

    // Getters
    int getCurrentSpeed() { return currentSpeed; }
};

// Instancias de motores
Motor motorL_1(MOTLeft_1_ENB, MOTLeft_1_IN3, MOTLeft_1_IN4);   // Frontal Izquierdo
Motor motorR_1(MOTRight_1_ENA, MOTRight_1_IN1, MOTRight_1_IN2); // Frontal Derecho
Motor motorL_2(MOTLeft_2_ENB, MOTLeft_2_IN3, MOTLeft_2_IN4);   // Medio Izquierdo
Motor motorR_2(MOTRight_2_ENA, MOTRight_2_IN1, MOTRight_2_IN2); // Medio Derecho
Motor motorL_3(MOTLeft_3_ENA, MOTLeft_3_IN1, MOTLeft_3_IN2);   // Trasero Izquierdo
Motor motorR_3(MOTRight_3_ENB, MOTRight_3_IN3, MOTRight_3_IN4); // Trasero Derecho

// Instancia del controlador principal
RockerBogieControl robot(motorL_1, motorR_1, motorL_2, motorR_2, motorL_3, motorR_3);

// Variable global para velocidad de prueba
int SpeedTest = MIN_SPEED;
int variableControl;

/**
 * Configuración inicial del sistema
 */
void setup() {
    Serial.begin(9600);
    printInstructions();
    variableControl = 1;
}

/**
 * Imprime las instrucciones de uso
 */
void printInstructions() {
    Serial.println(F("=== Control de Robot Rocker-Bogie ==="));
    Serial.println(F("Comandos disponibles:"));
    Serial.println(F("f: Adelante"));
    Serial.println(F("b: Atrás"));
    Serial.println(F("l: Giro izquierda"));
    Serial.println(F("r: Giro derecha"));
    Serial.println(F("s: Stop"));
    Serial.println(F("+: Aumentar velocidad"));
    Serial.println(F("-: Disminuir velocidad"));
    Serial.println(F("h: Mostrar comandos"));
    Serial.println(F("================================"));
}

/**
 * Bucle principal del programa
 */
void loop() {
    /*if (Serial.available() > 0) {
        char command = Serial.read();
        executeCommand(command);
    }*/
  //Código para pruebas de baterías. Capacidad de trabajo 100%
  if (variableControl <= 255)
  {
    variableControl += 1;
    delay(50);
  }
  robot.moveForward(variableControl);
  SpeedTest = robot.getCurrentSpeed();
  Serial.print(F("Velocidad: "));
  Serial.println(SpeedTest);
}

/**
 * Ejecuta el comando recibido
 * @param command Comando a ejecutar
 */
void executeCommand(char command) {
    switch(command) {
        case 'f':
            robot.moveForward(SpeedTest);
            Serial.println(F("Movimiento: Adelante"));
            break;
            
        case 'b':
            robot.moveBackward(SpeedTest);
            Serial.println(F("Movimiento: Atrás"));
            break;
            
        case 'l':
            robot.turnLeft(SpeedTest);
            Serial.println(F("Movimiento: Giro Izquierda"));
            break;
            
        case 'r':
            robot.turnRight(SpeedTest);
            Serial.println(F("Movimiento: Giro Derecha"));
            break;
            
        case 's':
            robot.stop();
            Serial.println(F("Movimiento: Stop"));
            break;
            
        case '+':
            robot.adjustSpeed(SPEED_STEP);
            SpeedTest = robot.getCurrentSpeed();
            Serial.print(F("Velocidad: "));
            Serial.println(SpeedTest);
            break;
            
        case '-':
            robot.adjustSpeed(-SPEED_STEP);
            SpeedTest = robot.getCurrentSpeed();
            Serial.print(F("Velocidad: "));
            Serial.println(SpeedTest);
            break;
            
        case 'h':
            printInstructions();
            break;
            
        default:
            // Ignora comandos no reconocidos
            break;
    }
}