/**
 * Control de Movimientos Básicos - Robot Rocker-Bogie
 * 
 * Este código implementa el control básico de movimientos para un robot tipo Rocker-Bogie
 * diseñado para entornos agrícolas, adaptado para motores tipo gusano con driver DBH-12.
 * Incluye control de traslación y rotación con protecciones básicas y diagnóstico.
 * 
 * Características del hardware:
 * - Arduino MEGA R3
 * - 6 Motores tipo gusano
 * - 3 Controladores DBH-12 H-Bridge (2 motores por driver)
 * 
 * @author: [Edgardo Tomas Martinez]
 * @version: 2.0
 * @date: Febrero 2025
 */

// Definiciones de pines para motores izquierdos
// Configuración para driver DBH-12 (2 pines PWM por motor)
#define MOTLeft_1_IN1 10    // Motor Frontal Izquierdo
#define MOTLeft_1_IN2 11
#define MOTLeft_2_IN1 8     // Motor Medio Izquierdo
#define MOTLeft_2_IN2 9
#define MOTLeft_3_IN1 6     // Motor Trasero Izquierdo
#define MOTLeft_3_IN2 7

// Definiciones de pines para motores derechos
#define MOTRight_1_IN1 12   // Motor Frontal Derecho
#define MOTRight_1_IN2 13
#define MOTRight_2_IN1 4    // Motor Medio Derecho
#define MOTRight_2_IN2 5
#define MOTRight_3_IN1 2    // Motor Trasero Derecho
#define MOTRight_3_IN2 3

// Definiciones de pines para lectura de corriente y voltaje
#define CT_Left_1 A0    // Pin CT del Drive 1 - Motor Izquierdo
#define CT_Right_1 A1   // Pin CT del Drive 1 - Motor Derecho
#define CT_Left_2 A2    // Pin CT del Drive 2 - Motor Izquierdo
#define CT_Right_2 A3   // Pin CT del Drive 2 - Motor Derecho
#define CT_Left_3 A4    // Pin CT del Drive 3 - Motor Izquierdo
#define CT_Right_3 A5   // Pin CT del Drive 3 - Motor Derecho

// Constantes de operación
#define MIN_SPEED 120      // Velocidad mínima
#define MAX_SPEED 250      // Velocidad máxima para DBH-12 (98% de 255)
#define SPEED_STEP 10      // Incremento de velocidad
#define TURN_INNER_RATIO 1  // Reducción de velocidad para rueda interior en giros
#define TURN_OUTER_RATIO 1  // Reducción de velocidad para rueda exterior en giros
#define DIRECTION_CHANGE_DELAY 100  // Retardo en ms para cambios de dirección
#define MOTOR_SAFE_PWM 150  // PWM considerado seguro para picos de corriente en giros
#define CHANGE_DIRECTION_TIMEOUT 1000 // Tiempo máximo para cambio de dirección (ms)

// Constante para la conversión de voltaje a corriente (V = I * 0.155)
//const float CT_CONVERSION_FACTOR = 0.155;
const float CT_CONVERSION_FACTOR = 0.55;
// Variables para el circuito rectangular
unsigned long forwardTime = 0;    // Tiempo para movimiento hacia adelante
unsigned long turnTime = 0;       // Tiempo para giro
bool isCircuitRunning = false;    // Estado del circuito
unsigned long lastStateChange = 0; // Control de tiempo
int circuitState = 0;            // Estado actual del circuito (0-3)
unsigned long circuitStartTime = 0;    // Tiempo cuando inicia el circuito
unsigned long totalRunTime = 0;        // Tiempo total de ejecución
bool stateInitialized = false;  // Variable global
int lastState = -1;  // Para control de estados del circuito
int SpeedTest = MIN_SPEED;  // Variable global para velocidad de prueba
int variableControl;

/**
 * Clase Motor: Control individual de cada motor DC
 * Adaptada para driver DBH-12 con configuración 2 PWM
 * Con protección contra picos de corriente en cambios de dirección
 */
class Motor {
  private:
    int in1Pin;     // Primer pin PWM
    int in2Pin;     // Segundo pin PWM
    int speed;      // Velocidad deseada (0-250)
    bool isRunning; // Estado del motor
    bool isForward; // Dirección actual (true = forward, false = backward)
    unsigned long lastDirectionChange; // Tiempo del último cambio de dirección
    unsigned long rampStartTime;  // Variables para rampa
    int targetSpeed;  // Velocidad final deseada
    int currentPWM;   // PWM actual durante la rampa
    static const int RAMP_TIME = 2000;    // 2 segundos de rampa
    static const int RAMP_THRESHOLD = 40;  // Diferencia para activar rampa
    static const int DIRECTION_CHANGE_PWM = 120; // PWM reducido para cambios de dirección
    static const int DECELERATION_TIME = 300;   // Tiempo de desaceleración en ms
    static const int DIRECTION_PAUSE = 100;     // Pausa entre cambio de dirección en ms

    void rampTo(int newSpeed) {
        int startPWM = currentPWM;
        targetSpeed = newSpeed;
        rampStartTime = millis();
        
        unsigned long elapsed = millis() - rampStartTime;
        if (elapsed < RAMP_TIME) {
            float progress = (float)elapsed / RAMP_TIME;
            currentPWM = startPWM + (targetSpeed - startPWM) * progress;
            // Actualizar el motor según la dirección
            if (isForward) {
                digitalWrite(in1Pin, LOW);
                analogWrite(in2Pin, currentPWM);
            } else {
                analogWrite(in1Pin, currentPWM);
                digitalWrite(in2Pin, LOW);
            }
        } else {
            currentPWM = targetSpeed;
            // Actualizar el motor según la dirección
            if (isForward) {
                digitalWrite(in1Pin, LOW);
                analogWrite(in2Pin, currentPWM);
            } else {
                analogWrite(in1Pin, currentPWM);
                digitalWrite(in2Pin, LOW);
            }
        }
    }
    
    // Función para cambio suave de dirección
    void smoothDirectionChange(bool toForward) {
        // Si ya está en la dirección correcta o no está en movimiento, no hacemos nada
        if (!isRunning || toForward == isForward) return;
        
        int originalSpeed = currentPWM;
        
        // 1. Desacelerar gradualmente
        for (int i = originalSpeed; i >= DIRECTION_CHANGE_PWM; i -= 10) {
            if (isForward) {
                digitalWrite(in1Pin, LOW);
                analogWrite(in2Pin, i);
            } else {
                analogWrite(in1Pin, i);
                digitalWrite(in2Pin, LOW);
            }
            delay(DECELERATION_TIME / ((originalSpeed - DIRECTION_CHANGE_PWM) / 10 + 1));
        }
        
        // 2. Detener el motor
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        delay(DIRECTION_PAUSE);
        
        // 3. Cambiar la dirección y actualizar la variable
        isForward = toForward;
        
        // 4. Iniciar con PWM bajo en la nueva dirección
        if (isForward) {
            digitalWrite(in1Pin, LOW);
            analogWrite(in2Pin, DIRECTION_CHANGE_PWM);
        } else {
            analogWrite(in1Pin, DIRECTION_CHANGE_PWM);
            digitalWrite(in2Pin, LOW);
        }
        
        // 5. Acelerar gradualmente hasta la velocidad original
        for (int i = DIRECTION_CHANGE_PWM; i <= originalSpeed; i += 10) {
            if (isForward) {
                digitalWrite(in1Pin, LOW);
                analogWrite(in2Pin, i);
            } else {
                analogWrite(in1Pin, i);
                digitalWrite(in2Pin, LOW);
            }
            delay(DECELERATION_TIME / ((originalSpeed - DIRECTION_CHANGE_PWM) / 10 + 1));
        }
        
        // Actualizar el PWM actual
        currentPWM = originalSpeed;
    }

  public:
    // Constructor por defecto
    Motor() : in1Pin(0), in2Pin(0), speed(0), isRunning(false), isForward(true),
              lastDirectionChange(0), currentPWM(0), targetSpeed(0) {}
    
    /**
     * Constructor con parámetros
     * @param in1Pin Primer pin PWM
     * @param in2Pin Segundo pin PWM
     */
    Motor(int in1Pin, int in2Pin)
        : in1Pin(in1Pin), in2Pin(in2Pin), 
          speed(0), isRunning(false), isForward(true), lastDirectionChange(0),
          currentPWM(0), targetSpeed(0) {
        init();
    }

    /**
     * Inicializa los pines del motor
     */
    void init() {
        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);
        stop();
    }

    /**
     * Mueve el motor hacia adelante
     * Para DBH-12: IN1=LOW, IN2=PWM para avanzar
     */
    void forward() {
        if (canChangeDirection()) {
            if (isRunning && !isForward && currentPWM > DIRECTION_CHANGE_PWM) {
                // Si estamos en movimiento en dirección contraria, usar cambio suave
                smoothDirectionChange(true);
            } else {
                // Operación normal
                digitalWrite(in1Pin, LOW);
                analogWrite(in2Pin, speed);
                isForward = true;
            }
            
            isRunning = true;
            lastDirectionChange = millis();
            currentPWM = speed;
        }
    }

    /**
     * Mueve el motor hacia atrás
     * Para DBH-12: IN1=PWM, IN2=LOW para retroceder
     */
    void backward() {
        if (canChangeDirection()) {
            if (isRunning && isForward && currentPWM > DIRECTION_CHANGE_PWM) {
                // Si estamos en movimiento en dirección contraria, usar cambio suave
                smoothDirectionChange(false);
            } else {
                // Operación normal
                analogWrite(in1Pin, speed);
                digitalWrite(in2Pin, LOW);
                isForward = false;
            }
            
            isRunning = true;
            lastDirectionChange = millis();
            currentPWM = speed;
        }
    }
    
    /**
     * Detiene el motor con desaceleración gradual
     * Para DBH-12: Ambos pines en LOW
     */
    void stop() {
        if (isRunning && currentPWM > 50) {
            // Desaceleración gradual antes de detener
            int originalSpeed = currentPWM;
            for (int i = originalSpeed; i >= 50; i -= 15) {
                if (isForward) {
                    digitalWrite(in1Pin, LOW);
                    analogWrite(in2Pin, i);
                } else {
                    analogWrite(in1Pin, i);
                    digitalWrite(in2Pin, LOW);
                }
                delay(5); // Breve pausa para desaceleración gradual
            }
        }
        
        // Detención completa
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        isRunning = false;
        currentPWM = 0;
    }
    
    /**
     * Establece la velocidad del motor
     * @param newSpeed Nueva velocidad (0-250)
     */
    void setSpeed(int newSpeed) {
        // Aseguramos que la velocidad esté en el rango válido para DBH-12
        newSpeed = constrain(newSpeed, MIN_SPEED, MAX_SPEED);
        
        if(abs(newSpeed - currentPWM) > RAMP_THRESHOLD && isRunning) {
            // Usar rampa para cambios grandes
            rampTo(newSpeed);
        } else {
            // Cambio directo para ajustes pequeños
            if (isRunning) {
                // Solo actualizar PWM si el motor está en marcha
                if (isForward) {
                    digitalWrite(in1Pin, LOW);
                    analogWrite(in2Pin, newSpeed);
                } else {
                    analogWrite(in1Pin, newSpeed);
                    digitalWrite(in2Pin, LOW);
                }
            }
            currentPWM = newSpeed;
        }
        this->speed = newSpeed;
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
        // Limitar velocidad para giros para reducir picos de corriente
        int adjustedSpeed = constrain(speed, MIN_SPEED, 
                            (speed > MOTOR_SAFE_PWM) ? MOTOR_SAFE_PWM : MAX_SPEED);
                            
        setSideSpeeds(adjustedSpeed * TURN_OUTER_RATIO, 
                     adjustedSpeed * TURN_INNER_RATIO);
        
        // Si cualquier motor estaba moviendo a velocidad alta, primero reducir velocidad
        bool needSlowdown = false;
        if (frontLeft.isActive() || middleLeft.isActive() || rearLeft.isActive() ||
            frontRight.isActive() || middleRight.isActive() || rearRight.isActive()) {
            stop();
            delay(50); // Breve pausa
            needSlowdown = true;
        }
        
        frontLeft.forward();
        middleLeft.forward();
        rearLeft.forward();
        
        frontRight.backward();
        middleRight.backward();
        rearRight.backward();
        
        // Si necesitamos limitar la velocidad y la original era mayor, incrementar gradualmente
        if (needSlowdown && speed > MOTOR_SAFE_PWM) {
            delay(200); // Permitir que se estabilice en velocidad baja
            adjustSpeed(speed - MOTOR_SAFE_PWM); // Incrementar a la velocidad deseada
        }
    }

    /**
     * Giro sobre eje central hacia la derecha
     * @param speed Velocidad base del giro
     */
    void turnRight(int speed) {
        // Limitar velocidad para giros para reducir picos de corriente
        int adjustedSpeed = constrain(speed, MIN_SPEED, 
                            (speed > MOTOR_SAFE_PWM) ? MOTOR_SAFE_PWM : MAX_SPEED);
                            
        setSideSpeeds(adjustedSpeed * TURN_INNER_RATIO, 
                     adjustedSpeed * TURN_OUTER_RATIO);
        
        // Si cualquier motor estaba moviendo a velocidad alta, primero reducir velocidad
        bool needSlowdown = false;
        if (frontLeft.isActive() || middleLeft.isActive() || rearLeft.isActive() ||
            frontRight.isActive() || middleRight.isActive() || rearRight.isActive()) {
            stop();
            delay(50); // Breve pausa
            needSlowdown = true;
        }
        
        frontLeft.backward();
        middleLeft.backward();
        rearLeft.backward();
        
        frontRight.forward();
        middleRight.forward();
        rearRight.forward();
        
        // Si necesitamos limitar la velocidad y la original era mayor, incrementar gradualmente
        if (needSlowdown && speed > MOTOR_SAFE_PWM) {
            delay(200); // Permitir que se estabilice en velocidad baja
            adjustSpeed(speed - MOTOR_SAFE_PWM); // Incrementar a la velocidad deseada
        }
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

// Instancias de motores con configuración DBH-12
Motor motorL_1(MOTLeft_1_IN1, MOTLeft_1_IN2);   // Frontal Izquierdo
Motor motorR_1(MOTRight_1_IN1, MOTRight_1_IN2); // Frontal Derecho
Motor motorL_2(MOTLeft_2_IN1, MOTLeft_2_IN2);   // Medio Izquierdo
Motor motorR_2(MOTRight_2_IN1, MOTRight_2_IN2); // Medio Derecho
Motor motorL_3(MOTLeft_3_IN1, MOTLeft_3_IN2);   // Trasero Izquierdo
Motor motorR_3(MOTRight_3_IN1, MOTRight_3_IN2); // Trasero Derecho

// Instancia del controlador principal
RockerBogieControl robot(motorL_1, motorR_1, motorL_2, motorR_2, motorL_3, motorR_3);

/**
 * Configuración inicial del sistema
 */
void setup() {
    Serial1.begin(9600);
    Serial.begin(9600);
    Serial.setTimeout(5);
    printInstructions();
    variableControl = 120;
    SpeedTest = MIN_SPEED;
}

/**
 * Imprime las instrucciones de uso
 */
void printInstructions() {
    Serial.println(F("=== Control de Robot Rocker-Bogie con DBH-12 ==="));
    Serial.println(F("Comandos disponibles:"));
    Serial.println(F("f: Adelante"));
    Serial.println(F("b: Atrás"));
    Serial.println(F("l: Giro izquierda"));
    Serial.println(F("r: Giro derecha"));
    Serial.println(F("s: Stop"));
    Serial.println(F("m: Monitoreo de corriente"));
    Serial.println(F("+: Aumentar velocidad"));
    Serial.println(F("-: Disminuir velocidad"));
    Serial.println(F("c: Configurar circuito rectangular"));
    Serial.println(F("h: Mostrar comandos"));
    Serial.println(F("======================================="));

    Serial1.println(F("=== Control de Robot Rocker-Bogie con DBH-12 ==="));
    Serial1.println(F("Comandos disponibles:"));
    Serial1.println(F("f: Adelante"));
    Serial1.println(F("b: Atrás"));
    Serial1.println(F("l: Giro izquierda"));
    Serial1.println(F("r: Giro derecha"));
    Serial1.println(F("s: Stop"));
    Serial1.println(F("m: Monitoreo de corriente"));
    Serial1.println(F("+: Aumentar velocidad"));
    Serial1.println(F("-: Disminuir velocidad"));
    Serial1.println(F("c: Configurar circuito rectangular"));
    Serial1.println(F("h: Mostrar comandos"));
    Serial1.println(F("======================================="));
}

/**
 * Bucle principal del programa
 */
void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        executeCommand(command);
    }
    
    // Serial Bluetooth
    if (Serial1.available() > 0) {
        char command = Serial1.read();  // Lee un solo carácter
        executeCommand(command);
    }

    if (isCircuitRunning) {
        executeCircuit();
    }
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
            Serial1.println(F("Movimiento: Adelante"));
            break;
            
        case 'b':
            robot.moveBackward(SpeedTest);
            Serial.println(F("Movimiento: Atrás"));
            Serial1.println(F("Movimiento: Atrás"));
            break;
            
        case 'l':
            robot.turnLeft(SpeedTest);
            Serial.println(F("Movimiento: Giro Izquierda"));
            Serial1.println(F("Movimiento: Giro Izquierda"));
            break;
            
        case 'r':
            robot.turnRight(SpeedTest);
            Serial.println(F("Movimiento: Giro Derecha"));
            Serial1.println(F("Movimiento: Giro Derecha"));
            break;
            
        case 's':
            Serial.print(F("DEBUG - Al detener - Estado actual: "));
            Serial.println(circuitState);
            robot.stop();
            if(isCircuitRunning) {
                totalRunTime = (millis() - circuitStartTime) / 1000;
                Serial.print(F("Tiempo total de ejecución: "));
                Serial.print(totalRunTime);
                Serial.println(F(" segundos"));
                Serial1.print(F("Tiempo total: "));
                Serial1.print(totalRunTime);
                Serial1.println(F(" seg"));
            }
            isCircuitRunning = false;
            Serial.println(F("Movimiento: Stop"));
            Serial1.println(F("Movimiento: Stop"));
            break;

        case 'c':
            requestCircuitTimes();
            break;
            
        case '+':
            robot.adjustSpeed(SPEED_STEP);
            SpeedTest = robot.getCurrentSpeed();
            Serial.print(F("Velocidad: "));
            Serial.println(SpeedTest);
            Serial1.print("Speed: ");
            Serial1.println(SpeedTest);
            break;
            
        case '-':
            robot.adjustSpeed(-SPEED_STEP);
            SpeedTest = robot.getCurrentSpeed();
            Serial.print(F("Velocidad: "));
            Serial.println(SpeedTest);
            Serial1.print(F("Velocidad: "));
            Serial1.println(SpeedTest);
            break;
            
        case 'h':
            printInstructions();
            break;
        case 'm':
            monitorCurrent();
            break;
        default:
            // Ignora comandos no reconocidos
            break;
    }
}

void requestCircuitTimes() {
    Serial.print(F("DEBUG - Antes de iniciar - Estado actual: "));
    Serial.println(circuitState);
    
    robot.stop();
    isCircuitRunning = false;
    
    Serial.println(F("Configuración de tiempos para circuito rectangular"));
    Serial1.println(F("Configuración de tiempos para circuito"));
    
    // Para movimiento hacia adelante
    Serial.println(F("Ingrese tiempo en segundos para movimiento hacia adelante:"));
    Serial1.println(F("Tiempo adelante (seg):"));
    while(true) {
        if(Serial.available() > 0) {
            forwardTime = Serial.parseInt() * 1000;
            break;
        }
        if(Serial1.available() > 0) {
            forwardTime = Serial1.parseInt() * 1000;
            break;
        }
    }
    
    // Limpiar buffer
    while(Serial.available()) Serial.read();
    while(Serial1.available()) Serial1.read();
    
    // Para giro
    Serial.println(F("Ingrese tiempo en segundos para giro:"));
    Serial1.println(F("Tiempo giro (seg):"));
    while(true) {
        if(Serial.available() > 0) {
            turnTime = Serial.parseInt() * 1000;
            break;
        }
        if(Serial1.available() > 0) {
            turnTime = Serial1.parseInt() * 1000;
            break;
        }
    }
    
    Serial.println(F("Iniciando circuito..."));
    Serial1.println(F("Iniciando circuito..."));   
    circuitState = 0;
    lastState = -1;  // Reiniciamos lastState
    lastStateChange = millis();
    circuitStartTime = millis();
    isCircuitRunning = true;
    
    Serial.print(F("DEBUG - Después de iniciar - Estado actual: "));
    Serial.println(circuitState);
}

void executeCircuit() {
    if (!isCircuitRunning) return;
    
    unsigned long currentTime = millis();
    unsigned long stateTime = currentTime - lastStateChange;
    
    // Si el estado cambió, ejecutar la acción correspondiente
    if (lastState != circuitState) {
        Serial.print(F("DEBUG - Cambio de estado - Anterior: "));
        Serial.print(lastState);
        Serial.print(F(" Nuevo: "));
        Serial.println(circuitState);
        
        switch(circuitState) {
            case 0: // Adelante
                robot.moveForward(SpeedTest);
                Serial.println(F("Circuito: Adelante"));
                Serial1.println(F("Circuito: Adelante"));
                break;
                
            case 1: // Giro derecha
                robot.turnRight(SpeedTest);
                Serial.println(F("Circuito: Giro Derecha"));
                Serial1.println(F("Circuito: Giro Derecha"));
                break;
                
            case 2: // Adelante
                robot.moveForward(SpeedTest);
                Serial.println(F("Circuito: Adelante"));
                Serial1.println(F("Circuito: Adelante"));
                break;
                
            case 3: // Giro derecha
                robot.turnRight(SpeedTest);
                Serial.println(F("Circuito: Giro Derecha"));
                Serial1.println(F("Circuito: Giro Derecha"));
                break;
        }
        lastState = circuitState;
    }
    
    // Verificar tiempo para cambio de estado
    if ((circuitState == 0 || circuitState == 2) && stateTime >= forwardTime) {
        robot.stop();
        lastStateChange = currentTime;
        circuitState = (circuitState + 1) % 4;
    }
    else if ((circuitState == 1 || circuitState == 3) && stateTime >= turnTime) {
        robot.stop();
        lastStateChange = currentTime;
        circuitState = (circuitState + 1) % 4;
    }
}

/*
 * Función para leer el voltaje y la corriente de un pin CT
 * @param pinCT Pin analógico conectado al CT
 * @param voltage Puntero para almacenar el voltaje leído
 * @param current Puntero para almacenar la corriente calculada
 */
void readCT(int pinCT, float* voltage, float* current) {
    int analogValue = analogRead(pinCT);              // Leer el valor analógico (0-1023)
    *voltage = (analogValue / 1023.0) * 5.0;         // Convertir a voltaje (0-5V)
    *current = *voltage / CT_CONVERSION_FACTOR;      // Calcular la corriente (A)
}

/*
 * Función para mostrar los valores de voltaje y corriente de todos los drivers
 */
void monitorCurrent() {
    float voltageLeft1, currentLeft1, voltageRight1, currentRight1;
    float voltageLeft2, currentLeft2, voltageRight2, currentRight2;
    float voltageLeft3, currentLeft3, voltageRight3, currentRight3;

    // Leer los valores de los pines CT
    readCT(CT_Left_1, &voltageLeft1, &currentLeft1);
    readCT(CT_Right_1, &voltageRight1, &currentRight1);
    readCT(CT_Left_2, &voltageLeft2, &currentLeft2);
    readCT(CT_Right_2, &voltageRight2, &currentRight2);
    readCT(CT_Left_3, &voltageLeft3, &currentLeft3);
    readCT(CT_Right_3, &voltageRight3, &currentRight3);

    // Calcular la suma total de las corrientes
    float totalCurrent = currentLeft1 + currentRight1 + currentLeft2 + currentRight2 + currentLeft3 + currentRight3;

    // Mostrar los valores en el monitor serial
    Serial.println(F("=== Monitoreo de Corriente ==="));
    Serial.println(F("Drive 1:"));
    Serial.print(F("  CT_Left_1: Corriente = ")); Serial.print(currentLeft1); Serial.println(F(" A"));
    Serial.print(F("  CT_Right_1: Corriente = ")); Serial.print(currentRight1); Serial.println(F(" A"));
    Serial.println(F("Drive 2:"));
    Serial.print(F("  CT_Left_2: Corriente = ")); Serial.print(currentLeft2); Serial.println(F(" A"));
    Serial.print(F("  CT_Right_2: Corriente = ")); Serial.print(currentRight2); Serial.println(F(" A"));
    Serial.println(F("Drive 3:"));
    Serial.print(F("  CT_Left_3: Corriente = ")); Serial.print(currentLeft3); Serial.println(F(" A"));
    Serial.print(F("  CT_Right_3: Corriente = ")); Serial.print(currentRight3); Serial.println(F(" A"));
    Serial.print(F("Corriente Total: ")); Serial.print(totalCurrent); Serial.println(F(" A"));
    Serial.println(F("============================="));

    // Mostrar los valores en Serial1 (Bluetooth)
    Serial1.println(F("=== Monitoreo de Corriente ==="));
    Serial1.println(F("Drive 1:"));
    Serial1.print(F("  CT_Left_1: Corriente = ")); Serial1.print(currentLeft1); Serial1.println(F(" A"));
    Serial1.print(F("  CT_Right_1: Corriente = ")); Serial1.print(currentRight1); Serial1.println(F(" A"));
    Serial1.println(F("Drive 2:"));
    Serial1.print(F("  CT_Left_2: Corriente = ")); Serial1.print(currentLeft2); Serial1.println(F(" A"));
    Serial1.print(F("  CT_Right_2: Corriente = ")); Serial1.print(currentRight2); Serial1.println(F(" A"));
    Serial1.println(F("Drive 3:"));
    Serial1.print(F("  CT_Left_3: Corriente = ")); Serial1.print(currentLeft3); Serial1.println(F(" A"));
    Serial1.print(F("  CT_Right_3: Corriente = ")); Serial1.print(currentRight3); Serial1.println(F(" A"));
    Serial1.print(F("Corriente Total: ")); Serial1.print(totalCurrent); Serial1.println(F(" A"));
    Serial1.println(F("============================="));
}


// Las funciones de test están comentadas, pero se podrían implementar si son necesarias
// Código para pruebas con motores tipo gusano y DBH-12
/*
void testMotores() {
    Serial.println(F("\n=== PRUEBAS CON MOTORES TIPO GUSANO Y DBH-12 ==="));
    
    // Test 1: Motores individuales
    Serial.println(F("\n-- Test 1: Motores Individuales --"));
    Motor* motores[] = {&motorL_1, &motorR_1, &motorL_2, &motorR_2, &motorL_3, &motorR_3};
    String nombresMotores[] = {"FL", "FR", "ML", "MR", "RL", "RR"};
    
    for(int i = 0; i < 6; i++) {
        Serial.print(F("\nPrueba Motor "));
        Serial.println(nombresMotores[i]);
        
        // Prueba a diferentes velocidades
        int velocidades[] = {120, 180, 250};
        for(int vel : velocidades) {
            Serial.print(F("Velocidad PWM: "));
            Serial.println(vel);
            motores[i]->setSpeed(vel);
            motores[i]->forward();
            delay(5000);  // Tiempo para tomar medidas
            motores[i]->stop();
            delay(2000);
        }
    }
    
    // Test 2: Prueba por lados
    Serial.println(F("\n-- Test 2: Prueba por Lados --"));
    delay(5000);  // Tiempo para preparar medición
    
    // Izquierda
    Serial.println(F("Motores Izquierdos"));
    motorL_1.setSpeed(180);
    motorL_2.setSpeed(180);
    motorL_3.setSpeed(180);
    motorL_1.forward();
    motorL_2.forward();
    motorL_3.forward();
    delay(5000);
    motorL_1.stop();
    motorL_2.stop();
    motorL_3.stop();
    
    delay(3000);
    
    // Derecha
    Serial.println(F("Motores Derechos"));
    motorR_1.setSpeed(180);
    motorR_2.setSpeed(180);
    motorR_3.setSpeed(180);
    motorR_1.forward();
    motorR_2.forward();
    motorR_3.forward();
    delay(5000);
    motorR_1.stop();
    motorR_2.stop();
    motorR_3.stop();
    
    // Test 3: Todos los motores
    Serial.println(F("\n-- Test 3: Todos los Motores --"));
    // Array con valores PWM a probar
    int pwmValues[] = {120, 180, 250};
    for(int pwm : pwmValues) {
        Serial.print(F("Prueba con PWM: "));
        Serial.println(pwm);
        delay(3000);  // Tiempo para preparar medición
        
        robot.moveForward(pwm);
        delay(10000); // Tiempo para mediciones
        robot.stop();
        
        Serial.println(F("Pausa entre pruebas"));
        delay(5000);  // Pausa entre pruebas
    }
    
    Serial.println(F("\n=== Fin de pruebas con motores tipo gusano y DBH-12 ==="));
}
*/
