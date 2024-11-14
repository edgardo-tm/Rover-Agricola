// Configuración de pines
const int coinPin = 19;  // Pin GPIO donde se conecta el pin COIN del aceptador
volatile int pulseCount = 0;  // Variable para contar los pulsos
int totalAmount = 0;  // Monto total acumulado

void IRAM_ATTR countPulse() {
  pulseCount++;  // Incrementa el contador de pulsos cada vez que se detecta una interrupción
  Serial.println("Pulso detectado!"); 
}

void setup() {
  // Inicializa el Serial Monitor
  Serial.begin(115200);
  
  // Configura el pin del aceptador de monedas como entrada
  pinMode(coinPin, INPUT_PULLUP);

  // Configura la interrupción para detectar los pulsos
  attachInterrupt(digitalPinToInterrupt(coinPin), countPulse, FALLING);

  // Inicializa el contador y el monto total
  pulseCount = 0;
  totalAmount = 0;
}

void loop() {
  //Serial.println("Esperando moneda...");
  // Si hay pulsos registrados
  if (pulseCount > 0) {
    delay(500);  // Espera un poco para asegurarse de que se registren todos los pulsos

    // Identifica la moneda en función del número de pulsos
    if (pulseCount == 10) {
      Serial.println("Moneda de 10 pesos detectada.");
      totalAmount += 10;
    } else if (pulseCount == 5) {
      Serial.println("Moneda de 5 pesos detectada.");
      totalAmount += 5;
    } else if (pulseCount == 2) {
      Serial.println("Moneda de 2 pesos detectada.");
      totalAmount += 2;
    } else if (pulseCount == 1) {
      Serial.println("Moneda de 1 peso detectada.");
      totalAmount += 1;
    } else {
      Serial.println("Moneda no reconocida.");
    }

    // Muestra el monto total
    Serial.print("Monto total: ");
    Serial.print(totalAmount);
    Serial.println(" pesos.");

    // Reinicia el contador de pulsos para la próxima moneda
    pulseCount = 0;
  }
}

