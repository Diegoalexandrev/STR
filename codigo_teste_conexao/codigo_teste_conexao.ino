#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definição dos pinos dos LEDs
#define LED_GREEN 25
#define LED_RED 26
#define LED_BLUE 27

// Configuração do DHT11
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Defina as dimensões do display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Endereço I2C do display SSD1306
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variáveis globais para temperatura e umidade
float temperature = 0;
float humidity = 0;

// Handles das tasks
TaskHandle_t TaskDHTHandle;
TaskHandle_t TaskOLEDHandle;
TaskHandle_t TaskWiFiHandle;
TaskHandle_t TaskBlinkGreenHandle;
TaskHandle_t TaskBlinkRedHandle;
TaskHandle_t TaskBlinkBlueHandle;

// Credenciais do Wi-Fi
const char* ssid = "Qnn18l16";
const char* password = "Qnn18CcL16";

// Variável para status do Wi-Fi
bool wifiConnected = false;

void setup() {
  Serial.begin(115200);

  // Configuração dos pinos dos LEDs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Inicializa o display SSD1306
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha na inicialização do SSD1306"));
    for (;;);
  }
  display.display();
  delay(2000);

  // Inicializa o sensor DHT11
  dht.begin();

  // Cria a task para ler o sensor DHT11
  xTaskCreatePinnedToCore(
      TaskDHT,          // Função da task
      "TaskDHT",        // Nome da task
      10000,            // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskDHTHandle,   // Handle da task
      0);               // Core 0

  // Cria a task para atualizar o display OLED
  xTaskCreatePinnedToCore(
      TaskOLED,         // Função da task
      "TaskOLED",       // Nome da task
      10000,            // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskOLEDHandle,  // Handle da task
      1);               // Core 1

  // Cria a task para gerenciar o Wi-Fi
  xTaskCreatePinnedToCore(
      TaskWiFi,         // Função da task
      "TaskWiFi",       // Nome da task
      10000,            // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskWiFiHandle,  // Handle da task
      1);               // Core 1

  // Cria as tasks para piscar os LEDs
  xTaskCreatePinnedToCore(
      TaskBlinkGreen,   // Função da task para piscar o LED verde
      "TaskBlinkGreen", // Nome da task
      1000,             // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskBlinkGreenHandle, // Handle da task
      0);               // Core 0

  xTaskCreatePinnedToCore(
      TaskBlinkRed,     // Função da task para piscar o LED vermelho
      "TaskBlinkRed",   // Nome da task
      1000,             // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskBlinkRedHandle, // Handle da task
      1);               // Core 1

  xTaskCreatePinnedToCore(
      TaskBlinkBlue,    // Função da task para piscar o LED azul
      "TaskBlinkBlue",  // Nome da task
      1000,             // Tamanho da pilha
      NULL,             // Parâmetro passado (não usado)
      1,                // Prioridade da task
      &TaskBlinkBlueHandle, // Handle da task
      1);               // Core 1
}

void TaskDHT(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Leitura do sensor DHT11
    float newTemperature = dht.readTemperature();
    float newHumidity = dht.readHumidity();

    // Verifica se as leituras são válidas
    if (!isnan(newTemperature) && !isnan(newHumidity)) {
      temperature = newTemperature;
      humidity = newHumidity;
    } else {
      Serial.println(F("Falha ao ler o sensor DHT"));
    }

    // Pausa de 2 segundos
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskOLED(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Limpa o display
    display.clearDisplay();

    // Exibe a temperatura e umidade no display
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Temp: ");
    display.print(temperature);
    display.println(" C");

    display.print("Umidade: ");
    display.print(humidity);
    display.println(" %");

    // Exibe o status do Wi-Fi
    display.print("WiFi: ");
    display.println(wifiConnected ? "Conectado" : "Desconectado");

    display.display();  // Atualiza o display

    // Pausa de 2 segundos
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskWiFi(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Conecta ao Wi-Fi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Conectando-se ao Wi-Fi: ");
      Serial.println(ssid);
      WiFi.begin(ssid, password);

      int attemptCount = 0;
      while (WiFi.status() != WL_CONNECTED && attemptCount < 10) {
        delay(1000);
        Serial.print(".");
        attemptCount++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConectado ao Wi-Fi!");
        wifiConnected = true;
      } else {
        Serial.println("\nFalha ao conectar ao Wi-Fi");
        wifiConnected = false;
      }
    }

    // Aguarda 10 segundos antes de tentar reconectar se desconectado
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// Task para piscar o LED verde
void TaskBlinkGreen(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_GREEN, HIGH);  // Acende o LED verde
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 500 ms
    digitalWrite(LED_GREEN, LOW);   // Apaga o LED verde
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 500 ms
  }
}

// Task para piscar o LED vermelho
void TaskBlinkRed(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_RED, HIGH);  // Acende o LED vermelho
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 1 segundo
    digitalWrite(LED_RED, LOW);   // Apaga o LED vermelho
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 1 segundo
  }
}

// Task para piscar o LED azul
void TaskBlinkBlue(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_BLUE, HIGH);  // Acende o LED azul
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 1,5 segundos
    digitalWrite(LED_BLUE, LOW);   // Apaga o LED azul
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pausa de 1,5 segundos
  }
}

void loop() {
  // O loop fica vazio porque todas as ações estão nas tasks do FreeRTOS
}
