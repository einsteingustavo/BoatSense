#include <Arduino.h>
#include <Wire.h>
#include <HX711_ADC.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_tockn.h>

#define SHT_LOX1 27
#define SHT_LOX2 26
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// -------------------------------------- TASK HANDLERS --------------------------------------
TaskHandle_t TaskHandle_loadCell;
TaskHandle_t TaskHandle_Positioning;
// -------------------------------------- TASK HANDLERS --------------------------------------

// -------------------------------------- INSTANCES --------------------------------------
// HX711 constructor (dout, sck)
HX711_ADC LoadCell(25, 33);

// GY-521 constructor (sda = 21, SCL = 22)
MPU6050 mpu = MPU6050(Wire);

// Laser sensor constructor (sda = 21, SCL = 22)
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
// -------------------------------------- INSTANCES --------------------------------------

// -------------------------------------- VARIABLES --------------------------------------
float roll = 0, pitch = 0, load = 0;
uint16_t distance1 = 0, distance2 = 0;
long t;
// -------------------------------------- VARIABLES --------------------------------------

// -------------------------------------- FUNCTIONS DECLARATIONS --------------------------------------
void setID();
void Task_loadCell(void *pvParameters);
void Task_Positioning(void *pvParameters);
// -------------------------------------- FUNCTIONS DECLARATIONS --------------------------------------

void setup()
{
  // -------------------------------------- PROTOCOLS SETUP --------------------------------------
  Wire.begin();
  Serial.begin(115200);
  // -------------------------------------- PROTOCOLS SETUP --------------------------------------

  xTaskCreatePinnedToCore(
      Task_loadCell,        /* Função responsável pelos calculos da celula de carga */
      "Task_loadCell",      /* nome da tarefa */
      10000,                /* memória alocada para a pilha da tarefa */
      NULL,                 /* parâmetro de entrada para a tarefa (pode ser NULL) */
      1,                    /* Nível de prioridade da tarefa (0 a N) */
      &TaskHandle_loadCell, /* referência para a tarefa (pode ser NULL) */
      0                     /* Núcleo que executará a tarefa */
  );

  xTaskCreatePinnedToCore(
      Task_Positioning,        /* Função responsável pelos calculos de posicionamento */
      "Task_Positioning",      /* nome da tarefa */
      10000,                   /* memória alocada para a pilha da tarefa */
      NULL,                    /* parâmetro de entrada para a tarefa (pode ser NULL) */
      1,                       /* Nível de prioridade da tarefa (0 a N) */
      &TaskHandle_Positioning, /* referência para a tarefa (pode ser NULL) */
      1                        /* Núcleo que executará a tarefa */
  );
}

void loop()
{
}

// -------------------------------------- FUNCTIONS IMPLEMENTATIONS --------------------------------------

void Task_loadCell(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(10000));
  // -------------------------------------- LOAD CELL SETUP --------------------------------------
  LoadCell.begin();
  long stabilisingtime = 5000; // tare preciscion can be improved by adding a few seconds of stabilising time
  Serial.println("Wait...");
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(1700.00); // user set calibration factor (float)
  Serial.println("Startup + tare is complete");
  // -------------------------------------- LOAD CELL SETUP --------------------------------------

  for (;;)
  {
    // -------------------------------------- LOAD CELL LOOP --------------------------------------
    // // update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
    // // longer delay in scetch will reduce effective sample rate (be carefull with delay() in loop)
    LoadCell.update();

    // get smoothed value from data set + current calibration factor
    if (millis() > t + 250)
    {
      load = LoadCell.getData();
      t = millis();
    }
    // -------------------------------------- LOAD CELL LOOP --------------------------------------
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void Task_Positioning(void *pvParameters)
{
  // -------------------------------------- MPU SETUP --------------------------------------
  mpu.begin();
  mpu.calcGyroOffsets(true);
  // -------------------------------------- MPU SETUP --------------------------------------

  // -------------------------------------- LASERS SETUP --------------------------------------
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println("Both in reset mode...(pins are low)");
  Serial.println("Starting...");

  setID();
  // -------------------------------------- LASERS SETUP --------------------------------------

  for (;;)
  {
    // -------------------------------------- MPU LOOP --------------------------------------
    // Read MPU-6050 data
    mpu.update();
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    // // Calculate roll and pitch
    roll = atan2(ay, az) * 180.0 / PI;
    pitch = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / PI;
    // -------------------------------------- MPU LOOP --------------------------------------

    // -------------------------------------- LASERS LOOP --------------------------------------
    // Read VL53L0X sensor data
    distance1 = sensor1.readRange();
    distance2 = sensor2.readRange();
    // -------------------------------------- LASERS LOOP --------------------------------------
    // -------------------------------------- RESULTS --------------------------------------
    Serial.print("Load_cell output: ");
    Serial.println(load);
    Serial.println();
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("\tPitch: ");
    Serial.println(pitch);
    Serial.println();
    Serial.print("Distance 1: ");
    Serial.print(distance1);
    Serial.print("\tDistance 2: ");
    Serial.println(distance2);
    Serial.println();
    // -------------------------------------- RESULTS --------------------------------------
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void setID()
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  // initing LOX1
  if (!sensor1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  // activating LOX2 and reseting LOX1
  digitalWrite(SHT_LOX2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  // initing LOX2
  if (!sensor2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}
// -------------------------------------- FUNCTIONS IMPLEMENTATIONS --------------------------------------
