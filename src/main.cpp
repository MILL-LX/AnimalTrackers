#include <TinyGPSPlus.h>
#include <DFRobot_DF1201S.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

#include <Adafruit_NeoPixel.h>

// Task MuscleWire
// TaskHandle_t Task_muscleWire;

// int shrinkMuscleWire = 0;
// int timesOfShrink = 1;

// void Task_muscleWire1_code(void *pvParameters);

#define speciesLED_pin D1
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, speciesLED_pin, NEO_GRB + NEO_KHZ800);

#define PIN_MUSCLE_WIRE D2
#define PIN_BUTTON D3

// #define OLED_RESET 3
// Adafruit_SSD1306 display(OLED_RESET);

static const int RXPin_gnss = D7, TXPin_gnss = D6;
static const uint32_t GPSBaud_gnss = 9600;

static const int RXPin_mp3 = 8, TXPin_mp3 = 9;
static const uint32_t GPSBaud_mp3 = 115200;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GNSS module
SoftwareSerial ss(RXPin_gnss, TXPin_gnss);

// The serial connection to the MP3 module
HardwareSerial DF1201SSerial(0);
DFRobot_DF1201S DF1201S;

// Coordinates
struct CoordinatesBirds
{
  float longitude;
  float latitude;
  int species;
};

int numCoordinates;
CoordinatesBirds *coordinates;

int shrunkTimes = 0;
bool shrunkState = false;
unsigned long startShrunkTime = 0;
bool UnshrunkState = false;
unsigned long startUnShrunkTime = 0;

// Earth radius in meters
const float R = 6371000;

// Functions headers
void displayInfo();
void saveCurrentLocation();
float haversineDistance(float lat1, float lon1, float lat2, float lon2);

CoordinatesBirds *loadCoordinatesFiles(int &numCoordinates);

void setup()
{
  Serial.begin(9600);

  pixels.begin();

  pixels.setBrightness(35);
  pixels.setPixelColor(0, pixels.Color(125, 125, 125));
  Serial.println("WHITE PIXEL!");

  pixels.show();

  ss.begin(GPSBaud_gnss);
  DF1201SSerial.begin(GPSBaud_mp3, SERIAL_8N1, RXPin_mp3, TXPin_mp3);

  while (!DF1201S.begin(DF1201SSerial))
  {
    Serial.println("Init failed, please check the wire connection!");
    delay(1000);
  }

  DF1201S.setVol(/*VOL = */ 40);
  DF1201S.switchFunction(DF1201S.MUSIC);
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);
  DF1201S.setPrompt(false);

  // display.begin(SSD1306_SWITCHCAPVCC, 0x3D);

  Serial.println("GPS device started!");

  // int numCoordinates;
  // CoordinatesBirds *coordinates = loadCoordinatesFiles(numCoordinates);
  coordinates = loadCoordinatesFiles(numCoordinates);

  pinMode(PIN_MUSCLE_WIRE, OUTPUT);
  digitalWrite(PIN_MUSCLE_WIRE, LOW);
  pinMode(PIN_BUTTON, INPUT);

  pixels.begin();

  pixels.setBrightness(55);
  pixels.setPixelColor(0, pixels.Color(125, 125, 125));
  Serial.println("WHITE PIXEL!");

  pixels.show();

  delay(2000);

  // Serial.println("PLAYING SOUND! Purple Heron!");
  // DF1201S.playSpecFile("/001.mp3");
  // DF1201S.start();

  // delay(5000);

  // Serial.println("PLAYING SOUND! Otter!");
  // DF1201S.playSpecFile("/002.mp3");
  // DF1201S.start();

  // delay(5000);

  // Serial.println("PLAYING SOUND! Crickets!");
  // DF1201S.playSpecFile("/003.mp3");
  // DF1201S.start();

  // // Multi-core
  // xTaskCreatePinnedToCore(
  //     Task_muscleWire1_code, /* Function to implement the task */
  //     "Task1",               /* Name of the task */
  //     10000,                 /* Stack size in words */
  //     &shrinkMuscleWire,     /* Task input parameter */
  //     0,                     /* Priority of the task */
  //     &Task_muscleWire,      /* Task handle. */
  //     0);                    /* Core where the task should run */
}

double currentLat = 0;
double currentLon = 0;

// SAMOUCO HOME 38°43'12"N 9°00'21"W
// double prev_Lat = 38.72;
// double prev_Lon = -9.0058;

bool insideZone = false;
int lastSpecies = 0;

void loop()
{
  // GPS data
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      // displayInfo();
      saveCurrentLocation();

      // display.clearDisplay();
      // display.setTextSize(1);
      // display.setTextColor(SSD1306_WHITE);

      // display.setCursor(0, 0);
      // display.println(F("Coordinates: "));
      // display.println(currentLat, 7);
      // display.println(currentLon, 7);

      // display.display();

      if (currentLat != 0 && currentLon != 0)
      {
        for (int i = 0; i < numCoordinates; i++)
        {
          float distance = haversineDistance(coordinates[i].latitude, coordinates[i].longitude, currentLat, currentLon);

          if (distance < 50)
          {
            if (!insideZone)
            {
              // MUSCLE WIRE
              Serial.println("MUSCLE WIRE! INSIDE ZONE!");
              shrunkState = true;
              shrunkTimes = 2;
              startShrunkTime = millis();
              digitalWrite(PIN_MUSCLE_WIRE, HIGH);
            }
            insideZone = true;

            lastSpecies = coordinates[i].species;

            switch (lastSpecies)
            {
            case 1:
              Serial.println("Mole Cricket!");
              pixels.setPixelColor(0, pixels.Color(0, 255, 0));
              break;
            case 2:
              Serial.println("Purple Heron!");
              pixels.setPixelColor(0, pixels.Color(160, 32, 240));
              break;
            case 3:
              Serial.println("River Otter!");
              pixels.setPixelColor(0, pixels.Color(0, 0, 255));
              break;
            default:
              break;
            }

            Serial.println(DF1201S.getFileName());

            pixels.show();
            break;
          }

          // OUTSIDE OF ZONE
          if ((i == numCoordinates - 1) && insideZone)
          {
            insideZone = false;
            lastSpecies = 0;
            pixels.clear();
            pixels.show();

            // MUSCLE WIRE
            Serial.println("MUSCLE WIRE! OUTSIDE ZONE!");
            shrunkState = true;
            shrunkTimes = 1;
            startShrunkTime = millis();
            digitalWrite(PIN_MUSCLE_WIRE, HIGH);
          }
        }
      }
    }
  }

  // Serial.println(shrinkMuscleWire);

  // Button behavior
  // PLAYS THE SOUND RELATED TO THE SPECIES
  if (digitalRead(PIN_BUTTON) == HIGH)
  {
    Serial.println("BUTTON PRESSED!");

    switch (lastSpecies)
    {
    case 1:
      Serial.println("PLAYING SOUND! Mole Cricket!");
      DF1201S.playSpecFile("/001.mp3");
      DF1201S.start();
      break;
    case 2:
      Serial.println("PLAYING SOUND! Purple Heron!");
      DF1201S.playSpecFile("/002.mp3");
      DF1201S.start();
      break;
    case 3:
      Serial.println("PLAYING SOUND! River Otter!");
      DF1201S.playSpecFile("/003.mp3");
      DF1201S.start();
      break;
    default:
      break;
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }

  if (millis() - startShrunkTime > 8000 && shrunkState)
  {
    shrunkState = false;
    digitalWrite(PIN_MUSCLE_WIRE, LOW);

    if (shrunkTimes == 2)
    {
      UnshrunkState = true;
      startUnShrunkTime = millis();
      shrunkTimes = 0;
    }
  }

  if (millis() - startUnShrunkTime > 5000 && UnshrunkState)
  {
    UnshrunkState = false;
    digitalWrite(PIN_MUSCLE_WIRE, HIGH);
    shrunkState = true;
    startShrunkTime = millis();
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.println(gps.location.lat(), 6);
    Serial.println(F(","));
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void saveCurrentLocation()
{
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();

    // Serial.print(F("Location: "));
    // Serial.print(currentLat, 6);
    // Serial.print(F(","));
    // Serial.println(currentLon, 6);

    // display.print(currentLat, 6);
    // display.print(F(","));
    // display.print(currentLon, 6);
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    pixels.show();
  }
}

float toRadians(float degree)
{
  return degree * (PI / 180);
}

float haversineDistance(float lat1, float lon1, float lat2, float lon2)
{
  // Convert latitude and longitude from degrees to radians
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  // Difference in coordinates
  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;

  // Haversine formula
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) *
                sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Distance in meters
  float distance = R * c;

  return distance;
}

// void Task_muscleWire1_code(void *pvParameters)
// {
//   Serial.println("Task_muscleWire_code running on core ");
//   Serial.println(xPortGetCoreID());

//   for (;;)
//   {
//     if (pvParameters > 0)
//     {
//       pvParameters = pvParameters - 1;
//       Serial.println("Start task!");
//       pixels.setPixelColor(0, pixels.Color(0, 0, 0));
//       pixels.show();
//       delay(8000);
//       pixels.setPixelColor(0, pixels.Color(255, 255, 255));
//       pixels.show();
//       delay(5000);
//       Serial.println("End task!");

//       pvParameters = pvParameters - 1;
//     }
//   }
// }

CoordinatesBirds *
loadCoordinatesFiles(int &numCoordinates)
{
  /**
   * Coordinates of the birds
   * Especies:
   * 1 - mole_cricket
   * 2 - Purple Heron
   * 3 - River otter
   */
  static CoordinatesBirds coordinates[] = {
      {-9.1658779979, 38.6631713197, 1},
      {-9.1659135371, 38.6631122686, 1},
      {-9.1692066193, 38.9959768782, 1},
      {-9.3311733194, 38.727139947, 1},
      {-9.3203191624, 38.7027131495, 1},
      {-9.1030329324, 38.9171898498, 1},
      {-8.8474855423, 38.8634567261, 1},
      {-9.4294956085, 38.8762395994, 1},
      {-9.0513426438, 38.6432073722, 1},
      {-9.44779667, 38.74188667, 1},
      {-9.4083513549, 38.8239542831, 1},
      {-9.43066, 38.703279, 1},
      {-9.1744031, 38.7608476, 1},
      {-9.4256679722, 38.705421, 1},
      {-9.4308088347, 38.8696385112, 1},
      {-9.4258449388, 38.7430314875, 1},
      {-9.104275, 38.6486783333, 1},
      {-9.462902007, 38.836381796, 1},
      {-8.9740430564, 38.849641503, 1},
      {-8.9198654264, 38.9293017674, 1},
      {-9.491848148, 38.7890424968, 1},
      {-9.1155917761, 38.6496771607, 1},
      {-9.3973633647, 38.819936505, 1},
      {-9.0951072425, 38.7825635812, 1},
      {-9.29698412, 38.81370358, 1},
      {-9.1817792719, 38.7541958739, 1},
      {-9.4155795034, 38.8098933455, 1},
      {-9.1671624277, 38.6649057702, 1},
      {-9.0922413021, 38.7826631582, 1},
      {-9.1571453959, 38.6470425786, 1},
      {-9.193326, 38.747775, 1},
      {-9.0103189275, 38.9764854481, 1},
      {-9.46648, 38.8279416667, 1},
      {-9.4487571716, 38.7516150347, 1},
      {-9.3620878575, 38.7089753322, 1},
      {-9.1541042164, 38.7710406726, 1},
      {-9.2162232585, 38.7010999682, 1},
      {-9.1951652722, 38.7209782892, 1},
      {-9.1608763486, 38.9060763194, 1},
      {-9.1941583659, 38.6455122714, 1},
      {-9.0512474, 38.6431989, 1},
      {-9.426287, 38.717653, 1},
      {-9.3043973297, 38.7290060035, 1},
      {-9.4453705414, 38.7274492395, 1},
      {-8.9529309, 39.0051259997, 1},
      {-9.4436271116, 38.7266969716, 1},
      {-9.4547673, 38.7465215, 1},
      {-9.0617040172, 38.8700921935, 1},
      {-9.194064, 38.747989, 1},
      {-9.3836035025, 38.8027175522, 1},
      {-9.200655, 38.6584166667, 1},
      {-9.3605764002, 38.9466337282, 1},
      {-9.2418045154, 38.9227359236, 1},
      {-9.052405, 38.64295, 1},
      {-9.1868874058, 38.7123747566, 1},
      {-9.3168226564, 38.8891812056, 1},
      {-9.2562168973, 38.9176770677, 1},
      {-9.1771305556, 38.9026194444, 1},
      {-9.2047911503, 38.8790887236, 1},
      {-9.3472050429, 38.9699733937, 1},
      {-9.1939638889, 38.6438583333, 1},
      {-9.226635, 38.815991, 1},
      {-9.0088316426, 38.9269304444, 1},
      {-9.1550257802, 38.6824131084, 1},
      {-9.140070198715136, 38.720580053770135, 1}, // "ROTUNDA"
      {-9.139884892969292, 38.722155150016846, 2}, // "FINAL DO PARQUE"
      {-9.141328880020183, 38.720367321881014, 3}, // MILL
      {-9.3519702147, 39.0099694614, 3},
      {-8.9790615848, 39.0118509605, 3},
      {-8.9152397469, 38.8174192937, 3},
      {-8.9506898496, 38.682677512, 3},
      {-8.9873556806, 38.9523385702, 3},
      {-9.0368390449, 38.8834354179, 3},
      {-8.9710458, 38.9327331, 3},
      {-9.0365547614, 38.8834992343, 3},
      {-9.3384057281, 38.8812239355, 3},
      {-9.2114589217, 38.8132340791, 3},
      {-9.313728789, 38.8772713307, 3},
      {-8.8992835209, 38.7265727296, 3},
      {-8.88014, 38.7085516667, 3},
      {-9.3905855402, 38.9474222457, 3},
      {-8.8802283333, 38.6525733333, 3},
      {-8.8808639794, 38.6518337942, 3},
      {-9.3678283689, 38.9034385681, 3},
      {-8.947615, 38.8323016667, 3},
      {-8.9742508591, 38.8435015508, 3},
      {-8.883197416866324, 38.70583145413107, 3},
      {-8.921297288484311, 38.75619824440719, 3},
      {-8.968287852556102, 38.83891158421909, 3},
      {-8.970964446124157, 38.839308478695784, 3},
      {-8.945327104920551, 38.91458569871941, 3},
      {-8.921, 38.749138888, 2},
      {-8.9469909668, 38.8899639292, 2},
      {-8.9809824149, 38.7437065052, 2},
      {-9.2406090115, 38.6710177005, 2},
      {-8.9537305556, 38.8633555556, 2},
      {-8.9740431, 38.8496358, 2},
      {-8.9739769974, 38.8467834762, 2},
      {-8.9699436165, 38.9423672729, 2},
      {-9.0511572361, 38.8625595945, 2},
      {-8.9664204146, 38.9045462408, 2},
      {-8.96267, 38.9020183333, 2},
      {-8.9635371383, 38.9232748426, 2},
      {-9.4107571973, 38.7484661603, 2},
      {-8.9609831494, 38.9041130056, 2},
      {-8.9739530095, 38.8462629123, 2},
      {-8.9689343535, 38.8761922259, 2},
      {-8.9740431, 38.8496358, 2},
      {-8.9957147158, 38.6854175889, 2},
      {-8.9529687, 39.0068029, 2},
      {-9.1383462633, 38.7424463724, 2},
      {-8.9676116667, 38.8657, 2},
      {-8.9743900066, 38.8463921261, 2},
      {-8.9693979719, 38.9418577525, 2},
      {-9.1181705322, 38.8402659655, 2},
      {-9.0397427, 38.9022048, 2},
      {-9.1502067517, 38.8516963179, 2},
      {-8.9573089999, 38.8715196926, 2},
      {-9.240648216, 38.6715390759, 2},
      {-8.9559416667, 38.8710466667, 2},
      {-8.8801695675, 38.6520515394, 2},
      {-8.9669536151, 38.9393811817, 2},
      {-8.9661550119, 38.8790284699, 2},
      {-8.9700435199, 38.941784504, 2},
      {-8.976909975, 38.9409834056, 2},
      {-8.9697001972, 38.9441877448, 2},
      {-8.8960750356, 38.73323849, 2},
      {-8.9779399433, 38.9473919391, 2},
      {-9.0477059237, 38.8637416438, 2},
      {-8.8926173, 38.7340864, 2},
      {-8.8945833333, 38.7326333333, 2},
      {-8.9020783333, 38.7298983333, 2},
      {-8.9742045088, 38.9443119243, 2},
      {-8.976909975, 38.9449888069, 2},
      {-8.9738200702, 38.9420515347, 2},
      {-8.8926173, 38.7340864, 2},
      {-8.9736841577, 38.8471729424, 2},
      {-8.9720198137, 38.8487229478, 2},
      {-8.9596168415, 38.8902466253, 2},
      {-8.8776633333, 38.703685, 2},
      {-8.9596168415, 38.8902466253, 2},
      {-8.9414233799, 38.9068111543, 2},
      {-8.9521946278, 38.8725661325, 2},
      {-8.9669052783, 38.8343741667, 2},
      {-8.9385533333, 38.889235, 2},
      {-8.9553933333, 38.8608541667, 2},
      {-8.9751933612, 38.9393811817, 2},
      {-8.9673016667, 38.87606, 2},
      {-8.9604533091, 38.8725962698, 2},
      {-8.9717601337, 38.9441580747, 2},
      {-8.974163393, 38.9430899773, 2},
      {-8.8935119195, 38.7324364575, 2},
      {-9.1551284789, 38.8510932922, 2},
      {-9.0440175747, 38.8657912467, 2},
      {-9.0366098073, 38.8832969536, 2},
      {-8.9806865253, 38.9422888937, 2},
      {-8.9723987862, 38.8482041895, 2},
      {-8.9733592927, 38.8458350518, 2},
      {-8.8453369078, 38.8554173739, 2},
      {-8.8621902488, 38.8793241844, 2},
      {-8.8651142534, 38.8691639227, 2},
      {-8.9891975349, 38.9630883202, 2},
      {-8.9889932, 38.9551563, 2},
      {-8.9758800067, 38.9433570032, 2},
      {-8.9811424845, 38.88807198, 2},
      {-8.9291033723, 38.9080608627, 2},
      {-8.8770557195, 38.7042516911, 2},
      {-8.9135884171, 38.7967463844, 2},
      {-8.9765666523, 38.9406866993, 2},
      {-8.9813731708, 38.9396185496, 2},
      {-8.959823629, 38.8724726368, 2},
      {-8.9889932, 38.9551563, 2},
      {-8.9935909464, 38.96528644, 2},
      {-8.9718443592, 38.944272898, 2},
      {-8.9671766449, 38.9041599949, 2},
      {-8.9630473968, 38.9239659198, 2},
      {-8.9765666523, 38.9433570032, 2},
      {-8.9745067157, 38.9446921174, 2},
      {-8.9421601374, 38.8448411671, 2},
      {-8.9740430564, 38.8496357586, 2},
      {-8.9634866667, 38.9232033333, 2},
      {-9.3273852955, 38.6962584716, 2},
      {-8.9799998798, 38.9404196634, 2},
      {-8.974786, 38.846705, 2},
      {-8.9628696, 38.7562753, 2},
      {-8.9786265888, 38.9390844687, 2},
      {-9.1283621256, 38.7514825524, 2},
      {-9.0773341583, 38.8412532065, 2},
      {-8.8819633333, 38.6526416667, 2},
      {-8.9757394546, 38.8483600113, 2},
      {-8.9021196448, 38.7303828731, 2},
      {-8.922584095, 38.74191188, 2},
      {-8.9806865253, 38.9401526264, 2},
      {-8.9765666523, 38.9422888937, 2},
      {-8.9718969775, 38.9520972256, 2},
      {-8.9459010507, 38.9096783946, 2},
      {-8.9738200702, 38.9340104996, 2},
      {-8.9799998798, 38.9366810549, 2},
      {-8.9724581647, 38.8490562702, 2},
      {-8.97120713, 38.8487692869, 2},
      {-8.9798842973, 38.9203019204, 2},
      {-8.8802316667, 38.652575, 2},
      {-8.9532723549, 38.9006859361, 2},
      {-8.9698657179, 38.8890668173, 2},
      {-8.9396054192, 38.7614695734, 2},
      {-8.9384375039, 38.884126822, 2},
      {-8.9647956074, 38.8728776553, 2},
      {-9.1472810882, 38.8496086042, 2},
      {-8.9690712749, 38.8616417899, 2},
      {-9.00567817789934, 38.719680632435, 2}};

  numCoordinates = sizeof(coordinates) / sizeof(coordinates[0]);

  Serial.println(numCoordinates);

  return coordinates;
}