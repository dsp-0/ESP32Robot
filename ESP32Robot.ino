#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
//#include <Preferences.h>
#include <memory>

#include <Freenove_WS2812_Lib_for_ESP32.h>

#include <GyverStepper2.h>

#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
                // 16 servo objects can be created on the ESP32

const BLEUUID SERVICE_UUID = BLEUUID("01942846-0661-7c4a-8953-e76f2ae2e6e2");
const BLEUUID PROG_CHARACTERISTIC_UUID = BLEUUID("01942846-0761-7c4a-8953-e76f2ae2e6e2");

const uint32_t LEDS_COUNT = 2;
const uint32_t LEDS_PIN =	13;
const uint32_t CHANNEL = 7;
const uint32_t ML_D1=14, ML_D2=27, ML_D3=26, ML_D4=25;
const uint32_t MR_D1=16, MR_D2=17, MR_D3=18, MR_D4=19;
const uint32_t SERVO=21;

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_RGB);

const uint8_t rbtable[]={0,1,2,3,5,7,11,16,24,35,52,78,116,172,255};
const uint8_t gtable[]={0,1,2,3,4,5,6,8,9,10,12,14,16,19,21,25,28,33,38,44,51,59,69,81,94,111,130,154,182,215,255};

uint16_t prog[1000];
uint16_t prog_len=0;

std::atomic_flag progReady;
std::atomic_flag connected;
//Preferences preferences;


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    prog_len=0;
    connected.clear();
  }
  void onDisconnect(BLEServer* pServer) {
    BLEDevice::startAdvertising();
    progReady.clear();
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    auto cUUID=pCharacteristic->getUUID();
    if(cUUID.equals(PROG_CHARACTERISTIC_UUID)) {
      uint8_t* data = pCharacteristic->getData();
      uint16_t len = pCharacteristic->getLength();
      Serial.print("Received bytes: ");
      Serial.println(len);
        uint8_t* progBuf=(uint8_t*)prog;
      for(int i=0;i<len;i++,data++){
        progBuf[prog_len]=*data;
        prog_len++;
      }
    }
  }
};

GStepper2<STEPPER4WIRE> L_stepper(2038, ML_D1, ML_D3, ML_D2, ML_D4);
GStepper2<STEPPER4WIRE> R_stepper(2038, MR_D1, MR_D3, MR_D2, MR_D4);
const int cSteps=2038;
const int cMm=100;

bool setRGB(uint8_t num, uint8_t r, uint8_t g, uint8_t b);

bool waitForFinish(){
  while(!L_stepper.ready()){
    L_stepper.tick();
    R_stepper.tick();
  }
  return true;
}

bool line(int32_t mm){ //100мм = 360 градусов = 2038 steps
  Serial.print("Line ");
  Serial.println(mm);
//  setRGB(0,255,0,255);
  L_stepper.setTarget(mm*cSteps/cMm,RELATIVE);
  R_stepper.setTarget(mm*cSteps/cMm,RELATIVE);
  waitForFinish();
//  setRGB(0,0,0,0);
  return true;
}

bool angle(int32_t deg){ //314mm - l O between wheel = 360 градусов
  Serial.print("Angle ");
  Serial.println(deg);
//  setRGB(0,255,255,0);
  L_stepper.setTarget(deg*314*cSteps/(360*100),RELATIVE);
  R_stepper.setTarget(-deg*314*cSteps/(360*100),RELATIVE);
  waitForFinish();
//  setRGB(0,0,0,0);
  return true;
}

bool feather(bool pos){
  Serial.print("Feather ");
  Serial.println(pos);
//  setRGB(0,0,255,0);
  if(pos){
    myservo.write(20);
  }else{
    myservo.write(10);
  }
  delay(250);
//  setRGB(0,0,0,0);
  return true;
}

bool setRGB(uint8_t num, uint8_t r, uint8_t g, uint8_t b){
  strip.setLedColorData(num,r,g,b);
  strip.show();
  delay(10);
  return true;
}

//union Cmd{
//  uint16_t d;
//  struct{uint16_t command:2; int16_t data:14;} s;
//};

enum commands{CMD_LINE=0,
              CMD_ANGLE,
              CMD_PEN,
              CMD_LED};

bool run(){
  setRGB(0,0,170,255);
  setRGB(1,0,170,255);
  auto p=prog;
  for(int i=0;i<prog_len/2;p++,i++){
    uint16_t cmd=*p>>14;
    int16_t data=(*p<<2);
    data>>=2;

    Serial.print("Command ");
    Serial.print(cmd);
    Serial.print(" data ");
    Serial.println(data);
    switch (cmd){
    case CMD_LINE:
      line(data);
      break;
    case CMD_ANGLE:
      angle(data);
      break;
    case CMD_PEN:
      feather(data<0);
      break;
    case CMD_LED:
      uint8_t num=data<0;
      uint8_t tmp=(data>>9)& 0xF;
      uint8_t r=rbtable[tmp];
      tmp=(data>>4)& 0x1F;
      uint8_t g=gtable[tmp];
      tmp=data& 0xF;
      uint8_t b=rbtable[tmp];
      Serial.print("LED "); Serial.print(num); Serial.print(" "); Serial.print(r); Serial.print(" "); Serial.print(g); Serial.print(" "); Serial.println(b);
      setRGB(!num,r,g,b);
      break;
    }
  }
  feather(1);
  setRGB(0,255,175,0);
  setRGB(1,255,175,0);
  return true;
}

void setup() {
//  preferences.begin("lamp");
  Serial.begin(115200);
  strip.begin();

  L_stepper.autoPower(true);
  R_stepper.autoPower(true);
  R_stepper.reverse(true);
  L_stepper.setMaxSpeed(2038);
  L_stepper.setAcceleration(8152);
  R_stepper.setMaxSpeed(2038);
  R_stepper.setAcceleration(8152);

  myservo.attach(SERVO);
  myservo.write(20);
  progReady.test_and_set();
  connected.test_and_set();

  setRGB(0,255,175,0);
  setRGB(1,255,175,0);

  //uint32_t settings=preferences.getUInt("s",2565);

  BLEDevice::init("Red Knight");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks);
  BLEDevice::setMTU(517);

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pProgCharacteristic = pService->createCharacteristic(
                                         PROG_CHARACTERISTIC_UUID,
 //                                        BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pProgCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

}

void loop() {
  if(!progReady.test_and_set()) run();
  if(!connected.test_and_set()){
    setRGB(0,108,255,0);
    setRGB(1,108,255,0);
  }
}
