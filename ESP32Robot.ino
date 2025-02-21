#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <memory>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <GyverStepper2.h>
#include <ESP32Servo.h>

void startOTA(String&);

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

String name;

std::atomic_flag progReady;
std::atomic_flag connected;
Preferences preferences;


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

const int cSteps=4076;
const int cMm=100;
GStepper2<STEPPER4WIRE_HALF> L_stepper(cSteps, ML_D1, ML_D3, ML_D2, ML_D4);
GStepper2<STEPPER4WIRE_HALF> R_stepper(cSteps, MR_D1, MR_D3, MR_D2, MR_D4);

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

uint16_t cDegInAngle;

bool angle(int32_t deg){ //314mm - l O between wheel = 360 градусов
  Serial.print("Angle ");
  Serial.println(deg);
//  setRGB(0,255,255,0);
  L_stepper.setTarget(deg*cDegInAngle*cSteps/(360*100),RELATIVE);
  R_stepper.setTarget(-deg*cDegInAngle*cSteps/(360*100),RELATIVE);
  waitForFinish();
//  setRGB(0,0,0,0);
  return true;
}

bool feather(bool pos){
  Serial.print("Feather ");
  Serial.println(pos);
//  setRGB(0,0,255,0);
  if(pos){
    myservo.write(40);
  }else{
    myservo.write(15);
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

bool showError(){
  setRGB(0,5,0,0);
  setRGB(1,0,0,0);
  return 1;
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
/*
    if (*p&0xC000==0)  // Цикл
    else if(*p&0xFC00==0x7000) // Условный вход в подпрограмму
    else if(*p&0xFC00==0x7400) // Условный переход
    else if(*p&0xFC00==0x7800) // Вход в подпрограмму
    else if(*p&0xFC00==0x7C00) // Безусловный переход
*/
    if(*p&0x8000==0) if(showError()) break;
    else if(*p&0xFC00==0x8000) line((int16_t(*p<<6))>>6); // Движение вперед/назад
    else if(*p&0xFE00==0x8400) angle((int16_t(*p<<7))>>7); // Поворот вправо/влево
    else if(*p&0xFFFE==0x87FE) feather(!!(*p&1)); // Поднять/опустить перо
    else if(*p==0x87FC){ // Принять настройки
      auto tmp = (char*)(p+1);
      auto param=tmp;
      tmp+=strlen(param)+1;
      if(param[0]=='n') preferences.putString(param,tmp);
      else preferences.putShort(param,*(uint16_t*)tmp);
      break;
    }
    else if(*p==0x87FD){ // Вызвать OTA
      startOTA(name);
    }
    else if(*p&0xC000==0xC000){ // Это цвет глаз
      uint16_t data=*p;
      uint8_t num=*p&0x2000;
      uint8_t tmp=(data>>9)& 0xF;
      uint8_t r=rbtable[tmp];
      tmp=(data>>4)& 0x1F;
      uint8_t g=gtable[tmp];
      tmp=data& 0xF;
      uint8_t b=rbtable[tmp];
      Serial.print("LED "); Serial.print(num); Serial.print(" "); Serial.print(r); Serial.print(" "); Serial.print(g); Serial.print(" "); Serial.println(b);
      setRGB(!num,r,g,b);
    }
    else if(showError()) break; // Это ошибочный код 

/*
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
    }*/
  }
  feather(1);
  setRGB(0,255,175,0);
  setRGB(1,255,175,0);
  return true;
}

void setup() {
  preferences.begin("settings");
  Serial.begin(115200);
  strip.begin();

  L_stepper.autoPower(true);
  R_stepper.autoPower(true);
  R_stepper.reverse(true);
  L_stepper.setMaxSpeed(1200);
  L_stepper.setAcceleration(1200);
  R_stepper.setMaxSpeed(1200);
  R_stepper.setAcceleration(1200);

  myservo.attach(SERVO);
  feather(1);
  progReady.test_and_set();
  connected.test_and_set();

  setRGB(0,255,175,0);
  setRGB(1,255,175,0);

  name = preferences.getString("name", "Clear Turtle");
  cDegInAngle=preferences.getUShort("cDegInAngle",314);

  BLEDevice::init(name); //Red Knight Green Dragon
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
