#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <initializer_list>
#include <lwip/sockets.h>
#include <errno.h>
#include <Preferences.h>
const char* WIFI_SSID="No_Disponible";
const char* WIFI_PASSWORD="Mapache2003";
constexpr int ENA=9, ENB=18, IN1=10, IN2=17, IN3=21, IN4=38;
constexpr int ENC1_A=5,ENC1_B=6,ENC2_A=7,ENC2_B=8;
constexpr int RELAY1=48,RELAY2=47;
constexpr bool RELAY_ACTIVE_LOW=true;
constexpr uint8_t OLED_ADDRS[]={0x3C,0x3D};
constexpr uint8_t PCA_ADDR=0x40;
constexpr uint16_t PORT=4211;
constexpr uint32_t WATCHDOG_MS=400;
constexpr uint8_t ARM_PROTOCOL=1;
// NC contact to GND: closed at rest; open (pressed or broken wire) is an emergency.
constexpr int ESTOP_PIN=1;
bool emergencyActive(){return ESTOP_PIN>=0 && digitalRead(ESTOP_PIN)==HIGH;}
WiFiServer listener(PORT); WiFiClient pc;
Adafruit_BNO055 imu28(55,0x28,&Wire),imu29(55,0x29,&Wire);
Adafruit_BNO055* bno=nullptr;
Adafruit_SSD1306 oled(128,64,&Wire,-1);
Adafruit_PWMServoDriver pca(PCA_ADDR, Wire);
bool oledReady=false,pcaReady=false;
constexpr uint8_t ARM_SERVO_COUNT=6;
constexpr float ARM_POWER_POSE[ARM_SERVO_COUNT]={108.5f,149.5f,180.0f,90.0f,90.0f,60.0f};
float armAngles[ARM_SERVO_COUNT];
float armStart[ARM_SERVO_COUNT],armTarget[ARM_SERVO_COUNT];
bool armMoving=false;
uint32_t armStartedAt=0,armDurationMs=0,lastArmWrite=0;
uint32_t tcpRxFrames=0,tcpTxFrames=0,lastDisplay=0;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
volatile int32_t ticksL=0,ticksR=0;
void IRAM_ATTR encL(){int v=digitalRead(ENC1_A)==digitalRead(ENC1_B)?1:-1;portENTER_CRITICAL_ISR(&mux);ticksL+=v;portEXIT_CRITICAL_ISR(&mux);}
void IRAM_ATTR encR(){int v=digitalRead(ENC2_A)==digitalRead(ENC2_B)?1:-1;portENTER_CRITICAL_ISR(&mux);ticksR+=v;portEXIT_CRITICAL_ISR(&mux);}
bool armed=false,pwmOK=false,serverStarted=false;
bool estopLatched=false,cutRelaysOnLoss=false,timeoutActive=false;
Preferences preferences;
int demandL=0,demandR=0,actualL=0,actualR=0;
uint32_t bootId,seq=0,lastCommand=0,lastTelemetry=0,lastImu=0,lastRamp=0,lastRetry=0;
float heading=0,rollDeg=0,pitchDeg=0;
uint8_t cs=0,cg=0,ca=0,cm=0;
char rx[100],tx[800];size_t ri=0,tn=0,to=0;bool overflow=false;
uint32_t txAt=0,imuReadAt=0;
bool imuReadOK=false;
uint32_t imuRetryAt=0;
float relativeHeading=0, previousHeading=0;
bool headingInitialized=false;
uint8_t oledAddress=0;
uint32_t lastI2CCheck=0;
bool i2cPresent(uint8_t address){
  Wire.beginTransmission(address);
  return Wire.endTransmission()==0;
}
uint16_t usToTicks(int us){
  us=constrain(us,500,2500);
  return static_cast<uint16_t>((static_cast<uint32_t>(us)*4096U)/20000U);
}
int angleToUs(uint8_t channel,float angle){
  angle=channel==5?constrain(angle,10.0f,75.0f):constrain(angle,0.0f,180.0f);
  return lroundf(600.0f+10.0f*angle);
}
void writeArmAngles(){
  if(!pcaReady)return;
  for(uint8_t channel=0;channel<ARM_SERVO_COUNT;channel++){
    int us=angleToUs(channel,armAngles[channel]);
    pca.setPWM(channel,0,usToTicks(us));
  }
}
void updateArmMotion(uint32_t now){
  if(!armMoving||!pcaReady||now-lastArmWrite<20)return;
  lastArmWrite=now;
  uint32_t elapsed=now-armStartedAt;
  float fraction=elapsed>=armDurationMs?1.0f:static_cast<float>(elapsed)/armDurationMs;
  for(uint8_t channel=0;channel<ARM_SERVO_COUNT;channel++)
    armAngles[channel]=armStart[channel]+(armTarget[channel]-armStart[channel])*fraction;
  writeArmAngles();
  if(fraction>=1.0f)armMoving=false;
}
void stopArmMotion(){
  updateArmMotion(millis());
  armMoving=false;
  for(uint8_t channel=0;channel<ARM_SERVO_COUNT;channel++)
    armStart[channel]=armTarget[channel]=armAngles[channel];
}
void startArmMotion(const int* requested,uint32_t duration,uint32_t now){
  updateArmMotion(now);
  for(uint8_t channel=0;channel<ARM_SERVO_COUNT;channel++){
    armStart[channel]=armAngles[channel];
    armTarget[channel]=static_cast<float>(requested[channel]);
  }
  armStartedAt=now;armDurationMs=duration;armMoving=true;
}
bool imuReady(){
  return bno&&imuReadOK&&millis()-imuReadAt<=250;
}
bool initPca(){
  if(!i2cPresent(PCA_ADDR))return false;
  pca.begin();pca.setPWMFreq(50);
  return true;
}
bool initOled(){
  for(uint8_t address:OLED_ADDRS){
    if(i2cPresent(address)&&oled.begin(SSD1306_SWITCHCAPVCC,address,false,false)){
      oledAddress=address;oled.setTextWrap(false);oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);return true;
    }
  }
  return false;
}
void checkI2CDevices(uint32_t now){
  if(now-lastI2CCheck<1000)return;
  lastI2CCheck=now;
  if(pcaReady)pcaReady=i2cPresent(PCA_ADDR);
  else pcaReady=initPca();
  if(!pcaReady&&armMoving)stopArmMotion();
  if(oledReady)oledReady=i2cPresent(oledAddress);
  else oledReady=initOled();
}
const char* statusText(){
  if(emergencyActive()||estopLatched)return "ERROR E-STOP";
  if(!pwmOK)return "ERROR MOTOR PWM";
  if(!imuReady())return "ERROR BNO055";
  if(!pcaReady)return "ERROR PCA9685";
  if(!oledReady)return "ERROR OLED";
  if(WiFi.status()!=WL_CONNECTED)return "WAITING WIFI";
  if(!serverStarted||!pc||!pc.connected())return "WAITING CONNECTION";
  if(actualL||actualR||armMoving)return "RUNNING";
  return "IDLE";
}
void drawOled(uint32_t now){
  if(!oledReady||now-lastDisplay<1000)return;
  char traffic[22];
  snprintf(traffic,sizeof(traffic),"TCP RX:%03lu TX:%03lu",
           (unsigned long)(tcpRxFrames%1000),(unsigned long)(tcpTxFrames%1000));
  lastDisplay=now;oled.clearDisplay();
  oled.setCursor(0,0);oled.println("SAR AGV");
  oled.setCursor(0,10);oled.print("IP:");
  if(WiFi.status()==WL_CONNECTED)oled.println(WiFi.localIP().toString());
  else oled.println("0.0.0.0");
  oled.setCursor(0,22);oled.println("STATUS:");
  oled.setCursor(0,32);oled.println(statusText());
  oled.setCursor(0,44);oled.print("BNO:");oled.print(imuReady()?"OK":"ERR");
  oled.print(" OLED:");oled.print(oledReady?"OK":"ERR");
  oled.print(" PCA:");oled.println(pcaReady?"OK":"ERR");
  oled.setCursor(0,56);oled.println(traffic);oled.display();
}
void motorOutputs(int l,int r){
  actualL=l;actualR=r;
  // Exact logical polarity from the supplied working firmware.
  digitalWrite(IN1,l<0); digitalWrite(IN2,l>=0);
  digitalWrite(IN3,r>=0);digitalWrite(IN4,r<0);
  ledcWrite(ENA,abs(l));ledcWrite(ENB,abs(r));
}
void relays(bool on){digitalWrite(RELAY1,RELAY_ACTIVE_LOW?!on:on);digitalWrite(RELAY2,RELAY_ACTIVE_LOW?!on:on);}
void halt(bool disarm){demandL=demandR=0;motorOutputs(0,0);stopArmMotion();if(disarm){armed=false;relays(false);}}
void disconnectPC(){halt(cutRelaysOnLoss);timeoutActive=true;pc.stop();ri=tn=to=0;overflow=false;}
void parseCommand(){
  if(!strcmp(rx,"ESTOP")){estopLatched=true;halt(true);return;}
  if(!strcmp(rx,"RESET_ESTOP")){halt(true);if(!emergencyActive())estopLatched=false;return;}
  if(!strcmp(rx,"POLICY:0")||!strcmp(rx,"POLICY:1")){
    bool requested=rx[7]=='1';
    if(requested!=cutRelaysOnLoss){cutRelaysOnLoss=requested;preferences.putBool("cut_loss",requested);}
    return;
  }

  if(!strcmp(rx,"STATUS")){lastTelemetry=millis()-100;return;}
  if(!strcmp(rx,"STOP")){halt(false);return;}
  if(!strcmp(rx,"POWER:0")){halt(true);return;}
  if(!strcmp(rx,"POWER:1")&&pwmOK&&!emergencyActive()&&!estopLatched){
    halt(false);
    if(!armed){
      if(!pcaReady)return;
      memcpy(armAngles,ARM_POWER_POSE,sizeof armAngles);
      writeArmAngles();
      relays(true);
    }
    armed=true;timeoutActive=false;lastCommand=millis();return;
  }
  int arm[ARM_SERVO_COUNT],duration=0,armFields=0;
  if(sscanf(rx,"ARM:%d,%d,%d,%d,%d,%d,%d%n",
      &arm[0],&arm[1],&arm[2],&arm[3],&arm[4],&arm[5],&duration,&armFields)==7
      &&armFields==(int)strlen(rx)&&pcaReady&&armed&&!emergencyActive()&&!estopLatched
      &&!timeoutActive&&duration>=100&&duration<=10000
      &&arm[0]>=0&&arm[0]<=180&&arm[1]>=0&&arm[1]<=180
      &&arm[2]>=0&&arm[2]<=180&&arm[3]>=0&&arm[3]<=180
      &&arm[4]>=0&&arm[4]<=180&&arm[5]>=10&&arm[5]<=75){
    lastCommand=millis();
    startArmMotion(arm,static_cast<uint32_t>(duration),lastCommand);
    return;
  }
  int l,r,n=0;
  if(sscanf(rx,"PWM:%d,%d%n",&l,&r,&n)==2&&n==(int)strlen(rx)&&l>=-255&&l<=255&&r>=-255&&r<=255){
    lastCommand=millis();
    if(l==0&&r==0)timeoutActive=false;
    if(armed&&!emergencyActive()&&!estopLatched&&!timeoutActive){demandL=l;demandR=r;}else halt(false);
    return;
  }
  halt(false);
}
int ramp(int v,int target){
  // Reverse through zero. Limit acceleration; zero commands stop immediately.
  if(target==0)return 0;
  if((v>0&&target<0)||(v<0&&target>0))target=0;
  if(v<target)return min(v+6,target);
  return max(v-6,target);
}
void setup(){
  memcpy(armAngles,ARM_POWER_POSE,sizeof armAngles);
  Serial.begin(115200);bootId=esp_random();
  preferences.begin("sar",false);cutRelaysOnLoss=preferences.getBool("cut_loss",false);
  if(ESTOP_PIN>=0)pinMode(ESTOP_PIN,INPUT_PULLUP);
  digitalWrite(RELAY1,RELAY_ACTIVE_LOW);digitalWrite(RELAY2,RELAY_ACTIVE_LOW);
  pinMode(RELAY1,OUTPUT);pinMode(RELAY2,OUTPUT);relays(false);
  for(int pin:{IN1,IN2,IN3,IN4})pinMode(pin,OUTPUT);
  pwmOK=ledcAttach(ENA,5000,8);pwmOK=ledcAttach(ENB,5000,8)&&pwmOK;halt(true);
  for(int pin:{ENC1_A,ENC1_B,ENC2_A,ENC2_B})pinMode(pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A),encL,CHANGE);attachInterrupt(digitalPinToInterrupt(ENC2_A),encR,CHANGE);
  Wire.begin(11,12,100000);Wire.setTimeOut(30);
  pcaReady=initPca();oledReady=initOled();lastI2CCheck=millis();
  Serial.printf("PCA9685 %s at 0x40; OLED %s\n",pcaReady?"ready":"missing",oledReady?"ready":"missing");
  if(imu29.begin())bno=&imu29;else if(imu28.begin())bno=&imu28;
  if(bno){bno->setExtCrystalUse(true);Serial.println(bno==&imu29?"BNO055 detected: 0x29":"BNO055 detected: 0x28");}
  else Serial.println("BNO055 not detected; check SDA11/SCL12 and power");
  WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(true);WiFi.setSleep(false);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.println("SAR AGV 1.0: power OFF; TCP 4211; waiting for WiFi");
}
void loop(){
  uint32_t now=millis();
  if(emergencyActive()){estopLatched=true;halt(true);}
  checkI2CDevices(now);drawOled(now);
  if(armed&&now-lastCommand>WATCHDOG_MS){halt(cutRelaysOnLoss);timeoutActive=true;}
  if(WiFi.status()!=WL_CONNECTED){if(armed||pc)disconnectPC();if(now-lastRetry>15000){lastRetry=now;WiFi.reconnect();}delay(1);return;}
  if(!serverStarted){listener.begin();serverStarted=true;Serial.print("AGV IP: ");Serial.println(WiFi.localIP());}
  if(pc&&!pc.connected())disconnectPC();
  if(!pc||!pc.connected()){WiFiClient incoming=listener.available();if(incoming){disconnectPC();pc=incoming;pc.setNoDelay(true);lastCommand=now;}}
  int budget=160;
  while(pc&&pc.available()&&budget-->0){char c=pc.read();if(c=='\n'||c=='\r'){if(!overflow&&ri){rx[ri]=0;++tcpRxFrames;parseCommand();}ri=0;overflow=false;}else if(!overflow){if(ri<sizeof(rx)-1)rx[ri++]=c;else{overflow=true;halt(false);}}}
  now=millis();
  if(armed&&now-lastCommand>WATCHDOG_MS){halt(cutRelaysOnLoss);timeoutActive=true;}
  updateArmMotion(now);
  if(now-lastRamp>=10){lastRamp=now;motorOutputs(armed&&!estopLatched?ramp(actualL,demandL):0,armed&&!estopLatched?ramp(actualR,demandR):0);}
  // Retry detection only while motor outputs are stopped.
  if(!bno && !armed && now-imuRetryAt>=5000){
    imuRetryAt=now;
    if(imu29.begin())bno=&imu29;else if(imu28.begin())bno=&imu28;
    if(bno){bno->setExtCrystalUse(true);Serial.println("BNO055 detected on retry");}
  }
  if(bno&&now-lastImu>=50){
    lastImu=now;uint8_t address=bno==&imu29?0x29:0x28;
    // BNO055 Euler registers: H, R, P, signed little-endian, 1/16 degree.
    // Read completion is checked; a failed transfer is never fresh heading=0.
    Wire.beginTransmission(address);Wire.write((uint8_t)0x1A);
    imuReadOK=false;
    if(Wire.endTransmission(false)==0 && Wire.requestFrom(address,(uint8_t)6)==6){
      uint8_t data[6];for(int i=0;i<6;i++)data[i]=Wire.read();
      heading=(int16_t)(data[0]|(data[1]<<8))/16.0f;
      rollDeg=(int16_t)(data[2]|(data[3]<<8))/16.0f;
      pitchDeg=(int16_t)(data[4]|(data[5]<<8))/16.0f;
      if(heading>=0 && heading<360){
        if(headingInitialized){
          float delta=heading-previousHeading;
          while(delta>180)delta-=360;while(delta<-180)delta+=360;
          relativeHeading+=delta;
        }else headingInitialized=true;
        previousHeading=heading;imuReadAt=millis();imuReadOK=true;
        bno->getCalibration(&cs,&cg,&ca,&cm); // informational, never a data gate
      }
    }
  }
  if(pc&&pc.connected()){
    if(!tn&&now-lastTelemetry>=100){lastTelemetry=now;int32_t l,r;portENTER_CRITICAL(&mux);l=ticksL;r=ticksR;portEXIT_CRITICAL(&mux);
      tn=snprintf(tx,sizeof(tx),
        "{\"protocol\":1,\"cut_relays_on_loss\":%d,\"physical_estop\":%d,\"boot_id\":%lu,\"seq\":%lu,\"ms\":%lu,"
        "\"tl\":%ld,\"tr\":%ld,\"pl\":%d,\"pr\":%d,\"armed\":%d,\"estop\":%d,\"pwm_ok\":%d,"
        "\"imu\":%d,\"imu_addr\":%d,\"heading_relative\":%.2f,\"heading\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,"
        "\"imu_age_ms\":%lu,\"imu_read_ok\":%d,\"oled_ready\":%d,\"pca_ready\":%d,\"arm_protocol\":%u,"
        "\"arm_busy\":%d,\"arm_angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
        "\"tcp_rx_frames\":%lu,\"tcp_tx_frames\":%lu,\"cs\":%d,\"cg\":%d,\"ca\":%d,\"cm\":%d}\n",
        cutRelaysOnLoss,ESTOP_PIN<0?-1:(emergencyActive()?1:0),(unsigned long)bootId,(unsigned long)++seq,
        (unsigned long)now,(long)l,(long)r,actualL,actualR,armed,estopLatched?1:0,pwmOK,bno!=nullptr,
        bno?(bno==&imu29?41:40):0,relativeHeading,heading,rollDeg,pitchDeg,(unsigned long)(millis()-imuReadAt),
        imuReadOK,oledReady,pcaReady,(unsigned int)ARM_PROTOCOL,armMoving,armAngles[0],armAngles[1],armAngles[2],
        armAngles[3],armAngles[4],armAngles[5],(unsigned long)tcpRxFrames,(unsigned long)tcpTxFrames,cs,cg,ca,cm);
      if(tn>=sizeof(tx)){disconnectPC();return;}to=0;txAt=millis();
    }
    if(tn){
      if(millis()-txAt>2500){Serial.println("TCP TX stalled; applying loss policy");disconnectPC();}
      else{
        int sent=::send(pc.fd(),tx+to,tn-to,MSG_DONTWAIT);
        if(sent>0){to+=sent;if(to>=tn){tn=to=0;++tcpTxFrames;}}
        else if(sent==0||(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR))disconnectPC();
      }
    }
  }
  delay(1);
}
