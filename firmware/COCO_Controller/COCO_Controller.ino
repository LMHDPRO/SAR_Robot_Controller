#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <initializer_list>
#include <lwip/sockets.h>
#include <errno.h>

const char* WIFI_SSID="No_Disponible";
const char* WIFI_PASSWORD="Mapache2003";
constexpr int ARM_X=2,ARM_Y=1,ARM_SW=5;
constexpr int AGV_X=4,AGV_Y=3,AGV_SW=6;
constexpr int ENC_CLK=7,ENC_DT=8,ENC_SW=9,DM_PIN=10;
constexpr int SDA_PIN=11,SCL_PIN=12;
constexpr int DISTANCE_AXIS=2,ROTATION_AXIS=3,MENU_AXIS=2;
constexpr int DISTANCE_SIGN=1,ROTATION_SIGN=1,MENU_SIGN=1;
constexpr int ACTIVATE_PERCENT=70;
constexpr int RELEASE_PERCENT=45;
constexpr int JOY_DEADZONE=150;
constexpr float FILTER_ALPHA=0.30f;
constexpr int TRANSITIONS_PER_CLICK=4;

Adafruit_SSD1306 oled(128,64,&Wire,-1);
bool oledReady=false;
const uint8_t COCO_BITMAP[] PROGMEM={
  0x7f,0xff,0xff,0xff,0x3f,0xff,0xff,0xff,
  0x1f,0xf9,0xff,0xff,0x00,0x61,0xff,0xff,
  0x00,0x01,0xff,0xff,0x00,0x01,0xff,0xff,
  0x00,0x01,0xff,0xff,0x00,0x03,0xff,0xff,
  0x02,0x01,0xff,0xff,0x07,0x00,0xcf,0xe1,
  0x07,0x00,0x00,0x00,0x17,0x00,0x00,0x00,
  0x3f,0x00,0x00,0x00,0xff,0xc0,0x08,0x00,
  0xf9,0xfe,0x3c,0x00,0xf9,0xfe,0x78,0x00,
  0xdb,0xfc,0x70,0x00,0xff,0xfd,0xe0,0x00,
  0x7f,0xf9,0xf0,0x00,0x7f,0xf1,0xfe,0x00,
  0x7f,0x81,0xfe,0x00,0x3e,0x01,0xff,0x00,
  0x38,0x01,0xff,0x80,0x18,0x03,0xd9,0x80,
  0x00,0x07,0x80,0x00,0x00,0x0e,0x00,0x00,
  0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0x00,0x00,0x00,0xff,0xff,0x07,0xef
};

struct Button{
  int pin;bool raw=false,down=false,rose=false,fell=false;
  uint32_t changed=0,pressed=0;
  explicit Button(int p):pin(p){}
  void begin(){pinMode(pin,INPUT_PULLUP);raw=down=digitalRead(pin)==LOW;changed=pressed=millis();}
  void update(uint32_t now){
    rose=fell=false;bool sample=digitalRead(pin)==LOW;
    if(sample!=raw){raw=sample;changed=now;}
    if(raw!=down&&now-changed>=30){down=raw;rose=down;fell=!down;if(rose)pressed=now;}
  }
  bool held()const{return raw&&down;}
  bool released()const{return !raw&&!down;}
};
Button armButton(ARM_SW),agvButton(AGV_SW),encoderButton(ENC_SW),deadman(DM_PIN);

struct Axis{
  int pin,sign,raw=0,center=1900,output=0;float filtered=1900;
  Axis(int p,int s):pin(p),sign(s){}
  void update(){
    raw=analogRead(pin);filtered+=FILTER_ALPHA*(raw-filtered);
    int d=(int)lroundf(filtered)-center;
    if(abs(d)<=JOY_DEADZONE){output=0;return;}
    int span=d>0?4095-center-JOY_DEADZONE:center-JOY_DEADZONE;
    if(span<=0){output=0;return;}
    int magnitude=constrain((abs(d)-JOY_DEADZONE)*100/span,0,100);
    output=sign*(d>0?magnitude:-magnitude);
  }
  bool centered()const{return output==0&&abs(raw-center)<=JOY_DEADZONE;}
};
Axis axes[]={Axis(ARM_X,-1),Axis(ARM_Y,1),Axis(AGV_X,-1),Axis(AGV_Y,1)};
bool allCentered(){for(int i=0;i<4;i++)if(!axes[i].centered())return false;return true;}
void calibrateJoysticks(){
  long total[4]={0,0,0,0};
  for(int n=0;n<64;n++){for(int i=0;i<4;i++)total[i]+=analogRead(axes[i].pin);delay(3);}
  for(int i=0;i<4;i++){
    axes[i].center=total[i]/64;axes[i].raw=axes[i].center;
    axes[i].filtered=axes[i].center;axes[i].output=0;
    Serial.printf("Axis %d center: %d\n",i,axes[i].center);
  }
}

portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoderTransitions=0;
volatile uint8_t encoderPrevious=0;
const int8_t ENCODER_TABLE[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
void IRAM_ATTR encoderISR(){
  portENTER_CRITICAL_ISR(&encoderMux);
  uint8_t s=(digitalRead(ENC_CLK)<<1)|digitalRead(ENC_DT);
  encoderTransitions+=ENCODER_TABLE[(encoderPrevious<<2)|s];encoderPrevious=s;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
int32_t readEncoder(){portENTER_CRITICAL(&encoderMux);int32_t n=encoderTransitions;portEXIT_CRITICAL(&encoderMux);return n;}
int32_t encoderConsumed=0,encoderTotal=0;int encoderDelta=0;uint32_t encoderDeltaAt=0;
int consumeEncoder(){
  int32_t n=readEncoder();int clicks=(n-encoderConsumed)/TRANSITIONS_PER_CLICK;
  encoderConsumed+=clicks*TRANSITIONS_PER_CLICK;encoderTotal+=clicks;
  if(clicks){encoderDelta=clicks;encoderDeltaAt=millis();}return clicks;
}

enum CocoPage:uint8_t{PAGE_MAIN,PAGE_MENU,PAGE_DEBUG};
enum CocoItem:uint8_t{ITEM_POWER,ITEM_MODE,ITEM_SPEED,ITEM_FEEL,ITEM_DEBUG,ITEM_RESET,ITEM_EXIT,ITEM_COUNT};
CocoPage page=PAGE_MAIN;
int selected=0,debugTab=0,controlMode=1,speedSetting=0,armFeel=0;
bool rotationMode=false,gripMode=false,armCenterRequired=false;
bool motionEnabled=false,releaseSeen=false,menuWaitRelease=false,navigationReady=false;
uint32_t navRepeatAt=0;bool wasReady=false;
float zChange=0,gripChange=0;
const char* MODE_NAMES[]={"Joint","Gamepad","TCP"};
const char* SPEED_NAMES[]={"Low","Medium","High"};
int wrapIndex(int n,int count){return(n%count+count)%count;}
void lockMotion(){motionEnabled=false;releaseSeen=false;}

WiFiServer server(4210);WiFiClient pythonClient;
bool serverStarted=false,stateValid=false,pythonControl=false,agvFresh=false,agvArmed=false;
uint32_t stateAt=0,bootId=0,seq=0,powerId=0,powerAt=0;
bool powerPending=false,powerEnable=false;
float agvTravel=0,agvYaw=0,agvSpeed=0;
float shownYaw=0,shownDelta=0,shownTurns=0;int yawValid=0;uint32_t yawAt=0;
long ticksL=0,ticksR=0;int imuPresent=0;
char rx[180],tx[950];size_t rxN=0,txN=0,txOffset=0;
bool rxOverflow=false;uint32_t txStarted=0,lastPacket=0;
bool linkFresh(){return pythonClient&&pythonClient.connected()&&stateValid&&millis()-stateAt<1500;}
bool robotReady(){return linkFresh()&&pythonControl&&agvFresh&&agvArmed;}
bool allowed(){return motionEnabled&&robotReady()&&deadman.held()&&page==PAGE_MAIN&&controlMode==1;}
int thresholdDirection(int value,int previous){
  if(previous && value*previous>=RELEASE_PERCENT)return previous;
  return abs(value)>=ACTIVATE_PERCENT?(value>0?1:-1):0;
}
int steerRequest(){static int previous=0;previous=thresholdDirection(axes[ROTATION_AXIS].output*ROTATION_SIGN,previous);return previous;}
int driveRequest(){
  // Rotation has priority, including intermediate diagonal positions.
  static int previous=0;
  if(abs(axes[ROTATION_AXIS].output)>20){previous=0;return 0;}
  previous=thresholdDirection(axes[DISTANCE_AXIS].output*DISTANCE_SIGN,previous);return previous;
}
void requestPower(){
  if(!linkFresh()||!pythonControl||!agvFresh||!deadman.released()||!allCentered()||powerPending)return;
  powerEnable=!agvArmed;powerPending=true;++powerId;powerAt=millis();
}
void closePython(const char* reason){
  if(pythonClient)pythonClient.stop();
  stateValid=pythonControl=agvFresh=agvArmed=powerPending=false;yawValid=0;lockMotion();
  rxN=txN=txOffset=0;rxOverflow=false;if(reason)Serial.println(reason);
}
void parsePython(){
  if(!strncmp(rx,"YAW:",4)){
    float yaw,delta,turns;int valid,n=0;
    if(sscanf(rx+4,"%f,%f,%f,%d%n",&yaw,&delta,&turns,&valid,&n)==4&&n==(int)strlen(rx+4)&&isfinite(yaw)&&isfinite(delta)&&isfinite(turns)){
      shownYaw=yaw;shownDelta=delta;shownTurns=turns;yawValid=valid==1;yawAt=millis();
    }return;
  }
  if(!strcmp(rx,"CTRL:1")){pythonControl=true;return;}
  if(!strcmp(rx,"CTRL:0")){pythonControl=false;lockMotion();return;}
  if(!strncmp(rx,"STATE:",6)){
    float x,y,z,a,b,g,v;int fresh,armed,fault,n=0;
    if(sscanf(rx+6,"%f,%f,%f,%f,%f,%f,%f,%d,%d,%d%n",&x,&y,&z,&a,&b,&g,&v,&fresh,&armed,&fault,&n)==10&&n==(int)strlen(rx+6)){
      stateValid=true;stateAt=millis();agvFresh=fresh==1&&fault==0;agvArmed=armed==1;
      if(powerPending&&agvFresh&&agvArmed==powerEnable)powerPending=false;
    }
  }else if(!strncmp(rx,"AGV:",4)){
    float travel,yaw,speed;long l,r;int imu,n=0;
    if(sscanf(rx+4,"%f,%f,%f,%ld,%ld,%d%n",&travel,&yaw,&speed,&l,&r,&imu,&n)==6&&n==(int)strlen(rx+4)&&isfinite(travel)&&isfinite(yaw)&&isfinite(speed)){
      agvTravel=travel;agvYaw=yaw;agvSpeed=speed;ticksL=l;ticksR=r;imuPresent=imu;
    }
  }
}
void serviceTCP(){
  if(WiFi.status()!=WL_CONNECTED){closePython(nullptr);return;}
  if(!serverStarted){server.begin();serverStarted=true;}
  if(!pythonClient||!pythonClient.connected()){
    closePython(nullptr);WiFiClient c=server.available();if(!c)return;
    pythonClient=c;pythonClient.setNoDelay(true);Serial.println("Python TCP connected; waiting for STATE");
  }
  int budget=400;
  while(pythonClient.available()&&budget-->0){
    char c=pythonClient.read();
    if(c=='\n'||c=='\r'){
      if(rxN&&!rxOverflow){rx[rxN]=0;parsePython();}
      rxN=0;rxOverflow=false;
    }else if(!rxOverflow){if(rxN<sizeof(rx)-1)rx[rxN++]=c;else{rxOverflow=true;stateValid=false;lockMotion();}}
  }
  if(!txN&&millis()-lastPacket>=50){
    lastPacket=millis();bool enabled=allowed();
    int n=snprintf(tx,sizeof(tx),
      "{\"type\":\"coco\",\"control_style\":\"manual90\",\"boot_id\":%lu,\"seq\":%lu,"
      "\"mode\":\"%c\",\"menu_open\":%d,\"debug_open\":%d,\"deadman\":%d,\"motion_enabled\":%d,"
      "\"arm_filtered_x\":%d,\"arm_filtered_y\":%d,\"agv_filtered_x\":%d,\"agv_filtered_y\":%d,"
      "\"drive\":%d,\"steer\":%d,\"arm_mode\":\"%s\",\"encoder_mode\":\"%s\","
      "\"distance_cm\":0,\"turn_deg\":0,\"turn_jog\":0,\"z_change_mm\":%.2f,\"grip_change_pct\":%.2f,"
      "\"speed_level\":%d,\"power_request_id\":%lu,\"power_pending\":%d,\"power_enable\":%d}\n",
      (unsigned long)bootId,(unsigned long)++seq,'A'+controlMode,page==PAGE_MENU,page==PAGE_DEBUG,
      deadman.held(),enabled,axes[0].output,axes[1].output,axes[2].output,axes[3].output,
      enabled?driveRequest():0,enabled?steerRequest():0,rotationMode?"RX_RY":"XY",gripMode?"GRIP":"Z",
      zChange,gripChange,speedSetting,(unsigned long)powerId,powerPending,powerEnable);
    if(n<=0||(size_t)n>=sizeof(tx)){closePython("TCP packet overflow");return;}
    txN=n;txOffset=0;txStarted=millis();
  }
  if(txN){
    if(millis()-txStarted>2500){closePython("TCP send timeout");return;}
    int n=::send(pythonClient.fd(),tx+txOffset,txN-txOffset,MSG_DONTWAIT);
    if(n>0){txOffset+=n;if(txOffset>=txN)txN=txOffset=0;}
    else if(n==0||(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)){closePython("TCP send failed");return;}
  }
  if(powerPending&&millis()-powerAt>2500)powerPending=false;
}

void updateControls(int clicks,uint32_t now){
  if(!deadman.held()||page!=PAGE_MAIN||controlMode!=1||!robotReady())motionEnabled=false;
  if(page==PAGE_DEBUG){
    if(deadman.rose)debugTab=(debugTab+1)%6;
    if(agvButton.fell){page=PAGE_MENU;navigationReady=false;lockMotion();}return;
  }
  if(page==PAGE_MAIN&&deadman.released()&&agvButton.held()&&now-agvButton.pressed>=600){
    page=PAGE_MENU;selected=ITEM_POWER;menuWaitRelease=true;navigationReady=false;lockMotion();return;
  }
  if(page==PAGE_MENU){
    int nav=axes[MENU_AXIS].output*MENU_SIGN;
    if(abs(nav)<15)navigationReady=true;
    if(abs(nav)>55&&(navigationReady||now-navRepeatAt>300)){selected=wrapIndex(selected+(nav>0?-1:1),ITEM_COUNT);navigationReady=false;navRepeatAt=now;}
    if(selected==ITEM_MODE && clicks)controlMode=wrapIndex(controlMode+clicks,3);
    if(selected==ITEM_SPEED)speedSetting=constrain(speedSetting+clicks,0,2);
    if(selected==ITEM_FEEL)armFeel=constrain(armFeel+clicks,0,1);
    if(menuWaitRelease){if(agvButton.released())menuWaitRelease=false;return;}
    if(agvButton.fell || encoderButton.rose){
      if(selected==ITEM_POWER){requestPower();return;}
      if(selected==ITEM_DEBUG){page=PAGE_DEBUG;debugTab=0;}
      else{if(selected==ITEM_RESET){zChange=0;gripChange=0;}page=PAGE_MAIN;}
      lockMotion();
    }return;
  }
  if(!robotReady())releaseSeen=false;
  if(deadman.released()&&robotReady())releaseSeen=true;
  if(deadman.rose){motionEnabled=releaseSeen&&allCentered()&&controlMode==1&&robotReady();releaseSeen=false;}
  if(controlMode==1&&armButton.rose){rotationMode=!rotationMode;armCenterRequired=true;}
  if(axes[0].centered()&&axes[1].centered())armCenterRequired=false;
  if(encoderButton.rose&&deadman.released()&&!agvArmed){requestPower();return;}
  if(controlMode==1&&encoderButton.rose){gripMode=!gripMode;clicks=0;}
  if(allowed()&&clicks){float step=armFeel==0?0.5f:1.0f;if(gripMode)gripChange+=clicks*step;else zChange+=clicks*step;}
}

void lineAt(int y,const char* text){oled.setCursor(0,y);for(int i=0;text[i]&&i<21;i++)oled.write(text[i]);}
void header(const char* title){oled.clearDisplay();oled.setTextSize(1);oled.setTextColor(SSD1306_WHITE);lineAt(0,title);oled.drawFastHLine(0,9,128,SSD1306_WHITE);}
void battery(){oled.drawRect(113,1,13,7,SSD1306_WHITE);oled.fillRect(126,3,2,3,SSD1306_WHITE);}
void drawMain(){
  char text[48];snprintf(text,sizeof(text),"COCO %c %s",'A'+controlMode,allowed()?"LIVE":"LOCK");header(text);battery();
  if(!linkFresh()){
    lineAt(12,"Python: waiting");
    if(WiFi.status()==WL_CONNECTED){snprintf(text,sizeof(text),"IP:%s",WiFi.localIP().toString().c_str());lineAt(24,text);lineAt(36,"TCP port: 4210");snprintf(text,sizeof(text),"WiFi RSSI:%ld",(long)WiFi.RSSI());lineAt(46,text);}
    else{lineAt(24,"IP: waiting for WiFi");lineAt(36,"TCP port: 4210");lineAt(46,"Connecting...");}
    lineAt(56,"Hold B: MENU");return;
  }
  snprintf(text,sizeof(text),"A:%s ENC:%s",rotationMode?"RxRy":"XY",gripMode?"Grip":"Z");lineAt(12,text);
  int d=allowed()?driveRequest():0,r=allowed()?steerRequest():0;
  snprintf(text,sizeof(text),"AGV:%s",r>0?"LEFT":r<0?"RIGHT":d>0?"FORWARD":d<0?"REVERSE":"STOP");lineAt(22,text);
  if(yawValid&&millis()-yawAt<1500){
    snprintf(text,sizeof(text),"Yaw:%+.1f T:%+.2f",shownYaw,shownTurns);lineAt(32,text);
    snprintf(text,sizeof(text),"Turn d:%+.1f deg",shownDelta);lineAt(42,text);
  }else{lineAt(32,"IMU: NO FRESH DATA");snprintf(text,sizeof(text),"Pose yaw:%+.1f",agvYaw);lineAt(42,text);}
  const char* why=allowed()?"LIVE":!pythonControl?"Select COCO on PC":!agvFresh?"AGV offline":!agvArmed?"Robot power OFF":controlMode!=1?"Select Gamepad B":!deadman.held()?"Hold DM to drive":!allCentered()?"Release DM + center":"Release / press DM";
  lineAt(56,why);
}
void drawMenu(){
  header("COCO OS | MENU");char rows[ITEM_COUNT][24];
  snprintf(rows[ITEM_POWER],24,"%s",powerPending?"Waiting for AGV":!linkFresh()?"Python offline":!pythonControl?"Select COCO in PC":!agvFresh?"AGV no data":agvArmed?"Disable robot":"Enable robot");
  snprintf(rows[ITEM_MODE],24,"Mode %c: %s",'A'+controlMode,MODE_NAMES[controlMode]);
  snprintf(rows[ITEM_SPEED],24,"PWM: %s",SPEED_NAMES[speedSetting]);
  snprintf(rows[ITEM_FEEL],24,"Encoder: %s",armFeel?"Normal":"Fine");
  snprintf(rows[ITEM_DEBUG],24,"Debug / info tabs");snprintf(rows[ITEM_RESET],24,"Reset arm requests");snprintf(rows[ITEM_EXIT],24,"Exit menu");
  int first=selected>=5?selected-4:0;
  for(int i=first;i<first+5&&i<ITEM_COUNT;i++){
    int y=12+(i-first)*8;if(i==selected){oled.fillRect(0,y,128,8,SSD1306_WHITE);oled.setTextColor(SSD1306_BLACK);}
    oled.setCursor(2,y);oled.print(rows[i]);oled.setTextColor(SSD1306_WHITE);
  }lineAt(56,"B:OK ENC:edit/click");
}
void drawDebug(){
  const char* names[]={"RAW ADC","FILTERED","BUTTONS","AGV INFO","ARM INFO","NETWORK"};
  char t[48];snprintf(t,sizeof(t),"%d/6 %s",debugTab+1,names[debugTab]);header(t);
  switch(debugTab){
    case 0:
      snprintf(t,sizeof(t),"A X:%d Y:%d",axes[0].raw,axes[1].raw);lineAt(12,t);
      snprintf(t,sizeof(t),"B X:%d Y:%d",axes[2].raw,axes[3].raw);lineAt(22,t);
      snprintf(t,sizeof(t),"Center A:%d,%d",axes[0].center,axes[1].center);lineAt(32,t);
      snprintf(t,sizeof(t),"Center B:%d,%d",axes[2].center,axes[3].center);lineAt(42,t);break;
    case 1:
      snprintf(t,sizeof(t),"A:%+d %+d",axes[0].output,axes[1].output);lineAt(12,t);
      snprintf(t,sizeof(t),"B:%+d %+d",axes[2].output,axes[3].output);lineAt(22,t);
      snprintf(t,sizeof(t),"D:%d R:%d",driveRequest(),steerRequest());lineAt(32,t);lineAt(42,"Trigger 70% / off 45%");break;
    case 2:
      snprintf(t,sizeof(t),"SW A:%d B:%d E:%d",armButton.down,agvButton.down,encoderButton.down);lineAt(12,t);
      snprintf(t,sizeof(t),"ENC:%ld d:%d",(long)encoderTotal,millis()-encoderDeltaAt<1000?encoderDelta:0);lineAt(22,t);
      snprintf(t,sizeof(t),"DM:%d STOP:not fitted",deadman.held());lineAt(32,t);lineAt(42,"Outputs: blocked");break;
    case 3:
      if(linkFresh()&&agvFresh){
        snprintf(t,sizeof(t),"Power:%d V:%.0fmm/s",agvArmed,agvSpeed);lineAt(12,t);
        snprintf(t,sizeof(t),"D:%.1fcm R:%.1f",agvTravel,agvYaw);lineAt(22,t);
        snprintf(t,sizeof(t),"L:%ld R:%ld",ticksL,ticksR);lineAt(32,t);
        snprintf(t,sizeof(t),"IMU OK:%d d:%+.1f",yawValid&&millis()-yawAt<1500,shownDelta);lineAt(42,t);
      }else{lineAt(12,"AGV NO RECENT DATA");lineAt(22,"D:? R:? Speed:?");}break;
    case 4:lineAt(12,"TCP X:? Y:? Z:?");lineAt(22,"Rx:? Ry:? Grip:?");lineAt(32,"Arm not integrated");lineAt(42,"IK:? Limits:?");break;
    case 5:
      snprintf(t,sizeof(t),"WiFi:%s",WiFi.status()==WL_CONNECTED?"ON":"OFF");lineAt(12,t);
      if(WiFi.status()==WL_CONNECTED)lineAt(22,WiFi.localIP().toString().c_str());else lineAt(22,"IP: waiting");
      snprintf(t,sizeof(t),"TCP:4210 Python:%d",linkFresh());lineAt(32,t);
      snprintf(t,sizeof(t),"Owner:%s",pythonControl?"COCO":"PC");lineAt(42,t);break;
  }lineAt(56,"DM: next B: return");
}
void updateDisplay(){if(!oledReady)return;if(page==PAGE_MENU)drawMenu();else if(page==PAGE_DEBUG)drawDebug();else drawMain();oled.display();}
void startDisplay(){
  Wire.begin(SDA_PIN,SCL_PIN,400000);Wire.setTimeOut(15);
  for(uint8_t addr:{0x3C,0x3D}){Wire.beginTransmission(addr);if(Wire.endTransmission()==0){oledReady=oled.begin(SSD1306_SWITCHCAPVCC,addr,false,false);if(oledReady)break;}}
  if(!oledReady){Serial.println("OLED not found");return;}
  oled.setTextWrap(false);oled.clearDisplay();oled.setTextColor(SSD1306_WHITE);
  for(int y=0;y<40;y++)for(int x=0;x<40;x++){
    int sx=x*32/40,sy=y*32/40;if(pgm_read_byte(&COCO_BITMAP[sy*4+sx/8])&(0x80>>(sx%8)))oled.drawPixel(44+x,y,SSD1306_WHITE);
  }
  oled.setTextSize(2);int16_t x,y;uint16_t w,h;oled.getTextBounds("COCO OS",0,0,&x,&y,&w,&h);oled.setCursor((128-(int)w)/2-x,46);oled.print("COCO OS");oled.display();delay(3000);
}
void setup(){
  Serial.begin(115200);bootId=esp_random();armButton.begin();agvButton.begin();encoderButton.begin();deadman.begin();
  pinMode(ENC_CLK,INPUT_PULLUP);pinMode(ENC_DT,INPUT_PULLUP);encoderPrevious=(digitalRead(ENC_CLK)<<1)|digitalRead(ENC_DT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK),encoderISR,CHANGE);attachInterrupt(digitalPinToInterrupt(ENC_DT),encoderISR,CHANGE);
  analogReadResolution(12);WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setAutoReconnect(true);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  startDisplay();calibrateJoysticks();encoderConsumed=readEncoder();Serial.println("COCO / SAR 1.0 / TCP 4210 / hold B: menu");
}
void loop(){
  static uint32_t inputAt=0,displayAt=0,serialAt=0,retryAt=0;static bool previousWifi=false;
  uint32_t now=millis();armButton.update(now);agvButton.update(now);encoderButton.update(now);deadman.update(now);
  if(now-inputAt>=5){inputAt=now;for(int i=0;i<4;i++)axes[i].update();}
  serviceTCP();updateControls(consumeEncoder(),millis());
  bool wifi=WiFi.status()==WL_CONNECTED;
  if(wifi&&!previousWifi){Serial.print("COCO IP: ");Serial.println(WiFi.localIP());}previousWifi=wifi;
  if(!wifi&&now-retryAt>=15000){retryAt=now;WiFi.reconnect();}
  if(now-displayAt>=100){displayAt=now;updateDisplay();}
  if(now-serialAt>=500 && Serial && Serial.availableForWrite()>160){serialAt=now;Serial.printf("PC:%d OwnerCOCO:%d AGV:%d Armed:%d DM:%d LIVE:%d D:%d R:%d\n",linkFresh(),pythonControl,agvFresh,agvArmed,deadman.held(),allowed(),allowed()?driveRequest():0,allowed()?steerRequest():0);}
  delay(1);
}
