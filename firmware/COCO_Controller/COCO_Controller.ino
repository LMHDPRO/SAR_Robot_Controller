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
uint8_t oledAddress=0;
size_t oledOffset=1024;
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
enum CocoItem:uint8_t{ITEM_OWNER,ITEM_POWER,ITEM_MODE,ITEM_ARM_SPEED,ITEM_AGV_SPEED,ITEM_TURN,ITEM_FEEL,ITEM_DEBUG,ITEM_RESET,ITEM_COUNT};
CocoPage page=PAGE_MAIN;
int selected=0,debugTab=0,controlMode=1,armSpeedSetting=1,agvSpeedSetting=0,turnStepIndex=1,armFeel=0,selectedArmJoint=0;
bool preciseTurnEnabled=false;
bool rotationMode=false,gripMode=false,armCenterRequired=false;
bool motionEnabled=false,releaseSeen=false,menuWaitRelease=false,navigationReady=false;
uint32_t navRepeatAt=0, splashAt=0;
float zChange=0,gripChange=0;
const char* MODE_NAMES[]={"JOINTS","DUAL","TCP","AGV"};
const char* SPEED_NAMES[]={"LOW","MID","HIGH"};
const char* JOINT_NAMES[]={"BASE","SHOULDER","ELBOW","WRIST R","WRIST E","GRIP"};
const int TURN_STEPS[]={45,90,180};
const float ARM_ENCODER_STEPS[][2]={{0.5f,1.0f},{10.0f,20.0f},{10.0f,15.0f}};
int wrapIndex(int n,int count){return(n%count+count)%count;}
void lockMotion(){motionEnabled=false;releaseSeen=false;}

WiFiServer server(4210);WiFiClient pythonClient;
bool serverStarted=false,stateValid=false,pythonControl=false,agvFresh=false,agvArmed=false,agvData=false;
uint32_t stateAt=0,bootId=0,seq=0,powerId=0,powerAt=0;
bool powerPending=false,powerEnable=false;
uint32_t ownerId=0,ownerAt=0,hostAt=0,ownerNoticeAt=0;
bool ownerPending=false,ownerTarget=false,host11=false,ownerDenied=false;
bool armPoseValid=false;
float joints[6],nodes[5][3];
int pwmL=0,pwmR=0,turnTarget=0;
float turnProgress=0;
char turnState[10]="IDLE";
float agvTravel=0,agvYaw=0,agvSpeed=0;
float armX=0,armY=0,armZ=0;
float shownYaw=0,shownDelta=0,shownTurns=0;int yawValid=0;uint32_t yawAt=0;
long ticksL=0,ticksR=0;int imuPresent=0;
char rx[180],tx[950];size_t rxN=0,txN=0,txOffset=0;
bool rxOverflow=false;uint32_t txStarted=0,lastPacket=0;
bool linkFresh(){return pythonClient&&pythonClient.connected()&&stateValid&&millis()-stateAt<1500;}
bool robotReady(){return linkFresh()&&pythonControl&&agvFresh&&agvArmed;}
bool splashActive(){return oledReady&&millis()-splashAt<1000;}
bool allowed(){return motionEnabled&&robotReady()&&deadman.held()&&page==PAGE_MAIN&&!ownerPending&&!powerPending&&!armCenterRequired&&!splashActive();}
bool driveAllowed(){return allowed()&&controlMode!=0;}
bool yawFresh(){return agvFresh&&yawValid&&millis()-yawAt<350;}
bool hostReady(){return linkFresh()&&host11&&millis()-hostAt<500;}
bool poseFresh(){return hostReady()&&agvData&&armPoseValid;}
bool turnMode(){return controlMode==1||controlMode==3;}
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
void requestPower(bool enable){
  if(!linkFresh()||!pythonControl||!agvFresh||!deadman.released()||!allCentered()||powerPending||ownerPending)return;
  if(enable==agvArmed)return;
  powerEnable=enable;powerPending=true;++powerId;powerAt=millis();lockMotion();
}
void requestOwner(){
  if(!hostReady()||!deadman.released()||!allCentered()||ownerPending||powerPending)return;
  ownerTarget=!pythonControl;ownerPending=true;ownerDenied=false;
  if(++ownerId==0)++ownerId;
  ownerAt=millis();lockMotion();
}
void closePython(const char* reason){
  if(pythonClient)pythonClient.stop();
  stateValid=pythonControl=agvFresh=agvArmed=powerPending=false;yawValid=0;lockMotion();
  agvData=ownerPending=host11=ownerDenied=armPoseValid=false;
  strcpy(turnState,"IDLE");
  rxN=txN=txOffset=0;rxOverflow=false;if(reason)Serial.println(reason);
}
void parsePython(){
  if(!strncmp(rx,"YAW:",4)){
    float yaw,delta,turns;int valid,n=0;
    if(sscanf(rx+4,"%f,%f,%f,%d%n",&yaw,&delta,&turns,&valid,&n)==4&&n==(int)strlen(rx+4)&&isfinite(yaw)&&isfinite(delta)&&isfinite(turns)){
      shownYaw=yaw;shownDelta=delta;shownTurns=turns;yawValid=valid==1;yawAt=millis();
    }return;
  }
  if(!strcmp(rx,"CTRL:1")||!strcmp(rx,"CTRL:0")){
    bool owner=rx[5]=='1';if(owner!=pythonControl)lockMotion();pythonControl=owner;return;
  }
  if(!strncmp(rx,"OWNER:",6)){
    unsigned long boot,id;int owner,n=0;
    if(sscanf(rx+6,"%lu,%lu,%d%n",&boot,&id,&owner,&n)==3&&n==(int)strlen(rx+6)&&boot==bootId&&ownerPending&&id==ownerId){
      ownerPending=false;pythonControl=owner==1;ownerDenied=pythonControl!=ownerTarget;ownerNoticeAt=millis();lockMotion();
    }return;
  }
  if(!strncmp(rx,"COCO11:",7)){
    int valid,l,r,n=0;
    if(sscanf(rx+7,"%d,%d,%d%n",&valid,&l,&r,&n)==3&&n==(int)strlen(rx+7)){
      host11=true;hostAt=millis();if(!valid)armPoseValid=false;pwmL=l;pwmR=r;
    }return;
  }
  if(!strncmp(rx,"POSE:",5)){
    const char* next=rx+5;
    float pose[21];
    for(int i=0;i<21;i++){
      char* end;pose[i]=strtof(next,&end);
      if(end==next||!isfinite(pose[i])||*end!=(i==20?'\0':',')){armPoseValid=false;return;}
      next=end+1;
    }
    memcpy(joints,pose,sizeof(joints));memcpy(nodes,pose+6,sizeof(nodes));armPoseValid=true;return;
  }
  if(!strncmp(rx,"TURN:",5)){
    char state[10];int target,n=0;float progress;
    if(sscanf(rx+5,"%9[^,],%d,%f%n",state,&target,&progress,&n)==3&&n==(int)strlen(rx+5)&&isfinite(progress)){
      strcpy(turnState,state);turnTarget=target;turnProgress=progress;
    }return;
  }
  if(!strncmp(rx,"STATE:",6)){
    float x,y,z,a,b,g,v;int fresh,armed,fault,n=0;
    if(sscanf(rx+6,"%f,%f,%f,%f,%f,%f,%f,%d,%d,%d%n",&x,&y,&z,&a,&b,&g,&v,&fresh,&armed,&fault,&n)==10&&n==(int)strlen(rx+6)&&isfinite(x)&&isfinite(y)&&isfinite(z)&&isfinite(v)){
      armX=x;armY=y;armZ=z;stateValid=true;stateAt=millis();agvData=fresh==1;agvFresh=agvData&&fault==0;agvArmed=armed==1;
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
  int budget=512;
  while(pythonClient.available()&&budget-->0){
    char c=pythonClient.read();
    if(c=='\n'||c=='\r'){
      if(rxN&&!rxOverflow){rx[rxN]=0;parsePython();}
      rxN=0;rxOverflow=false;
    }else if(!rxOverflow){if(rxN<sizeof(rx)-1)rx[rxN++]=c;else{rxOverflow=true;stateValid=false;lockMotion();}}
  }
  if(!txN&&millis()-lastPacket>=50){
    lastPacket=millis();bool enabled=allowed(),driving=driveAllowed();
    int n=snprintf(tx,sizeof(tx),
      "{\"type\":\"coco\",\"control_style\":\"manual90\",\"boot_id\":%lu,\"seq\":%lu,"
      "\"mode\":\"%c\",\"menu_open\":%d,\"debug_open\":%d,\"deadman\":%d,\"motion_enabled\":%d,"
      "\"arm_filtered_x\":%d,\"arm_filtered_y\":%d,\"agv_filtered_x\":%d,\"agv_filtered_y\":%d,"
      "\"arm_joint_index\":%d,"
      "\"drive\":%d,\"steer\":%d,\"arm_mode\":\"%s\",\"encoder_mode\":\"%s\","
      "\"distance_cm\":0,\"turn_deg\":0,\"turn_jog\":0,\"z_change_mm\":%.2f,\"grip_change_pct\":%.2f,"
      "\"speed_level\":%d,\"power_request_id\":%lu,\"power_pending\":%d,\"power_enable\":%d,"
      "\"coco_os\":11,\"arm_speed_level\":%d,\"agv_speed_level\":%d,\"turn_step_deg\":%d,"
      "\"owner_request_id\":%lu,\"owner_pending\":%d,\"owner_target\":\"%s\"}\n",
      (unsigned long)bootId,(unsigned long)++seq,'A'+controlMode,page==PAGE_MENU,page==PAGE_DEBUG,
      deadman.held(),enabled,controlMode==3?0:axes[0].output,controlMode==3?0:axes[1].output,axes[2].output,axes[3].output,
      controlMode==3?-1:selectedArmJoint,driving?driveRequest():0,
      driving&&(!turnMode()||!preciseTurnEnabled||hostReady())?steerRequest():0,rotationMode?"RX_RY":"XY",gripMode?"GRIP":"Z",
      controlMode==3?0:zChange,controlMode==3?0:gripChange,agvSpeedSetting,(unsigned long)powerId,powerPending,powerEnable,
      armSpeedSetting,agvSpeedSetting,turnMode()&&preciseTurnEnabled?TURN_STEPS[turnStepIndex]:0,
      (unsigned long)ownerId,ownerPending,ownerTarget?"COCO":"PC");
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
  if(ownerPending&&millis()-ownerAt>2500){ownerPending=false;ownerDenied=true;ownerNoticeAt=millis();lockMotion();}
}

void updateControls(int clicks,uint32_t now){
  if(splashActive()){lockMotion();return;}
  if(!deadman.held()||page!=PAGE_MAIN||!robotReady())motionEnabled=false;
  if(page==PAGE_DEBUG){
    if(deadman.rose)debugTab=(debugTab+1)%6;
    if(agvButton.fell){page=PAGE_MENU;navigationReady=false;lockMotion();}return;
  }
  if(page==PAGE_MAIN&&deadman.released()&&agvButton.held()&&now-agvButton.pressed>=600){
    page=PAGE_MENU;selected=ITEM_OWNER;menuWaitRelease=true;navigationReady=false;lockMotion();return;
  }
  if(page==PAGE_MENU){
    if(menuWaitRelease){if(agvButton.released())menuWaitRelease=false;return;}
    if(agvButton.fell){page=PAGE_MAIN;lockMotion();return;}
    if(ownerPending||powerPending)return;
    int nav=axes[MENU_AXIS].output*MENU_SIGN;
    if(abs(nav)<15)navigationReady=true;
    if(abs(nav)>55&&(navigationReady||now-navRepeatAt>300)){
      selected=wrapIndex(selected+(nav>0?-1:1),ITEM_COUNT);navigationReady=false;navRepeatAt=now;
    }
    if(!clicks)return;
    switch(selected){
      case ITEM_OWNER:requestOwner();break;
      case ITEM_POWER:requestPower(clicks>0);break;
      case ITEM_MODE:controlMode=wrapIndex(controlMode+clicks,4);break;
      case ITEM_ARM_SPEED:if(hostReady())armSpeedSetting=constrain(armSpeedSetting+clicks,0,2);break;
      case ITEM_AGV_SPEED:agvSpeedSetting=constrain(agvSpeedSetting+clicks,0,2);break;
      case ITEM_TURN:if(hostReady())preciseTurnEnabled=clicks>0;break;
      case ITEM_FEEL:armFeel=constrain(armFeel+clicks,0,1);break;
      case ITEM_DEBUG:page=PAGE_DEBUG;debugTab=0;break;
      case ITEM_RESET:zChange=0;gripChange=0;break;
    }return;
  }
  if(!robotReady())releaseSeen=false;
  if(deadman.released()&&robotReady())releaseSeen=true;
  if(deadman.rose){motionEnabled=releaseSeen&&allCentered()&&robotReady()&&!ownerPending&&!powerPending;releaseSeen=false;}
  if(controlMode==3)armCenterRequired=false;
  if((controlMode==1||controlMode==2)&&armButton.rose){rotationMode=!rotationMode;armCenterRequired=true;}
  if(axes[0].centered()&&axes[1].centered())armCenterRequired=false;
  if(controlMode==3){
    if(deadman.released()&&clicks)turnStepIndex=wrapIndex(turnStepIndex+clicks,3);
    return;
  }
  if(encoderButton.rose&&deadman.released()&&!agvArmed){requestPower(true);return;}
  if(controlMode>0&&encoderButton.rose){gripMode=!gripMode;clicks=0;}
  if(controlMode==0&&clicks)selectedArmJoint=wrapIndex(selectedArmJoint+clicks,6);
  else if(allowed()&&clicks){
    float delta=clicks*ARM_ENCODER_STEPS[armSpeedSetting][armFeel];
    if(gripMode)gripChange+=delta;else zChange+=delta;
  }
}

void textAt(int x,int y,const char* text){
  oled.setCursor(x,y);
  oled.print(text);
}
void lineAt(int y,const char* text){textAt(0,y,text);}
void header(const char* title){
  oled.clearDisplay();oled.setTextSize(1);oled.setTextColor(SSD1306_WHITE);
  lineAt(0,title);oled.drawFastHLine(0,10,128,SSD1306_WHITE);
}

void micro(int x,int y,const char* text,uint16_t color=SSD1306_WHITE){
  // Each octal digit is one row of the 3x5 glyph, top to bottom.
  static const char glyphs[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-+./:%";
  static const uint16_t rows[]={
    025755,065656,034443,065556,074647,074644,034757,055755,072227,011153,055655,044447,057755,
    057775,025552,065644,025573,065655,034216,072222,055557,055552,055775,055255,055222,071247,
    075557,026227,071247,071317,055711,074717,074757,071111,075757,075717,
    000700,002720,000002,011244,002020,051245
  };
  static_assert(sizeof(rows)/sizeof(rows[0])==sizeof(glyphs)-1,"Incomplete OLED font");
  for(;*text;++text,x+=4){
    char c=*text>='a'&&*text<='z'?*text-'a'+'A':*text;
    const char* glyph=strchr(glyphs,c);
    if(!glyph)continue;
    uint16_t bits=rows[glyph-glyphs];
    for(int row=0;row<5;row++)
      for(int col=0;col<3;col++)
        if(bits&(1u<<((4-row)*3+2-col)))oled.drawPixel(x+col,y+row,color);
  }
}
void badge(int x,int w,const char* label,bool filled){
  if(filled)oled.fillRect(x,1,w,7,SSD1306_WHITE);
  else oled.drawRect(x,1,w,7,SSD1306_WHITE);
  micro(x+(w-4*strlen(label)+1)/2,2,label,filled?SSD1306_BLACK:SSD1306_WHITE);
}
void topBar(const char* title){
  header(title);
  badge(44,20,ownerPending?"WAIT":!linkFresh()?"--":pythonControl?"COCO":"PC",linkFresh()&&pythonControl&&!ownerPending);
  badge(66,16,!linkFresh()||!agvData?"--":agvArmed?"ON":"OFF",linkFresh()&&agvData&&agvArmed);
  badge(84,20,allowed()?"LIVE":"LOCK",allowed());
  if(linkFresh())oled.fillCircle(108,4,1,SSD1306_WHITE);
  if(WiFi.status()==WL_CONNECTED){
    long rssi=WiFi.RSSI();
    int level=rssi>=-55?4:rssi>=-67?3:rssi>=-78?2:1;
    for(int i=0;i<4;i++){
      int h=i*2+2;
      if(i<level)oled.fillRect(113+i*4,9-h,3,h,SSD1306_WHITE);
      else oled.drawFastHLine(113+i*4,8,3,SSD1306_WHITE);
    }
  }
}
void bar(int x,int y,int w,int h,float value){
  oled.drawRect(x,y,w,h,SSD1306_WHITE);
  int filled=lroundf(constrain(value,0.0f,1.0f)*(w-2));
  if(filled>0)oled.fillRect(x+1,y+1,filled,h-2,SSD1306_WHITE);
}
void motorBar(int x,int y,int w,int pwm){
  const int center=x+w/2;
  oled.drawRect(x,y,w,5,SSD1306_WHITE);
  oled.drawFastVLine(center,y+1,3,SSD1306_WHITE);
  int span=pwm<0?center-x-1:x+w-center-2;
  int fill=lroundf(constrain(abs(pwm)/135.0f,0.0f,1.0f)*span);
  if(fill>0)oled.fillRect(pwm<0?center-fill:center+1,y+1,fill,3,SSD1306_WHITE);
}
void radar(int x,int y,int r){
  float angle=(millis()%1800)*2.0f*PI/1800.0f;
  oled.drawCircle(x,y,r,SSD1306_WHITE);oled.drawCircle(x,y,r/2,SSD1306_WHITE);
  oled.drawFastHLine(x-r,y,2*r+1,SSD1306_WHITE);oled.drawFastVLine(x,y-r,2*r+1,SSD1306_WHITE);
  oled.drawLine(x,y,x+lroundf(r*cosf(angle)),y+lroundf(r*sinf(angle)),SSD1306_WHITE);
}
void drawArm(int left,int width,int highlight=-1){
  micro(left+2,13,poseFresh()?"ARM EST":"ARM --");
  if(!poseFresh())return;
  // The PC sends FK points in millimetres. Render their isometric projection.
  float u[5],v[5],floorV[5];
  for(int i=0;i<5;i++){
    float x=nodes[i][0],y=nodes[i][1],z=nodes[i][2];
    u[i]=(x-y)*.866f;floorV[i]=(x+y)*.5f;v[i]=floorV[i]-z;
  }
  float uMin=0,uMax=0,vMin=0,vMax=0;
  for(int i=0;i<5;i++){
    uMin=fminf(uMin,u[i]);uMax=fmaxf(uMax,u[i]);
    vMin=fminf(vMin,fminf(v[i],floorV[i]));vMax=fmaxf(vMax,fmaxf(v[i],floorV[i]));
  }
  float scale=fminf(.13f,fminf((width-12)/fmaxf(1,uMax-uMin),30/fmaxf(1,vMax-vMin)));
  int x[5],y[5],floorY[5];
  for(int i=0;i<5;i++){
    x[i]=left+width/2+lroundf((u[i]-(uMin+uMax)/2)*scale);
    y[i]=38+lroundf((v[i]-(vMin+vMax)/2)*scale);
    floorY[i]=38+lroundf((floorV[i]-(vMin+vMax)/2)*scale);
  }
  for(int i=0;i<4;i++){
    for(int dot=0;dot<8;dot+=2)
      oled.drawPixel(x[i]+(x[i+1]-x[i])*dot/8,floorY[i]+(floorY[i+1]-floorY[i])*dot/8,SSD1306_WHITE);
    oled.drawLine(x[i],y[i],x[i+1],y[i+1],SSD1306_WHITE);
    oled.drawLine(x[i]+1,y[i],x[i+1]+1,y[i+1],SSD1306_WHITE);
  }
  oled.fillRect(x[0]-3,y[0]-1,7,3,SSD1306_WHITE);
  for(int i=1;i<4;i++)oled.fillCircle(x[i],y[i],2,SSD1306_WHITE);
  int dx=x[4]-x[3],dy=y[4]-y[3];
  float length=hypotf(dx,dy);
  if(length>1){
    int reachX=lroundf(3*dx/length),reachY=lroundf(3*dy/length);
    float opening=1.0f+2.0f*(joints[5]-10)/65;
    int jawX=lroundf(-dy*opening/length),jawY=lroundf(dx*opening/length);
    oled.drawLine(x[4]-jawX,y[4]-jawY,x[4]+jawX,y[4]+jawY,SSD1306_WHITE);
    oled.drawLine(x[4]-jawX,y[4]-jawY,x[4]-jawX+reachX,y[4]-jawY+reachY,SSD1306_WHITE);
    oled.drawLine(x[4]+jawX,y[4]+jawY,x[4]+jawX+reachX,y[4]+jawY+reachY,SSD1306_WHITE);
  }else oled.fillCircle(x[4],y[4],1,SSD1306_WHITE);
  const int jointNode[]={0,1,2,3,3,4};
  if(highlight>=0)oled.drawCircle(x[jointNode[highlight]],y[jointNode[highlight]],4,SSD1306_WHITE);
}
void drawXYZ(){
  oled.drawFastVLine(72,11,53,SSD1306_WHITE);
  char label[18];
  snprintf(label,sizeof(label),"%s / %s",rotationMode?"RxRy":"XY",gripMode?"GRIP":"Z");
  micro(76,13,label);micro(115,13,"MM");
  const float values[]={armX,armY,armZ};
  for(int i=0;i<3;i++){
    if(poseFresh())snprintf(label,sizeof(label),"%c %+.0f",'X'+i,values[i]);
    else snprintf(label,sizeof(label),"%c --",'X'+i);
    textAt(77,23+i*10,label);
  }
  micro(76,56,SPEED_NAMES[armSpeedSetting]);
  if(poseFresh())bar(98,55,27,7,(joints[5]-10)/65);
}
void drawJoints(){
  drawArm(0,72,selectedArmJoint);
  oled.drawFastVLine(72,11,53,SSD1306_WHITE);
  char label[20];
  snprintf(label,sizeof(label),"J%d %s",selectedArmJoint+1,JOINT_NAMES[selectedArmJoint]);
  micro(77,14,label);
  if(poseFresh())snprintf(label,sizeof(label),"%3.0f",joints[selectedArmJoint]);else strcpy(label,"--");
  oled.setTextSize(2);textAt(77,23,label);oled.setTextSize(1);micro(115,31,"DEG");
  if(poseFresh()){
    float value=selectedArmJoint==5?(joints[5]-10)/65:joints[selectedArmJoint]/180;
    bar(77,42,48,8,value);
  }
  micro(77,56,SPEED_NAMES[armSpeedSetting]);
}
void drawAGV(int left,int width){
  char label[24];
  micro(left,13,agvFresh?"AGV":"AGV --");micro(left+width-16,13,SPEED_NAMES[agvSpeedSetting]);
  if(linkFresh()&&agvFresh)snprintf(label,sizeof(label),"D %+.1f CM",agvTravel);else strcpy(label,"D -- CM");
  micro(left,22,label);
  if(linkFresh()&&agvFresh)snprintf(label,sizeof(label),"V %+.0f MM/S",agvSpeed);else strcpy(label,"V -- MM/S");
  micro(left,30,label);
  if(controlMode==1){
    if(yawFresh())snprintf(label,sizeof(label),"YAW %+.0f DEG",fmodf(shownYaw,360));else strcpy(label,"YAW --");
    micro(left,36,label);
  }
  micro(left,42,"L -");micro(left,49,"R -");
  micro(left+width-3,42,"+");micro(left+width-3,49,"+");
  if(hostReady()&&agvFresh){
    motorBar(left+13,42,width-18,pwmL);motorBar(left+13,49,width-18,pwmR);
  }else{micro(left+width/2-3,42,"--");micro(left+width/2-3,49,"--");}

  const int step=TURN_STEPS[turnStepIndex];
  const bool currentTurn=hostReady()&&abs(turnTarget)==step;
  const bool running=currentTurn&&!strcmp(turnState,"RUN");
  if(!preciseTurnEnabled){
    if(controlMode==3)snprintf(label,sizeof(label),"TURN OFF %d",step);else strcpy(label,"TURN OFF");
  }else if(running){
    snprintf(label,sizeof(label),"%c %.0f/%d",turnTarget>0?'L':'R',fabsf(turnProgress),step);
    bar(left+41,57,width-41,5,turnProgress/turnTarget);
  }else if(!yawFresh())snprintf(label,sizeof(label),"%d NO IMU",step);
  else if(currentTurn&&(!strcmp(turnState,"DONE")||!strcmp(turnState,"CANCEL")))snprintf(label,sizeof(label),"%s %d",turnState,step);
  else snprintf(label,sizeof(label),"TURN %d DEG",step);
  micro(left,57,label);
}
void drawCompass(){
  const int x=25,y=36,r=16;
  oled.drawCircle(x,y,r,SSD1306_WHITE);
  for(int i=0;i<4;i++){
    int dx=i%2?(i==1?r:-r):0,dy=i%2?0:(i==0?-r:r);
    oled.drawLine(x+dx,y+dy,x+dx*3/4,y+dy*3/4,SSD1306_WHITE);
  }
  micro(18,13,"REF");
  if(yawFresh()){
    float angle=shownYaw*PI/180.0f;
    oled.drawLine(x,y,x-lroundf(13*sinf(angle)),y-lroundf(13*cosf(angle)),SSD1306_WHITE);
    oled.fillCircle(x,y,2,SSD1306_WHITE);
    char label[15];snprintf(label,sizeof(label),"%+.0f DEG",fmodf(shownYaw,360));
    micro(9,56,label);
  }else micro(11,56,"YAW --");
}
void drawMain(){
  topBar(MODE_NAMES[controlMode]);
  if(!linkFresh()){
    radar(25,33,15);
    micro(2,55,"HOLD B: MENU");
    if(controlMode==2){drawXYZ();return;}
    textAt(49,17,"WAIT FOR PC");
    if(WiFi.status()==WL_CONNECTED){micro(49,31,WiFi.localIP().toString().c_str());micro(49,42,"TCP 4210");}
    else micro(49,32,"CONNECT WIFI");
    return;
  }
  switch(controlMode){
    case 0:drawJoints();break;
    case 1:
      drawAGV(2,59);oled.drawFastVLine(63,11,53,SSD1306_WHITE);drawArm(64,64);
      if(poseFresh()){
        char label[16];snprintf(label,sizeof(label),"Z %+.0f MM",armZ);micro(66,57,label);
      }break;
    case 2:drawArm(0,72);drawXYZ();break;
    case 3:drawCompass();oled.drawFastVLine(49,11,53,SSD1306_WHITE);drawAGV(53,73);break;
  }
}
void drawMenu(){
  topBar("MENU");char rows[ITEM_COUNT][24];
  snprintf(rows[ITEM_OWNER],24,"%s",ownerPending?"CONTROL: WAIT":ownerDenied&&millis()-ownerNoticeAt<1800?"CONTROL: RETRY":!hostReady()?"CONTROL: OFFLINE":pythonControl?"GIVE TO PC":"TAKE CTRL");
  snprintf(rows[ITEM_MODE],24,"MODE %c: %s",'A'+controlMode,MODE_NAMES[controlMode]);
  snprintf(rows[ITEM_ARM_SPEED],24,"ARM SPEED: %s",hostReady()?SPEED_NAMES[armSpeedSetting]:"--");
  snprintf(rows[ITEM_AGV_SPEED],24,"AGV SPEED: %s",SPEED_NAMES[agvSpeedSetting]);
  snprintf(rows[ITEM_TURN],24,"PRECISE TURN: %s",!hostReady()?"--":preciseTurnEnabled?"ON":"OFF");
  const char* relayState=powerPending?"WAIT":!agvFresh||!pythonControl?"LOCK":agvArmed?"ON":"OFF";
  snprintf(rows[ITEM_POWER],24,"RELAYS: %s",relayState);
  snprintf(rows[ITEM_FEEL],24,"ENCODER: %s",armFeel?"NORMAL":"FINE");
  strcpy(rows[ITEM_DEBUG],"DIAGNOSTICS");strcpy(rows[ITEM_RESET],"RESET ARM INPUT");
  int first=constrain(selected-2,0,ITEM_COUNT-5);
  for(int i=first;i<first+5;i++){
    int y=12+(i-first)*8;
    if(i==selected)oled.fillRect(0,y,128,8,SSD1306_WHITE);
    micro(3,y+1,rows[i],i==selected?SSD1306_BLACK:SSD1306_WHITE);
  }
  micro(2,57,"B: EXIT  TURN ENC: APPLY");
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
      snprintf(t,sizeof(t),"D:%d R:%d",driveAllowed()?driveRequest():0,driveAllowed()?steerRequest():0);lineAt(32,t);lineAt(42,"Trigger 70% / off 45%");break;
    case 2:
      snprintf(t,sizeof(t),"SW A:%d B:%d E:%d",armButton.down,agvButton.down,encoderButton.down);lineAt(12,t);
      snprintf(t,sizeof(t),"ENC:%ld d:%d",(long)encoderTotal,millis()-encoderDeltaAt<1000?encoderDelta:0);lineAt(22,t);
      snprintf(t,sizeof(t),"DM:%d",deadman.held());lineAt(32,t);lineAt(42,"Outputs: blocked");break;
    case 3:
      if(linkFresh()&&agvFresh){
        snprintf(t,sizeof(t),"Power:%d V:%.0fmm/s",agvArmed,agvSpeed);lineAt(12,t);
        snprintf(t,sizeof(t),"D:%.1fcm R:%.1f",agvTravel,agvYaw);lineAt(22,t);
        snprintf(t,sizeof(t),"L:%ld R:%ld",ticksL,ticksR);lineAt(32,t);
        snprintf(t,sizeof(t),"IMU OK:%d d:%+.1f",yawValid&&millis()-yawAt<1500,shownDelta);lineAt(42,t);
      }else{lineAt(12,"AGV NO RECENT DATA");lineAt(22,"D:? R:? Speed:?");}break;
    case 4:
      snprintf(t,sizeof(t),"X:%+.1f Y:%+.1f",armX,armY);lineAt(12,poseFresh()?t:"X:-- Y:--");
      snprintf(t,sizeof(t),"Z:%+.1f mm",armZ);lineAt(22,poseFresh()?t:"Z:--");
      snprintf(t,sizeof(t),"Mode:%c %s",'A'+controlMode,MODE_NAMES[controlMode]);lineAt(32,t);
      snprintf(t,sizeof(t),"Joint:%s Stick:%+d",JOINT_NAMES[selectedArmJoint],axes[1].output);lineAt(42,t);break;
    case 5:
      snprintf(t,sizeof(t),"WiFi:%s",WiFi.status()==WL_CONNECTED?"ON":"OFF");lineAt(12,t);
      if(WiFi.status()==WL_CONNECTED)lineAt(22,WiFi.localIP().toString().c_str());else lineAt(22,"IP: waiting");
      snprintf(t,sizeof(t),"TCP:4210 Python:%d",linkFresh());lineAt(32,t);
      snprintf(t,sizeof(t),"Owner:%s",pythonControl?"COCO":"PC");lineAt(42,t);break;
  }lineAt(56,"DM: next B: return");
}
void drawSplash(){
  oled.clearDisplay();oled.setTextColor(SSD1306_WHITE);
  for(int y=0;y<40;y++)for(int x=0;x<40;x++){
    int sx=x*32/40,sy=y*32/40;if(pgm_read_byte(&COCO_BITMAP[sy*4+sx/8])&(0x80>>(sx%8)))oled.drawPixel(44+x,y,SSD1306_WHITE);
  }
  oled.setTextSize(1);textAt(31,43,"COCO OS 1.1");
  bar(19,55,90,5,(millis()-splashAt)/1000.0f);
}
void updateDisplay(){
  if(!oledReady||oledOffset<1024)return;
  if(splashActive())drawSplash();else if(page==PAGE_MENU)drawMenu();else if(page==PAGE_DEBUG)drawDebug();else drawMain();
  // SSD1306 horizontal addressing. Keep the framebuffer fixed until all chunks are sent.
  const uint8_t addressWindow[]={0x00,0x21,0,127,0x22,0,7};
  Wire.beginTransmission(oledAddress);Wire.write(addressWindow,sizeof(addressWindow));
  if(Wire.endTransmission()==0)oledOffset=0;
}
void serviceDisplay(){
  if(oledOffset>=1024)return;
  constexpr size_t chunk=64;
  Wire.beginTransmission(oledAddress);Wire.write(0x40);Wire.write(oled.getBuffer()+oledOffset,chunk);
  if(Wire.endTransmission()==0)oledOffset+=chunk;else oledOffset=1024;
}
void startDisplay(){
  Wire.begin(SDA_PIN,SCL_PIN,400000);Wire.setTimeOut(5);
  for(uint8_t addr:{0x3C,0x3D}){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission()==0&&oled.begin(SSD1306_SWITCHCAPVCC,addr,false,false)){
      oledReady=true;oledAddress=addr;break;
    }
  }
  if(oledReady){Wire.setClock(400000);oled.setTextWrap(false);}else Serial.println("OLED not found");
}
void setup(){
  Serial.begin(115200);bootId=esp_random();armButton.begin();agvButton.begin();encoderButton.begin();deadman.begin();
  pinMode(ENC_CLK,INPUT_PULLUP);pinMode(ENC_DT,INPUT_PULLUP);encoderPrevious=(digitalRead(ENC_CLK)<<1)|digitalRead(ENC_DT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK),encoderISR,CHANGE);attachInterrupt(digitalPinToInterrupt(ENC_DT),encoderISR,CHANGE);
  analogReadResolution(12);WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setAutoReconnect(true);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  startDisplay();calibrateJoysticks();encoderConsumed=readEncoder();splashAt=millis();
  Serial.println("COCO OS 1.1 / TCP 4210 / hold B: menu");
}
void loop(){
  static uint32_t inputAt=0,displayAt=0,serialAt=0,retryAt=0;static bool previousWifi=false;
  uint32_t now=millis();armButton.update(now);agvButton.update(now);encoderButton.update(now);deadman.update(now);
  if(now-inputAt>=5){inputAt=now;for(int i=0;i<4;i++)axes[i].update();}
  updateControls(consumeEncoder(),millis());serviceTCP();
  bool wifi=WiFi.status()==WL_CONNECTED;
  if(wifi&&!previousWifi){Serial.print("COCO IP: ");Serial.println(WiFi.localIP());}previousWifi=wifi;
  if(!wifi&&now-retryAt>=15000){retryAt=now;WiFi.reconnect();}
  if(now-displayAt>=100){displayAt=now;updateDisplay();}
  serviceDisplay();
  if(now-serialAt>=500 && Serial && Serial.availableForWrite()>160){serialAt=now;Serial.printf("PC:%d OwnerCOCO:%d AGV:%d Armed:%d DM:%d LIVE:%d D:%d R:%d\n",linkFresh(),pythonControl,agvFresh,agvArmed,deadman.held(),allowed(),driveAllowed()?driveRequest():0,driveAllowed()?steerRequest():0);}
  delay(1);
}
