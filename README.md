# Techno-MCU-Home-automation-v8
/*
 TechnoMCU ULTIMATE - Ultimate Advanced Control System
 - Advanced 3-button control for 2 servos, 2 relays, and sharpy mode
 - Zero latency servo control
 - Spectacular animations without performance impact
 - By Raktim Hazra
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Servo.h>
#include <EEPROM.h>

// Pin assignments
#define SERVO_X_PIN 14
#define SERVO_Y_PIN 12
#define RELAY_PIN    13
#define RELAY2_PIN   16  // D0 - New relay
#define BTN_C_PIN    15  // D8 - New button
#define SDA_PIN       4
#define SCL_PIN       5
#define OLED_ADDR   0x3C

#define BTN_A_PIN 0
#define BTN_B_PIN 2

// WiFi AP
const char* apSsid = "TechnoMCU_Ultimate";
const char* apPass = "raktim52";
IPAddress apIP(192,168,4,1);

// Globals
Servo sX, sY;
ESP8266WebServer server(80);
DNSServer dnsServer;

volatile int servoXpos = 90;
volatile int servoYpos = 90;
volatile bool relayState = false;
volatile bool relay2State = false;  // New relay state

// Operation Modes
enum OperationMode { 
  MODE_MANUAL = 0, 
  MODE_SHARPY = 1, 
  MODE_RELAY = 2 
};

OperationMode operationMode = MODE_MANUAL;

// Display States
enum DisplayState { 
  STATE_BOOT = 0,
  STATE_MAIN = 1,
  STATE_MENU = 2,
  STATE_MANUAL = 3,
  STATE_SHARPY = 4,
  STATE_RELAY = 5
};

DisplayState displayState = STATE_BOOT;

uint8_t ssd_buf[1024];
unsigned long lastOledUpdate = 0;
unsigned long oledInterval = 80;

// Button handling - ULTRA RESPONSIVE
unsigned long btnAPressTime = 0;
unsigned long btnBPressTime = 0;
unsigned long btnCPressTime = 0;  // New button
bool btnAPressed = false;
bool btnBPressed = false;
bool btnCPressed = false;  // New button
bool bothButtonsPressed = false;
unsigned long bothButtonsStartTime = 0;

// Enhanced control variables
unsigned long btnHoldStartTime = 0;
bool rapidMode = false;
int rapidIncrement = 3;
unsigned long lastServoUpdate = 0;
#define SERVO_UPDATE_INTERVAL 20 // Ultra responsive

// Animation variables - SPECTACULAR
unsigned long bootStartTime = 0;
bool bootComplete = false;
#define BOOT_DURATION 5000

unsigned long menuScrollTime = 0;
int currentMenu = 0;
const char* menuItems[] = {"MAIN", "MANUAL", "SHARPY", "RELAY", "INFO"};
int menuCount = 5;
int menuScrollOffset = 0;
bool menuScrolling = false;

unsigned long sharpyPatternTime = 0;
int sharpyPattern = 0;
bool sharpyActive = false;

// Particle system for animations
struct Particle {
  float x, y;
  float vx, vy;
  int life;
};
Particle particles[20];

// EEPROM
#define EEPROM_SIZE 64
#define ADDR_MAGIC 0
#define ADDR_RELAY 1
#define ADDR_RELAY2 2  // New relay EEPROM address
#define ADDR_SERVO_X 3
#define ADDR_SERVO_Y 4
#define ADDR_MODE 5
const uint8_t MAGIC = 0xA5;

// Function declarations
void setOperationMode(OperationMode newMode);
void updateOLED();
void applyServos();
void applyRelay();
void handleButtons();
void updateSharpyMode();
void saveState();
void loadState();
bool isCaptivePortal();
String webpage();
void initParticles();
void updateParticles();

// Minimal I2C for SSD1306 - Optimized for speed
void i2c_beginPins(){
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}

void i2c_start(){
  digitalWrite(SDA_PIN,HIGH); digitalWrite(SCL_PIN,HIGH);
  digitalWrite(SDA_PIN,LOW);  digitalWrite(SCL_PIN,LOW);
}

void i2c_stop(){
  digitalWrite(SDA_PIN,LOW); digitalWrite(SCL_PIN,HIGH);
  digitalWrite(SDA_PIN,HIGH);
}

void i2c_writeByte(uint8_t b){
  for(uint8_t i=0;i<8;i++){
    digitalWrite(SDA_PIN,(b & 0x80)?HIGH:LOW);
    digitalWrite(SCL_PIN,HIGH);
    delayMicroseconds(1);
    digitalWrite(SCL_PIN,LOW);
    b <<= 1;
  }
  pinMode(SDA_PIN, INPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);
}

void ssd_cmd(uint8_t c){
  i2c_start();
  i2c_writeByte((OLED_ADDR<<1)|0);
  i2c_writeByte(0x00);
  i2c_writeByte(c);
  i2c_stop();
}

void ssd_data(uint8_t d){
  i2c_start();
  i2c_writeByte((OLED_ADDR<<1)|0);
  i2c_writeByte(0x40);
  i2c_writeByte(d);
  i2c_stop();
}

// SSD1306 Init
void ssd_init(){
  i2c_beginPins();
  delay(50);
  ssd_cmd(0xAE);
  ssd_cmd(0xD5); ssd_cmd(0x80);
  ssd_cmd(0xA8); ssd_cmd(0x3F);
  ssd_cmd(0xD3); ssd_cmd(0x00);
  ssd_cmd(0x40);
  ssd_cmd(0x8D); ssd_cmd(0x14);
  ssd_cmd(0x20); ssd_cmd(0x00);
  ssd_cmd(0xA1);
  ssd_cmd(0xC8);
  ssd_cmd(0xDA); ssd_cmd(0x12);
  ssd_cmd(0x81); ssd_cmd(0xCF);
  ssd_cmd(0xD9); ssd_cmd(0xF1);
  ssd_cmd(0xDB); ssd_cmd(0x40);
  ssd_cmd(0xA4);
  ssd_cmd(0xA6);
  ssd_cmd(0xAF);
  memset(ssd_buf,0,sizeof(ssd_buf));
}

void ssd_displayBuffer(){
  for(uint8_t p=0;p<8;p++){
    ssd_cmd(0xB0|p);
    ssd_cmd(0x00);
    ssd_cmd(0x10);
    i2c_start();
    i2c_writeByte((OLED_ADDR<<1)|0);
    i2c_writeByte(0x40);
    for(int c=0;c<128;c++){
      i2c_writeByte(ssd_buf[p*128 + c]);
    }
    i2c_stop();
  }
}

void ssd_clearBuffer(){ 
  memset(ssd_buf,0,sizeof(ssd_buf)); 
}

// Font 5x7
const uint8_t font5x7[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x00,0x00,0x00,0x07,0x00,0x07,0x00,
  0x14,0x7F,0x14,0x7F,0x14,0x24,0x2A,0x7F,0x2A,0x12,0x23,0x13,0x08,0x64,0x62,
  0x36,0x49,0x55,0x22,0x50,0x00,0x05,0x03,0x00,0x00,0x00,0x1C,0x22,0x41,0x00,
  0x00,0x41,0x22,0x1C,0x00,0x14,0x08,0x3E,0x08,0x14,0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x60,0x60,0x00,0x00,
  0x20,0x10,0x08,0x04,0x02,0x3E,0x51,0x49,0x45,0x3E,0x00,0x42,0x7F,0x40,0x00,
  0x42,0x61,0x51,0x49,0x46,0x21,0x41,0x45,0x4B,0x31,0x18,0x14,0x12,0x7F,0x10,
  0x27,0x45,0x45,0x45,0x39,0x3C,0x4A,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36,0x06,0x49,0x49,0x29,0x1E,0x00,0x36,0x36,0x00,0x00,
  0x00,0x56,0x36,0x00,0x00,0x08,0x14,0x22,0x41,0x00,0x14,0x14,0x14,0x14,0x14,
  0x00,0x41,0x22,0x14,0x08,0x02,0x01,0x51,0x09,0x06,0x32,0x49,0x79,0x41,0x3E,
  0x7E,0x11,0x11,0x11,0x7E,0x7F,0x49,0x49,0x49,0x36,0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C,0x7F,0x49,0x49,0x49,0x41,0x7F,0x09,0x09,0x09,0x01,
  0x3E,0x41,0x49,0x49,0x7A,0x7F,0x08,0x08,0x08,0x7F,0x00,0x41,0x7F,0x41,0x00,
  0x20,0x40,0x41,0x3F,0x01,0x7F,0x08,0x14,0x22,0x41,0x7F,0x40,0x40,0x40,0x40,
  0x7F,0x02,0x0C,0x02,0x7F,0x7F,0x04,0x08,0x10,0x7F,0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06,0x3E,0x41,0x51,0x21,0x5E,0x7F,0x09,0x19,0x29,0x46,
  0x46,0x49,0x49,0x49,0x31,0x01,0x01,0x7F,0x01,0x01,0x3F,0x40,0x40,0x40,0x3F,
  0x1F,0x20,0x40,0x20,0x1F,0x3F,0x40,0x38,0x40,0x3F,0x63,0x14,0x08,0x14,0x63,
  0x07,0x08,0x70,0x08,0x07,0x61,0x51,0x49,0x45,0x43,0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20,0x00,0x41,0x41,0x7F,0x00,0x04,0x02,0x01,0x02,0x04,
  0x40,0x40,0x40,0x40,0x40,0x00,0x01,0x02,0x04,0x00,0x20,0x54,0x54,0x54,0x78,
  0x7F,0x48,0x44,0x44,0x38,0x38,0x44,0x44,0x44,0x20,0x38,0x44,0x44,0x48,0x7F,
  0x38,0x54,0x54,0x54,0x18,0x08,0x7E,0x09,0x01,0x02,0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78,0x00,0x44,0x7D,0x40,0x00,0x20,0x40,0x44,0x3D,0x00,
  0x7F,0x10,0x28,0x44,0x00,0x00,0x41,0x7F,0x40,0x00,0x7C,0x04,0x18,0x04,0x78,
  0x7C,0x08,0x04,0x04,0x78,0x38,0x44,0x44,0x44,0x38,0x7C,0x14,0x14,0x14,0x08,
  0x08,0x14,0x14,0x18,0x7C,0x7C,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20,0x3C,0x40,0x40,0x20,0x7C,0x1C,0x20,0x40,0x20,0x1C,
  0x3C,0x40,0x30,0x40,0x3C,0x44,0x28,0x10,0x28,0x44,0x0C,0x50,0x50,0x50,0x3C,
  0x44,0x64,0x54,0x4C,0x44,0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x7F,0x00,0x00,
  0x00,0x41,0x36,0x08,0x00,0x10,0x08,0x08,0x10,0x08,0x78,0x46,0x41,0x46,0x78
};

void ssd_drawChar(int col,int page,char c){
  if(c<32||c>127) c='?';
  int idx=(c-32)*5;
  for(int x=0;x<5;x++){
    if(col+x>=0&&col+x<128&&page>=0&&page<8)
      ssd_buf[page*128+(col+x)] = font5x7[idx+x];
  }
  if(col+5>=0&&col+5<128&&page>=0&&page<8) ssd_buf[page*128+(col+5)]=0x00;
}

void ssd_printStr(int x,int page,const char *s){
  int cx=x;
  while(*s && cx<122){ ssd_drawChar(cx,page,*s++); cx+=6; }
}

// Particle system for advanced animations
void initParticles(){
  for(int i=0; i<20; i++){
    particles[i].life = 0;
  }
}

void updateParticles(){
  for(int i=0; i<20; i++){
    if(particles[i].life > 0){
      particles[i].x += particles[i].vx;
      particles[i].y += particles[i].vy;
      particles[i].life--;
      
      // Draw particle
      int px = (int)particles[i].x;
      int py = (int)particles[i].y;
      if(px>=0 && px<128 && py>=0 && py<8){
        ssd_buf[py*128 + px] |= (1 << (particles[i].life % 8));
      }
    }
  }
}

void createParticleBurst(int x, int y, int count){
  for(int i=0; i<20; i++){
    if(particles[i].life == 0){
      particles[i].x = x;
      particles[i].y = y;
      particles[i].vx = (random(100)-50)/50.0;
      particles[i].vy = (random(100)-50)/50.0;
      particles[i].life = random(10,30);
      if(--count <= 0) break;
    }
  }
}

// SPECTACULAR Boot Animation
void showBootAnimation(){
  unsigned long currentTime = millis() - bootStartTime;
  float progress = (float)currentTime / BOOT_DURATION;
  
  ssd_clearBuffer();
  
  // Phase 1: Matrix-style text reveal
  if(progress < 0.3){
    int revealHeight = (progress / 0.3) * 24;
    for(int y=0; y<revealHeight; y++){
      if(y < 8){
        // TECHNO MCU text with digital rain effect
        ssd_printStr(10, y, "TECHNO MCU");
        // Digital rain
        for(int x=0; x<128; x+=4){
          if(random(100) < 30) ssd_buf[y*128 + x] = 0xFF;
        }
      }
    }
  }
  // Phase 2: Creator name with particle explosion
  else if(progress < 0.6){
    ssd_printStr(15, 2, "TECHNO MCU");
    ssd_printStr(25, 4, "ULTIMATE");
    
    // Particle explosion from center
    if((int)(currentTime/100) % 5 == 0){
      createParticleBurst(64, 3, 5);
    }
    updateParticles();
  }
  // Phase 3: Final presentation with advanced effects
  else{
    ssd_printStr(15, 1, "TECHNO MCU");
    ssd_printStr(25, 2, "ULTIMATE");
    ssd_printStr(10, 4, "By Raktim Hazra");
    
    // Animated progress bar with glow effect
    int barWidth = ((progress-0.6)/0.4) * 100;
    for(int i=0; i<barWidth; i++){
      if(14+i < 114){
        int intensity = (sin((i + currentTime/100.0) * 0.5) + 1) * 4;
        ssd_buf[6*128 + 14 + i] = intensity;
        if(i > 5) ssd_buf[6*128 + 14 + i - 5] = intensity/2;
      }
    }
    
    // Rotating cube animation
    int cubeTime = currentTime / 50;
    int cubeX = 100;
    int cubeY = 7;
    for(int i=0; i<4; i++){
      int angle = (cubeTime + i*90) % 360;
      int x = cubeX + 8 * cos(angle * 3.14159 / 180);
      int y = cubeY + 8 * sin(angle * 3.14159 / 180);
      if(x>=0 && x<128 && y>=0 && y<8){
        ssd_buf[y*128 + x] = 0xFF;
      }
    }
    
    updateParticles();
  }
  
  ssd_displayBuffer();
  
  if(currentTime >= BOOT_DURATION){
    bootComplete = true;
    displayState = STATE_MAIN;
    createParticleBurst(64, 3, 20); // Final explosion
  }
}

// Enhanced Menu with 3D effects
void showMenu(){
  unsigned long currentTime = millis();
  
  ssd_clearBuffer();
  
  // Animated 3D header
  ssd_printStr(0, 0, "CONTROL SYSTEM");
  int wave = (currentTime / 200) % 8;
  for(int i=0; i<8; i++){
    if(80+i*6 < 128) {
      int height = (sin((i + wave) * 0.8) + 1) * 2;
      for(int j=0; j<height; j++){
        ssd_buf[j*128 + 80+i*6] = 0xFF;
      }
    }
  }
  
  // 3D menu items with perspective
  int centerX = 64;
  int perspective = sin(currentTime * 0.002) * 3;
  
  for(int i=-2; i<=2; i++){
    int itemIndex = (currentMenu + i + menuCount) % menuCount;
    if(itemIndex >= 0 && itemIndex < menuCount){
      int yPos = 2 + i + perspective;
      int xOffset = abs(i) * 5;
      int alpha = 255 - abs(i) * 80;
      
      if(yPos >= 0 && yPos < 8){
        if(i == 0){
          // Center item - highlighted
          ssd_printStr(centerX - strlen(menuItems[itemIndex])*3 + xOffset, yPos, menuItems[itemIndex]);
          // Glow effect
          for(int x=centerX-20; x<centerX+20; x++){
            if(x>=0 && x<128) ssd_buf[yPos*128 + x] |= 0x18;
          }
        } else {
          // Side items - dimmed
          ssd_printStr(centerX - strlen(menuItems[itemIndex])*3 + xOffset, yPos, menuItems[itemIndex]);
        }
      }
    }
  }
  
  // Animated selection indicator
  int pulse = (currentTime / 300) % 2;
  if(pulse){
    ssd_printStr(10, 7, "< SELECT >");
    createParticleBurst(5, 7, 2);
    createParticleBurst(122, 7, 2);
  }
  
  updateParticles();
  ssd_displayBuffer();
}

// ULTRA RESPONSIVE Main Display
void showMainDisplay(){
  static unsigned long lastAnimTime = 0;
  unsigned long currentTime = millis();
  
  ssd_clearBuffer();
  
  // Animated header with real-time clock
  ssd_printStr(0, 0, "TECHNO MCU");
  
  // Real-time clock (seconds since boot)
  unsigned long uptime = currentTime / 1000;
  char timeStr[10];
  snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", (uptime/60)%60, uptime%60);
  ssd_printStr(90, 0, timeStr);
  
  // 3D Mode indicator with particle trail
  int modeX[] = {30, 60, 90};
  for(int i=0; i<3; i++){
    int intensity = (operationMode == i) ? 0xFF : 0x18;
    const char* modeText = (i==0)?"MAN":(i==1)?"SHP":"RLY";
    
    if(operationMode == i){
      // Active mode - glowing
      ssd_printStr(modeX[i], 1, modeText);
      // Create particles around active mode
      if(currentTime - lastAnimTime > 200){
        createParticleBurst(modeX[i]+8, 2, 3);
        lastAnimTime = currentTime;
      }
    } else {
      // Inactive modes
      ssd_printStr(modeX[i], 1, modeText);
    }
  }
  
  // Real-time servo visualization with smooth interpolation
  int barX = map(servoXpos, 0, 180, 10, 118);
  int barY = map(servoYpos, 0, 180, 4, 7);
  
  // Dynamic crosshair that follows servos
  for(int i=-3; i<=3; i++){
    for(int j=-1; j<=1; j++){
      if(barX+i >=0 && barX+i <128 && barY+j >=0 && barY+j <8){
        int dist = abs(i) + abs(j);
        ssd_buf[(barY+j)*128 + barX+i] = (dist == 0) ? 0xFF : 
                                         (dist <= 2) ? 0xAA : 0x55;
      }
    }
  }
  
  // Live data with smooth transitions
  char dataStr[32];
  snprintf(dataStr, sizeof(dataStr), "X:%3d Y:%3d R1:%s R2:%s", servoXpos, servoYpos, relayState?"ON":"OFF", relay2State?"ON":"OFF");
  ssd_printStr(0, 3, dataStr);
  
  // Spectrum analyzer style bars
  int xBar = map(servoXpos, 0, 180, 0, 40);
  int yBar = map(servoYpos, 0, 180, 0, 40);
  
  for(int i=0; i<xBar; i+=2){
    if(10+i < 50) ssd_buf[5*128 + 10+i] = 0xFF;
  }
  for(int i=0; i<yBar; i+=2){
    if(70+i < 110) ssd_buf[5*128 + 70+i] = 0xFF;
  }
  
  // Interactive control hints
  int hintCycle = (currentTime / 1500) % 4;
  const char* hints[] = {
    "A:Mode B:Control C:Relay2",
    "Hold:Rapid Fire",
    "A+B:Special",
    "Web:Full Control"
  };
  ssd_printStr(0, 7, hints[hintCycle]);
  
  updateParticles();
  ssd_displayBuffer();
}

// Advanced Manual Control
void showManualControl(){
  unsigned long currentTime = millis();
  
  ssd_clearBuffer();
  
  // Holographic interface
  ssd_printStr(0, 0, "MANUAL CONTROL");
  
  // Real-time position with large display
  char posStr[16];
  snprintf(posStr, sizeof(posStr), "X:%3d", servoXpos);
  ssd_printStr(10, 2, posStr);
  snprintf(posStr, sizeof(posStr), "Y:%3d", servoYpos);
  ssd_printStr(70, 2, posStr);
  
  // Interactive radar display
  int radarX = map(servoXpos, 0, 180, 20, 108);
  int radarY = map(servoYpos, 0, 180, 4, 7);
  
  // Radar sweep
  int sweepAngle = (currentTime / 100) % 360;
  for(int r=5; r<=15; r+=5){
    int sweepX = 64 + r * cos(sweepAngle * 3.14159 / 180);
    int sweepY = 5 + r * sin(sweepAngle * 3.14159 / 180);
    if(sweepX>=0 && sweepX<128 && sweepY>=0 && sweepY<8){
      ssd_buf[sweepY*128 + sweepX] = 0x18;
    }
  }
  
  // Target position
  for(int i=-2; i<=2; i++){
    for(int j=-1; j<=1; j++){
      if(radarX+i >=0 && radarX+i <128 && radarY+j >=0 && radarY+j <8){
        ssd_buf[(radarY+j)*128 + radarX+i] = 0xFF;
      }
    }
  }
  
  // Control status
  if(rapidMode){
    ssd_printStr(0, 7, "RAPID MODE ACTIVE");
    createParticleBurst(50, 7, 2);
  } else {
    ssd_printStr(0, 7, "A:Mode B:Move C:Relay2");
  }
  
  updateParticles();
  ssd_displayBuffer();
}

// Cinematic Sharpy Mode
void showSharpyMode(){
  unsigned long currentTime = millis();
  
  ssd_clearBuffer();
  
  // Laser show header
  ssd_printStr(0, 0, "LASER SHARPY");
  
  // Pattern visualization
  const char* patterns[] = {"DIAGONAL", "SQUARE", "CIRCLE", "RANDOM"};
  ssd_printStr(0, 1, patterns[sharpyPattern]);
  
  // Status
  ssd_printStr(70, 1, sharpyActive?"ACTIVE":"PAUSED");
  
  // Laser beam visualization
  int beamIntensity = (sin(currentTime * 0.01) + 1) * 4;
  for(int i=0; i<4; i++){
    int angle = (sharpyPattern * 90 + i * 90 + currentTime/50) % 360;
    for(int r=5; r<=20; r++){
      int x = 64 + r * cos(angle * 3.14159 / 180);
      int y = 4 + r * sin(angle * 3.14159 / 180);
      if(x>=0 && x<128 && y>=0 && y<8){
        ssd_buf[y*128 + x] = beamIntensity;
      }
    }
  }
  
  // Current position
  char posStr[16];
  snprintf(posStr, sizeof(posStr), "X:%3d Y:%3d", servoXpos, servoYpos);
  ssd_printStr(0, 6, posStr);
  
  // Control hints
  ssd_printStr(0, 7, "A:Mode B:Toggle C:Relay2");
  
  updateParticles();
  ssd_displayBuffer();
}

// High-Tech Relay Control
void showRelayControl(){
  unsigned long currentTime = millis();
  
  ssd_clearBuffer();
  
  // Cybernetic interface
  ssd_printStr(0, 0, "POWER CONTROL");
  
  // Relay status with advanced visualization
  if(relayState || relay2State){
    // ACTIVE - Energy flow visualization
    ssd_printStr(20, 2, "POWER: ACTIVE");
    
    // Animated energy bars
    int energyWave = (currentTime / 100) % 8;
    for(int i=0; i<8; i++){
      int height = (sin((i + energyWave) * 0.8) + 1) * 3;
      for(int j=0; j<height; j++){
        if(30+i*12 < 128 && 6-j >= 0){
          ssd_buf[(6-j)*128 + 30+i*12] = 0xFF;
        }
      }
    }
    
    // Particle energy
    if(currentTime % 100 == 0){
      createParticleBurst(64, 3, 5);
    }
  } else {
    // INACTIVE - Standby mode
    ssd_printStr(20, 2, "POWER: STANDBY");
    
    // Pulsing border
    int pulse = (sin(currentTime * 0.005) + 1) * 2;
    for(int i=20; i<40; i++){
      ssd_buf[4*128 + i] = pulse;
      ssd_buf[5*128 + i] = pulse;
    }
  }
  
  // Relay status display
  char relayStr[32];
  snprintf(relayStr, sizeof(relayStr), "R1:%s R2:%s", relayState?"ON ":"OFF", relay2State?"ON ":"OFF");
  ssd_printStr(0, 4, relayStr);
  
  // Control interface
  ssd_printStr(0, 7, "A:Mode B:R1 C:R2");
  
  updateParticles();
  ssd_displayBuffer();
}

// Main OLED update function - ZERO LATENCY
void updateOLED(){
  unsigned long now = millis();
  if(now - lastOledUpdate < oledInterval) return;
  
  lastOledUpdate = now;
  
  switch(displayState){
    case STATE_BOOT:
      showBootAnimation();
      break;
    case STATE_MAIN:
      showMainDisplay();
      break;
    case STATE_MENU:
      showMenu();
      break;
    case STATE_MANUAL:
      showManualControl();
      break;
    case STATE_SHARPY:
      showSharpyMode();
      break;
    case STATE_RELAY:
      showRelayControl();
      break;
  }
}

// ULTRA FAST Servo Control - ZERO LATENCY
void applyServos(){ 
  unsigned long now = millis();
  if(now - lastServoUpdate >= SERVO_UPDATE_INTERVAL){
    sX.write(servoXpos); 
    sY.write(servoYpos);
    lastServoUpdate = now;
  }
}

void applyRelay(){ 
  digitalWrite(RELAY_PIN, relayState ? LOW : HIGH); 
  digitalWrite(RELAY2_PIN, relay2State ? LOW : HIGH);  // New relay
}

// Mode management
void setOperationMode(OperationMode newMode){
  operationMode = newMode;
  
  // Reset servos to center when switching to manual mode
  if(newMode == MODE_MANUAL){
    servoXpos = 90;
    servoYpos = 90;
    applyServos();
  }
  
  // Set appropriate display state
  switch(newMode){
    case MODE_MANUAL:
      displayState = STATE_MANUAL;
      break;
    case MODE_SHARPY:
      displayState = STATE_SHARPY;
      sharpyActive = false; // Start paused
      break;
    case MODE_RELAY:
      displayState = STATE_RELAY;
      break;
  }
  
  saveState();
}

// ULTIMATE Button Control System
void handleButtons(){
  bool btnA = digitalRead(BTN_A_PIN) == LOW;
  bool btnB = digitalRead(BTN_B_PIN) == LOW;
  bool btnC = digitalRead(BTN_C_PIN) == LOW;  // New button
  
  // Menu navigation
  if(displayState == STATE_MENU){
    if(btnA && !btnAPressed){
      btnAPressed = true;
      currentMenu = (currentMenu - 1 + menuCount) % menuCount;
      menuScrolling = true;
      menuScrollTime = millis();
      createParticleBurst(10, 4, 3);
    }
    if(!btnA && btnAPressed){
      btnAPressed = false;
    }
    
    if(btnB && !btnBPressed){
      btnBPressed = true;
      currentMenu = (currentMenu + 1) % menuCount;
      menuScrolling = true;
      menuScrollTime = millis();
      createParticleBurst(118, 4, 3);
    }
    if(!btnB && btnBPressed){
      btnBPressed = false;
    }
    
    // Button C for additional menu control
    if(btnC && !btnCPressed){
      btnCPressed = true;
      createParticleBurst(64, 7, 3);
      // Additional menu function
    }
    if(!btnC && btnCPressed){
      btnCPressed = false;
    }
    
    // Both buttons to select menu item
    if(btnA && btnB && !bothButtonsPressed){
      bothButtonsPressed = true;
      createParticleBurst(64, 4, 10);
      switch(currentMenu){
        case 0: displayState = STATE_MAIN; break;
        case 1: setOperationMode(MODE_MANUAL); break;
        case 2: setOperationMode(MODE_SHARPY); break;
        case 3: setOperationMode(MODE_RELAY); break;
        case 4: displayState = STATE_MAIN; break;
      }
    }
    if(!(btnA && btnB) && bothButtonsPressed){
      bothButtonsPressed = false;
    }
    return;
  }
  
  // ULTIMATE MODE CONTROL: Button A cycles modes
  if(btnA && !btnAPressed && !bothButtonsPressed){
    btnAPressed = true;
    createParticleBurst(10, 0, 5);
    
    // Cycle through operation modes
    OperationMode newMode = (OperationMode)((operationMode + 1) % 3);
    setOperationMode(newMode);
  }
  if(!btnA && btnAPressed){
    btnAPressed = false;
  }
  
  // ULTIMATE SERVO CONTROL: Button B controls everything
  if(btnB && !btnBPressed){
    btnBPressed = true;
    btnHoldStartTime = millis();
    
    // Mode-specific single press actions
    switch(operationMode){
      case MODE_MANUAL:
        // Single press: Move both servos +5
        servoXpos = constrain(servoXpos + 5, 0, 180);
        servoYpos = constrain(servoYpos + 5, 0, 180);
        break;
      case MODE_SHARPY:
        // Toggle sharpy mode on/off
        sharpyActive = !sharpyActive;
        sharpyPatternTime = sharpyActive ? millis() : 0;
        break;
      case MODE_RELAY:
        // Toggle relay 1
        relayState = !relayState;
        applyRelay();
        break;
    }
    applyServos();
    saveState();
    createParticleBurst(118, 0, 3);
  }
  
  // NEW BUTTON C: Controls relay 2
  if(btnC && !btnCPressed){
    btnCPressed = true;
    
    // Toggle relay 2 in any mode
    relay2State = !relay2State;
    applyRelay();
    saveState();
    createParticleBurst(64, 0, 3);
  }
  if(!btnC && btnCPressed){
    btnCPressed = false;
  }
  
  // RAPID FIRE MODE: Hold detection
  if(btnB && btnBPressed) {
    if(millis() - btnHoldStartTime > 400) { // Ultra fast response
      rapidMode = true;
      
      switch(operationMode){
        case MODE_MANUAL:
          // Rapid movement for both servos
          servoXpos = constrain(servoXpos + rapidIncrement, 0, 180);
          servoYpos = constrain(servoYpos + rapidIncrement, 0, 180);
          break;
        case MODE_SHARPY:
          // Cycle through patterns rapidly
          if(millis() - btnHoldStartTime > 1000){
            sharpyPattern = (sharpyPattern + 1) % 4;
            btnHoldStartTime = millis(); // Reset for continuous cycling
          }
          break;
        case MODE_RELAY:
          // Rapid relay 1 toggle (protection included)
          if(millis() - btnHoldStartTime > 800){
            relayState = !relayState;
            applyRelay();
            btnHoldStartTime = millis();
          }
          break;
      }
      applyServos();
      saveState();
    }
  }
  
  // Button B release
  if(!btnB && btnBPressed) {
    btnBPressed = false;
    rapidMode = false;
  }
  
  // ULTIMATE COMBO: Both buttons for special functions
  if(btnA && btnB && !bothButtonsPressed){
    bothButtonsPressed = true;
    bothButtonsStartTime = millis();
  }
  
  if(bothButtonsPressed && (btnA && btnB)){
    if(millis() - bothButtonsStartTime > 1200){
      // Long press: Enter menu with explosion
      displayState = STATE_MENU;
      createParticleBurst(64, 3, 15);
      bothButtonsPressed = false;
    } else if(millis() - bothButtonsStartTime > 300){
      // Short press: Special functions
      createParticleBurst(64, 3, 8);
      switch(operationMode){
        case MODE_MANUAL:
          // Center both servos
          servoXpos = 90;
          servoYpos = 90;
          break;
        case MODE_SHARPY:
          // Reset to pattern 0
          sharpyPattern = 0;
          break;
        case MODE_RELAY:
          // Force both relays off (safety)
          relayState = false;
          relay2State = false;
          applyRelay();
          break;
      }
      applyServos();
      saveState();
      bothButtonsPressed = false;
    }
  }
  
  if(!(btnA && btnB) && bothButtonsPressed){
    bothButtonsPressed = false;
  }
}

// Sharpy mode pattern - Optimized
void updateSharpyMode(){
  if(operationMode != MODE_SHARPY || !sharpyActive) return;
  
  unsigned long now = millis();
  if(now - sharpyPatternTime < 500) return;
  
  sharpyPatternTime = now;
  
  // Advanced patterns
  switch(sharpyPattern){
    case 0: // Diagonal sweep
      servoXpos = 45 + (sharpyPattern % 4) * 30;
      servoYpos = 45 + (sharpyPattern % 4) * 30;
      break;
    case 1: // Square pattern
      servoXpos = (sharpyPattern % 2 == 0) ? 45 : 135;
      servoYpos = (sharpyPattern % 4 < 2) ? 45 : 135;
      break;
    case 2: // Circle pattern
      servoXpos = 90 + 45 * cos(sharpyPattern * 3.14159 / 2);
      servoYpos = 90 + 45 * sin(sharpyPattern * 3.14159 / 2);
      break;
    case 3: // Random pattern
      servoXpos = random(30, 150);
      servoYpos = random(30, 150);
      break;
  }
  
  sharpyPattern = (sharpyPattern + 1) % 4;
  applyServos();
}

// EEPROM functions
void saveState(){
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(ADDR_MAGIC, MAGIC);
  EEPROM.write(ADDR_RELAY, relayState ? 1 : 0);
  EEPROM.write(ADDR_RELAY2, relay2State ? 1 : 0);  // New relay
  EEPROM.write(ADDR_SERVO_X, constrain(servoXpos, 0, 180));
  EEPROM.write(ADDR_SERVO_Y, constrain(servoYpos, 0, 180));
  EEPROM.write(ADDR_MODE, operationMode);
  EEPROM.commit();
  EEPROM.end();
}

void loadState(){
  EEPROM.begin(EEPROM_SIZE);
  if(EEPROM.read(ADDR_MAGIC) == MAGIC){
    relayState = EEPROM.read(ADDR_RELAY) == 1;
    relay2State = EEPROM.read(ADDR_RELAY2) == 1;  // New relay
    servoXpos = EEPROM.read(ADDR_SERVO_X);
    servoYpos = EEPROM.read(ADDR_SERVO_Y);
    operationMode = (OperationMode)EEPROM.read(ADDR_MODE);
    
    if(servoXpos < 0 || servoXpos > 180) servoXpos = 90;
    if(servoYpos < 0 || servoYpos > 180) servoYpos = 90;
    if(operationMode < MODE_MANUAL || operationMode > MODE_RELAY) {
      operationMode = MODE_MANUAL;
    }
  } else {
    relayState = false; 
    relay2State = false;  // New relay
    servoXpos = 90; 
    servoYpos = 90; 
    operationMode = MODE_MANUAL;
    EEPROM.write(ADDR_MAGIC, MAGIC);
    EEPROM.commit();
  }
  EEPROM.end();
}

// Captive portal detection
bool isCaptivePortal(){
  String host = server.hostHeader();
  return host != String(apIP.toString()) && host != "" && host != "192.168.4.1";
}

// Web interface - Optimized for mobile
String webpage(){
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>TechnoMCU ULTIMATE</title>";
  html += "<style>";
  html += "body{margin:0;background:#0a0a0a;color:#0ff;font-family:Arial,sans-serif;text-align:center}";
  html += ".top{padding:15px;background:#002a3a;box-shadow:0 4px 20px rgba(0,255,255,0.1)}";
  html += ".box{padding:15px;margin:10px;border-radius:10px;background:rgba(0,40,60,0.3);border:1px solid rgba(0,255,255,0.1);display:inline-block;vertical-align:top}";
  html += "#joy{background:#001a22;border-radius:12px;border:2px solid #0ff;cursor:pointer}";
  html += ".slider{width:90%;height:25px;background:#001a22;border-radius:12px;outline:none;opacity:0.7}";
  html += ".btn{padding:10px 15px;margin:5px;border-radius:8px;border:none;background:#00ccff;color:#000;font-weight:700;cursor:pointer;font-size:14px}";
  html += ".ghost{background:transparent;border:2px solid #00ccff;color:#00ccff}";
  html += ".small{font-size:12px;color:#8af}";
  html += "#status{font-size:14px;margin-top:10px;color:#8af;padding:8px;background:rgba(0,50,80,0.3);border-radius:8px}";
  html += ".mode-active{background:#00ff88}";
  html += "h2{text-shadow:0 0 10px rgba(0,255,255,0.5)}";
  html += "</style></head><body>";
  
  html += "<div class='top'><h2>TechnoMCU ULTIMATE Control</h2>";
  html += "<div class='small'>By Raktim Hazra | Zero Latency | Advanced Animations</div></div>";
  
  html += "<div class='box'><canvas id='joy' width='200' height='200'></canvas>";
  html += "<div class='small'>Drag to control servos</div></div>";
  
  html += "<div class='box' style='width:300px;text-align:left'>";
  
  html += "<div style='margin-bottom:10px'><strong>Mode Control:</strong></div>";
  html += "<button id='modeManual' class='btn ";
  html += (operationMode==MODE_MANUAL?"mode-active":"");
  html += "'>MANUAL</button>";
  html += "<button id='modeSharpy' class='btn ";
  html += (operationMode==MODE_SHARPY?"mode-active":"");
  html += "'>SHARPY</button>";
  html += "<button id='modeRelay' class='btn ";
  html += (operationMode==MODE_RELAY?"mode-active":"");
  html += "'>RELAY</button>";
  
  html += "<div style='margin-top:15px'><strong>Servo Control:</strong></div>";
  html += "<div>X: <span id='sxv'>";
  html += String(servoXpos);
  html += "</span>°</div>";
  html += "<input id='sx' class='slider' type='range' min='0' max='180' value='";
  html += String(servoXpos);
  html += "'><br>";
  html += "<div>Y: <span id='syv'>";
  html += String(servoYpos);
  html += "</span>°</div>";
  html += "<input id='sy' class='slider' type='range' min='0' max='180' value='";
  html += String(servoYpos);
  html += "'><br>";
  
  html += "<div style='margin-top:10px'>";
  html += "<button id='stepL' class='btn ghost'>X-5</button>";
  html += "<button id='stepR' class='btn ghost'>X+5</button>";
  html += "<button id='stepU' class='btn ghost'>Y+5</button>";
  html += "<button id='stepD' class='btn ghost'>Y-5</button>";
  html += "</div>";
  
  html += "<div style='margin-top:15px'><strong>Relay Control:</strong></div>";
  html += "<button id='relayBtn' class='btn ";
  html += (relayState?"mode-active":"");
  html += "'>";
  html += (relayState?"RELAY 1 ON":"RELAY 1 OFF");
  html += "</button>";
  
  html += "<button id='relay2Btn' class='btn ";
  html += (relay2State?"mode-active":"");
  html += "'>";
  html += (relay2State?"RELAY 2 ON":"RELAY 2 OFF");
  html += "</button>";
  
  html += "<div id='status' class='small'>";
  html += "Mode: ";
  if(operationMode == MODE_MANUAL) html += "MANUAL";
  else if(operationMode == MODE_SHARPY) html += "SHARPY";
  else html += "RELAY";
  html += " | X:";
  html += String(servoXpos);
  html += " Y:";
  html += String(servoYpos);
  html += " | Relay1:";
  html += (relayState?"ON":"OFF");
  html += " Relay2:";
  html += (relay2State?"ON":"OFF");
  html += "</div></div>";
  
  html += "<script>";
  html += "let joy=document.getElementById('joy'),ctx=joy.getContext('2d'),dragging=false;";
  html += "let sx=document.getElementById('sx'),sy=document.getElementById('sy'),sxv=document.getElementById('sxv'),syv=document.getElementById('syv');";
  html += "let lastSend=0;";
  
  html += "function drawJoy(x,y){";
  html += "ctx.clearRect(0,0,200,200);";
  html += "ctx.fillStyle='#002a3a';ctx.fillRect(0,0,200,200);";
  html += "ctx.strokeStyle='#00ccff';ctx.lineWidth=2;ctx.strokeRect(10,10,180,180);";
  html += "ctx.fillStyle='#00ccff';ctx.beginPath();ctx.arc(100+x,100+y,15,0,Math.PI*2);ctx.fill();";
  html += "}";
  
  html += "function updateServos(x,y){";
  html += "let now=Date.now();";
  html += "if(now-lastSend>50){";
  html += "fetch('/move?x='+x+'&y='+y).then(r=>r.text()).then(updateStatus);";
  html += "lastSend=now;";
  html += "}";
  html += "sx.value=x;sxv.textContent=x;";
  html += "sy.value=y;syv.textContent=y;";
  html += "}";
  
  html += "function updateStatus(){";
  html += "fetch('/status').then(r=>r.json()).then(data=>{";
  html += "document.getElementById('status').innerHTML='Mode: '+(data.mode==0?'MANUAL':(data.mode==1?'SHARPY':'RELAY'))+' | X:'+data.x+' Y:'+data.y+' Relay1:'+(data.relay?'ON':'OFF')+' Relay2:'+(data.relay2?'ON':'OFF');";
  html += "});";
  html += "}";
  
  html += "joy.addEventListener('mousedown',e=>{dragging=true;handleJoy(e);});";
  html += "joy.addEventListener('mousemove',e=>{if(dragging)handleJoy(e);});";
  html += "joy.addEventListener('mouseup',()=>{dragging=false;drawJoy(0,0);updateServos(90,90);});";
  html += "joy.addEventListener('touchstart',e=>{e.preventDefault();dragging=true;handleJoy(e.touches[0]);});";
  html += "joy.addEventListener('touchmove',e=>{e.preventDefault();if(dragging)handleJoy(e.touches[0]);});";
  html += "joy.addEventListener('touchend',()=>{dragging=false;drawJoy(0,0);updateServos(90,90);});";
  
  html += "function handleJoy(e){";
  html += "let rect=joy.getBoundingClientRect();";
  html += "let x=((e.clientX-rect.left-100)/80)*90+90;";
  html += "let y=((e.clientY-rect.top-100)/80)*90+90;";
  html += "x=Math.max(0,Math.min(180,x));y=Math.max(0,Math.min(180,y));";
  html += "drawJoy((x-90)/90*80,(y-90)/90*80);";
  html += "updateServos(Math.round(x),Math.round(y));";
  html += "}";
  
  html += "sx.oninput=()=>{updateServos(sx.value,sy.value);};";
  html += "sy.oninput=()=>{updateServos(sx.value,sy.value);};";
  
  html += "document.getElementById('stepL').onclick=()=>updateServos(parseInt(sx.value)-5,parseInt(sy.value));";
  html += "document.getElementById('stepR').onclick=()=>updateServos(parseInt(sx.value)+5,parseInt(sy.value));";
  html += "document.getElementById('stepU').onclick=()=>updateServos(parseInt(sx.value),parseInt(sy.value)+5);";
  html += "document.getElementById('stepD').onclick=()=>updateServos(parseInt(sx.value),parseInt(sy.value)-5);";
  
  html += "document.getElementById('relayBtn').onclick=()=>{";
  html += "fetch('/relay').then(r=>r.text()).then(t=>{";
  html += "document.getElementById('relayBtn').textContent=t=='ON'?'RELAY 1 ON':'RELAY 1 OFF';";
  html += "document.getElementById('relayBtn').className=t=='ON'?'btn mode-active':'btn';";
  html += "updateStatus();";
  html += "});";
  html += "};";
  
  html += "document.getElementById('relay2Btn').onclick=()=>{";
  html += "fetch('/relay2').then(r=>r.text()).then(t=>{";
  html += "document.getElementById('relay2Btn').textContent=t=='ON'?'RELAY 2 ON':'RELAY 2 OFF';";
  html += "document.getElementById('relay2Btn').className=t=='ON'?'btn mode-active':'btn';";
  html += "updateStatus();";
  html += "});";
  html += "};";
  
  html += "document.getElementById('modeManual').onclick=()=>{fetch('/setmode?mode=0').then(()=>location.reload());};";
  html += "document.getElementById('modeSharpy').onclick=()=>{fetch('/setmode?mode=1').then(()=>location.reload());};";
  html += "document.getElementById('modeRelay').onclick=()=>{fetch('/setmode?mode=2').then(()=>location.reload());};";
  
  html += "drawJoy(0,0);";
  html += "setInterval(updateStatus,1500);";
  html += "</script>";
  
  html += "</body></html>";
  return html;
}

// Setup
void setup(){
  // Initialize pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);  // New relay
  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(BTN_C_PIN, INPUT_PULLUP);  // New button
  
  // Initialize OLED
  ssd_init();
  
  // Initialize particle system
  initParticles();
  
  // Start SPECTACULAR boot animation
  bootStartTime = millis();
  displayState = STATE_BOOT;
  
  // Load saved state
  loadState();
  
  // Initialize servos and relay
  sX.attach(SERVO_X_PIN); 
  sY.attach(SERVO_Y_PIN);
  applyServos(); 
  applyRelay();

  // Setup WiFi AP
  WiFi.softAP(apSsid, apPass);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  dnsServer.start(53, "*", apIP);

  // Setup web server routes
  server.on("/", [](){ 
    if(isCaptivePortal()){
      server.sendHeader("Location", String("http://") + apIP.toString(), true);
      server.send(302, "text/plain", "");
    } else {
      server.send(200, "text/html", webpage()); 
    }
  });
  
  server.on("/move", [](){ 
    if(server.hasArg("x")) servoXpos = server.arg("x").toInt();
    if(server.hasArg("y")) servoYpos = server.arg("y").toInt();
    applyServos();
    saveState();
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/relay", [](){ 
    relayState = !relayState; 
    applyRelay(); 
    saveState(); 
    server.send(200, "text/plain", relayState ? "ON" : "OFF"); 
  });
  
  // New relay2 route
  server.on("/relay2", [](){ 
    relay2State = !relay2State; 
    applyRelay(); 
    saveState(); 
    server.send(200, "text/plain", relay2State ? "ON" : "OFF"); 
  });
  
  server.on("/setmode", [](){ 
    if(server.hasArg("mode")){
      int mode = server.arg("mode").toInt();
      if(mode >= 0 && mode <= 2){
        setOperationMode((OperationMode)mode);
      }
    }
    server.send(200, "text/plain", "OK"); 
  });
  
  server.on("/status", [](){ 
    String json = "{";
    json += "\"x\":" + String(servoXpos) + ",";
    json += "\"y\":" + String(servoYpos) + ",";
    json += "\"relay\":" + String(relayState ? 1 : 0) + ",";
    json += "\"relay2\":" + String(relay2State ? 1 : 0) + ",";  // New relay
    json += "\"mode\":" + String(operationMode);
    json += "}";
    server.send(200, "application/json", json); 
  });
  
  server.onNotFound([](){
    if(isCaptivePortal()){
      server.sendHeader("Location", String("http://") + apIP.toString(), true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "Not Found");
    }
  });
  
  server.begin();
}

// Main loop - ULTIMATE PERFORMANCE
void loop(){
  // Handle network requests (non-blocking)
  dnsServer.processNextRequest();
  server.handleClient();
  
  // Handle inputs - ULTRA RESPONSIVE
  handleButtons();
  
  // Update sharpy pattern if active
  updateSharpyMode();
  
  // Update OLED display with ZERO LATENCY
  updateOLED();
}
