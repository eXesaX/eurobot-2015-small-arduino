#include "DualVNH5019MotorShield.h"
#include <StackList.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <registers.h>

// motors
DualVNH5019MotorShield motors;

// task queue type
struct task {
  int command;
  int x;
  int y;
};

StackList <task> commandStack;
task currentTask;

//proximity sensors
int infraRedSensorSmall = A5;
int infraRedSensorBig = A2;
const int CDIRsmall = 70; // critical distance infrared small
const int CDIRbig = 470; // critical distance infrared big

//general timing
const long shutdownTime = 89000;
long startTime = 0;
long time = 0;

//startingCord
int starterCordPin = 20;
volatile bool plugged = true;

//laser sensors
byte initComplete=0;
byte testctr=0;
unsigned long currTime;
unsigned long timer;
volatile int sensor0_data[4];
volatile int sensor1_data[4];
volatile byte movementflag0 = 0;
volatile byte movementflag1 = 0;
const int SS0 = 3; //Select Slave port for first sensor
const int SS1 = 5; //Select Slave port for second sensor

//global position
int x = 0;
int y = 0;
int angle_degrees = 0;
const float cpiToAngle = 4.7;
const float radiansToDegrees = 57.2957795;
//

//firmware upload
extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];

void setup() {
  Serial.begin(115200);
  
  //laser sensors setup
  pinMode(1, INPUT);
  digitalWrite(1, HIGH);
  pinMode(SS0, OUTPUT);
  pinMode(SS1, OUTPUT);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);

  //pinMode(10, OUTPUT);
  //analogWrite(10, 0);
  
  performStartup(SS0);
  delay(20);
  dispRegisters(SS0);
  
  performStartup(SS1);
  delay(20);  
  dispRegisters(SS1);
  
  delay(100);
  initComplete=9;
  
  
  
  //proximity sensor pins
  pinMode(infraRedSensorSmall, INPUT);
  pinMode(infraRedSensorBig, INPUT);
  
  //starting cord
  pinMode(starterCordPin, INPUT_PULLUP);
  attachInterrupt(3, starterCord, RISING);

  //filling command queue
  if (digitalRead(31)) {
   commandStack.push((task){1, 50, 50});
//   commandStack.push((task){2, -10});
  } else { //yellow side
   commandStack.push((task){1, 50, 50});
   //commandStack.push((task){1, 100});
   //commandStack.push((task){2, 90});
   //commandStack.push((task){1, 230});
//   commandStack.push((task){2, -10});
 }
}

void starterCord() {
  plugged = false;
  startTime = millis();
  //Serial.println("START");
  //motors.setSpeeds(150, 150);
}

void runTask() {
  if (!commandStack.isEmpty()) {
     currentTask = commandStack.pop();
     Serial.println("Command popped");
     
     int curX = x;
     int curY = y;
     int curAngle = angle_degrees;
     
     int localX = currentTask.x - curX;
     int localY = currentTask.y - curY;
     
     int localDistance = floor(sqrt(pow(localX, 2) + pow(localY, 2)));
     int localAngle = atan2(localY, localX)*radiansToDegrees;
     
     Serial.println("D: " + String(localDistance) + "A: " + String(localAngle));
     
     if (currentTask.command == 1) {
        //straight move
        Serial.println("straight move");
        int target = curX + localDistance;
        int diff = abs(target - x);
        //Serial.println(String(diff) + " to go");
        int dir = 1;
        if (localDistance < 0) { dir = -1; }
        else {dir = 1;}
        
        while (diff != 0) {
         if ((analogRead(infraRedSensorSmall) > CDIRsmall) || 
             (analogRead(infraRedSensorBig) > CDIRbig)) { 
            motors.setBrakes(400, 400);
            delay(250); 
         }
         updatePosition();
         diff = abs(target - x);
         //Serial.println(String(diff) + " = " + String(target) + " - " + String(x));
         motors.setSpeeds(200*dir, 170*dir); 
        }
     } 
     if (currentTask.command == 2) {
        //turn
        Serial.println("turn");
        int target = curAngle + localAngle;
        Serial.println(angle_degrees);
        int diff = abs(target - angle_degrees);
        
        Serial.println(String(diff) + " to go");
        int dir = 1;
        if (localAngle < 0) { dir = -1; }
        else {dir = 1;}
        
        while (diff != 0) {
         if ((analogRead(infraRedSensorSmall) > CDIRsmall) || 
             (analogRead(infraRedSensorBig) > CDIRbig)) { 
            motors.setBrakes(400, 400);
            delay(250); 
         }
         updatePosition();
         diff = abs(target - angle_degrees);
//         Serial.println(String(diff) + " = " + String(target) + " - " + String(angle_degrees));
         motors.setSpeeds(200*dir, -200*dir); 
        }
     }
     motors.setBrakes(400, 400);
  } else {
     motors.setBrakes(400, 400); 
  }
}

void adns_com_begin(byte ss){
  digitalWrite(ss, LOW);
}

void adns_com_end(byte ss){
  digitalWrite(ss, HIGH);
}

byte adns_read_reg(byte reg_addr, byte ss){
  adns_com_begin(ss);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end(ss);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data, byte ss){
  adns_com_begin(ss);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end(ss);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(byte ss){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02, ss); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d, ss); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18, ss); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(ss);
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end(ss);
  }


void performStartup(byte ss){
  adns_com_end(ss); // ensure that the serial port is reset
  adns_com_begin(ss); // ensure that the serial port is reset
  adns_com_end(ss); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a, ss); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion, ss);
  adns_read_reg(REG_Delta_X_L, ss);
  adns_read_reg(REG_Delta_X_H, ss);
  adns_read_reg(REG_Delta_Y_L, ss);
  adns_read_reg(REG_Delta_Y_H, ss);
  // upload the firmware
  adns_upload_firmware(ss);
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0, ss);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0, ss);
  adns_write_reg(REG_Configuration_I, 0x01, ss);
  
  delay(1);

  Serial.println("Optical Chip Initialized");
  }



void dispRegisters(byte ss){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(ss, LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);
    delay(1);
  }
  digitalWrite(ss, HIGH);
}


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }
  
void UpdatePointer0(byte ss){
  if (initComplete==9){
    if (adns_read_reg(REG_Motion, ss) & 0b10000000) {
      digitalWrite(ss, LOW);
      sensor0_data[0] = (int)adns_read_reg(REG_Delta_X_L, ss);
      sensor0_data[1] = (int)adns_read_reg(REG_Delta_Y_L, ss);
      sensor0_data[2] = (int)adns_read_reg(REG_Delta_X_H, ss);
      sensor0_data[3] = (int)adns_read_reg(REG_Delta_Y_H, ss);
      digitalWrite(ss, HIGH);  
  
      movementflag0 = 1;
    }
  }
}

void UpdatePointer1(byte ss){
  if (initComplete==9){
    if (adns_read_reg(REG_Motion, ss) & 0b10000000) {
      digitalWrite(ss, LOW);
      sensor1_data[0] = (int)adns_read_reg(REG_Delta_X_L, ss);
      sensor1_data[1] = (int)adns_read_reg(REG_Delta_Y_L, ss);
      sensor1_data[2] = (int)adns_read_reg(REG_Delta_X_H, ss);
      sensor1_data[3] = (int)adns_read_reg(REG_Delta_Y_H, ss);
      digitalWrite(ss, HIGH);  
  
      movementflag1 = 1;
    }
  }
}
  
int x0low = 0;
int y0low = 0;
int x0high = 0;
int y0high = 0;
int x0distance = 0;
int y0distance = 0;

int x1low = 0;
int y1low = 0;
int x1high = 0;
int y1high = 0;
int x1distance = 0;
int y1distance = 0;

void updatePosition() {
 UpdatePointer0(SS0);
 UpdatePointer1(SS1);
 if (movementflag0){
    x0low = convTwosComp(sensor0_data[0]);
    y0low = convTwosComp(sensor0_data[1]);
    x0high = convTwosComp(sensor0_data[2]);
    y0high = convTwosComp(sensor0_data[3]);
    x0high = x0high << 8;
    y0high = y0high << 8;
    x0distance += x0high | x0low;
    y0distance += y0high | y0low;
    movementflag0 = 0;
//    Serial.print("X0d: " + String(x0distance));
//    Serial.println(" Y0d: " + String(y0distance));
 }
 if (movementflag1){   
    x1low = convTwosComp(sensor1_data[0]);
    y1low = convTwosComp(sensor1_data[1]);
    x1high = convTwosComp(sensor1_data[2]);
    y1high = convTwosComp(sensor1_data[3]);
    x1high = x1high << 8;
    y1high = y1high << 8;
    x1distance += x1high | x1low;
    y1distance += y1high | y1low;
    int tempX = floor(x1distance * 1.25);
    
    movementflag1 = 0;
//    Serial.print("X1d: " + String(x1distance));
//    Serial.println(" Y1d: " + String(y1distance));
    //Serial.println("Xhigh: " + String(xhigh));
  }
  //800 per 10 cm
  x = 0; //floor(x1distance / 80.0);
  //Serial.println(x0distance);
  y = 0; //floor(y1distance / 80.0);
  int xdir = 1;
  if (x1distance - x0distance >= 0) {xdir = 1;}
  else {xdir = -1;}
  angle_degrees = sqrt(pow(x0distance - x1distance, 2) + pow(y0distance - y1distance, 2)) / cpiToAngle * xdir;
  
  //Serial.println(angle_degrees);
}

void loop() {
  //Serial.println(digitalRead(starterCordPin));
  updatePosition();
  if (1) {
   updatePosition();
   //delay(200);
   //Serial.println(String(analogRead(infraRedSensorSmall)) + " " + String(analogRead(infraRedSensorBig)));
   if (millis() - startTime < 89000) {
    
     /*if (millis() - startTime > 10000) {
       motors.setBrakes(400, 400);
     }
     
     if ((analogRead(infraRedSensorSmall) > CDIRsmall) || 
             (analogRead(infraRedSensorBig) > CDIRbig)) {
               motors.setBrakes(400, 400);
             }*/
     
     runTask();
   } else  { 
     Serial.println("FULL STOP"); 
     motors.setBrakes(400,400);
   }
  }
}
