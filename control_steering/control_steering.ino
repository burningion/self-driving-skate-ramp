#include <Packet.h>
#include <PacketCRC.h>
#include <SerialTransfer.h>

SerialTransfer steerTransfer;

struct __attribute__((packed)) STRUCT {
  char d;
  int x;
} steerStruct;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  steerTransfer.begin(Serial);
  // left and right
  pinMode(A9, OUTPUT);
  pinMode(A8, OUTPUT);
  // up and down
  pinMode(A7, OUTPUT);
  pinMode(A6, OUTPUT);

  digitalWrite(A9, LOW);
  digitalWrite(A8, LOW);
  digitalWrite(A7, LOW);
  digitalWrite(A6, LOW);
  
  
}

void loop() {
 if(steerTransfer.available()) {
  uint16_t recSize = 0;
  recSize = steerTransfer.rxObj(steerStruct, recSize);
  if (steerStruct.d == 'R') {
    if (steerStruct.x == 1) {
      digitalWrite(A9, HIGH);
    }
    else {
      digitalWrite(A9, LOW);
      digitalWrite(A8, LOW);
    }
  }
  else if (steerStruct.d == 'L') {
    if (steerStruct.x == 1) {
      digitalWrite(A8, HIGH);
    }
    else {
      digitalWrite(A9, LOW);
      digitalWrite(A8, LOW);
    }
  }
    else if (steerStruct.d == 'U') {
      if (steerStruct.x == 1) {
        digitalWrite(A6, HIGH);
      }
      else {
        digitalWrite(A7, LOW);
        digitalWrite(A6, LOW);
      }
    }
    else if (steerStruct.d == 'D') {
      if (steerStruct.x == 1) {
        digitalWrite(A7, HIGH);
      }
      else {
        digitalWrite(A7, LOW);
        digitalWrite(A6, LOW);
      }


  }

 }
}
