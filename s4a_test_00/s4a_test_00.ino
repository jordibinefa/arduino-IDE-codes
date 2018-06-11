#define BOTO_02 2
#define LED_13 13
#define BOTO_03 3
#define LED_12 12
#define BOTO_A5 A5
#define LED_11 11
#define BOTO_A4 A4
#define LED_10 10

void setup(){
  Serial.begin(9600);
  pinMode(BOTO_02,INPUT);
  pinMode(LED_13,OUTPUT);
  pinMode(BOTO_03,INPUT);
  pinMode(LED_12,OUTPUT);
  pinMode(BOTO_A5,INPUT);
  pinMode(LED_11,OUTPUT);
  pinMode(BOTO_A4,INPUT);
  pinMode(LED_10,OUTPUT);
}

boolean bPremut(int nQuinBoto){
  if(digitalRead(nQuinBoto))
    return false;
  return true;
}

void vTrametEstat(boolean b){
  if(b)
    Serial.print("1");
  else
    Serial.print("0");
}

void vTrametValorsJoystick(){
  Serial.print("(");
  Serial.print(analogRead(A1));
  Serial.print(",");
  Serial.print(analogRead(A2));
  Serial.print(")");
}

void loop(){
  static boolean bEstavaPremutBoto02 = false;
  static boolean bEstavaPremutBoto03 = false;
  static boolean bEstavaPremutBotoA5 = false;
  static boolean bEstavaPremutBotoA4 = false;
  static boolean bEstatLED_13 = false,bUltimEstatLED_13 = false;
  static boolean bEstatLED_12 = false,bUltimEstatLED_12 = false;
  static boolean bEstatLED_11 = false,bUltimEstatLED_11 = false;
  static boolean bEstatLED_10 = false,bUltimEstatLED_10 = false;
  boolean bEstatBOTO_02 = bPremut(BOTO_02);
  boolean bEstatBOTO_03 = bPremut(BOTO_03);
  boolean bEstatBOTO_A5 = bPremut(BOTO_A5);
  boolean bEstatBOTO_A4 = bPremut(BOTO_A4);
  char ch;
  
  if(bEstavaPremutBoto02 != bEstatBOTO_02){
    delay(10);
    // Hi ha hagut canvi
    if(bEstatBOTO_02){
      bEstatLED_13 = !bEstatLED_13;
      //Serial.print("BOTO_02 premut");
    }
    bEstavaPremutBoto02 = bEstatBOTO_02;
  }
  if(bEstavaPremutBoto03 != bEstatBOTO_03){
    delay(10);
    // Hi ha hagut canvi
    if(bEstatBOTO_03){
      bEstatLED_12 = !bEstatLED_12;
    }
    bEstavaPremutBoto03 = bEstatBOTO_03;
  }
  if(bEstavaPremutBotoA5 != bEstatBOTO_A5){
    delay(10);
    // Hi ha hagut canvi
    if(bEstatBOTO_A5){
      bEstatLED_11 = !bEstatLED_11;
    }
    bEstavaPremutBotoA5 = bEstatBOTO_A5;
  }
  if(bEstavaPremutBotoA4 != bEstatBOTO_A4){
    delay(10);
    // Hi ha hagut canvi
    if(bEstatBOTO_A4){
      bEstatLED_10 = !bEstatLED_10;
    }
    bEstavaPremutBotoA4 = bEstatBOTO_A4;
  }

  if (Serial.available() > 0) {
    ch = Serial.read();
    if(ch == 'a')
      bEstatLED_13 = false;
    if(ch == 'e')
      bEstatLED_13 = true;
    if(ch == 'A')
      bEstatLED_12 = false;
    if(ch == 'E')
      bEstatLED_12 = true;
    if(ch == 'f')
      bEstatLED_11 = false;
    if(ch == 'n')
      bEstatLED_11 = true;
    if(ch == 'F')
      bEstatLED_10 = false;
    if(ch == 'N')
      bEstatLED_10 = true;
  }
  
  digitalWrite(LED_13, bEstatLED_13);
  digitalWrite(LED_12, bEstatLED_12);
  digitalWrite(LED_11, bEstatLED_11);
  digitalWrite(LED_10, bEstatLED_10);
  
  vTrametEstat(bEstatBOTO_02);
  vTrametEstat(bEstatBOTO_03);
  vTrametEstat(bEstatBOTO_A5);
  vTrametEstat(bEstatBOTO_A4);
  vTrametEstat(bEstatLED_13);
  vTrametEstat(bEstatLED_12);
  vTrametEstat(bEstatLED_11);
  vTrametEstat(bEstatLED_10);
  vTrametValorsJoystick();
  Serial.println("");
  
  delay(200);
}
