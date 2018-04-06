//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
double Channel_1;
double Channel_2;
void setup()
{

  Serial.begin(115200);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Welcome");
  lcd.setCursor(0,1);
  lcd.print("Loading Stand By");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PWM CH1");
  lcd.setCursor(8,0);
  lcd.print("PWM CH2");
  lcd.setCursor(0,1);
}


void loop()
{
Channel_1 = pulseIn(8,HIGH);
Channel_2 = pulseIn(9,HIGH);
Serial.print("PWM CH1: ");
Serial.println(Channel_1);
Serial.print("PWM CH2: ");
Serial.println(Channel_2);
delay(300);
lcd.setCursor(0,1);
lcd.print(Channel_1);
lcd.setCursor(9,1);
lcd.print(Channel_2);



  
}
