#include <I2Cdev.h>
#include <PS2X_lib.h>
#include <PinChangeInt.h>

#define PS2_DAT        A0
#define PS2_CMD        A1
#define PS2_SEL        A2
#define PS2_CLK        A3

#define PWM1           9
#define PWM2           10
#define PWM11          44
#define PWM22          45

#define PWM_max        200
#define PWM_min        160
#define IN1            4
#define IN2            5
#define IN3            6
#define IN4            7
#define IN11           48
#define IN22           49
#define IN33           50
#define IN44           51
#define pressures   true
#define rumble      true

#define INTERRUPT_PIN_RR A8
#define INTERRUPT_PIN_RL A9
#define INTERRUPT_PIN_FR A10
#define INTERRUPT_PIN_FL A11




#define DEBUG      false




PS2X ps2x;

char dane2[60];
float rozpedzanie = 0;
float rozpedzanie1 = 0;
float rozpedzanie2 = 0;
float rozpedzanie11 = 0;
float rozpedzanie22 = 0;
boolean rozpedzaj = false;
volatile uint8_t ograniczenie = 0;
uint8_t ograniczenie2 = 255;
int licznik;

unsigned long czas_wczesniejszy = 0;
unsigned long timer;
unsigned long nieaktywny;

boolean flaga_prosto = false;
boolean flaga_wstecz = false;
boolean flaga_zawracanie = false;
boolean flaga_zawracanie_l = false;
boolean flaga_zawracanie_p = false;
boolean flaga_jedzie = false;
boolean flaga_czasu_zatrzymania = true;

boolean sprawdz = true;

int postoj = 100;            // Czas przerwy miedzy zmiana kierunku obrotow silnika [ms]
float przyspieszenie = 20;      // Wartosc przyspieszenia <0.01, 255>
float predkosc = PWM_min;         // Maksymalna predkosc poczatkowa <0, 255>
float przyr_predkosci = 2;

volatile long licznik_RL = 0;
volatile long licznik_RR = 0;
volatile uint8_t licznik_RLL = 0;
volatile uint8_t licznik_RRR = 0;
volatile uint8_t licznik_FLL = 0;
volatile uint8_t licznik_FRR = 0;
volatile float predkosc_RL = 0;
volatile float predkosc_RR = 0;
volatile float predkosc_FL = 0;
volatile float predkosc_FR = 0;
volatile unsigned long t1, t2 = 0;
volatile unsigned long t3, t4 = 0;
volatile unsigned long t5, t6 = 0;
volatile unsigned long t7, t8 = 0;

byte error = 0;
byte vibrate = 0;
unsigned long odczyt = 0;

boolean ustawienia = false;
float napiecie = 0;
float offset1 = 0;
float offset2 = 0;
float offset11 = 0;
float offset22 = 0;


void setup()
{
  TCCR2B = TCCR2B & B11111000 | B00000001;                     // MEGA         set timer 2 divisor to     1 for PWM RRequency of 31372.55 Hz
  TCCR5B = TCCR5B & B11111000 | B00000001;

  if (DEBUG)
    Serial.begin(115200); //Uruchomienie komunikacji
  Serial1.begin(57600);
  Serial2.begin(57600);
  Wire.begin(1);
  Wire.onReceive(receiveEvent);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  delay(500);
  if (error != 1)
  {
    ps2x.read_gamepad(true, 0);
    delay(500);
  }
  pinMode(PWM1, OUTPUT); //Sygnał PWM silnika nr 1
  pinMode(PWM2, OUTPUT); //Sygnał PWM silnika nr 2
  pinMode(PWM11, OUTPUT); //Sygnał PWM silnika nr 3
  pinMode(PWM22, OUTPUT); //Sygnał PWM silnika nr 4
  pinMode(IN1, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 1
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 2
  pinMode(IN4, OUTPUT);
  pinMode(IN11, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 1
  pinMode(IN22, OUTPUT);
  pinMode(IN33, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 2
  pinMode(IN44, OUTPUT);
  digitalWrite(IN1, LOW); //zwarcie silnikow
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN11, LOW); //zwarcie silnikow
  digitalWrite(IN22, LOW);
  digitalWrite(IN33, LOW);
  digitalWrite(IN44, LOW);
  pinMode(INTERRUPT_PIN_RL, INPUT_PULLUP);
  attachPinChangeInterrupt(INTERRUPT_PIN_RL, RL, CHANGE);
  pinMode(INTERRUPT_PIN_RR, INPUT_PULLUP);
  attachPinChangeInterrupt(INTERRUPT_PIN_RR, RR, CHANGE);
  pinMode(INTERRUPT_PIN_FL, INPUT_PULLUP);
  attachPinChangeInterrupt(INTERRUPT_PIN_FL, FL, CHANGE);
  pinMode(INTERRUPT_PIN_FR, INPUT_PULLUP);
  digitalWrite(20, HIGH);
  digitalWrite(21, HIGH);
  attachPinChangeInterrupt(INTERRUPT_PIN_FR, FR, CHANGE);
  napiecie = analogRead(A5);
  napiecie = napiecie / 117;
  t3 = millis();
  t4 = millis();
  t5 = millis();
  t6 = millis();
}

void loop()
{
  if (sprawdz)
  {
    nieaktywny = millis();
    sprawdz = false;
  }

  if (millis() - nieaktywny > 500)                            //zatrzymaj jak zgubi zasieg
  {
    wyslij_dane();
    zatrzymaj();
    if (flaga_czasu_zatrzymania)
    {
      czas_wczesniejszy = millis();
      flaga_czasu_zatrzymania = false;
    }
  }

  if (rozpedzaj == true)
  {
    if (rozpedzanie < predkosc)
    {
      rozpedzanie += przyspieszenie;
      if (rozpedzanie > 255)
        rozpedzanie = 255;
    }
    else
      rozpedzanie = predkosc;

    if (rozpedzanie1 < predkosc + offset1)
    {
      analogWrite(PWM1, rozpedzanie1);
      rozpedzanie1 += przyspieszenie;
      if (rozpedzanie1 + offset1 > 255)
        rozpedzanie1 = 255;
    }
    else
    {
      rozpedzanie1 = predkosc + offset1;
      analogWrite(PWM1, rozpedzanie1);
    }

    if (rozpedzanie2 < predkosc + offset2)
    {
      analogWrite(PWM2, rozpedzanie2);
      rozpedzanie2 += przyspieszenie;
      if (rozpedzanie2 + offset2 > 255)
        rozpedzanie2 = 255;
    }
    else
    {
      rozpedzanie2 = predkosc + offset2;
      analogWrite(PWM2, rozpedzanie2);
    }

    if (rozpedzanie11 < predkosc + offset11)
    {
      analogWrite(PWM11, rozpedzanie11);
      rozpedzanie11 += przyspieszenie;
      if (rozpedzanie11 + offset11 > 255)
        rozpedzanie11 = 255;
    }
    else
    {
      rozpedzanie11 = predkosc + offset11;
      analogWrite(PWM11, rozpedzanie11);
    }

    if (rozpedzanie22 < predkosc + offset22)
    {
      analogWrite(PWM22, rozpedzanie22);
      rozpedzanie22 += przyspieszenie;
      if (rozpedzanie22 + offset22 > 255)
        rozpedzanie22 = 255;
    }
    else
    {
      rozpedzanie22 = predkosc + offset22;
      analogWrite(PWM22, rozpedzanie22);
    }
  }

  ograniczenie = map(predkosc, PWM_min, PWM_max, 12, 5);

  if (millis() - odczyt > 20 && error != 1)
  {
    ps2x.read_gamepad();
    if (ps2x.Analog(PSS_LY) < 127 && !(ps2x.Analog(PSS_RX) < 127) && !(ps2x.Analog(PSS_RX) > 128))
    {
      sprawdz = true;
      czas();
      prosto();
    }
    else if (ps2x.Analog(PSS_LY) > 128  && !(ps2x.Analog(PSS_RX) < 127) && !(ps2x.Analog(PSS_RX) > 128))
    {
      sprawdz = true;
      czas();
      wstecz();
    }
    else if (ps2x.Analog(PSS_RX) < 127)
    {
      sprawdz = true;
      czas();
      lewo();
    }
    else if (ps2x.Analog(PSS_RX) > 128)
    {
      sprawdz = true;
      czas();
      prawo();
    }
    else
    {
      zatrzymaj();
      if (flaga_czasu_zatrzymania)
      {
        czas_wczesniejszy = millis();
        flaga_czasu_zatrzymania = false;
      }
    }
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
    {
      sprawdz = true;
      szybciej();
    }
    if (ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2))
    {
      sprawdz = true;
      wolniej();
    }
    odczyt = millis();
  }

  wyslij_dane();
}

void serialEvent1()
{
  int bytes = Serial1.available();
  if (bytes > 0)
  {
    String dane = "";
    dane += char(Serial1.read());
    if (dane.equals("u"))
    {
      if (ustawienia)
        ustawienia = false;
      else
        ustawienia = true;
    }
    else if (ustawienia)
    {
      for (int i = 0; i < 3; i++)
        dane += char(Serial1.read());
      if (dane.equals("off,"))
      {
        char zmienna = char(Serial1.read());
        dane = "";
        char znak = char(Serial1.read());
        dane += znak;
        for (int i = 0; znak != '\n'; i++)
        {
          znak = char(Serial1.read());
          if (znak != '\n')
            dane += znak;
        }
        if (zmienna == '2')
        {
          offset1 = dane.toFloat();
          if (offset1 + PWM_min < 0)
            offset1 = -PWM_min;
          if (offset1 + PWM_max > 255)
            offset1 = 255 - PWM_max;
        }
        else if (zmienna == '1')
        {
          offset2 = dane.toFloat();
          if (offset2 + PWM_min < 0)
            offset2 = -PWM_min;
          if (offset2 + PWM_max > 255)
            offset2 = 255 - PWM_max;
        }
        else if (zmienna == '4')
        {
          offset11 = dane.toFloat();
          if (offset11 + PWM_min < 0)
            offset11 = -PWM_min;
          if (offset11 + PWM_max > 255)
            offset11 = 255 - PWM_max;
        }
        else if (zmienna == '3')
        {
          offset22 = dane.toFloat();
          if (offset22 + PWM_min < 0)
            offset22 = -PWM_min;
          if (offset22 + PWM_max > 255)
            offset22 = 255 - PWM_max;
        }
      }
    }
    else
      while (Serial1.available() > 0)
        Serial1.read();
  }
}

void serialEvent2()
{
  if (Serial2.available() > 0)
  {
    sprawdz = true;
    char dane = Serial2.read();

    if (dane == '1')
    {
      czas();
      prosto();
    }
    else if (dane == '2')
    {
      czas();
      wstecz();
    }
    else if (dane == '3')
    {
      czas();
      lewo();
    }
    else if (dane == '4')
    {
      czas();
      prawo();
    }
    else if (dane == '0')
    {
      zatrzymaj();
      if (flaga_czasu_zatrzymania)
      {
        czas_wczesniejszy = millis();
        flaga_czasu_zatrzymania = false;
      }
    }
    else if (dane == '8')
      szybciej();
    else if (dane == '9')
      wolniej();

    while (Serial2.available())             //czyszczenie buffora
      Serial2.read();
  }
}

void receiveEvent(int bytes) {
  if (Wire.available() > 0)
    Wire.readBytesUntil('\n', dane2, bytes);
}

void RL()
{
  if (millis() - t1 >= ograniczenie)
  {
    if (flaga_prosto || flaga_zawracanie_p)
    {
      licznik_RL++;
      licznik_RLL++;
    }
    else if (flaga_wstecz || flaga_zawracanie_l)
    {
      licznik_RL--;
      licznik_RLL++;
    }
  }
  t1 = millis();
  if (licznik_RLL == 2)
  {
    predkosc_RL = 2.04 / 2 / (millis() - t3) * 1000;
    licznik_RLL = 0;
    t3 = millis();
  }
}

void RR()
{
  if (millis() - t2 >= ograniczenie)
  {
    if (flaga_prosto || flaga_zawracanie_l)
    {
      licznik_RR++;
      licznik_RRR++;
    }
    else if (flaga_wstecz || flaga_zawracanie_p)
    {
      licznik_RR--;
      licznik_RRR++;
    }
  }
  t2 = millis();
  if (licznik_RRR == 2)
  {
    predkosc_RR = 2.04 / 2 / (millis() - t4) * 1000;
    licznik_RRR = 0;
    t4 = millis();
  }
}

void FL()
{
  if (millis() - t5 >= ograniczenie)
    licznik_FLL++;
  t5 = millis();
  if (licznik_FLL == 2)
  {
    predkosc_FL = 2.04 / 2 / (millis() - t7) * 1000;
    licznik_FLL = 0;
    t7 = millis();
  }
}

void FR()
{
  if (millis() - t6 >= ograniczenie)
    licznik_FRR++;
  t6 = millis();
  if (licznik_FRR == 2)
  {
    predkosc_FR = 2.04 / 2 / (millis() - t8) * 1000;
    licznik_FRR = 0;
    t8 = millis();
  }
}
//********************************************************************************************************************************************************************

void wyslij_dane()
{
  if (DEBUG)
  {
    Serial.print(napiecie);
    Serial.print(", ");
    Serial.print(predkosc / 255, 2);
    Serial.print(", ");
    Serial.print(rozpedzanie / (predkosc + 0.01), 2);
    Serial.print(", ");
    Serial.print(licznik_RL);
    Serial.print(", ");
    Serial.print(licznik_RR);
    Serial.print(", ");
    if (millis() - t3 > ograniczenie2)
      Serial.print("0");
    else
      Serial.print(predkosc_RL);
    Serial.print(", ");
    if (millis() - t4 > ograniczenie2)
      Serial.print("0");
    else
      Serial.print(predkosc_RR);
    Serial.print(", ");
    if (millis() - t7 > ograniczenie2)
      Serial.print("0");
    else
      Serial.print(predkosc_FL);
    Serial.print(", ");
    if (millis() - t8 > ograniczenie2)
      Serial.print("0");
    else
      Serial.print(predkosc_FR);
    Serial.print(", ");
    Serial.println(dane2);
  }

  if (ustawienia)
  {
    Serial1.print("ust, ");
    Serial1.print(predkosc / 255, 2);
    Serial1.print(", ");
    Serial1.print(offset1 / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offset2 / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offset11 / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offset22 / 2.55, 1);
    Serial1.print(", ");
    //    Serial1.print(rozpedzanie / (predkosc + 0.01), 2);
    //   Serial1.print(", ");
    //    Serial1.print(licznik_RL);
    //    Serial1.print(", ");
    //    Serial1.print(licznik_RR);
    //    Serial1.print(", ");
    if (millis() - t3 > ograniczenie2)
      Serial1.print("0");
    else
      Serial1.print(predkosc_RL);
    Serial1.print(", ");
    if (millis() - t4 > ograniczenie2)
      Serial1.print("0");
    else
      Serial1.print(predkosc_RR);
    Serial1.print(", ");
    if (millis() - t7 > ograniczenie2)
      Serial1.print("0");
    else
      Serial1.print(predkosc_FL);
    Serial1.print(", ");
    if (millis() - t8 > ograniczenie2)
      Serial1.print("0");
    else
      Serial1.print(predkosc_FR);
    Serial1.print(", ");
    Serial1.println(dane2);
  }
  else
  {
    Serial1.print(predkosc / 255, 2);
    Serial1.print(", ");
    Serial1.print(napiecie);
    Serial1.print(", ");
    Serial1.print(licznik_RL);
    Serial1.print(", ");
    Serial1.print(licznik_RR);
    Serial1.print(", ");
    Serial1.println(dane2);
  }

  Serial2.print(predkosc / 255, 2);
  Serial2.print(", ");
  Serial2.println(rozpedzanie / (predkosc + 0.01), 2);
}

void czas()
{
  timer = millis() - czas_wczesniejszy;
}

void szybciej()
{
  predkosc += przyr_predkosci;
  if (predkosc > PWM_max)
  {
    if (error != 1)
      ps2x.read_gamepad(true, 0);
    predkosc = PWM_max;
  }
}

void wolniej()
{
  predkosc -= przyr_predkosci;
  if (predkosc < PWM_min)
  {
    if (error != 1)
      ps2x.read_gamepad(true, 0);
    predkosc = PWM_min;
  }
}

void prosto()
{
  if ((flaga_wstecz == true || flaga_zawracanie == true) && flaga_jedzie == true)
  {
    zatrzymaj();
    wyslij_dane();

    delay(postoj);
  }
  if (timer < postoj && flaga_prosto == false)
  {
    zatrzymaj();
    wyslij_dane();

    licznik = postoj - timer;
    delay(licznik);
  }

  digitalWrite(IN1, LOW); //Silnik prawy - obroty w lewo
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); //Silnik lewy - obroty w prawo
  digitalWrite(IN4, LOW);

  digitalWrite(IN11, LOW); //Silnik prawy - obroty w lewo
  digitalWrite(IN22, HIGH);
  digitalWrite(IN33, HIGH); //Silnik lewy - obroty w prawo
  digitalWrite(IN44, LOW);

  flaga_jedzie = true;
  flaga_prosto = true;
  flaga_wstecz = false;
  flaga_zawracanie = false;
  flaga_czasu_zatrzymania = true;
  flaga_zawracanie_l = false;
  flaga_zawracanie_p = false;
  rozpedzaj = true;
}

void wstecz()
{
  if ((flaga_prosto == true || flaga_zawracanie == true) && flaga_jedzie == true)
  {
    zatrzymaj();
    wyslij_dane();

    delay(postoj);
  }
  if (timer < postoj && flaga_wstecz == false)
  {
    zatrzymaj();
    wyslij_dane();

    licznik = postoj - timer;
    delay(licznik);
  }

  digitalWrite(IN1, HIGH); //Silnik prawy - obroty w lewo
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); //Silnik lewy - obroty w prawo
  digitalWrite(IN4, HIGH);

  digitalWrite(IN11, HIGH); //Silnik prawy - obroty w lewo
  digitalWrite(IN22, LOW);
  digitalWrite(IN33, LOW); //Silnik lewy - obroty w prawo
  digitalWrite(IN44, HIGH);

  flaga_jedzie = true;
  flaga_wstecz = true;
  flaga_prosto = false;
  flaga_zawracanie = false;
  flaga_czasu_zatrzymania = true;
  flaga_zawracanie_l = false;
  flaga_zawracanie_p = false;
  rozpedzaj = true;
}

void prawo()
{
  if (flaga_jedzie == false || flaga_zawracanie == true)
  {
    if (timer < postoj && flaga_zawracanie_p == false)
    {
      zatrzymaj();
      wyslij_dane();

      licznik = postoj - timer;
      delay(licznik);
    }
    else if (flaga_zawracanie_l == true)
    {
      zatrzymaj();
      wyslij_dane();

      delay(postoj);
    }

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    digitalWrite(IN11, HIGH);
    digitalWrite(IN22, LOW);
    digitalWrite(IN33, HIGH);
    digitalWrite(IN44, LOW);

    rozpedzaj = true;

    flaga_zawracanie_p = true;
    flaga_zawracanie_l = false;
    flaga_jedzie = true;
    flaga_zawracanie = true;
    flaga_prosto = false;
    flaga_wstecz = false;
    flaga_czasu_zatrzymania = true;
  }
  else
  {
    if (flaga_wstecz == true)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      digitalWrite(IN11, LOW);
      digitalWrite(IN22, LOW);
      digitalWrite(IN33, LOW);
      digitalWrite(IN44, HIGH);
    }
    else if (flaga_prosto == true)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

      digitalWrite(IN11, LOW);
      digitalWrite(IN22, LOW);
      digitalWrite(IN33, HIGH);
      digitalWrite(IN44, LOW);
    }
    flaga_zawracanie = false;
    flaga_jedzie = true;
    flaga_czasu_zatrzymania = true;
  }
}

void lewo()
{
  if (flaga_jedzie == false || flaga_zawracanie == true)
  {
    if (timer < postoj && flaga_zawracanie_l == false)
    {
      zatrzymaj();
      wyslij_dane();

      licznik = postoj - timer;
      delay(licznik);
    }
    else if (flaga_zawracanie_p == true)
    {
      zatrzymaj();
      wyslij_dane();

      delay(postoj);
    }

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    digitalWrite(IN11, LOW);
    digitalWrite(IN22, HIGH);
    digitalWrite(IN33, LOW);
    digitalWrite(IN44, HIGH);

    rozpedzaj = true;

    flaga_zawracanie_p = false;
    flaga_zawracanie_l = true;
    flaga_jedzie = true;
    flaga_zawracanie = true;
    flaga_prosto = false;
    flaga_wstecz = false;
    flaga_czasu_zatrzymania = true;
  }
  else
  {
    if (flaga_wstecz == true)
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);

      digitalWrite(IN11, HIGH);
      digitalWrite(IN22, LOW);
      digitalWrite(IN33, LOW);
      digitalWrite(IN44, LOW);
    }
    else if (flaga_prosto == true)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);

      digitalWrite(IN11, LOW);
      digitalWrite(IN22, HIGH);
      digitalWrite(IN33, LOW);
      digitalWrite(IN44, LOW);
    }
    flaga_zawracanie = false;
    flaga_jedzie = true;
    flaga_czasu_zatrzymania = true;
  }
}

void zatrzymaj()
{
  digitalWrite(IN1, LOW); //zwarcie silnikow
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(IN11, LOW); //zwarcie silnikow
  digitalWrite(IN22, LOW);
  digitalWrite(IN33, LOW);
  digitalWrite(IN44, LOW);

  analogWrite(PWM1, PWM_min);
  analogWrite(PWM2, PWM_min);
  analogWrite(PWM11, PWM_min);
  analogWrite(PWM22, PWM_min);
  rozpedzaj = false;
  rozpedzanie = 0;
  rozpedzanie1 = 0;
  rozpedzanie2 = 0;
  rozpedzanie11 = 0;
  rozpedzanie22 = 0;
  flaga_jedzie = false;
}

void delay(int x)
{
  long y = millis();
  while (millis() - y < x)
    wyslij_dane();
}
