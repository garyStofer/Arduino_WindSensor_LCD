#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>

#define Enc_A_PIN 2    // This genetates the interrupt, providing the click
#define Enc_B_PIN 10    // This is providing the direction 
#define Enc_PRESS_PIN 3

#define LONGPRESS_TIMEOUT 100000
char EncoderCnt = 0;
char EncoderPressedCnt = 0;
unsigned char ShortPressCnt = 0;
unsigned char LongPressCnt = 0;

#define UPDATE_PER 1000000   // one second, this is the measure interval and also the LCD update interval
#define GUST_PER 600         // that is 600 one seconds worth

struct tagCalData
{
  int WDir_min;
  int WDir_max;
  int WDir_offs;
  unsigned char AnemoCnt;
  float AnemoFact;
} WXCalib = {63, 602, 0, 8, 2.50};

// array to keep 10 minutes of winddata for gust and average calculations
unsigned char Wind_Gust[GUST_PER];



// Using the nLCD library from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
// or the Arduino LCD lib that comes with it.
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // in 4 bit interface mode

/* for conntrastcontrol
#define   CONTRAST_PIN   12
#define   CONTRAST       110

  // Set PWM for contrast levels
  pinMode(CONTRAST_PIN, OUTPUT);
  analogWrite ( CONTRAST_PIN, CONTRAST );
/*
  This ISR handles the debounce and reading of a quadrature encoder knob and increments or decrements  two global encoder count  variables
  depending on the direction of the encoder and the state of the button. The encoder functions at 1/4 the maximal possible resolution and generally provides one inc/dec
  per mechanical detent.

  One of the quadrature inputs is used to trigger this interupt handler, which checks the other quadrature input to decide the direction.
  A dead time after an initial deection is used to block out bounces.  It is assumed that the dirction qaudrature signal is not bouncing while
  the first phase is causing the initial interrupt as it's signal is 90deg opposed.

  Two encoder count variables are beeing modified by this ISR
     A: EncoderCnt when the knob is turned whithot the button being press
     B: EncoderPressCnt when the knob is  turned while the button is also pressed. This happens only after a LONG press timeout
  */

void
ISR_0( void)
{
  if ( digitalRead( Enc_PRESS_PIN ) )
  {
    if ( digitalRead( Enc_B_PIN ) )
      EncoderCnt++;
    else
      EncoderCnt--;

  }
  else
  {
    if ( digitalRead( Enc_B_PIN ) )
      EncoderPressedCnt++;
    else
      EncoderPressedCnt--;
  }

}

/*
  ISR to handle the button press interrupt. Two modes of button presses are recognized. A short, momentary press and a long, timing out press.
   While the hardware debounced button signal is sampled for up to TIMEOUT time in this ISR no other code is beeing executed. If the timeout occurs
   a long button press-, otherwise a short button press is registered. Timing has to be done by a software counter since interrupts are disabled and function millis() and micros()
   don't advance in time. Software timing is of course CPU clock dependent an d therefore has to be adjusted to the clock frequency if it is not running at 16Mhz.
*/
void
ISR_1(void)
{
  volatile unsigned long t = 0;
  while ( !digitalRead( Enc_PRESS_PIN ))
  {
    if (t++ > LONGPRESS_TIMEOUT)
    {
      LongPressCnt++;
      return;
    }
  }
  ShortPressCnt++;
}

static volatile unsigned short WindCnt = 0;

// Pin change interrupt to capture the edges of the wind speed interrupter
ISR(PCINT1_vect)
{
  // check PCINT1 interrupt flags for the wind_count pin if any other pinchange interrupts are used in this code
  WindCnt++;  // count every edge from the wind sensor
}

void setup() {


  // Setup the Encoder pins to be inputs with pullups
  pinMode(Enc_A_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_B_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_PRESS_PIN, INPUT);// Use external 10K pullup and 100nf to gnd for debounce

  pinMode(13, OUTPUT);      // the LED
  pinMode(14, INPUT);       // the Windspeed count

  lcd.begin(20, 4);              // initialize the lcd coloums and rows

  lcd.home ();                   // go home
  lcd.print("Wind Display");
  lcd.setCursor ( 0, 1 );
  lcd.print("DIR: ");
  lcd.setCursor ( 0, 2 );
  lcd.print("SPD: ");
  lcd.setCursor ( 0, 3 );
  lcd.print("Gst: ");

  attachInterrupt(0, ISR_0, FALLING);    // for the roraty encoder knob
  attachInterrupt(1, ISR_1, FALLING);    // for the roraty encoder knob

  //Serial.begin(57600);
  //Serial.print("Wind Display Demo\n");

  // enable the pin change interrupt for PC0, aka, A0, aka pin14
  PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka Portc0, aka A0
  PCICR |= 1 << PCIE1; // enable PCinterupt 1 vector


}

void loop()
{
  unsigned short adc_val;
  unsigned short wind_count;
  float wind_speed;
  long wind_dir;    // needs to be long for overflow protection in math
  static unsigned long t = 0;
  static unsigned short GustNdx = 0;
  unsigned char wind_gust;


  if ( micros() < t)  // wait until next update period
  {
    // Todo: menu stuff in here
    return;
  }


  // disable the interrupt so we can get a two byte variable without getting interrupted
  PCICR = 0;  // disable PCinterupt 1 vector
  wind_count = WindCnt;     // get the count from the pin change interrupt
  WindCnt = 0;
  t = micros() + UPDATE_PER; // every second we update the display with new data
  PCICR |= 1 << PCIE1; // enable PCinterupt 1 vector

  // calc and store the current wind speed
  wind_speed = (wind_count * WXCalib.AnemoFact) / (2*WXCalib.AnemoCnt);   // 2.5 miles/rev/sec; div by 2x counts per revolution because we count both edges in the interrupt.
  Wind_Gust[GustNdx] = (unsigned char) (wind_speed +0.5); // store current measure wind speed in uchar, use rounding. 
  GustNdx = ++GustNdx % GUST_PER;  // Advance to next position, wrap around 

  // calc and store the current wind dir
  adc_val = analogRead(3);    // read the input pin
  wind_dir = ((adc_val - WXCalib.WDir_min) *  360L) / (WXCalib.WDir_max - WXCalib.WDir_min);
  wind_dir += WXCalib.WDir_offs;

  // Precaution in case the calib values are wrong or slightly off
  if (wind_dir > 360)
    wind_dir = 360;
  else if (wind_dir < 0)
    wind_dir = 0;

  wind_dir %= 360;

  // now, finally, go throuh the 1 second array and take the highest number for the last 10 minutes.
  wind_gust = 0;
  for ( unsigned short i = 0; i < GUST_PER; i++)
  {
    if (Wind_Gust[i] > wind_gust)
      wind_gust = Wind_Gust[i] ;    // this is the highest in the last 10 minutes
  }

  lcd.setCursor ( 5, 1 );
  lcd.print(wind_dir);
  lcd.print(char(223)); // degree symbol
  lcd.print("  "); // to clear lcd after change from 3 to 1 digit display

  lcd.setCursor ( 5, 2 );
  lcd.print(wind_speed);
  lcd.print(" mph  "); // to clear lcd after change from 3 to 1 digit displ

  lcd.setCursor ( 5, 3 );
  lcd.print(wind_gust);
  lcd.print(" mph  ");


  if (digitalRead(13) == HIGH)
    digitalWrite(13, LOW);
  else
    digitalWrite(13, HIGH);

}
