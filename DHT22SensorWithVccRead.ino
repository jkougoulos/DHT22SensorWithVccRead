// TODO: power off DHT when in SLEEP -> does not help, 2s delay

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

//#define HWUNO
#define HWMOTE
#define HWVCCREAD

#define DEBUGSENSOR
//#define DEBUGSENSORTIMING

//#define STATICID 3
#ifdef DEBUGSENSOR
  #define NUM_8S_SLEEP 1
#else
  #define NUM_8S_SLEEP 15
#endif

#ifdef HWUNO
  #define RNDDELAY 1000
  #define RANDOMSEED_PINUSED  A0
#endif

#ifdef HWVCCREAD
#define BATDIV_PIN 3
#define VCCREAD_PIN A0
#endif

#ifndef STATICID
  #define ID_DIP3_PIN 7
  #define ID_DIP2_PIN 6
  #define ID_DIP1_PIN 5
#endif

#ifdef HWUNO
  #include <RH_ASK.h>
  #define RADIO_SPEED 2400
  #define RADIO_RX_PIN 11
  #define RADIO_TX_PIN 12
  #define RADIO_PTT_PIN 10
  RH_ASK driver( RADIO_SPEED, RADIO_RX_PIN, RADIO_TX_PIN, RADIO_PTT_PIN); // speed RX,TX, ptt?
#endif

#ifdef HWMOTE
  #include <RH_RF69.h>
  RH_RF69 driver;
#endif

#include <DHT.h>
#define DHTTYPE DHT22
#define DHTPIN 4            // Data pin (do not add 5.1k-10k resistor between vcc & data, is defined as PULLUP input )
DHT dht(DHTPIN, DHTTYPE);

#ifdef HWUNO
  #define TXLED LED_BUILTIN
#endif
#ifdef HWMOTE
  #define TXLED 9 // is Moteino LED_BUILTIN
#endif 

#define SERIAL_SPEED 9600

char data[255];
int Temp = 0;
int Humidity = 0 ;
int myID = 9999;
int Vcc = 0;
bool criticalSection = 1;
unsigned long Cycle = 0;


void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds
  sei();                           // re-enable interrupts 
}

ISR(WDT_vect)
{
   //wdt_disable();
   if( !criticalSection )
   {
      wdt_reset();
   }
   else
   {
      // must be rebooted
      // configure
      MCUSR = 0;                          // reset flags
       
                                            // Put timer in reset-only mode:
      WDTCSR |= 0b00011000;               // Enter config mode.
      WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                            // set WDE (reset enable...4th from left), and set delay interval
                                            // reset system in 16 ms...
                                            // unless wdt_disable() in loop() is reached first
                                       
      // reboot
      while(1);
   }
}
  
void checkRFINIT()
{
  digitalWrite( TXLED, LOW );
  if (!driver.init())
  {
    Serial.println("Driver init failed");
  }
#ifdef HWMOTE
  if ( !driver.setFrequency(433.0) )
  {
    Serial.println("Radio setFrequency failed");
  }
//  driver.setTxPower(-18);
#endif
  return;
}

void getConfig()
{
#ifdef STATICID
  myID = STATICID;
#else
  pinMode( ID_DIP3_PIN, INPUT_PULLUP );
  pinMode( ID_DIP2_PIN, INPUT_PULLUP );
  pinMode( ID_DIP1_PIN, INPUT_PULLUP );

  int idbit0 = digitalRead( ID_DIP3_PIN );
  int idbit1 = digitalRead( ID_DIP2_PIN );
  int idbit2 = digitalRead( ID_DIP1_PIN );

  myID = 0;
  bitWrite(myID,2,!idbit2);
  bitWrite(myID,1,!idbit1);
  bitWrite(myID,0,!idbit0);

  #ifdef DEBUGSENSOR
  Serial.print("CFGB0:");
  Serial.print(idbit2);  
  Serial.print(" CFGB1:");
  Serial.print(idbit1);  
  Serial.print(" CFGB2:");
  Serial.print(idbit0);  
  Serial.print(" ID:");
  Serial.print(myID);
  Serial.println();
  delay(100);
  #endif

  pinMode( ID_DIP3_PIN, OUTPUT);
  digitalWrite( ID_DIP3_PIN, LOW );
  pinMode( ID_DIP2_PIN, OUTPUT);
  digitalWrite( ID_DIP2_PIN, LOW );
  pinMode( ID_DIP1_PIN, OUTPUT);
  digitalWrite( ID_DIP1_PIN, LOW );

#endif  
}

void transmitInfo()
{
  digitalWrite( TXLED, HIGH );
  driver.send( (uint8_t *)data, strlen(data));
  driver.waitPacketSent();
  digitalWrite( TXLED, LOW );
  return;  
}

void sleepNow(int num8s)         // here we put the arduino to sleep
{
#ifdef DEBUGSENSOR
    Serial.println("Going to sleep!");
    delay(100);
#endif

#ifdef HWMOTE
    driver.sleep();
#endif
    for( int i = 0 ; i < num8s ; i++ )
    {
        ADCSRA &= ~(1 << ADEN);
        power_adc_disable();
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
        cli();
        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
        sei();
    }

#ifdef DEBUGSENSOR
    delay(100);
    Serial.println("Woke up!");
    delay(100);
#endif
}

void setup() {
  // put your setup code here, to run once:
  configure_wdt();
  
  Serial.begin(SERIAL_SPEED);
  Serial.println("Initializing");
  pinMode( TXLED, OUTPUT );
#ifdef HWVCCREAD
  pinMode( BATDIV_PIN, OUTPUT );
  digitalWrite( BATDIV_PIN, LOW );
#endif
  
  checkRFINIT();
  getConfig();

#ifdef HWUNO
  randomSeed(analogRead(RANDOMSEED_PINUSED));
#endif

  dht.begin();  
  delay(2000);  //DHT says that needs to wait 2 seconds
  wdt_reset(); 
  
  Serial.println("Everything ok, Starting loop!");
  delay(100);
  wdt_reset();  
}

void loop() {

#ifdef DEBUGSENSORTIMING
  long a1 = millis();
#endif  
  int Humidity = (int)(dht.readHumidity(true)*10);  // adafruit library returns float... useless in arduino
  int Temp = (int)(dht.readTemperature(false,true)*10);   // adafruit library returns float... useless in arduino
  // Read temperature as Fahrenheit (isFahrenheit = true)
  if (isnan(Humidity) || isnan(Temp)) {
    Temp = 0; Humidity = 0;
    Serial.println("Failed to read from DHT sensor!");
  }

#ifdef HWVCCREAD
  power_adc_enable();
  ADCSRA |= (1 << ADEN);
  analogReference(INTERNAL);
  digitalWrite(BATDIV_PIN, HIGH);
  delay(100); 
#ifdef DEBUGSENSOR
  delay(5000); wdt_reset(); // just to have time to measure things
#endif
  int Vraw = analogRead(A0);
  digitalWrite(BATDIV_PIN, LOW);
  float vRef = 1.1;
  float divider = (10.0 + 47.0 + 47.0) / 10.0;
  float Volt = Vraw *( vRef / 1024.0 ) * divider ;
  Vcc = Volt * 100;
#endif

#ifdef DEBUGSENSOR
  Serial.print("TEMP: ");
  Serial.print(Temp);
  Serial.print("/");
  Serial.print(" HUM: ");
  Serial.print(Humidity);
  Serial.print(" VOLT: ");
  Serial.print(Vcc);
  Serial.print(":");
  Serial.print(Vraw);
  Serial.print(" ID: ");
  Serial.print(myID);
  Serial.print(" CYCLE: ");
  Serial.print(Cycle);
  Serial.println();
  delay(100);
#endif
  
  sprintf(data,"<ID0:%d/MS0:%ld/C00:%ld/V02:%d/T01:%d/H01:%d>\n", myID, millis(),++Cycle,Vcc,Temp,Humidity);
  transmitInfo();
#ifdef DEBUGSENSORTIMING
  long a2 = millis();
#endif
  criticalSection = 0;
  sleepNow(NUM_8S_SLEEP); 
  criticalSection = 1;
#ifdef DEBUGSENSORTIMING
  long a3 = millis();
  Serial.print( "ms in sleep: " );
  Serial.println(a3 - a2);
  Serial.print( "ms in loop: " );
  Serial.println(a2 - a1);
//  delay(100);
#endif

#ifdef HWUNO  
  delay( random(0,RNDDELAY) ); //random delay to prevent collisions
#endif  
}
