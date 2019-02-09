/* 
  * A modification of the Adafruit_TSL2561_U library is
  * included under the name JASON_TSL2561_U . The modification
  * took place in order to be able to use the convenient 
  * functions provided in the Adafruit library with the
  * SoftwareWire I2C instance.
  * 
  * The aim of the project is to collect data (temperature, lux
  * humidity) using the custom made board. 
  * Collected data are logged into an SD card.
  * 
  * @author Jason Manoloudis
  *
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <JASON_TSL2561_U.h>
#include <SPI.h>
#include "SparkFunTMP102.h"
#include "Adafruit_Si7021.h"
#include <SoftwareWire.h>
#include <Adafruit_TSL2561_U.h>
#include <SD.h>
#include <string.h>

//Instances with I2C addresses, where applicable
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 1); 
Adafruit_TSL2561_Unified tsl2 = Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 2); // R5 GND
Adafruit_TSL2561_Unified tsl3 = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 3); // R4 VCC
TMP102 tmp(0x48); // R7 GND
TMP102 tmp2(0x4B); //R3 SCL
TMP102 tmp3(0x4A); //R6 SDA
Adafruit_Si7021 si7021 = Adafruit_Si7021();

SoftwareWire myWire(8,9);

JASON_TSLJASON_Unified tslll = JASON_TSLJASON_Unified(TSL2561_ADDR_FLOAT,4);
JASON_TSLJASON_Unified tslll2 = JASON_TSLJASON_Unified(TSL2561_ADDR_HIGH,5); //R4 VCC
TMP102 tmppp(0x48); // R7 GND
TMP102 tmppp2(0x4B); //R3 SCL


float temperature;
long lux;
float humidity;
char dataString[81] = "";
const int CSpin = 10; //SD breakout board Chip Select
File root;
String newestName;
long converted;


/* @brief: save data in SD
 * @param filename: the name of file to save into
 * @param dtString: the char array to be written in file
*/
void saveData(String filename, char dtString[])
{
  File file = SD.open(filename,FILE_WRITE);
 // Serial.print(F("FreeRam: "));Serial.println(FreeRam());
  if(file)
  {
    file.print(dtString);
    file.close();
  }else
  {
    //Serial.println(F("Error writing in file"));
    while(1);
  }
}

/* @brief: Count the number of existing files in SD to generate next file
 * @param root: the reference file  
 * @return newName: the name of the new file (num_of_files+1.csv)
*/
String countFiles(File root)
{
  int fileCounter = 0;
  while(1)
  {
    File file2 = root.openNextFile();
    if(!file2) 
    {
      String newName = "data" + String(fileCounter) + ".csv";
      return newName;
    }
    fileCounter ++;
    file2.close();
  }
}

/* Optional configuration settings */
void configure()
{
  tmp.setConversionRate(2);
  tmp.setExtendedMode(0);
  tmp2.setConversionRate(2);
  tmp2.setExtendedMode(0);
  tmp3.setConversionRate(2);
  tmp3.setExtendedMode(0);
  tsl.enableAutoRange(true); //Auto gain
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
  tsl2.enableAutoRange(true); //Auto gain
  tsl2.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS); 
  tsl3.enableAutoRange(true); //Auto gain
  tsl3.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
}

void setup() 
{
  //Serial.begin(9600);
  if(!si7021.begin()) {/*Serial.println(F("Problem with si7021"));*/ while(true);}
  tmp.begin();
  tmp2.begin();
  tmp3.begin();
  if(!tsl.begin()) {/*Serial.println(F("Problem with tsl 1")); */ while(true);}
  if(!tsl2.begin()) {/*Serial.println(F("Problem with tsl 2")); */while(true);}
  if(!tsl3.begin()) {/*Serial.println(F("Problem with tsl 3"));*/ while(true);}

  myWire.begin();

  if(!tslll.begin(&myWire)) {/*Serial.println(F("problem with soft i2c tsl"));*/ while(true);}
  if(!tslll2.begin(&myWire)) {/*Serial.println(F("problem with soft i2c tsl 2")); */while(true);}
  
  configure();

  pinMode(CSpin, OUTPUT); //Set SD ChipSelect pin as output even if it is not used

  /* Generate new filename & add "headers" line */
  if(!SD.begin(CSpin)) {/*Serial.println(F("Card failed"));*/ while(1);}
  root = SD.open("/"); 
  newestName = countFiles(root);
  root.close();
  char firstRow[] = "temp1;temp2;temp3;temp4;temp5;hum;lux1;lux2;lux3;lux4;lux5;\n"; 
  saveData(newestName, firstRow);
}

/* @brief: convert float to char array and append it in dataString
 * @param f: the number to be converted & saved
*/
void changeToStringAndConcat(float f)
{
  char temp[12];
  dtostrf(f,6,2,temp);
  strcat(dataString, temp);
  strcat(dataString, ";");
}

void loop() 
{
  sensors_event_t event;
  sensors_event_t event2;
  sensors_event_t event3;
  sensors_event_t event10;
  sensors_event_t event11; 

 /* Hardware I2C TMP102 */
  tmp.wakeup();
  temperature = tmp.readTempC();
  tmp.sleep();
  changeToStringAndConcat(temperature);
  
  tmp2.wakeup();
  temperature = tmp2.readTempC();
  tmp2.sleep();
  changeToStringAndConcat(temperature);

  tmp3.wakeup();
  temperature = tmp3.readTempC();
  tmp3.sleep();
  changeToStringAndConcat(temperature);

  /* Software I2C TMP102 */
  myWire.beginTransmission(0x48);
  myWire.write(uint8_t(0x00));
  myWire.endTransmission();
  myWire.requestFrom(0x48, 2);
  byte c = myWire.read();
  byte c2 = myWire.read();
  int temp = (((c<<8)|c2)>>4) & 0xFFF;
  if(bitRead(temp,11)==1) //Negative value is denoted with MSB = 1
  {
    temp -= 0x1000;
  }
  float temp2 = temp*0.0625;
  changeToStringAndConcat(temp2);
  
  myWire.beginTransmission(0x4B);
  myWire.write(uint8_t(0x00));
  myWire.endTransmission();
  myWire.requestFrom(0x4B, 2);
  c = myWire.read();
  c2 = myWire.read();
  temp = (((c<<8)|c2)>>4) & 0xFFF;
  if(bitRead(temp,11)==1) //Negative value is denoted with MSB = 1
  {
    temp -= 0x1000;
  }
  temp2 = temp*0.0625;
  changeToStringAndConcat(temp2);

  /* read from Si7021 humidity sensor */
  humidity = si7021.readHumidity();
  changeToStringAndConcat(humidity);

  /* Hardware I2C TSL2561 */
  tsl.getEvent(&event);
  lux = (long)event.light;
  changeToStringAndConcat(lux);
 
  tsl2.getEvent(&event2);
  lux = (long)event2.light;
  changeToStringAndConcat(lux);

  tsl3.getEvent(&event3); 
  lux = (long)event3.light;
  changeToStringAndConcat(lux);
  
  /* Software I2C TSL2561 */
  tslll.getEvent(&event10);
  lux = (long)event10.light;
  changeToStringAndConcat(lux);
   
  tslll2.getEvent(&event11);
  lux = (long)event11.light;
  changeToStringAndConcat(lux);

  strcat(dataString, "\n");
  saveData(newestName,dataString);
  dataString[0]=(char)0;
  delay(2000);
}
