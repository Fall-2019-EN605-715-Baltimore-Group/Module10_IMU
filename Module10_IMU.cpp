/*
 * Module10_IMU
 * Reads the measurement of the BNO055 Orientation sensor.
 * Results are placed on the serial connection at a 10 Hz Rate
 */

#include <Wire.h>               // I2C connectivity
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>    // IMU connectivity
#include <utility/imumaths.h>   // Used for orientation calculations

/*
 * The following macros enable functionality to print
 * measurements in the units present by the name.
 */

//#define FEATURE_EVENT
#define FEATURE_EULER

// Create the sensor object
Adafruit_BNO055 imuObj = Adafruit_BNO055(55);

void setup(void)
{
    // Init serial communications
    Serial.begin(115200);

    // Init sensor
    if(!imuObj.begin())
    {
        //There was a problem detecting the BNO055 ... check your connections
        Serial.print("IMU not detected, check connections and restart");
        while(1);
    }
    else
    {
        Serial.println("IMU connected"); Serial.println("");
    }

    delay(1000);

    imuObj.setExtCrystalUse(true);
}

void loop(void)
{
#ifdef FEATURE_EVENT
    //Get a new sensor measurement
    sensors_event_t event;
    imuObj.getEvent(&event);

    //Display the floating point data
    Serial.print("X: ");    Serial.print(event.orientation.x, 4);
    Serial.print(" Y: ");  Serial.print(event.orientation.y, 4);
    Serial.print(" Z: ");  Serial.print(event.orientation.z, 4);
    Serial.println("");
#elif defined(FEATURE_EULER)
    //Get a new sensor measurement
    imu::Vector<3> euler = imuObj.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");    Serial.print(euler.x());
    Serial.print(" Y: ");   Serial.print(euler.y());
    Serial.print(" Z: ");   Serial.print(euler.z());
    Serial.println("");
#endif

    delay(100);     // wait until next measuremnt is needed.
}   // END void loop()
