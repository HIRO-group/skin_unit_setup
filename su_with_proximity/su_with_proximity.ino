#include <ESP8266WiFi.h>
#include <SparkFunLSM6DS3.h>
#include <SparkFun_VL53L1X.h>
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>


//////////////////////////////////////
//                                  //
//          Dependencies            //
//                                  //
//////////////////////////////////////
// Linux
// 1. Install rosserial, rosserial_arduino
// 2. Make ROS Libraries and Import them to Arduino
//    - rosrun rosserial_arduino make_libraries.py <Arduino Directory>/libraries/
// 3. After roboskin is ready
// 4. Launch roscore
// 5. Transmit serial info to ros topics
//    - rosrun rosserial_python serial_node.py tcp
// 6. rostopic echo /imu_data1


// Arduino
// 1. Install ESP8266 Board by adding a link to preferences page
// 2. Install Libraries  (Install zip and open from Arduino IDE)
//    - ESP8266Wifi Library
//    - SparkFunLSM6DS3 Library
//    - SparkFunVL53L1X Library

//////////////////////////////////////
//                                  //
//  THINGS THAT NEED TO BE CHANGED  //
//                                  //
//////////////////////////////////////
// WiFi Setting
const char* ssid     = "2995-euclid-7";
const char* password = "2995-euclid-7!";
//const char* ssid     = "HIROlab2";
//const char* password = "HIROlab322";
//IPAddress server(10,0,0,155);  // Chang/e this to ROS_MASTER_IP
//IPAddress server(172, 16, 0, 205);  // Chang/e this to ROS_MASTER_IP/
IPAddress server(10, 0, 0, 165);  // Chang/e this to ROS_MASTER_IP/
const uint16_t serverPort = 11411;
// ROS Setting
char imu_topic_name[] = "/imu_data1";
float publish_interval = 10;  // millisec
char distance_topic_name[] = "/proximity_data1";
int distance_g;
int seq = 0;

//////////////////////////////////////
//                                  //
//          Static Settings         //
//////////////////////////////////////
// ROS Setting
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_publisher(imu_topic_name, &imu_msg);
unsigned long imu_time_counter;
sensor_msgs::LaserScan distance_msg;
ros::Publisher distance_publisher(distance_topic_name, &distance_msg);


////////// Sensors Setting //////////
LSM6DS3 myIMU(I2C_MODE, 0x6a); //Default constructor is I2C, addr 0x6B
SFEVL53L1X distanceSensor;
bool startRanging = false;

void setupWiFi()
{
  WiFi.disconnect();
  // Connect the ESP8266 to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(200);
}


void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("Start setup");

  ////////// WiFi Setting //////////
  setupWiFi();

  ////////// ROS Setting //////////
  Serial.println("Connecting to ros master ...");
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  Serial.println("Registering IMU Publisher ...");
  nh.advertise(imu_publisher);
  nh.advertise(distance_publisher);
  imu_msg.header.frame_id = imu_topic_name;
  distance_msg.header.frame_id = distance_topic_name;
  distance_msg.angle_min = -0.0001;
  distance_msg.angle_max = 0.0001;
  distance_msg.angle_increment = 1;
  distance_msg.scan_time = distanceSensor.getIntermeasurementPeriod();
  distance_msg.range_min = 0.01;
  distance_msg.range_max = 1.0;
  float * ranges = new float[1];
  distance_msg.ranges = ranges;
  distance_msg.ranges_length = 1;

  ////////// Sensors Setting //////////
  Serial.println("Starting IMU LSM6DS3 ...");
  myIMU.settings.accelRange = 2;  // maximum of 2g
  myIMU.settings.accelBandWidth = 50;  // low pas filter to remove noise at 50 Hz
  myIMU.begin();

  if (distanceSensor.begin() == 0)
  {
    Serial.println("Sensor online!");
  }
}

void (*resetFunc)(void)=0;

void publishImu()
{

  float accelx = myIMU.readFloatAccelX();
  float accely = myIMU.readFloatAccelY();
  float accelz = myIMU.readFloatAccelZ();
  float gyrox = myIMU.readFloatGyroX();
  float gyroy = myIMU.readFloatGyroY();
  float gyroz = myIMU.readFloatGyroZ();
  // storing in IMU msg
  imu_msg.header.stamp = nh.now();
  imu_msg.linear_acceleration.x = accelx;
  imu_msg.linear_acceleration.y = accely;
  imu_msg.linear_acceleration.z = accelz;
  imu_msg.angular_velocity.x = gyrox;
  imu_msg.angular_velocity.y = gyroy;
  imu_msg.angular_velocity.z = gyroz;
  // publish or perish
  imu_publisher.publish(&imu_msg);

  // printing ...
//  Serial.println("Accelerometer:");
//  Serial.print(" X = ");  Serial.println(accelx, 4);
//  Serial.print(" Y = ");  Serial.println(accely, 4);
//  Serial.print(" Z = ");  Serial.println(accelz, 4);
//  Serial.println("Gyroscope:");
//  Serial.print(" X = ");  Serial.println(gyrox, 4);
//  Serial.print(" Y = ");  Serial.println(gyroy, 4);
//  Serial.print(" Z = ");  Serial.println(gyroz, 4);
}

void publishDistance(int distance)
{
  float dist = (float)distance /1000.0;
  distance_msg.header.stamp = nh.now();
  distance_msg.header.seq = seq;
  seq++;
  distance_msg.ranges[0] = dist; // [m]
//  Serial.println(dist);
  distance_publisher.publish(&distance_msg);
}

void loop()
{
  if ( WiFi.status() ==  WL_CONNECTED )
  {
    if (nh.connected())
    {
      unsigned long currentMillis = millis();
      if (currentMillis - imu_time_counter >= publish_interval)
      {
        imu_time_counter = currentMillis;
        publishImu();
      }

      // Distance Sensor
      if (startRanging == false)
      {
        distanceSensor.startRanging();
        startRanging = true;
      } else {
        if (distanceSensor.checkForDataReady())
        {
          int distance = distanceSensor.getDistance(); // [mm]
//          Serial.println(distance);
          distanceSensor.clearInterrupt();
          distanceSensor.stopRanging();
          publishDistance(distance);
          startRanging = false;
        }
      }
    }
  } else
  {
  resetFunc();
  }


  nh.spinOnce();
}
