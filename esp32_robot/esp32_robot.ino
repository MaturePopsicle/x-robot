// #include "./controller/MotorControl.hxx"
#include "controller/MotorControl.h"
#if 0
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    Wire.beginTransmission(0x68); //开启MPU6050的传输
    Wire.write(0x6B); //指定寄存器地址
    Wire.write(0); //写入一个字节的数据
    Wire.endTransmission(true); //结束传输，true表示释放总线
}

void loop() {
    #if 0
    // put your main code here, to run repeatedly:
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(PWMA, HIGH); //PWM调速a==0-255  51

    Wire.beginTransmission(0x68); //开启MPU6050的传输
    Wire.write(0x3B); //指定寄存器地址
    Wire.requestFrom(0x68, 2, true); //将输据读出到缓存
    //Wire.endTransmission(true); //关闭传输模式
    int val = Wire.read() << 8 | Wire.read(); //两个字节组成一个16位整数

    Serial.print("val:");
    Serial.print(val, DEC);
    Wire.write(0x41); //指定寄存器地址
    Wire.requestFrom(0x68, 2, true); //将输据读出到缓存  
    val = Wire.read() << 8 | Wire.read();
    Serial.print("val2:");
    Serial.print(val, DEC);
    Wire.endTransmission(true); //结束传输，true表示释放总线
    #endif

    delay(700);
}
#endif

#if 0
//读取MPU6050数据1 
#define IMUAddress 0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint8_t i2cData[14];

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
{
    uint32_t timeOutTimer;
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
    if (rcode) 
    {
        Serial.print(F("i2cRead failed: "));
        Serial.println(rcode);
        return rcode; //
    }

    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++) 
    {
        if (Wire.available())
        {
            data[i] = Wire.read();
            Serial.print( data[i]);
        }
        else
        {   

            Serial.println(F("i2cRead timeout"));
        }
        
    }
    return 0; // Success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
 
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data, length);
    uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
    if (rcode) {
      Serial.print(F("i2cWrite failed: "));
      Serial.println(rcode);
    }
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

 
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void setup() {
    Wire.begin();
    Serial.begin(9600);

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
 
    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) 
    { // Read "WHO_AM_I" register
        Serial.println(F("Error reading sensor"));
        while (1);
    }
    else 
    {
      Serial.println(F("success reading sensor"));
    }
}
 
void loop() {
    // put your main code here, to run repeatedly:
    byte c = readByte(IMUAddress, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("mpu: "); Serial.println(c);
    delay(100);
    i2cRead(0x3B, i2cData, 14);
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = ((i2cData[6] << 8) | i2cData[7]);
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    Serial.print("accX:\t"); Serial.print(accX); Serial.print("\t");
    Serial.print("accY:\t"); Serial.print(accY); Serial.print("\t");
    Serial.print("accZ:\t"); Serial.print(accZ); Serial.print("\t");
    Serial.print("tempRaw:\t"); Serial.print(tempRaw); Serial.print("\t");
    Serial.print("gyroX:\t"); Serial.print(gyroX); Serial.print("\t");
    Serial.print("gyroY:\t"); Serial.print(gyroY); Serial.print("\t");
    Serial.print("gyroZ:\t"); Serial.print(gyroZ); Serial.print("\n");
    delay(1000);
}

#endif


#if 0
//读取MPU6050数据
const uint8_t IMUAddress = 0x68;
const uint16_t I2C_TIMEOUT = 100;
double accX, accY, accZ;
uint8_t i2cData[14]; // Buffer for I2C data


uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
 
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data, length);
    uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
    if (rcode) {
      Serial.print(F("i2cWrite failed: "));
      Serial.println(rcode);
    }
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
    uint32_t timeOutTimer;
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
    if (rcode) {
      Serial.print(F("i2cRead failed: "));
      Serial.println(rcode);
      return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
    }
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++) {
      if (Wire.available())
        data[i] = Wire.read();
      else {
        timeOutTimer = micros();
        while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
        if (Wire.available())
          data[i] = Wire.read();
        else {
          Serial.println(F("i2cRead timeout"));
          return 5; // This error value is not already taken by endTransmission
        }
      }
    }
    return 0; // Success
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin(); 
    
    Wire.beginTransmission(IMUAddress); //开启MPU6050的传输
    Wire.write(0x6B); //指定寄存器地址
    Wire.write(0); //写入一个字节的数据
    Wire.endTransmission(true); //结束传输，true表示释放总线

   #if 0 
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
 
    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.println(F("Error reading sensor"));
        while (1);
    }
    else {
      Serial.println(F("success reading sensor"));
      Serial.print(i2cData[0], DEC );
    }
 
    delay(20); // Wait for sensor to stabilize
 
    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];
    #endif
 
}

void loop() {
    float realVals[7];
#if 0
    Wire.beginTransmission(IMUAddress);
    Wire.write(0x75);
     Serial.print("val2:\n");
     delay(700);
#endif
    Wire.beginTransmission(0x68); //开启MPU6050的传输
    Wire.write(0x3B); //指定寄存器地址
    Wire.requestFrom(0x68, 2, true); //将输据读出到缓存
    Wire.endTransmission(true); //关闭传输模式
    int val = Wire.read() << 8 | Wire.read(); //两个字节组成一个16位整数
    delay(500);

    Serial.print(val);
}
#endif



#if 1
//PWM控制 PIN4 PIN12输出  控制电机转速
//已经测试

// #define AIN1  22
// #define AIN2  23
// #define BIN1  19
// #define BIN2  18
// int freq = 10000;
// int channel = 0;
// int channel2 = 1;
// int resolution = 8;

control::MotorControl robot_control;

void setup()
 {
    Serial.begin(115200);
    // ledcSetup(channel, freq, resolution);
    // ledcAttachPin(4, channel);
    // ledcAttachPin(12, channel2);

    // pinMode(AIN1, OUTPUT);
    // pinMode(AIN2, OUTPUT);
    // digitalWrite(AIN1, HIGH);
    // digitalWrite(AIN2, LOW);

    // pinMode(BIN1, OUTPUT);
    // pinMode(BIN2, OUTPUT);
    // digitalWrite(BIN1, HIGH);
    // digitalWrite(BIN2, LOW);
    robot_control.setWheelsPwnCfg(10000, 0, 8, 10000, 1, 8);
    robot_control.initialize(AIN1, AIN2, PWM2, BIN1, BIN2, PWM2);
}

void loop() 
{
    // ledcWriteTone(channel, freq);
    int dutyCycle;
    for (dutyCycle = 0; dutyCycle <= 255; dutyCycle=dutyCycle+10)
    {
        Serial.print("value is: ");
        Serial.println(dutyCycle);
        // ledcWrite(channel, dutyCycle);
        robot_control.setLinearVelocity(dutyCycle, 255-dutyCycle);
        delay(1000);
    }

    
    
    // for (int dutyCycle2 = 0; dutyCycle2 < 255; dutyCycle2 = dutyCycle2 + 10)
    // {
    //     Serial.println(dutyCycle2);
    //     // ledcWriteTone(channel2, dutyCycle2);
    //     ledcWrite(channel, dutyCycle -= 10);
    //     ledcWrite(channel2, dutyCycle2);
    //     delay(1000);
    // }

}
#endif
