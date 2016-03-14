//INCOMPLETE: CANNOT READ MAGNETOMETER DATA
    #include<Wire.h>
    const int MPU=0x68;  // I2C address of the MPU-6050
    const int Mag=0x0C;  // I2C addres of Magnetometer
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,MaX,MaY,MaZ;
    void setup(){
      Wire.begin();
      Wire.beginTransmission(MPU);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      Wire.beginTransmission(Mag);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission(true);
      Serial.begin(9600);
    }
    void loop(){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(true);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers //MPU is address of device. copy with MPU as MAG(netometer), where MAG is address of magnetometer, which is probably 0x0C
      AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      Wire.beginTransmission(Mag);
      Wire.write(0x0A);
      Wire.endTransmission(true);
      Wire.requestFrom(Mag,6,true);
      MaX=Wire.read()<<8|Wire.read();
      MaY=Wire.read()<<8|Wire.read();
      MaZ=Wire.read()<<8|Wire.read();
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | GyX = "); Serial.print(GyX);
      Serial.print(" | GyY = "); Serial.print(GyY);
      Serial.print(" | GyZ = "); Serial.print(GyZ);
      Serial.print(" | MaX = "); Serial.print(MaX);
      Serial.print(" | MaY = "); Serial.print(MaY);
      Serial.print(" | MaZ = "); Serial.println(MaZ);
      delay(33);
    }

