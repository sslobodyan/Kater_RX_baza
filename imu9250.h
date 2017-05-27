#include <Wire.h>

#define BUFFER_LENGTH 6
#define PRINT_CALC_BIAS_
 
class Compass9250 {
//***************************************************************************************************************
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
//***************************************************************************************************************
  private:
    // для акселя: буфер, среднее, сумма по осям
    int16_t fax[BUFFER_LENGTH];
    int16_t fay[BUFFER_LENGTH];
    float dax, day;
    int16_t sax = 0; int16_t say = 0;
    
    int index = 0; // индекс записи в буфер
    
    // для магнитометра: буфер, среднее, сумма по осям
    int16_t  fmx[BUFFER_LENGTH];
    int16_t  fmy[BUFFER_LENGTH];
    int16_t  fmz[BUFFER_LENGTH];
    float dmx, dmy, dmz;
    int16_t  smx = 0; int16_t  smy = 0; int16_t  smz = 0;
    bool update_bias = false;
    
    const float grad2rad = (float) PI / 180.0;
    const float rad2grad = (float) 180.0 / PI;
    int16_t  mx_min,mx_max,my_min,my_max,mz_min,mz_max;
    int16_t  ax_min,ax_max,ay_min,ay_max;
    int16_t  mx, my, mz, ax, ay;

  public:
    int16_t  mx_bias=0,my_bias=0,mz_bias=0;
    int16_t  ax_bias=0,ay_bias=0;
    float roll;
    float pitch;
    void begin();
    void update();
    void start_update_bias();
    void stop_update_bias();
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
    int tilledHeading();
    int Heading();
    void mag_bias(int x, int y, int z);
    void acc_bias(int x, int y);
    void calc_bias_mag();
    void calc_bias_acc();
};

void Compass9250::mag_bias(int x, int y, int z){
  mx_bias = x;
  my_bias = y;
  mz_bias = z;
}

void Compass9250::acc_bias(int x, int y){
  ax_bias = x;
  ay_bias = y;
}

void Compass9250::calc_bias_mag(){
  if (mx < mx_min) mx_min = mx;
  if (mx > mx_max) mx_max = mx;
  mx_bias = (mx_min+mx_max)/2;
  
  if (my < my_min) my_min = my;
  if (my > my_max) my_max = my;
  my_bias = (my_min+my_max)/2;
  
  if (mz < mz_min) mz_min = mz;
  if (mz > mz_max) mz_max = mz;
  mz_bias = (mz_min+mz_max)/2;

#ifdef PRINT_CALC_BIAS
      Serial.print (" Mx ");
      Serial.print (mx, DEC);
      Serial.print(" (");Serial.print(mx_min);Serial.print(",");Serial.print(mx_max);Serial.print(",");Serial.print(mx_bias);Serial.print(")");
      Serial.print (" My ");
      Serial.print (my, DEC);
      Serial.print(" (");Serial.print(my_min);Serial.print(",");Serial.print(my_max);Serial.print(",");Serial.print(my_bias);Serial.print(")");
      Serial.print (" Mz ");
      Serial.print (mz, DEC);
      Serial.print (" (");Serial.print(mz_min);Serial.print(",");Serial.print(mz_max);Serial.print(",");Serial.print(mz_bias);Serial.print(")");
      Serial.println();
#endif
  
}

void Compass9250::calc_bias_acc(){
  if (ax < ax_min) ax_min = ax;
  if (ax > ax_max) ax_max = ax;
  ax_bias = (ax_min+ax_max)/2;
  
  if (ay < ay_min) ay_min = ay;
  if (ay > ay_max) ay_max = ay;
  ay_bias = (ay_min+ay_max)/2;

#ifdef PRINT_CALC_BIAS
      Serial.print (" Ax ");
      Serial.print (ax, DEC);
      Serial.print(" (");Serial.print(ax_min);Serial.print(",");Serial.print(ax_max);Serial.print(",");Serial.print(ax_bias);Serial.print(")");
      Serial.print (" Ay ");
      Serial.print (ay, DEC);
      Serial.print(" (");Serial.print(ay_min);Serial.print(",");Serial.print(ay_max);Serial.print(",");Serial.print(ay_bias);Serial.print(")");
#endif
  
}


void Compass9250::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void Compass9250::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void Compass9250::begin() {

  Wire.begin();
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);

  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  index = 0;
  
  sax = 0; say = 0;     // sum for accellaration
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    fax[i] = 0;                  // arrays for 8 values acceleration  init
    fay[i] = 0;
  }

  smx = 0; smy = 0; smz = 0;    // sum for mag
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    fmx[i] = 0;                  // arrays for 8 values mag  init
    fmy[i] = 0;
    fmz[i] = 0;
  }

}

void Compass9250::start_update_bias() {
  ax_min=10000;ay_min=10000;mx_min=10000;my_min=10000;mz_min=10000;
  ax_max=-10000;ay_max=-10000;mx_max=-10000;my_max=-10000;mz_max=-10000;
  ax_bias=0;ay_bias=0;mx_bias=0;my_bias=0;mz_bias=0;
  update_bias = true;
}

void Compass9250::stop_update_bias(){
  update_bias = false;
}


void Compass9250::update(){
  // Read accelerometer
  uint8_t Buf[4];
  I2Cread(MPU9250_ADDRESS, 0x3B, 4, Buf);

  // Create 16 bits values from 8 bits data
  ax = -(Buf[0] << 8 | Buf[1]);
  ay = -(Buf[2] << 8 | Buf[3]);

  if ( update_bias ) {
    calc_bias_acc();
  }

  ax = ax - ax_bias;
  ay = ay - ax_bias;

  sax = sax - fax[index];     // one value subtract
  fax[index] = ax ;   // new value insert
  sax = sax + fax[index];     // new sum
  dax = (float) sax / 8192.0 / BUFFER_LENGTH;       // average

  say = say - fay[index];     // same for acc y
  fay[index] = -ay;
  say = say + fay[index]; 
  day = (float) say  / 8192.0 / BUFFER_LENGTH;

  roll = asin(day);
  pitch = asin(dax); 
    
  // _____________________
  // :::  Magnetometer :::

  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Create 16 bits values from 8 bits data
  mx = -(Mag[3] << 8 | Mag[2]);
  my = -(Mag[1] << 8 | Mag[0]);
  mz = -(Mag[5] << 8 | Mag[4]);

  if ( update_bias ) {
    calc_bias_mag();
  }

  mx = mx - mx_bias;
  my = my - my_bias;
  mz = mz - mz_bias;

  smx = smx - fmx[index];     // one value subtract
  fmx[index] = mx;            // new value insert
  smx = smx + fmx[index];     // new sum
  dmx = (float) -smx / BUFFER_LENGTH;      // average

  smy = smy - fmy[index];     // same for mag y
  fmy[index] = my;
  smy = smy + fmy[index]; 
  dmy = (float) -smy / BUFFER_LENGTH;

  smz = smz - fmz[index];      // same for mag z
  fmz[index] = mz;
  smz = smz + fmz[index]; 
  dmz = (float) smz / BUFFER_LENGTH;

  index++;
  if (index >= BUFFER_LENGTH) {        //   wrap index
    index = 0;
  } 
}

int Compass9250::tilledHeading()
{
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  float Xh, Yh;
  Xh = dmx * cosPitch + dmz * sinPitch;
  Yh = dmx * sinRoll * sinPitch + dmy * cosRoll - dmz * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  float const PI2 = 2.0*PI;
  if (heading < 0) {
    heading += PI2;
  }
  if (heading > PI2) {
    heading -= PI2;
  }

  heading = heading * rad2grad;
  
  return (int)heading;
}

int Compass9250::Heading()
{
  float head = atan2( -(dmy), -(dmx));
  head = head * rad2grad + 180;
  if (head < 0) head +=360;
  if (head > 360) head -=360;

  return (int)head;
}


