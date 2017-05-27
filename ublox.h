
#define serial Serial

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  unsigned char cls=0x01;   // 0x01
  unsigned char id=0x02;    // 0x02
  unsigned short len=28;  // 28
  unsigned long iTOW;  // GPS Millisecond Time of Week
  long lon;            // Longitude
  long lat;            // Latitude
  long height;         // Height above Ellipsoid
  long hMSL;           // Height above mean sea level
  unsigned long hAcc;  // Horizontal Accuracy Estimate
  unsigned long vAcc;  // Vertical Accuracy Estimate
  unsigned char valid=0;
};

NAV_POSLLH posllh;

struct NAV_SOL {
  unsigned char cls=0x01;   // 0x01
  unsigned char id=0x06;    // 0x06
  unsigned short len=52;  // 52
  unsigned long iTOW;  // GPS Millisecond Time of Week
  long fTOW;  // Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000
  short week; // GPS week (GPS time)
  unsigned char gpsFix; /* GPSfix Type, range 0..5
                            0x00 = No Fix
                            0x01 = Dead Reckoning only
                            0x02 = 2D-Fix
                            0x03 = 3D-Fix
                            0x04 = GPS + dead reckoning combined
                            0x05 = Time only fix
                            0x06..0xff: reserved
                         */
  unsigned char flags; // младший бит GPSfixOK - признак фиксации координат                       
  long ecefX;
  long ecefY;
  long ecefZ;
  unsigned long pAcc; // position accuracy estimate
  long ecefVX;
  long ecefVY;
  long ecefVZ;
  unsigned long sAcc;
  unsigned short pDop;
  unsigned char reserved1;
  unsigned char numSv; // количество активных спутников
  unsigned long reserved2;
  unsigned char valid=0;
};

NAV_SOL nav_sol;

struct NAV_VELNED {
  unsigned char cls=0x01;   // 
  unsigned char id=0x12;    // 
  unsigned short len=36;  // 
  unsigned long iTOW;  // GPS Millisecond Time of Week
  long velN;           // NED north velocity
  long velE;           // NED east velocity
  long velD;           // NED down velocity
  unsigned long speed; // Speed (3-D)
  unsigned long gSpeed;  // Ground Speed (2-D)
  long heading;  // Heading of motion 2-D
  unsigned long sAcc;  // Speed Accuracy Estimate
  unsigned long cAcc;  // Course / Heading Accuracy Estimate
  unsigned char valid=0;
};

NAV_VELNED velned;

struct NAV_TIMEUTC {
  unsigned char cls=0x01;   // 
  unsigned char id=0x21;    // 
  unsigned short len=20;  // 
  unsigned long iTOW;  // GPS Millisecond Time of Week
  unsigned long tAcc;  // 
  signed long nano;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  unsigned char flags;
  unsigned char valid=0;
};

NAV_TIMEUTC timeutc;

struct NAV_DOP {
  unsigned char cls=0x01;   // 
  unsigned char id=0x04;    // 
  unsigned short len=18;  // 
  unsigned long iTOW;  // GPS Millisecond Time of Week
  uint16_t gDOP;
  uint16_t pDOP;
  uint16_t tDOP;
  uint16_t vDOP;
  uint16_t hDOP;
  uint16_t nDOP;
  uint16_t eDOP;
  unsigned char valid=0;
};

NAV_DOP navdop;

unsigned char* curr_structure;
unsigned int payloadSize=10;
unsigned short len;
unsigned short grd=0;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < len+4; i++) {      
    CK[0] += ((unsigned char*)(curr_structure))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  static unsigned char class_id[2]; // храним класс и ид входящего пакета
  static unsigned char crc[2]; // храним CRC входящего пакета
  
  while ( serial.available() ) {
    byte c = serial.read();
    //Serial.println(c,HEX);
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] ) {
        fpos++;
        main_paket_otvet.status.gps = true; // признак что ГПС работает в принципе
      }
      else
        fpos = 0;
    }
    else {
      if ( fpos == 2 ) {
        class_id[0] = c;
      }
      if ( fpos == 3 ) {
        class_id[1] = c;
        
        // подбираем структуру под входящий пакет
        if ((class_id[0] == posllh.cls) && (class_id[1] == posllh.id)) {
          curr_structure = (unsigned char *) (&posllh);
        }
        else if ((class_id[0] == nav_sol.cls) && (class_id[1] == nav_sol.id)) {
          curr_structure = (unsigned char *) (&nav_sol);
        }
        else if ((class_id[0] == velned.cls) && (class_id[1] == velned.id)) {
          curr_structure = (unsigned char *) (&velned);
        }
        else if ((class_id[0] == timeutc.cls) && (class_id[1] == timeutc.id)) {
          curr_structure = (unsigned char *) (&timeutc);
        }
        else if ((class_id[0] == navdop.cls) && (class_id[1] == navdop.id)) {
          curr_structure = (unsigned char *) (&navdop);
        }
       else fpos=0; // неизвестный пакет
        
        if ( fpos > 0 ) {
          len = ((unsigned short*)(curr_structure))[1];
          payloadSize = len + 4; // плюс класс структуры и ее длина
        }
        
      }
      
      if ( (fpos-2) < payloadSize ) { // запоминаем данные в структуру
        if (fpos > 4) { // шапку с длинами и номерами структур не затираем
          ((unsigned char*)(curr_structure))[fpos-2] = c;
        }  
      }
      
      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum); // считает ЦРЦ и прописывает в массив checksum
      }
      
      if ( fpos == (payloadSize+3) ) { // принят первый байт CRC
        crc[0] = c;
        if ( c != checksum[0] ) {
          fpos = 0;
        }  
      }
      
      if ( fpos == (payloadSize+4) ) { // принят второй байт CRC
        crc[1] = c;
        fpos = 0;
        if ( c == checksum[1] || 1==1 ) {
          ((unsigned char*)(curr_structure))[payloadSize] = 1; // устанавливаем флаг принятия пакета valid = 1
          return true;
        }
      }
      
      if ( fpos > (payloadSize+4) ) { // переприем? теоретически быть не должно
        fpos = 0;
      }
      
    }
  }
  return false;
}

void test_gps(void) {
  
  if ( processGPS() ) {
      
      if (posllh.valid ) {
        Serial.println();
        Serial.print(F("SERIAL_RX_BUFFER_SIZE "));
        Serial.println(SERIAL_RX_BUFFER_SIZE);

        
        Serial.print(F("iTOW:"));      Serial.print(posllh.iTOW);
        Serial.print(F(" lat/lon: ")); 
        Serial.print(posllh.lat/10000000.0f, 7); 
        Serial.print(","); 
        Serial.print(posllh.lon/10000000.0f, 7);
        Serial.println();
        posllh.valid = 0;
      }

      if (nav_sol.valid ) {
        Serial.print(F("gpsFix:"));      
        Serial.print(nav_sol.gpsFix);
        Serial.print(F(" flags: ")); 
        Serial.print(nav_sol.flags); 
        Serial.print(F(" numSv: "));  
        Serial.print(nav_sol.numSv);
        Serial.println();
        nav_sol.valid = 0;
      }
      
      if (velned.valid ) {
        Serial.print(F("gSpeed:"));      
        Serial.print(velned.gSpeed);
        Serial.print(F(" heading: ")); 
        Serial.print(velned.heading); 
        Serial.println();
        velned.valid = 0;
      }
      
      if (navdop.valid ) {
        Serial.print(F("hDOP:"));      
        Serial.print(navdop.hDOP);
        Serial.println();
        navdop.valid = 0;
      }   
       
      if (timeutc.valid ) {
        Serial.print(F("day:"));      
        Serial.print(timeutc.day);
        Serial.print(F(" month:"));      
        Serial.print(timeutc.month);
        Serial.print(F(" year:"));      
        Serial.print(timeutc.year);
        Serial.println();
        timeutc.valid = 0;
      }        
  }  
}

void update_gps() {
      processGPS();
      if (posllh.valid ) {
        gps_paket_otvet.lat = posllh.lat;
        gps_paket_otvet.lon = posllh.lon;
        if (main_paket_otvet.status.has_home == false) {
          main_paket_otvet.status.has_home = true;
        }
        posllh.valid = 0;
      }
      if (nav_sol.valid ) {
        gps_paket_otvet.fix = nav_sol.gpsFix;
        gps_paket_otvet.nSv = nav_sol.numSv;
        nav_sol.valid = 0;
      }
      if (velned.valid ) {
        gps_paket_otvet.gSpeed = velned.gSpeed;
        gps_paket_otvet.heading = velned.heading / 100000;
        velned.valid = 0;
      }
      if (navdop.valid ) {
        gps_paket_otvet.hdop = navdop.hDOP;
        navdop.valid = 0;
      }   
}


void ublox_setup() {
  static const char PROGMEM to_19200_UBX[] = {
    0xb5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,
    0x00,0x00,0xd0,0x08,0x00,0x00,0x00,0x4b,
    0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,
    0x00,0x00,0x46,0x4b}; 
  static const char PROGMEM  UBX_5_Hz[] = {0xb5,0x62,0x06,0x08,0x06,0x00,0xc8,0x00,0x01,0x00,0x01,0x00,0xde,0x6a};
  static const char PROGMEM  UBX_NAV[5][11] = {
    { 0xb5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0e,0x47},
    { 0xb5,0x62,0x06,0x01,0x03,0x00,0x01,0x21,0x01,0x2d,0x85},
    { 0xb5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1e,0x67},
    { 0xb5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4f},
    { 0xb5,0x62,0x06,0x01,0x03,0x00,0x01,0x04,0x01,0x10,0x4b}
    };
  static const char PROGMEM  UBX_Pedestrian[] = {
    0xb5,0x62,0x06,0x24,0x24,0x00,0xff,0xff,0x03,0x02,0x00,0x00,0x00,0x00,0x10,0x27,
    0x00,0x00,0x05,0x00,0xfa,0x00,0xfa,0x00,0x64,0x00,0x2c,0x01,0x05,0x00,0x00,0x00,
    0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4e,0x9a
    };

  delay(50);
  serial.begin(9600);
  serial.println("Setup UBLOX");
  delay(100);
  for (byte i=0; i<sizeof(to_19200_UBX); i++) {
    serial.write( pgm_read_byte(&(to_19200_UBX[i]) ) );
  }
  serial.flush();
  delay(100);
  serial.begin(19200);
 
  for (byte i=0; i<11; i++) serial.write( pgm_read_byte(&(UBX_NAV[0][i])));
  serial.flush();
  delay(50);
  for (byte i=0; i<11; i++) serial.write( pgm_read_byte(&(UBX_NAV[1][i])));
  serial.flush();
  delay(50);
  for (byte i=0; i<11; i++) serial.write( pgm_read_byte(&(UBX_NAV[2][i])));
  serial.flush();
  delay(50);
  for (byte i=0; i<11; i++) serial.write( pgm_read_byte(&(UBX_NAV[3][i])));
  serial.flush();
  delay(50);
  for (byte i=0; i<11; i++) serial.write( pgm_read_byte(&(UBX_NAV[4][i])));
  serial.flush();
  delay(50);

  for (byte i=0; i<sizeof(UBX_Pedestrian); i++) serial.write( pgm_read_byte(&(UBX_Pedestrian[i])));
  serial.flush();
  delay(50);

  for (byte i=0; i<sizeof(UBX_5_Hz); i++) serial.write( pgm_read_byte(&(UBX_5_Hz[i])));
  serial.flush();
  delay(50);

}

