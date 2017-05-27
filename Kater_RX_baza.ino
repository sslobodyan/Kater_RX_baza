#include <Servo.h>

#include <SPI.h>
#include "NRF24.h"

#include <Wire.h>
#include <EEPROM.h>

#include <avr/wdt.h>
#include "kater_rx_baza.h"
#include "imu9250.h"
#include "radio.h"
#include "ublox.h"
#include "navigate.h"
#include "sonar.h"

Compass9250 Compass;

#define EEPROM_HOPS_ADDR 4
#define EEPROM_BIAS 24

void write_int_to_eeprom(int adr, int data){
  byte lowByte = ((data >> 0) & 0xFF);
  byte highByte = ((data >> 8) & 0xFF);
  EEPROM.write(adr, lowByte);
  EEPROM.write(adr + 1, highByte);
}

int read_int_from_eeprom(int adr){
  byte lowByte = EEPROM.read(adr);
  byte highByte = EEPROM.read(adr + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void write_to_eeprom(){
  int adr=EEPROM_HOPS_ADDR;
  for(byte i=0; i<sizeof(hops); i++) {
    EEPROM.write(adr, bind_paket.mas[i]);
    adr++;
  }
}

void read_from_eeprom(){
  int adr=EEPROM_HOPS_ADDR;
  for(byte i=0; i<sizeof(hops); i++) {
    hops[i] = EEPROM.read(adr);
    adr++;
  }  
}

void read_bias_from_eeprom() {
  int adr = EEPROM_BIAS;
  Compass.mx_bias = read_int_from_eeprom( adr ); adr+=2;
  Compass.my_bias = read_int_from_eeprom( adr ); adr+=2;
  Compass.mz_bias = read_int_from_eeprom( adr ); adr+=2;
  Compass.ax_bias = read_int_from_eeprom( adr ); adr+=2;
  Compass.ay_bias = read_int_from_eeprom( adr ); adr+=2;
}

void write_bias_to_eeprom() {
  int adr = EEPROM_BIAS;
  write_int_to_eeprom( adr, Compass.mx_bias ); adr+=2;
  write_int_to_eeprom( adr, Compass.my_bias ); adr+=2;
  write_int_to_eeprom( adr, Compass.mz_bias ); adr+=2;
  write_int_to_eeprom( adr, Compass.ax_bias ); adr+=2;
  write_int_to_eeprom( adr, Compass.ay_bias ); adr+=2;
}

void get_analogs(void) {
  //#define ADC_NULL_TOK 512
  #define PITANIE_MEGA 5.0  
  #define PITANIE_ASC 5.0  

  if ( adc_busy ) return;

  int16_t dat;  
  float f;
  dat = 0;

  analogReference(DEFAULT); // относительно питания
  
  dat = analogRead(pinADC_TOK) ;
  //dat=410; //test 5A
  //dat=922; //test 20A
  //Serial.print("tok=");Serial.print(dat);Serial.print(" ");
  dat = abs(dat - adc_null_tok);
  f =  (float) dat * 04.8828125; //dat*(5/1024)/0.066 * 10 !! для 30А или 5/1024 / 0.100 для 20А
  //Serial.print(dat);Serial.print(" ");Serial.print("f=");Serial.print(f,3);
  tok = tok*(1-K_tok) + K_tok * f;
  if (tok > 2549) tok = 2549;
  //Serial.print(tok);
  //Serial.println();

  dat = analogRead(pinADC_BORT);
  f = (float) 01.5283203125 * dat; // 3.13 * 500 / 1024   *10 !!
  napruga =  (1-K_napruga)*napruga + K_napruga*( f );
  //Serial.print(F("Bort="));Serial.print(dat);Serial.print(" ");Serial.println(napruga);
}


void setup()
{
  wdt_enable(WDTO_8S);

  pinMode(pinTrap, INPUT_PULLUP);
  bool need_compas_calibrate = false;
  if ( !digitalRead(pinTrap) ) { // перемычка стоит - калибруем ИМУ
    need_compas_calibrate = true;
  }

  armed = false;
  kil_servo.attach(5,800,2200);
  kil_servo.write(90);
  bunker_servo.attach(9, 800, 2200);
  bunker_servo.write(0);
  gaz_servo.attach(6,800,2200);
  gaz_servo.write(90);
  
  old_bunker = 0;

  main_paket_otvet.status.gps = false;
  main_paket_otvet.status.autocourse = false;
  main_paket_otvet.status.almost = false;
  main_paket_otvet.status.has_home = false;
  
  pinMode(pinLight1, OUTPUT);
  digitalWrite(pinLight1, LOW);
  pinMode(pinLight2, OUTPUT);
  digitalWrite(pinLight2, LOW);

  pinMode(pinIRQ, INPUT_PULLUP);

  Serial.begin(19200); // потом ублокс перекроет своим значением при старте
  Serial.println(F("CarpWinner RX"));
  Serial.print(__DATE__);
  Serial.print("  ");
  Serial.println(__TIME__);
  Wire.begin();

  //read_from_eeprom(); пока все по 100
  
  hop_idx = 0;
  sonar_setup();

  Compass.begin();
  read_bias_from_eeprom();
  if (need_compas_calibrate) {
    Serial.println(F("Calculate biases"));
    Compass.start_update_bias();
    while ( !digitalRead(10) ) { // пока стоит перемычка
      wdt_reset();
      Compass.update();
      delay(50);  
    }
    Compass.stop_update_bias();
    write_bias_to_eeprom();
  }

  Serial.print(F("Biases: Ax "));Serial.print(Compass.ax_bias);
  Serial.print(F(" Ay "));Serial.print(Compass.ay_bias);
  Serial.print(F(" Mx "));Serial.print(Compass.mx_bias);
  Serial.print(F(" My "));Serial.print(Compass.my_bias);
  Serial.print(F(" Mz "));Serial.print(Compass.mz_bias);
  Serial.println();

  //Compass.mag_bias(-153, -121, -16);
  //Compass.acc_bias(-314, -66);
  for (byte nn=0; nn<15; nn++) {
    Compass.update();
  }
  start_heading = Compass.tilledHeading();

  trap_servo.attach(pinTrap, 800, 2200);
  trap_servo.write(0);

  long tok_summa=0;
  analogReference(DEFAULT);
  for(byte i=0; i<32; i++) {
    tok_summa += analogRead(pinADC_TOK);  
    delay(10);
  }
  adc_null_tok = tok_summa / 32; 
  Serial.print(F("Null current=")); Serial.println(adc_null_tok);

#ifndef PRINT_SONAR
  ublox_setup();
#endif

  Serial.println();

  //radio.pwrdown();

  radio_setup( );
  time_radio_filesafe = millis() + TIME_RADIO_FILESAFE;

  pinMode(pinDallas, INPUT);
  if ( digitalRead(pinDallas)==0 ) {
    binding();
  }
  
}


void failsave() {
  main_paket.x = 0;
  main_paket.y = 0;
}

bool controls_in_neutrals() {
  if ( abs(main_paket.x) > NEITRAL_CONTROLS_DELTA ) return false;
  if ( abs(main_paket.y) > NEITRAL_CONTROLS_DELTA ) return false;
  return true;
}

void set_bunker_servo( byte b ) {
  state_bunker = b;
  time_bunker = millis() + TIME_BUNKER_PAUSE;
}

void open_bunker() {
  set_bunker_servo( 180 );
}

void close_bunker() {
  set_bunker_servo( 0 );
}

void update_bunker(){
  if (old_bunker != state_bunker) {
    old_bunker += state_bunker>old_bunker ? 1 : -1 ;
    bunker_servo.write(old_bunker);
    if (old_bunker == state_bunker) {
      if (state_bunker > 100) {
        Serial.println(F("Bunker opened.")); 
        main_paket_otvet.status.bunker = 1;
      }
      if (state_bunker < 10)  {
        Serial.println(F("Bunker closed.")); 
        main_paket_otvet.status.bunker = 0;        
      }
    } else {
      //Serial.print(F("bunker "));Serial.println(old_bunker);
    }
    time_bunker = millis() + TIME_BUNKER_PAUSE;
  } 
  else time_bunker = 0;
}

void do_autopilot(void) {
  // идем к выбраной точке
  // heading_base - направление движения относительно северного полюса по IMU
  // vector - направление на цель (точку) из текущего положения
  int vector;
  int distance_to_home; // расстояние до дома в метрах
  int angle; // угол между направлением на точку и текущим курсом

  int servo_x, servo_y;

  float lat, lon; // текущее положение
  lat = gps_paket_otvet.lat / 10000000.0f;
  lon = gps_paket_otvet.lon / 10000000.0f;
  
  vector = GetHeading(lat, lon, target_lat, target_lon); // направление на точку по ГПС
  distance = abs( GetDistanceInM(lat, lon, target_lat, target_lon) );
  //distance_to_home = GetDistanceInM(lat, lon, home_lat, home_lon);
  //Serial.print(lat);Serial.print("-");Serial.print(lon);Serial.print(", ");Serial.print(target_lat);Serial.print("-");Serial.println(target_lon);

  servo_y = 0;
  if (distance > 400) {
      // ошибка с координатами точек - отключить автопилот
      active_action = ACTION_LAST;
      active_point = 255;
      Serial.print(F("Target "));Serial.print(target_lat);Serial.print(F("  "));Serial.println(target_lon); 
      Serial.print(F("Current "));Serial.print(lat);Serial.print(F("  "));Serial.println(lon); 
      Serial.println(F("Invalid point!"));
  }
  if ( active_action != ACTION_LAST ) {
      if (distance < 3) {
        // мы у цели
      } else if (distance > 5) {
        servo_y = -127; // еще далеко - полный газ
      } else servo_y = -40; // близко - по чуть-чуть
    
      if (active_action ==  ACTION_TARGET) {
        if (distance < 3) {
          active_action = ACTION_LAST;
          active_point = 255;
          Serial.println(F("Target done!"));
          Serial.print(F("Lat "));Serial.print(target_lat,7);
          Serial.print(F(" Lon "));Serial.println(target_lon,7);
        }
      }
    
      if (active_action ==  ACTION_FEED) {
        if (distance < 3) {
          // прибыли на точку кормления
          active_action = ACTION_GOHOME;
          active_point = 254; // признак что будем потом туда идти
          // открываем бункер
          open_bunker();
        }
      }

      if (active_action == ACTION_GOHOME) {
          // ждем закрытия бункера при автокормежке
          servo_y = 0;         
          if ( state_bunker > 10 && old_bunker==state_bunker ) { // бункер полностью открылся
            // закрываем и идем домой
            target_lat = home_lat / 10000000.0;
            target_lon = home_lon / 10000000.0;;
            active_point = 0; 
            active_action = ACTION_TARGET;
            close_bunker();
            Serial.print(F("Lat "));Serial.print(target_lat);
            Serial.print(F(" Lon "));Serial.println(target_lon);
          }    
      }
      
      int kurs = heading_base; // курс по компасу

      // угол отклонения от курса на цель
      angle = vector - kurs;
      if (angle < 0) angle += 360;
    
      if (servo_y != 0) {
          // поворачиваем руль в зависимости от скорости - чем быстрее едем, тем плавнее
          int u;
          if (angle < 180) {
            // поворачиваем вправо
            u = -angle * 2;
          } else {
            u = 2 * (360-angle) ;
          }
          int spd = 1 + gps_paket_otvet.gSpeed / 100; // скорость в м\с
          servo_x = u / spd; 
          if (servo_x > 90) servo_x = 90;
          if (servo_x < -90) servo_x = -90;
      }
      kil_servo.write(  map(servo_x, -127, 127, 0, 180)  );
  }  
  
  gaz_servo.write(  map(servo_y, -127, 127, 0, 180)  );

  //Serial.print("vector=");Serial.print(vector);Serial.print(" dist=");Serial.print(distance);Serial.print(" angle=");Serial.print(angle);Serial.print(" head=");Serial.println(heading_base);Serial.println();
  //Serial.print(" angle=");Serial.print(angle);Serial.print(" X=");Serial.print(servo_x);Serial.print(" Y=");Serial.println(servo_y);
}


void update_autopilot() {
    if ( !controls_in_neutrals() ) {
      Serial.println(F("deactivate autopilot"));
      Serial.print(F("x "));Serial.print(main_paket.x);Serial.print(F(", y "));Serial.println(main_paket.y);Serial.println();
      active_point = 255; // двинули стиками - отключаем автопилот и переходим на ручки 
      active_action = ACTION_LAST;
    } else {
      if ( millis() - time_autopilot > 100 ) {
        time_autopilot = millis();
        do_autopilot();  
      }
    } 
}

void no_radio() {  // нет радиосигнала - раз в 2 секунды

        // перезапускаем радиоканал
        radio_setup( );

#ifdef USE_FAILSAFE        
        // потеря радиосигнала - включаем файлсейв (ручки в нули)
        failsave();
#endif        
        time_radio_filesafe = millis() + TIME_RADIO_FILESAFE; // пауза до следующего файлсейва
        if ( millis() > time_gohome ) {
          // 20 секунд без связи и автопилота - самостоятельно идем домой
#ifdef USE_AUTOHOME
          if ( home_lat !=0 && home_lon !=0 ) {
            active_point = 0;
            active_action = ACTION_TARGET;
            target_lat = home_lat / 10000000.0f;
            target_lon = home_lon / 10000000.0f;
            time_gohome = 0;
            Serial.print(F("Go HOME! ")); 
            Serial.print(target_lat,7); Serial.print(F("  "));Serial.println(target_lon,7);
          }
#endif        
        } 
}


void manual_pilot(){

      if ( time_radio_filesafe < millis()  ) {
        no_radio();
      }
      delta = 0;

#ifdef USE_AUTOCURS
      if ( free_heading < 0 ) { // еще не запоминали направление движения
        if ( (abs(main_paket.x) < 10) && (abs(main_paket.y)) > 50 ) { // руль по центру и газ более чем на половину
          free_heading = heading_base;  // запоминаем текущее направление как желаемое
          main_paket_otvet.status.autocourse = 1;
        }
      }

      if (( abs(main_paket.x) > 20 ) || ( abs(main_paket.y) ) < 20 ) { // дернули ручками или сбросили газ - отключаем подруливание
        //if ( free_heading > 0 ) Serial.println("отключили дoруливание");
        free_heading = -1;
        main_paket_otvet.status.autocourse = 0;
      }

      if ( (free_heading >= 0) && (heading_base != free_heading) ) { // угол свободного плавания установлен, но текущий угол другой
          delta = free_heading - heading_base;
          if (delta < 0) delta += 360;
          if (delta > 180) { // отклонились по часовой (вправо)
            delta = 360 - delta;
            delta = delta > MAX_AUTOCOURSE_DELTA ? MAX_AUTOCOURSE_DELTA : delta;
          } else { // отклонились против часовой (влево)
            delta = delta > MAX_AUTOCOURSE_DELTA ? -MAX_AUTOCOURSE_DELTA : -delta;
          }
          //Serial.print(free_heading);Serial.print(" h=");Serial.println(heading_base);
          //Serial.print(main_paket.x);Serial.print(" доруливаем ");Serial.println(delta);
      }
#endif      
      
      // устанавливаем сервы куда запросили
      kil_servo.write(  map(main_paket.x + delta, -127, 127, 0, 180)  );
      
      if ( (main_paket.y > -10) && (main_paket.y < 10) ) armed = true; // прошли джойстиком срединку
      if ( armed ) {
        gaz_servo.write(  map(main_paket.y, -127, 127, 0, 180)  );
      } else {
        gaz_servo.write(  90  );  
      }
      
      //set_bunker_servo( map(main_paket.command.bunker, 0, 31, 0, 180) );
      // переделал на замедлитель
      trap_servo.write(  map(main_paket.command.trap, 0, 1, 0, 180)  );
}

void correct_heading_by_gps(){
  if ( main_paket_otvet.status.gps ) { // работает ЖПС
    if ( gps_paket_otvet.fix == 3 ) {
      if (gps_paket_otvet.gSpeed > 100 ) {
        // на скорости больше 1m\s доверяем скорее ЖПС чем ИМУ
        const float k_kurs=0.8f;
        heading_base = (float) gps_paket_otvet.heading*k_kurs + heading_base*(1-k_kurs);
      }
    }
  }
}

void print_debug_info() {
  #ifdef TEST_RADIO
    if (Serial.available()) {
      char cmd=Serial.read();
      if (cmd=='r') radio.print_registers();
      if (cmd=='f') {
        radio.flushRX();
        radio.flushTX();
        Serial.println(F("Flushed!"));
        radio.print_registers();
      }
    }
  #endif

  #ifdef PRINT_X_Y
    Serial.print(F("main_paket.x="));
   Serial.print(main_paket.x);
   Serial.print(F(" main_paket.y="));
   Serial.println(main_paket.y);
  #endif

  #ifdef PRINT_HEADING      
    Serial.print(F("Hd ")); Serial.print(heading);
    Serial.print(F("  Hd world ")); Serial.println(heading_base);
  #endif      

  #ifdef PRINT_SONAR
    if ( sonar_done ) {
      for(i=0; i<sizeof(sonar); i++) {
        Serial.print( sonar[i], HEX ); Serial.print(" ");
      }
      Serial.println(  );
      sonar_done = false;
    }
  #endif   
}


unsigned long test_time=0; //byte n_time=0;

void loop()   
{
  byte i;

  //test_time = micros();
  //test_time = micros()-test_time; Serial.print(F("time")); n_time+=1; Serial.print(n_time); Serial.print(" "); Serial.println( test_time ); test_time = micros(); // 

  wdt_reset();
  update_gps();

  Compass.update(); // 3.5ms

  if ( time_bunker && (time_bunker < millis()) ) {
    update_bunker();
  }

#ifndef RADIO_USE_IRQ
    radio.check(); // 0.3ms
    //test_time = micros()-test_time; Serial.println( test_time ); test_time = micros(); // 
    test_time = millis()-test_time; Serial.println( test_time ); test_time = millis(); // 
#endif  

  if (millis() > time_next_paket) {
    time_next_paket += TIME_SLOT_MS * 2;
    do_hop();
    do_hop();
    Serial.println("#");
  }
  
  if (active_point < 255) {
    // активный автопилот без контроля радио
    update_autopilot();
  } else {  
    // идем на ручках и контролируем радио
    manual_pilot(); // 0.4ms  3.5ms_from_start
  }

  if ( time_adc < millis() ) {
    time_adc = millis() + 50;
    get_analogs(); // 0.3ms
    heading_base = Compass.tilledHeading(); // 0.8ms
    correct_heading_by_gps();
    
    heading = heading_base - start_heading;
    if (heading < 0) heading += 360;
    if (heading > 359 ) heading -= 360;
  }

  if ( time_sonar < millis() ) {
    sonar_setup(); // перезапуск зависшего сонара
    time_sonar = millis()+2000;
  }
  
}
