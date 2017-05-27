#define CE_PIN  8
#define CSN_PIN 7
#define pinIRQ 2

#define DEBUG_RADIO_
#ifdef DEBUG_RADIO
  #define TIME_SLOT_MS 500
  #define TIME_RADIO_FILESAFE 30000
  #define TIME_RADIO_GOHOME 35000
#else
  #define TIME_SLOT_MS 35
  #define TIME_RADIO_FILESAFE 500
  #define TIME_RADIO_GOHOME 15000
#endif  

#define RADIO_USE_IRQ

NRF24 radio(CE_PIN, CSN_PIN, pinIRQ); 

//=================================================//

enum Type_Paket: byte {
  NO_PAKET = 0,
  MAIN_PAKET=1,
  SONAR_PAKET=2,
  BIND_PAKET=3, 
  GPS_PAKET=6, 
  AUTOPILOT_PAKET=7
} curr_paket;

struct stCommand {
  byte light:2 ;
  byte trap:1 ;
  byte bunker:1 ;
  byte reserve:4 ;
};

struct stMain_Paket {
  byte type=MAIN_PAKET;
  int8_t x=127;
  int8_t y=127;
  stCommand command ;
} main_paket;

struct stStatus {
  byte sonar:1; // сонар получил полное эхо (заполнен весь буфер)
  byte gps:1; // GPS отдал любой пакет - просто модуль работает
  byte autocourse:1; // автокурс держит направление
  byte almost:1; // почти подошли к точке назначения
  byte has_home:1; // прописали домашнюю точку
  byte bunker:1; // 0 - закрыт, 1 - открыт
  byte other:2; // потом придумаю для чего надо
};

struct stMain_Paket_Otvet { // =7
  byte type=MAIN_PAKET+0x10;
  byte v;
  byte a;
  byte head;
  int16_t heading_base; //2 направление по ИМУ
  stStatus status; 
} main_paket_otvet;

struct stGps { // =12
  byte type=GPS_PAKET;
  uint8_t test1 = 0xBE;
  uint16_t test2 = 0xD000;  
  int32_t home_lat=3; // координаты "дома"
  int32_t home_lon=4;
} gps_paket;

struct stGpsOtvet { // =19
  byte type=GPS_PAKET+0x10;
  byte nSv; // 1
  int16_t heading; // 2  
  long lat; // 4
  long lon; // 4
  uint32_t gSpeed; // 4 скорость в см/сек
  uint16_t hdop; // 2
  byte fix; // 1
} gps_paket_otvet;

struct stAutopilot_Paket { // =12
  byte type=AUTOPILOT_PAKET;
  enActions action;
  byte active_point; // или 255 если автопилот отключен
  byte gluke; // лишний байт в пакете - выравнивание в структуре
  long lat; // координаты точки закорма
  long lon;
} autopilot_paket;

struct stAutopilot_Paket_Otvet { // =3
  byte type=(0x10+AUTOPILOT_PAKET);
  byte action;
  byte cur_point; // или 255 если автопилот отключен
} autopilot_paket_otvet;

struct stSonar_Paket { // =4
  byte type=SONAR_PAKET;
  byte idx=0; // индекс на прием
  byte speed19k=1;
  byte deep=0; // минимальная глубина (начало прослушки эха)
} sonar_paket;

struct stSonar_Paket_Otvet { // =32
  byte type=SONAR_PAKET+0x10;
  byte idx=0; // индекс на прием
  byte mas[30]; // массив данных с сонара
} sonar_paket_otvet;

struct stBind_Paket {
  byte type = BIND_PAKET;
  byte idx;
  byte mas[10];
} bind_paket;

struct stBind_Paket_Otvet {
  byte type = (0x10 + BIND_PAKET);
  byte pusto;
} bind_paket_otvet;


/////////////////////////////////////////////////////////////////////

void RXhandler(uint8_t len){
    // обрабатываем принятый пакет
    use_paket();
   
    time_radio_filesafe = millis() + TIME_RADIO_FILESAFE;
    time_gohome = millis() + TIME_RADIO_GOHOME;

    // отправляем запрошенный пакет
    send_otvet();
}

void TXhandler(){
    // уходим на следующую частоту
    do_hop();
    // запоминаем время для следующего хопа
    time_next_paket = millis() + TIME_SLOT_MS + TIME_SLOT_MS/2 ;
}

#ifdef RADIO_USE_IRQ
void radio_assert_irq(){
  attachInterrupt( digitalPinToInterrupt(pinIRQ), radio_irq, LOW);  
}

void radio_deassert_irq(){
  detachInterrupt( digitalPinToInterrupt(pinIRQ) );
}

void radio_irq(){
  //radio_deassert_irq();
  radio.handler();
  //radio_assert_irq();
}
#endif

void radio_setup( )
{
  #ifdef RADIO_USE_IRQ
    radio_deassert_irq();
  #endif
  byte adr[] = {">CARP"};
  radio.local_address = adr;
  radio.remote_address= adr;

  hop_idx = 0;
  radio.channel = hops[ hop_idx ];

  radio.RX_handler = RXhandler;
  radio.TX_handler = TXhandler;

  radio.rx_buffer = (uint8_t*)&mas;

  radio.begin();

  radio.listening();
  
  #ifdef RADIO_USE_IRQ
    radio_assert_irq();
  #endif

  uint8_t a[6];
  radio.getAddr(a, 0);
  Serial.print(F("Reset Radio "));Serial.print(hops[ hop_idx ]);Serial.print(" ");Serial.println((char*)a);
  Serial.print(F("Hops: "));
  for (byte i=0; i< sizeof(hops); i++) {
    Serial.print( hops[i] ); Serial.print(" ");
  }
  Serial.println();

#ifdef RADIO_USE_IRQ
  radio_deassert_irq();
  radio_assert_irq();
#endif  
  
  time_next_paket = micros();
}

void send_otvet(void){
    int hdtmp;
    
    if ( main_paket_otvet.status.sonar ) { // готовы данные сонара для отправки - приоритет отправки
      curr_paket = SONAR_PAKET;  
    } else {
      if (curr_paket == SONAR_PAKET) curr_paket = MAIN_PAKET; // на запрос параметров сонара отвечаем стандартным пакетом
      // иначе что запросили - то и отправляем
    }
    
    #ifdef PRINT_SEND_PAKET
      Serial.print("Otv ");Serial.println(curr_paket);
    #endif
    
    switch (curr_paket) {
      case MAIN_PAKET:
        //main_paket_otvet.v = main_paket.x ;
        main_paket_otvet.v = (napruga+1)/10;
        main_paket_otvet.a = (tok+1)/10;
        hdtmp = heading;
        if ( (hdtmp >=349) || (hdtmp<=11) ) {
          hdtmp = 0;
        } else {
          hdtmp = heading + 11; // сдвигаем на полшкалы
          hdtmp = (float) hdtmp / 22.5;
        }          
        #ifdef PRINT_HEADING_SECTOR
          Serial.print(F("head "));Serial.println(hdtmp);
        #endif
        main_paket_otvet.head = hdtmp;
        main_paket_otvet.heading_base = heading_base;
        radio.send_data( (uint8_t*)&main_paket_otvet, sizeof(main_paket_otvet) );    
        //Serial.println("MAIN_PAKET_OTVET");
        break;
                
      case AUTOPILOT_PAKET:
        // текущая цель с учетом доезда и мануала
        autopilot_paket_otvet.cur_point = active_point;
        autopilot_paket_otvet.action = active_action;
        radio.send_data( (uint8_t*)&autopilot_paket_otvet, sizeof(autopilot_paket_otvet) );          
/*  
        Serial.print("Otvet ");
        Serial.println(autopilot_paket_otvet.cur_point);
*/               
        //Serial.print(" AUTOPILOT_PAKET_OTVET ");Serial.println(millis());
    
        break;  

      case GPS_PAKET:
        radio.send_data( (uint8_t*)&gps_paket_otvet, sizeof(gps_paket_otvet) );          
        break;  

      case BIND_PAKET:
        radio.send_data( (uint8_t*)&bind_paket_otvet, sizeof(bind_paket_otvet) );          
        //for(byte i=0; i<sizeof(hops); i++) hops[i] = bind_paket.mas[i]; 
        write_to_eeprom();
        //read_from_eeprom();
        Serial.println("BINDING OFF!");
        Serial.print("New hops=");
        for(byte i=0; i<sizeof(hops); i++) {Serial.print(bind_paket.mas[i]); Serial.print(" ");}
        Serial.println();
        break;  

      case SONAR_PAKET: 
        sonar_paket_otvet.idx = sonar_idx_tx;
        for (byte i=0; i<sizeof(sonar_paket_otvet.mas); i++) {
          sonar_paket_otvet.mas[i] = sonar[sonar_idx_tx];
          if ( ++sonar_idx_tx >= sizeof(sonar) ) {
            main_paket_otvet.status.sonar = false; // передали все данные сонара
            //sonar_setup(); // перезапускаем цикл эхолота
            time_sonar = millis()+50;
            sonar_idx_tx = 0;
            break;
          }
        }
        radio.send_data( (uint8_t*)&sonar_paket_otvet, sizeof(sonar_paket_otvet) );      
        //Serial.print("SONAR_PAKET_OTVET ");Serial.println(sonar_paket_otvet.idx);
        break;  
 
      default:
        return;
    } 
    //Serial.println();
}

void use_paket(void) {
  curr_paket = (Type_Paket) mas[0];
#ifdef PRINT_RECV_PAKET
  Serial.print(millis() - time_paket);Serial.print(" ");Serial.print(curr_paket);Serial.print(" ");
  time_paket = millis();
  switch (curr_paket) {
    case MAIN_PAKET:
        Serial.println(F("MAIN")); break;
    case AUTOPILOT_PAKET:
        Serial.println(F("AUTO")); break;
    case SONAR_PAKET:
        Serial.print(F("SONAR ")); Serial.print( mas[1] ); Serial.print(" ");  break;
    case BIND_PAKET:
        Serial.println(F("BIND")); break;
    case GPS_PAKET:
        Serial.println(F("GPS")); break;
    defaault: Serial.println(F("Unknown"));
  }
#endif  
  switch (curr_paket) {
    case MAIN_PAKET:
      //time_next_paket = micros(); // синхронизируемся
      memcpy(&main_paket, (uint8_t*)mas, sizeof(main_paket));

      if ((main_paket.command.light & 0b01) < 1 ) digitalWrite(pinLight1, LOW);
      else digitalWrite(pinLight1, HIGH);
      
      if ((main_paket.command.light & 0b10) < 1) digitalWrite(pinLight2, LOW);
      else digitalWrite(pinLight2, HIGH);

      if ( active_point != 254 ) { // не работает автопилот
        if ( main_paket.command.bunker ) open_bunker();
        else close_bunker();
      }

#ifdef PRINT_CONTROLS
      Serial.print(F("x "));
      Serial.print(main_paket.x);
      Serial.print(F(" y "));
      Serial.print(main_paket.y);
      Serial.print(F(" bunker "));
      Serial.print(main_paket.command.bunker);
      Serial.print(F(" trap "));
      Serial.print(main_paket.command.trap);
      Serial.print(F(" light "));      
      Serial.println(main_paket.command.light);
#endif
      break;
      
    case AUTOPILOT_PAKET: 
      
      memcpy(&autopilot_paket, (uint8_t*)mas, sizeof(autopilot_paket));
      //Serial.print("Point ");Serial.print(autopilot_paket.cur_point);Serial.print(" Action ");Serial.println(autopilot_paket.action);
      //for (byte k=0; k<sizeof(mas); k++) { Serial.print(mas[k],HEX);Serial.print(","); } Serial.println();

      switch ( autopilot_paket.action ) {
        case ACTION_FEED:
        case ACTION_TARGET:
            // устанавливаем запрошенную цель
            active_action = autopilot_paket.action;
            active_point = autopilot_paket.active_point;
            if (active_action != ACTION_LAST) {
              target_lat = (float) autopilot_paket.lat / 10000000.0;
              target_lon = (float) autopilot_paket.lon / 10000000.0;
            }
            //Serial.println(active_point);
            break;
        default: // запрос состояния автопилота
            //Serial.print("Unknown action "); Serial.println(autopilot_paket.action);
            break;
      }
      //Serial.println("AUTOPILOT_PAKET");
#ifdef PRINT_AUTOPILOT      
      Serial.print(F("Apnt ")); Serial.print(autopilot_paket.active_point);
      Serial.print(F(" Ac="));Serial.print(active_action);Serial.print(F(" Trgt "));
      Serial.print(target_lat,7);Serial.print(F(" - "));Serial.print(target_lon,7);Serial.print(", H=");Serial.print(home_lat/ 10000000.0,7);Serial.print(" - ");Serial.println(home_lon/ 10000000.0,7);
#endif      
      break;  

    case SONAR_PAKET: 
      memcpy(&sonar_paket, (uint8_t*)mas, sizeof(sonar_paket));
#ifdef PRINT_RECV_PAKET
      Serial.print(" S_P_Z ");Serial.print(sonar_paket.idx);
#endif      
      break;  

    case GPS_PAKET: 
      memcpy(&gps_paket, (uint8_t*)mas, sizeof(gps_paket));
      home_lat =  gps_paket.home_lat ;
      home_lon =  gps_paket.home_lon ;
#ifdef PRINT_RECV_HOME      
      Serial.print(F("Hlat "));Serial.print(home_lat);
      Serial.print(F(" Hlon "));Serial.println(home_lon);
      for (byte k=0; k<sizeof(mas); k++) { Serial.print(mas[k],HEX);Serial.print(","); } Serial.println();
#endif
      break;  

    case BIND_PAKET: 
      memcpy(&bind_paket, (uint8_t*)mas, sizeof(bind_paket));
/*      
      Serial.println();
      Serial.print(F("BIND PAKET "));
      Serial.print(bind_paket.idx);
      Serial.print("=");
      for (byte i=0; i<bind_paket.idx; i++) { 
        Serial.print(bind_paket.mas[i]);Serial.print(","); 
      } 
      Serial.println();
*/
      break;  
      
    default:
      Serial.print(F("UNKNOWN PAKET 0x"));
      Serial.println(curr_paket, HEX);
      for (byte k=0; k<sizeof(mas); k++) { Serial.print(mas[k],HEX);Serial.print(","); } Serial.println();
      return;
  }
  mas[0] = NO_PAKET;
}

void do_hop(void){
  if ( ++hop_idx >= sizeof(hops) ) hop_idx=0;
  if ( radio.channel != hops[ hop_idx ] ) {
    radio.channel = hops[ hop_idx ];
    radio.update_channel();    
  }
}

void binding(){
  byte adr[] = {"BINDC"};

  time_next_paket = millis() + 300000L;
  time_radio_filesafe = millis() + 300000L;
  time_gohome = millis() + 300000L;
  
  radio.local_address = adr;
  radio.remote_address= adr;
  radio.channel = 120;
  for (byte i=0; i<sizeof(hops); i++) hops[i]=120;
  radio.begin();
  Serial.println();
  Serial.println(F("BINDING MODE"));
  Serial.print(F("Hops="));
  for (byte i=0; i<sizeof(hops); i++) {
    Serial.print(hops[i]); Serial.print(" ");
  }
  Serial.println();
}
  
