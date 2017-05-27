volatile int sonar_skip_idx=0;

#define sonar_pin A6
#define sonar_pin_trig 3

#define DISARM_RADIO

void sonar_start_cycle(void);


void sonar_setup() {
  pinMode(sonar_pin_trig, INPUT);
  // ждем посылку строба
  EIFR = _BV( INTF1 );
  attachInterrupt( 1, sonar_start_cycle, FALLING );
}


ISR(ADC_vect) {
  uint8_t data;

  data = ADCH;

  if ( sonar_skip_idx ) {
      sonar_skip_idx -= 1; // ждем запрошенную глубину
  } else {
      sonar[sonar_idx] = data;
      sonar_idx++;
      if (sonar_idx >= sizeof(sonar) ){
        // набрали полный буфер, заканчиваем
        ADCSRA = _BV(ADEN)  | // ADC enable
                 _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
        // устанавливаем флаг готовности данных сонара
        main_paket_otvet.status.sonar = true;
        sonar_done = true;
        adc_busy = false;
        
#ifdef RADIO_USE_IRQ 
#ifdef DISARM_RADIO       
        radio_assert_irq(); // включаем контроль радиоканала
#endif        
#endif 
        //Serial.println("SF");
      }
  }
}

void sonar_start_cycle(void) { // попадаем сюда по внешнему прерыванию от сонара
  
  detachInterrupt( 1 ); // отключаем слежение за триггером сонара
#ifdef RADIO_USE_IRQ  
#ifdef DISARM_RADIO       
  radio_deassert_irq(); // отключаем реакцию на радиоканал
#endif  
#endif  

  EIFR = _BV( INTF1 );
  adc_busy = true;
  // настраиваем АЦП на чтение с ножки эха постоянно и запускаем чтение
  ADMUX  = ((sonar_pin - 14)&0x07) | bit(ADLAR) | bit(REFS0); // Channel sel, left-adj, use AVCC
  //ADMUX  = 0b11000000 | ((sonar_pin - 14)&0x07) | bit(ADLAR) | bit(REFS0); // Channel sel, left-adj, use REF1V1
  ADCSRB = 0;                // Free run mode, no high MUX bit
  if ( sonar_paket.speed19k ) { 
      // глубина до 9.048м
      ADCSRA = _BV(ADEN)  | // ADC enable
               _BV(ADSC)  | // ADC start
               _BV(ADATE) | // Auto trigger
               _BV(ADPS2) | _BV(ADPS1) ; // 64:1 / 13 = 19230 Hz
      sonar_skip_idx = (float) sonar_paket.deep * 240/9.048; 
  } else { 
      // меньше частота опроса - больше глубина до 18.096
      ADCSRA = _BV(ADEN)  | // ADC enable
               _BV(ADSC)  | // ADC start
               _BV(ADATE) | // Auto trigger
               _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
      sonar_skip_idx = (float) sonar_paket.deep * 240/18.096; 
  }
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  sonar_idx = 0;
}

