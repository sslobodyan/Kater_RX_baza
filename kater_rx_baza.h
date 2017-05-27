
#define pinADC_TOK A0 
#define pinADC_BORT A7
#define pinLight1 A2
#define pinLight2 A3
#define pinDallas PD4

#define pinTrap 10

uint8_t hops[] = {50, 70, 100, 80}; // набор каналов для прыжков
//uint8_t hops[] = {100, 100, 100, 100}; // набор каналов для прыжков
uint8_t hop_idx=0; // текущий прыжок
uint32_t time_next_paket=0; //

#define USE_AUTOCURS
#define USE_FAILSAFE
#define USE_AUTOHOME

#define TEST_RADIO_

#define PRINT_HEADING_
#define PRINT_MAG_RAW_
#define PRINT_ACC_RAW_
#define PRINT_GYR_RAW_
#define PRINT_ACC_PITCH_
#define PRINT_TILLED_COMPASS_
#define PRINT_MAG_FTR_
#define PRINT_MAG_PLOT_
#define PRINT_PLOT_PITCH_
#define PRINT_COMPASS_PLOT_
#define PRINT_MAG_MAGNETO_

#define PRINT_X_Y_

#define PRINT_AUTOPILOT_
#define PRINT_RECV_PAKET_
#define PRINT_SEND_PAKET_
#define PRINT_RECV_HOME_
#define PRINT_CONTROLS_
#define PRINT_SONAR_

#define NEITRAL_CONTROLS_DELTA 100
#define TIME_BUNKER_PAUSE 20
#define MAX_AUTOCOURSE_DELTA 60 // максимальная корректировка руля при автокурсе

volatile byte mas[32];

Servo kil_servo, gaz_servo, bunker_servo, trap_servo;

int tok, napruga;
int16_t adc_null_tok=0;
volatile unsigned long time_adc=0, time_radio_filesafe=0, time_autopilot=0, time_gohome=0, time_bunker=0, time_sonar, time_paket;

int16_t XAxis, ZAxis, YAxis;
int16_t old_XAxis, old_YAxis;

bool compass_ok, compas_filtrate=true;

uint8_t compass_buf[6];

int start_heading = -1; // компас после старта - строго от берега
int heading=0, heading_base;
#define K_compass 0.8f // фильтр нижних частот для компаса
#define K_tok 0.4f
#define K_napruga 0.25f


void test_gps(void);
bool tilled_compass_setup(void);
int get_tilled_compass(bool);

enum enActions: byte {
  ACTION_SAVE,
  ACTION_CLEAR,
  ACTION_TARGET,
  ACTION_FEED,
  ACTION_LAST,
  ACTION_NONE,
  ACTION_GOHOME
};

enActions active_action=ACTION_LAST;
byte active_point=255;

float target_lat, target_lon, home_lat, home_lon;

volatile uint8_t sonar[240];
volatile bool adc_busy=false; // флаг занятости ацп под сонар

int16_t free_heading = -1; 
int16_t delta;

bool sonar_done;
volatile byte sonar_idx, sonar_idx_tx;

int distance; // расстояние до цели (точки) в метрах

int angle_up, angle_down; // для фильтрации азимута

byte state_bunker=0, old_bunker;

bool armed = false;

void open_bunker();
void close_bunker();
void sonar_setup();
void do_hop();
void priem_paketa(void);
void use_paket(void);
void send_otvet(void);

void radio_assert_irq();
void radio_deassert_irq();
void radio_irq();
void update_bunker();
void binding();
void write_to_eeprom();
void read_from_eeprom();



