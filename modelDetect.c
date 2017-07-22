#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h> // rand
#include <fcntl.h>
#include <unistd.h>
#include "gpio.h"
#include <stdbool.h>
#include <sys/time.h> // gettimeofday   
#include <string.h> // fsprintf
#include <time.h> // localtime
// xenomai
#include <native/task.h>
#include <native/timer.h>

//  valves
#define pin_spi_cs1   P9_16 // 1_19=51
#define pin_spi_other P9_22 // 0_2=2 
#define pin_spi_mosi  P9_30 // 3_15=112
#define pin_spi_sclk  P9_21 // 0_3=3 
#define pin_spi_cs2   P9_42 // 0_7 =7
#define NUM_OF_CHANNELS 16 
// sensors
#define pin_din_sensor    P9_11 // 0_30=30 
#define pin_clk_sensor    P9_12 // 1_28=60 
#define pin_cs_sensor     P9_13 // 0_31=31 
#define pin_dout1_sensor  P9_14 // 1_18=50 
#define pin_dout2_sensor  P9_15 // 1_16=48 
//#define pin_dout3_sensor  P9_26 // 0_14=14 
//#define pin_dout3_sensor  P9_23 // s-mori compsition 
#define NUM_ADC_PORT 8
#define NUM_ADC 2

#define NANO_TO_SEC 0.000000001
//#define MICRO_TO_SEC 0.000001
//#define MS_TO_SEC 0.001

#define EXHAUST 0.0

// files
#define LINE_NUM 20000
#define STR_NUM 4096
#define DELIMITER ","
#define FILENAME_FORMAT "data/%04d%02d%02d/%02d%02d%02d.dat"
#define ANGLE_LIMIT_FILE (const char*) "data/angle_limit.dat"

// control
#define JOINT_NUM 2
#define LIMIT_NUM 2
unsigned long target_angle[JOINT_NUM];
unsigned long angle_limit[JOINT_NUM][LIMIT_NUM];

// loop
#define END_TIME 2.0
//#define STEP_NUM 1000
#define CHANGE_STEP 100
unsigned int step   = 0;
//unsigned int is_end = 0;
unsigned int is_init = 0;
//#define REPEAT_NUM 100
//#define LOOP_TIME 0.1
//#define PRESSURE_MAX 0.3
//#define PRESSURE_CHANGE 0.05
RTIME ini_t, now_t;

// xenomai
#define PRIORITY 99
#define PERIOD  1000000 // nano sec 
#define ONE_IN_NANO 1000000000

// data
unsigned long sensor_data[LINE_NUM][NUM_ADC][NUM_ADC_PORT];
double valve_data[LINE_NUM][NUM_OF_CHANNELS];
double time_data[LINE_NUM];

double valve_now[NUM_OF_CHANNELS];
double valve_old[NUM_OF_CHANNELS];

// SPI for valves 
bool clock_edge = false;
unsigned short resolution = 0x0FFF;
void set_SCLK(bool value) { digitalWrite(pin_spi_sclk, value); } 
void set_OTHER(bool value) { digitalWrite(pin_spi_other, value); } 
void set_MOSI(bool value) { digitalWrite(pin_spi_mosi, value); } 
void setCS1(bool value){ digitalWrite(pin_spi_cs1, value); } 
void setCS2(bool value){ digitalWrite(pin_spi_cs2, value); } 
void set_clock_edge(bool value){ clock_edge = value; } 
bool get_MISO(void) { return false; } // dummy 
void wait_SPI(void){} 

// value 1: Enable chipx
void chipSelect1(bool value){ setCS1(!value); wait_SPI(); wait_SPI(); }
void chipSelect2(bool value){ setCS2(!value); wait_SPI(); wait_SPI(); }

unsigned char transmit8bit(unsigned char output_data){
  unsigned char input_data = 0;
  int i;
  for(i = 7; i >= 0; i--){
    // MOSI - Master : write with down trigger
    //        Slave  : read with up trigger
    // MISO - Master : read before down trigger
    //        Slave  : write after down trigger
    set_SCLK(!clock_edge);
    set_MOSI( (bool)((output_data>>i)&0x01) );
    input_data <<= 1;
    wait_SPI();
    set_SCLK(clock_edge);
    input_data |= get_MISO() & 0x01;
    wait_SPI();
  }
  return input_data;
}

unsigned short transmit16bit(unsigned short output_data){
  unsigned char input_data_H, input_data_L;
  unsigned short input_data;
  input_data_H = transmit8bit( (unsigned char)(output_data>>8) );
  input_data_L = transmit8bit( (unsigned char)(output_data) );
  input_data = (((unsigned short)input_data_H << 8)&0xff00) | (unsigned short)input_data_L;
  return input_data;
}


void setDARegister(unsigned char ch, unsigned short dac_data){
  unsigned short register_data;

  if (ch < 8) {
    register_data = (((unsigned short)ch << 12) & 0x7000) | (dac_data & 0x0fff);
    chipSelect1(true);
    transmit16bit(register_data);
    chipSelect1(false);
  }
  else if (ch >= 8) {
    register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
    chipSelect2(true);
    transmit16bit(register_data);
    chipSelect2(false);
  }
}

// pressure coeff: [0.0, 1.0]
void setState(unsigned int ch, double pressure_coeff)
{
  setDARegister(ch, (unsigned short)(pressure_coeff * resolution));
}

// **** SPI for sensors ****
void set_DIN_SENSOR(bool value) { digitalWrite(pin_din_sensor, value); }
void set_CLK_SENSOR(bool value) { digitalWrite(pin_clk_sensor, value); }
void set_CS_SENSOR(bool value) { digitalWrite(pin_cs_sensor, value); }

int get_DOUT_SENSOR(int adc_num) { 
  if(adc_num==0){
    digitalRead(pin_dout1_sensor); 
    //}else if(adc_num==1){
    //digitalRead(pin_dout2_sensor); 
  }else{
    digitalRead(pin_dout2_sensor); 
    //digitalRead(pin_dout3_sensor); 
  }
}

unsigned long *read_sensor(unsigned long adc_num,unsigned long* sensorVal){
  
  unsigned long pin_num=0x00;
  unsigned long sVal;
  unsigned long commandout=0x00;
  
  int i;
  
  for(pin_num=0;pin_num<NUM_ADC_PORT;pin_num++){
    sVal=0x00;
    set_CS_SENSOR(true);
    set_CLK_SENSOR(false);
    set_DIN_SENSOR(false);
    set_CS_SENSOR(false);
    
    commandout=pin_num;
    commandout|=0x18;
    commandout<<=3;
    
    for(i=0;i<5;i++){
      if(commandout&0x80){
	set_DIN_SENSOR(true);
      }
      else{
	set_DIN_SENSOR(false);
      }
      commandout<<=1;
      set_CLK_SENSOR(true);
      set_CLK_SENSOR(false);
    }
    for(i=0;i<2;i++){
      set_CLK_SENSOR(true);
      set_CLK_SENSOR(false);
    }
    for(i=0;i<12;i++){
      set_CLK_SENSOR(true);
      sVal<<=1;
      if(get_DOUT_SENSOR(adc_num)){
	sVal|=0x01;
      }
      set_CLK_SENSOR(false);
    }
    sensorVal[pin_num]=sVal;
  }
  return(sensorVal);
}

// *******************************************
//               Init Functions              
// *******************************************
void init_pins()
{
  set_SCLK(LOW);
  set_MOSI(LOW);
  set_OTHER(LOW);
  setCS1(HIGH);
  setCS2(HIGH);
  
  //set_SCLK(HIGH);
  //set_MOSI(HIGH);
  //set_OTHER(HIGH);
  //setCS1(HIGH);
  //setCS2(HIGH);
  
  //analog_pin[0] = P9_33;
  //analog_pin[1] = P9_35;
  //analog_pin[2] = P9_36;
  //analog_pin[3] = P9_37;
  //analog_pin[4] = P9_38;
  //analog_pin[5] = P9_39;
  //analog_pin[6] = P9_40;
  
}


void init_DAConvAD5328(void) {
  set_clock_edge(false);// negative clock (use falling-edge)

  // initialize chip 1
  chipSelect1(true);
  transmit16bit(0xa000);// synchronized mode
  chipSelect1(false);

  chipSelect1(true);
  transmit16bit(0x8003);// Vdd as reference
  chipSelect1(false);

  // initialize chip 2
  chipSelect2(true);
  transmit16bit(0xa000);// synchronized mode
  chipSelect2(false);

  chipSelect2(true);
  transmit16bit(0x8003);// Vdd as reference
  chipSelect2(false);
}

void init_sensor(void) {
  set_DIN_SENSOR(false);
  set_CLK_SENSOR(false);
  set_CS_SENSOR(false);
}

//--------------------------------------------------------------
// below my function
//---------------------------------------------------------------
double getTime(void){
  now_t = rt_timer_read();
  return NANO_TO_SEC*( now_t - ini_t );
}

void exhaustAll(){
  int c;
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    setState( c, EXHAUST );
}

void getSensors( unsigned int n ){
  unsigned int b, p, c;
  unsigned long tmp_val[NUM_ADC_PORT];
  unsigned long *tmp_val0;
  // sensors
  //printf( "sensor: " );
  for ( b=0; b<NUM_ADC; b++){
    tmp_val0 = read_sensor( b, tmp_val );
    for ( p=0; p<NUM_ADC_PORT; p++ ){
      sensor_data[n][b][p] = tmp_val0[p]; 
      //printf( "%d ", tmp_val0[p] );
    }
  }
  //printf("\n");
  // valves
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    valve_data[n][c] = valve_now[c];
  // time
  time_data[n] = getTime();
}

long rand_in_range( double min, double max ){
  return min + (long)(( max - min )*( rand()/ RAND_MAX ));
}

void changeTargetAngle(void){
  unsigned int j, l;
  for ( j = 0; j < JOINT_NUM; j++ )
    target_angle[j] = rand_in_range( angle_limit[j][0], angle_limit[j][1] );
}

void xen_thread(void *arg __attribute__((__unused__))) {
  printf( "starting real time thread\n" ); 
  // wait for stabilization
  rt_task_sleep( ONE_IN_NANO );
  // set periodic time
  if( rt_task_set_periodic( NULL, TM_NOW, PERIOD ))
    fprintf( stderr, "Set Periodic Error!", 1 );
  printf("set loop period\n");
  // set init
  ini_t   = rt_timer_read();
  is_init = 1;
  // task
  while(1) {
    // wait to keep periodic time
    if(rt_task_wait_period(NULL)){                  
      fprintf( stderr, "Loop Error!\n", 1 ); // too fast loop
    }
    // change target angle
    if( step % CHANGE_STEP == 0 )
      changeTargetAngle();
    // control
    //pControl();
    // measure
    getSensors(step);
    // next
    step++;
  }
}



void loadAngleLimit(void) {
  FILE *fp;
  char str[STR_NUM];
  unsigned int j, l;
  // open file
  fp = fopen( ANGLE_LIMIT_FILE, "r" );
  if (fp == NULL){
    printf( "File open error: %s\n", ANGLE_LIMIT_FILE );
    return;
  }
  // read
  fscanf( fp, "%lu,%lu,%lu,%lu", &angle_limit[0][0], &angle_limit[0][1], &angle_limit[1][0], &angle_limit[1][1] );
  // close
  fclose(fp);
  // print
  printf("angle limit: \n");
  for ( j=0; j<JOINT_NUM; j++ ){
    for ( l=0; l<LIMIT_NUM; l++ )
      printf( "%04d ", angle_limit[j][l] );
    printf("\n");
  }
}

void saveResults(unsigned int end_step) {
  FILE *fp;
  char results_file[STR_NUM];
  char str[STR_NUM];
  char tmp_char[STR_NUM];
  unsigned int n, b, p, v;
  time_t timer;
  struct tm *local;
  struct tm *utc;
  // generate file name
  timer = time(NULL);
  local = localtime(&timer);
  int year   = local->tm_year + 1900;
  int month  = local->tm_mon + 1;
  int day    = local->tm_mday;
  int hour   = local->tm_hour;
  int minute = local->tm_min;
  int second = local->tm_sec;
  sprintf( results_file, FILENAME_FORMAT, year, month, day, hour, minute, second );
  // open file
  fp = fopen( results_file, "w");
  if (fp == NULL){
    printf( "File open error: %s\n", results_file );
    return;
  }
  // write file
  for ( n=0; n<end_step; n++ ){
    // time
    sprintf( str, "%lf%s", time_data[n], DELIMITER ); 
    // sensor
    for ( b = 0; b<NUM_ADC; b++ ) {
      for ( p = 0; p<NUM_ADC_PORT; p++ ) {
	sprintf( tmp_char, "%lu%s", sensor_data[n][b][p], DELIMITER ); 
	strcat( str, tmp_char );      
      }
    }
    // command
    for ( v = 0; v<NUM_OF_CHANNELS; v++) {
      sprintf( tmp_char, "%lf%s", valve_data[n][v], DELIMITER ); 
      strcat( str, tmp_char );
    }
    // end of line
    strcat( str, "\n" );
    // write
    fputs( str, fp );
  }
  // close
  fclose(fp);
  printf("saved %s. %d lines\n", results_file, end_step );
}

int main( int argc, char *argv[] ){
  double now_time;
  RT_TASK thread_desc; 
  // load angle limit
  loadAngleLimit();
  /*
  // initialize
  init();
  init_pins(); // ALL 5 pins are HIGH except for GND
  init_DAConvAD5328();
  init_sensor();  
  exhaustAll();
  // create tasks
  if( rt_task_create( &thread_desc, "BBB_RT", 0, PRIORITY, 0 )){
    fprintf( stderr, "Task Create Error!\n", 1 );
    return(0);
  }
  // start tasks
  if(rt_task_start( &thread_desc, &xen_thread, NULL )){
    fprintf( stderr, "Task Start Error!\n",1 );
    return(0);
  }
  // initialize
  srand((unsigned)time(NULL));
  ini_t = rt_timer_read();
  // loop
  now_time = getTime();
  //while( getTime() < END_TIME || is_init < 1 ){
  while( now_time < END_TIME || is_init < 1 ){
    //printf( "%d %d %lf\n", is_init, step, getTime() );
    //rt_task_sleep( 100 );
    now_time = getTime();
  }
  // delete tasks
  if( rt_task_delete( &thread_desc )){
    fprintf( stderr, "Task Delete Error!\n", 1 );
    //return(0);
  }
  // unitialize
  exhaustAll();
  // save data
  saveResults(step);
  */
  return 0;
}

