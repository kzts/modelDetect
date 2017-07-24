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
#define LINE_NUM 5000
#define STR_NUM 4096
#define DELIMITER ","
#define FILENAME_FORMAT "data/%04d%02d%02d/%02d%02d%02d.dat"
#define COMMANDS_FILE "data/commands.dat"

// control
#define JOINT_NUM 2
#define LIMIT_NUM 2
#define CHAMBER_NUM 2
#define ANGLE_BOARD 1
#define ANGLE_PORT 0
double p_gain, i_gain, d_gain;

// loop
unsigned int step = 0; // counter
unsigned int is_stop = 0;
unsigned int is_end = 0;
#define PRESSURE_MAX 0.06
#define PRESSURE_FIX 0.06
//#define PRESSURE_CHANGE 0.05
RTIME ini_t, now_t;

// xenomai
#define PRIORITY 99
//#define PERIOD  1000000 // nano sec 
#define PERIOD  2000000 // nano sec 
#define ONE_IN_NANO 1000000000

// motion
#define STOP_VELOCITY 2
#define WAIT_TIME 1
#define NEAR_ANGLE 30
#define PRESSURE_STOP 0.5
#define OTHER 3
#define FORWARD 0
#define BACK 1
unsigned long ini_angle;
unsigned long tar_angle;
double pressure_forward;
double pressure_back;

// data
unsigned long sensor_data[LINE_NUM][NUM_ADC][NUM_ADC_PORT];
unsigned long target_data[LINE_NUM][JOINT_NUM];
double valve_data[LINE_NUM][NUM_OF_CHANNELS];
double time_data[LINE_NUM];
double valve_now[NUM_OF_CHANNELS];

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

void exhaustAll(void){
  int c;
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    setState( c, EXHAUST );
}

void getSensors(void){
  unsigned int b, p;
  unsigned long tmp_val[NUM_ADC_PORT];
  unsigned long *tmp_val0;
  // sensors
  //printf( "sensor: " );
  for ( b=0; b<NUM_ADC; b++){
    tmp_val0 = read_sensor( b, tmp_val );
    for ( p=0; p<NUM_ADC_PORT; p++ ){
      sensor_data[step][b][p] = tmp_val0[p]; 
      //printf( "%d ", tmp_val0[p] );
    }
  }
  //printf("\n");
  // time
  time_data[step] = getTime();
}

void getValves(void){
  unsigned int c;
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    valve_data[step][c] = valve_now[c];
}

unsigned long getAngle(unsigned int n, unsigned int angle_port){
  return sensor_data[n][ANGLE_BOARD][angle_port];
}

long getVelocity(unsigned int n, unsigned int p){
  if( n == 0 )
    return 0;
  else
    return getAngle(n,p) - getAngle(n-1,p);
}

int getMaxVelocity(unsigned int n){
  long vel0 = abs( getVelocity( n, 0 ));
  long vel1 = abs( getVelocity( n, 1 ));
  if( vel0 > vel1 )
    return vel0;
  else
    return vel1;
}

int is_reach(unsigned int n){
  if( getAngle(n,0) < ANGLE_END0 )
    return 1;
  if( getAngle(n,1) < ANGLE_END1 )
    return 1;
  else
    return 0;
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
  for ( c=0; c< NUM_OF_CHANNELS; c++ ){
    setState( c, pressure[c] );
    valve_now[c] = pressure[c];
  }
  ini_t = rt_timer_read();
  // task
  while(1) {
    // detect delay
    if(rt_task_wait_period(NULL))
      fprintf( stderr, "Loop Error!\n", 1 ); 
    // measure
    getSensors();
    // phase switch
    if ( is_stop < 0 ){
      // swing phase
      // get angle
      if( find_reach() > 0 ){
	// phase end
	is_acce = 0;
	// stop arm
	for ( c=0; c< NUM_OF_CHANNELS; c++ ){
	  setState( c, pressure_stop[c] );
	  valve_now[c] = pressure_stop[c];
	}
	is_stop = 1;
	printf("arm reached. stop.\n");	  
      }
    }else{
      // stop phase
      if( abs( getMaxVelocity(step) ) < STOP_VELOCITY ){
	// motion end
	is_stop = 0;
	is_end  = 1;
	printf("motion end.\n");	  
      }
    }
    // get command value
    getValves();
    // next
    step++;
  }// while
}//function

void saveResults(unsigned int end_step) {
  FILE *fp;
  char results_file[STR_NUM];
  char str[STR_NUM];
  char tmp_char[STR_NUM];
  unsigned int n, b, p, v, j;
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

int loadCommands(void){
  FILE *fp;
  char str[STR_NUM];
  char *tmp;
  int c;
  // open file
  fp = fopen( COMMAND_FILE, "w");
  if (fp == NULL){
    printf( "File open error: %s\n", COMMAND_FILE );
    return 0;
  }
  // read
  fgets( str, STR_NUM, fp );
  tmp = strtok( str,  DELIMITER ); 
  for ( c=0; c< NUM_OF_CHANNELS; c++ ){
    tmp = strtok( NULL, " " ); 
    pressure[c] = atof( tmp );
  }
  // close
  fclose(fp);
  // print
  printf("pressure: ");
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    printf( "%5.4f%s", pressure[c], DELIMITER );
  printf("\n");
  return 1;
}

int main( int argc, char *argv[] ){
  RT_TASK thread_desc; 
  // load commands
  if( loadCommands() < 1 )
    return 0;
  // initialize
  ini_t = rt_timer_read();
  init();
  init_pins(); // ALL 5 pins are HIGH except for GND
  init_DAConvAD5328();
  init_sensor();  
  exhaustAll();
  // take initial posture
  init_posture();
  // create tasks
  if( rt_task_create( &thread_desc, "BBB_RT", 0, PRIORITY, 0 )){
    fprintf( stderr, "Task Create Error!\n", 1 );
    return 0;
  }
  // start tasks
  if(rt_task_start( &thread_desc, &xen_thread, NULL )){
    fprintf( stderr, "Task Start Error!\n",1 );
    return 0;
  }
  // move arm
  while( is_end < 1 && step < LINE_NUM )
    getTime();
  // delete tasks
  if( rt_task_delete( &thread_desc )){
    fprintf( stderr, "Task Delete Error!\n", 1 );
    return 0;
  }
  // unitialize
  exhaustAll();
  // save data
  saveResults(step);
  
  return 0;
}

