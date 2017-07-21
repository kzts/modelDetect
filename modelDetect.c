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

#define MICRO_TO_SEC 0.000001
#define MS_TO_SEC 0.001

#define EXHAUST 0.0

#define LINE_NUM 20000
#define STR_NUM 4096

#define DELIMITER ","
#define FILENAME_FORMAT "data/%04d%02d%02d/%02d%02d%02d.dat"

#define REPEAT_NUM 50
#define LOOP_TIME 0.2
#define PRESSURE_MAX 0.3
#define PRESSURE_CHANGE 0.025

struct timeval ini_t, now_t, loop_ini_t;

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
void exhaustAll(){
  int c;
  for ( c = 0; c< NUM_OF_CHANNELS; c++ )
    setState( c, EXHAUST );
}

double getTime(void){
  gettimeofday( &now_t, NULL );
  double now_time = + 1.0*( now_t.tv_sec - ini_t.tv_sec ) + MICRO_TO_SEC*( now_t.tv_usec - ini_t.tv_usec );
  return now_time;
}

double getTimeLoop(void){
  gettimeofday( &now_t, NULL );
  double now_time = + 1.0*( now_t.tv_sec - loop_ini_t.tv_sec ) 
    + MICRO_TO_SEC*( now_t.tv_usec - loop_ini_t.tv_usec );
  return now_time;
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

double rand_in_range( double min, double max ){
  return min + ( rand() * ( max - min + 0.0) / (0.0 + RAND_MAX) );
}

void generateCommands(void){
  int c;
  double mid0 = rand_in_range( PRESSURE_CHANGE, PRESSURE_MAX - PRESSURE_CHANGE );
  double mid1 = rand_in_range( PRESSURE_CHANGE, PRESSURE_MAX - PRESSURE_CHANGE );
  double chg0 = rand_in_range( 0.0, PRESSURE_CHANGE );
  double chg1 = rand_in_range( 0.0, PRESSURE_CHANGE );
  double pm0  = 1.0;
  double pm1  = 1.0;
  // initilize
  for ( c = 0; c< NUM_OF_CHANNELS; c++ ){
    valve_old[c] = valve_now[c];
    valve_now[c] = 0.0; 
  }
  //valve_now[c] = PRESSURE_MAX* rand()/ RAND_MAX;
  if ( rand() > 0.5* RAND_MAX )
      pm0 = -1.0;
  if ( rand() > 0.5* RAND_MAX )
      pm1 = -1.0;
    
  valve_now[0] = mid0 + pm0* chg0;
  valve_now[1] = mid0 - pm0* chg0;
  valve_now[2] = mid1 + pm1* chg1;
  valve_now[3] = mid1 - pm1* chg1;
}

int modelDetect(void){
  int r, c;
  int n = 0;
  // initialize
  gettimeofday( &ini_t, NULL );
  srand((unsigned)time(NULL));
  // loop
  for ( r = 0; r < REPEAT_NUM; r++ ){
    // reset timer
    gettimeofday( &loop_ini_t, NULL );
    // get commands
    generateCommands();
    // set commands
    for ( c = 0; c< NUM_OF_CHANNELS; c++ )
      setState( c, valve_now[c] );
    // get sensors
    while( getTimeLoop() < LOOP_TIME ){
      getSensors(n);
      n++;
    }
  }
  return n;
}

void saveResults(int end_step) {
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
  // initialize
  init();
  init_pins(); // ALL 5 pins are HIGH except for GND
  init_DAConvAD5328();
  init_sensor();  
  exhaustAll();
  // generate and measure motions
  int n = modelDetect();
  // unitialize
  exhaustAll();
  // save data
  saveResults(n);
  return 0;
}

