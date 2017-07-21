#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "gpio.h"
#include <stdbool.h>
#include <time.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


// for valves
//#define pin_spi_cs1  P9_28 // 3_17=113
//#define pin_spi_other P9_29 // 3_16=111
//#define pin_spi_mosi P9_30 // 3_15=112
//#define pin_spi_sclk P9_31 // 3_14=110
//#define pin_spi_cs2  P9_42 // 0_7 =7
//#define NUM_OF_CHANNELS 16

// for valves
#define pin_spi_cs1  P9_16 // 1_19=51
#define pin_spi_other P9_22 // 0_2=2
#define pin_spi_mosi P9_30 // 3_15=112
#define pin_spi_sclk P9_21 // 0_3=3
#define pin_spi_cs2  P9_42 // 0_7 =7
#define NUM_OF_CHANNELS 16

// for sensors
#define pin_din_sensor  P9_11 // 0_30=30
#define pin_clk_sensor P9_12 // 1_28=60
#define pin_cs_sensor P9_13 // 0_31=31
#define pin_dout1_sensor P9_14 // 1_18=50
#define pin_dout2_sensor  P9_15 // 1_19=51
#define NUM_ADC_PORT 8
#define NUM_ADC 2

// for analog input
//#define NUM_OF_AINS 7
//#define AIO_NUM 7

/**** analog sensors ****/
//PIN analog_pin[NUM_OF_AINS];

/*
FILE* fd[AIO_NUM] = {};
void initAIO(void)
{
int i;
for (i = 0; i < AIO_NUM; i++) {
char filename[64];
sprintf(filename, "/sys/devices/ocp.3/helper.12/AIN%d", i);
if ((fd[i] = fopen(filename, "r")) == NULL) {
perror("cannot open:");
exit(1);
}
}
}
void closeAIO(void)
{
  int i;
  for (i = 0; i < AIO_NUM; i++)
    fclose(fd[i]);
}
uint32_t myAnalogRead(int i)
{
  uint32_t num;
  fseek(fd[i], 0, SEEK_SET);
  fscanf(fd[i], "%d", &num);
  fflush(fd[i]);
  return num;
}
*/

/**** SPI for valves ****/
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

/**** SPI for sensors ****/
void set_DIN_SENSOR(bool value) { digitalWrite(pin_din_sensor, value); }
void set_CLK_SENSOR(bool value) { digitalWrite(pin_clk_sensor, value); }
void set_CS_SENSOR(bool value) { digitalWrite(pin_cs_sensor, value); }
int get_DOUT_SENSOR(int adc_num) { 
  if(adc_num==0){
    digitalRead(pin_dout1_sensor); 
  }
  else{
    digitalRead(pin_dout2_sensor); 
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

/*******************************************/
/*              Init Functions              /
		/*******************************************/
void init_pins()
{
  set_SCLK(LOW);
  set_MOSI(LOW);
  set_OTHER(LOW);
  setCS1(HIGH);
  setCS2(HIGH);

  /*set_SCLK(HIGH);
    set_MOSI(HIGH);
    set_OTHER(HIGH);
    setCS1(HIGH);
    setCS2(HIGH);
  */
  /*
    analog_pin[0] = P9_33;
    analog_pin[1] = P9_35;
    analog_pin[2] = P9_36;
    analog_pin[3] = P9_37;
    analog_pin[4] = P9_38;
    analog_pin[5] = P9_39;
    analog_pin[6] = P9_40;
  */
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



int main(int argc, char *argv[]) {
  struct timeval start_timeval, end_timeval;
  double elasped_time;

  if ( argc != 2){
    printf("error: input tesst time [s].\n");
    return 0;
  }


  int n;
  int i,j,k;
  unsigned int ch_num;

  //double Sec_continue = 5.0;
  double Sec_continue = atof(argv[1]);

  gettimeofday( &start_timeval, NULL );
          
  init();
  init_pins(); // ALL 5 pins are HIGH except for GND
  init_DAConvAD5328();
  init_sensor();
  
  //for (ch_num = 0; ch_num< NUM_OF_CHANNELS; ch_num++)
  //setState(ch_num, Exhaust); 
  
  usleep(500000);

  unsigned long *tmp_val0;
  unsigned long tmp_val[NUM_ADC_PORT];

  gettimeofday( &end_timeval, NULL );  
  elasped_time = end_timeval.tv_sec - start_timeval.tv_sec;

  //printf("%lf\t", elasped_time);

  while( elasped_time < Sec_continue){
    gettimeofday( &end_timeval, NULL );  
    elasped_time = end_timeval.tv_sec - start_timeval.tv_sec;
    
    printf("%08.3lf: ", elasped_time);
    
    for (j = 0; j< NUM_ADC; j++){
      tmp_val0 = read_sensor(j,tmp_val);
      for (k = 0; k< NUM_ADC_PORT; k++){
	printf("%04d,", tmp_val0[k]);
      }
    }
    printf("\n");
    usleep(100000);
  }
  return 0;
}
