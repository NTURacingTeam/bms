/************************* Includes ***************************/
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LT_I2C.h"
#include "QuikEval_EEPROM.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include "mcp2515.h"

/************************* Defines *****************************/
#define DEBUG_MODE 0 //1: debugging, 0: racing                    <-------要改-----------------------------
#define DISCHARGE_MODE 0 //1 for allowing discharge
const unsigned int ALLOWED_VOLTAGE_ERROR[7]={
  0b000000000000,
  0b000000000000,
  0b000000000000,
  0b000000000011,
  0b000000000011,
  0b000000000001,
  0b000000000001
  };

 const unsigned int ALLOWED_TEMPERATURE_ERROR[7]={
  0b000000001111,
  0b000000000100,
  0b100000000000,
  0b000000010000,
  0b000000000000,
  0b101000000100,
  0b000000000001
  };

#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/**************** Local Function Declaration *******************/
void measurement_loop(uint8_t datalog_en);
void print_menu(void);
void print_wrconfig(void);
void print_rxconfig(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);
void print_sumofcells(void);
void check_mux_fail(void);
void print_selftest_errors(uint8_t adc_reg , int8_t error);
void print_overlap_results(int8_t error);
void print_digital_redundancy_errors(uint8_t adc_reg , int8_t error);
void print_open_wires(void);
void print_pec_error_count(void);
int8_t select_s_pin(void);
void print_wrpwm(void);
void print_rxpwm(void);
void print_wrsctrl(void);
void print_rxsctrl(void);
void print_wrcomm(void);
void print_rxcomm(void);
void print_conv_time(uint32_t conv_time);
void check_error(int error);
void serial_print_text(char data[]);
void serial_print_hex(uint8_t data);
char read_hex(void);
char get_char(void);
void START_I2C_COMM(bool is_L, int s_pin);
double VOLT_READING_TO_TEMP(double v_read, double _r_pull_up = 4700.0);
void LTC6811_single_cell_discharge(int Cell, uint8_t the_IC, cell_asic *ic, bool discharging); //

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 7;//!< Number of ICs in the daisy chain <----------------------------------------
const uint8_t BEGIN_IC = 1; //Start detect flaw IC. 1 is the first IC
const uint8_t END_IC = TOTAL_IC; //END detect flaw IC. 8 is last IC
const double ERROR_COMPENSATION = 0.00;
const double OVER_VOLTAGE = 4.2+ERROR_COMPENSATION; //Volt       <----------------------------------------
const double UNDER_VOLTAGE = 3.0+ERROR_COMPENSATION;//           <----------------------------------------
const double OVER_TEMPERATURE = 60.0; //Degree Celcius           <----------------------------------------
const double UNDER_TEMPERATURE = 0.0;//                          <----------------------------------------

const int MEASURE_INTERVAL = 100; //milliseconds                <----------------------------------------
const int OUTPUT_MULTIPLIER = 3;
int output_counter = 0;

const int STATUS_PIN = 32;// Pin 9 is for output stats, HIGH when error <------------------------------------

//ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED; //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
const uint8_t SEL_REG_A = REG_1; //!< Register Selection
const uint8_t SEL_REG_B = REG_2; //!< Register Selection

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = (OVER_VOLTAGE * 10000); //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = (UNDER_VOLTAGE * 10000); //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = DISABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/************************************
  END SETUP
*************************************/

/******************************************************
  Global Battery Variables received from 681x commands.
  These variables store the results from the LTC6811
  register reads and the array lengths must be based
  on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

struct cell_data {
  double cell_voltage[12];
  double temperature[12];
};

cell_data BMS_DATA[TOTAL_IC]; //data to be read

/*********************************************************
  Set the configuration bits.
  Refer to the Configuration Register Group from data sheet.
**********************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {true, true, false, false, false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV = UV_THRESHOLD; //!< Under-voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCTOBITS[4] = {true, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 0.5 min
//DCTO       0        1   2 3 4 5 6 7  8  9  A  B  C  D  E  F
//Time (min) Disabled 0.5 1 2 3 4 5 10 15 20 30 40 60 75 90 120

/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

// MCP2515
MCP2515 mcp2515(48); // set pin 48 as cs pin for mcp2515
//const int frame_num = 2 * 12 * 7 / 8; // frame: 8 bytes, data_type: uint16_t(2 bytes)
//const int frame_num = 28;
//struct can_frame can_data[frame_num];
struct can_frame can_data;
struct can_frame error_status;
// For tuning NTC resistor
const uint16_t resistor[7] = {10000, 4700, 4700, 4700, 4700, 10000, 10000};
long int t1, last = 0;

uint8_t error_code[12 * 7] = {0};
int8_t error_vol_state[12 * 7] = {0};
int8_t error_temp_state[12 * 7] = {0};

bool discharge[12 * 7] = {false};
double discharge_vol[12 * 7] = {0};
long int t2, discharge_last = 0;

// SOC
double soc[141] = {0.0};

/*!**********************************************************************
  \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  
  for (int i = 0; i < TOTAL_IC; i++) {
    memset(BMS_DATA[i].cell_voltage, 0, 12 * sizeof(double));
    memset(BMS_DATA[i].temperature, 0, 12 * sizeof(double));
  }

  //quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
  }
  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);

  run_command(1); //write config 
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);

  //mcp2515 setup
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode();

  uint16_t can_init_id = 0x600;  // 0xB00 ~ 0xB14
  //uint16_t temp_init_id = 0x615; // 0xB15 ~ 0xB29
  can_data.can_id = 0x600;
  can_data.can_dlc = 8;

  error_status.can_id = 0x601;
  error_status.can_dlc = 1;

  /*
  for (uint16_t i = 0; i < frame_num; i++) {
    can_data[i].can_id = can_init_id + i;
    can_data[i].can_dlc = 8;
    can_data[i].data[0] = (i / 4) + ((i % 4) << 4);
  }*/

  // SOC Total 5650
  double level[4] = {300.0 / 5650.0, 2400.0 / 5650.0, 2600.0 / 5650.0, 100.0 / 5650.0};
  double value;
  for (int i = 0; i < 141; i++) {
      value = (double)i / 100 + 2.8;
      if (value <= 3.3) { // 5400 ~ 5650
          soc[i] = 1 - level[3] - level[2] - level[1] - level[0] - (3.3 - value) / 0.5 * 250 / 5650;
          if (soc[i] < 0) {
              soc[i] = 0.0;
          }
      } else if ((value > 3.3) && (value <= 3.4)) { // 5100 ~ 5400
          soc[i] = 1 - level[3] 
          - level[2] - level[1] - (3.4 - value) / 0.1 * 300 / 5650;
      } else if ((value > 3.4) && (value <= 3.65)) { // 2700 ~ 5100
          soc[i] = 1 - level[3] - level[2] - (3.65 - value) / 0.25 * 2400 / 5650;
      } else if ((value > 3.65) && (value <= 4.1)) { // 100 ~ 2700
          soc[i] = 1 - level[3] - (4.1 - value) / 0.45 * 2600 / 5650;
      } else { // 100
          soc[i] =  1 - (4.2 - value) / 0.1 * 100 / 5650;
      }
  }

#if DEBUG_MODE
  print_menu();
#endif
}

/*!*********************************************************************
  \brief Main loop
  @return void
***********************************************************************/
void loop()
{
  if (DEBUG_MODE) {
    if (Serial.available())           // Check for user input
    {
      uint32_t user_command;
      user_command = read_int();      // Read the user command
      if (user_command == 'm'){
        print_menu();
      }else{
        Serial.println(user_command);
        run_command(user_command);
      }
    }
  } else { //racing function
    run_command(3); //LTC6811 measure cell volt
    run_command(4); //read cell volt from LTC6811
    
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {  //loop in segments
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        BMS_DATA[current_ic].cell_voltage[i] = (BMS_IC[current_ic].cells.c_codes[i] * 0.0001); //get all voltage
        //can_data[current_ic * 4 + i / 3].data[(i % 3) * 2 + 1] = (uint8_t)((BMS_DATA[current_ic].cell_voltage[i] - 2) / 0.01); // 2 ~ 4.55, 0.01/tick
        //canVol[(current_ic * TOTAL_IC + i) / 8].data[(current_ic * TOTAL_IC + i) % 8] = (uint8_t)((BMS_DATA[current_ic].cell_voltage[i] - 1.56) / 0.01) // 1.56 ~ 4.2, 0.01/tick
      }
    }
    for(int i=0; i<6; i++){
      //GPIOBITS_A = {true, true, ((i&0b001)>>0), ((i&0b010)>>1), ((i&0b100)>>2)}; //set channel of mux
      GPIOBITS_A[2] = (i&0b001)>>0;
      GPIOBITS_A[3] = (i&0b010)>>1;
      GPIOBITS_A[4] = (i&0b100)>>2;
      run_command(31); //reset gpio status
      run_command(5); //start aux measuing
      run_command(6); //read back aux measuing
      for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
        BMS_DATA[current_ic].temperature[i] = VOLT_READING_TO_TEMP(BMS_IC[current_ic].aux.a_codes[0] * 0.0001, resistor[current_ic]); //GPIO 1 Left
        BMS_DATA[current_ic].temperature[i+6] = VOLT_READING_TO_TEMP(BMS_IC[current_ic].aux.a_codes[1] * 0.0001, resistor[current_ic]); //GPIO 2 Right

        //can_data[current_ic * 4 + i / 3].data[((i % 3) + 1) * 2] = (uint8_t)((BMS_DATA[current_ic].temperature[i] + 20) / 0.3125);
        //can_data[current_ic * 4 + (i + 6) / 3].data[(((i + 6) % 3) + 1) * 2] = (uint8_t)((BMS_DATA[current_ic].temperature[i + 6] + 20) / 0.3125); //-20 ~ 60, 0.3125/tick
        //canTemp[(current_ic * TOTAL_IC + i) / 8].data[(current_ic * TOTAL_IC + i) % 8] = (uint8_t)((BMS_DATA[current_ic].temperature[i] + 20) / 0.3125); //-20 ~ 60, 0.3125/tick
        //canTemp[(current_ic * TOTAL_IC + i + 6) / 8].data[(current_ic * TOTAL_IC + i + 6) % 8] = (uint8_t)((BMS_DATA[current_ic].temperature[i + 6] + 20) / 0.3125); //-20 ~ 60, 0.3125/tick
      }
    }

    //temperature and voltage measured and saved, now print results
    Serial.println("$STAT$");
    t1 = millis();
    if (t1 - last > 500) {
      for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
        Serial.print(current_ic + 1);
        for (int i = 0; i < 12; i++) {
          Serial.print("$");
          if (discharge[current_ic * 12 + i] == 0) {
            Serial.print(BMS_DATA[current_ic].cell_voltage[i]);
          } else {
            Serial.print(discharge_vol[current_ic * 12 + i]);
          }
        } 
        for (int i = 0; i < 12; i++) {
          Serial.print("$");
          Serial.print(BMS_DATA[current_ic].temperature[i]);
        }
        Serial.println("");
      }
      Serial.println("$ENDSTAT$");
      last = t1;
    }

    //error detection
    bool error_flag = false;
    bool error_vol_h = false; //false for voltage, true for temperature
    bool error_vol_l = false; //false for under, true for over
    bool error_temp_h = false;
    bool error_temp_l = false;
    //uint8_t error_ic; //starts with 1
    //uint8_t error_cell; //starts with 1
    for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
      for (int i = 0; i < 12; i++) {
        if ((UNDER_VOLTAGE > BMS_DATA[current_ic].cell_voltage[i])&& !((ALLOWED_VOLTAGE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true;
          error_code[current_ic * 7 + i] += 1;
          error_vol_state[current_ic * 7 + i] = -1;
          error_vol_l = true;
        } else if ((BMS_DATA[current_ic].cell_voltage[i] > OVER_VOLTAGE)&& !((ALLOWED_VOLTAGE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true;
          error_code[current_ic * 7 + i] += 1;
          error_vol_state[current_ic * 7 + i] = 1;
          error_vol_h = true;
        }

        if ((UNDER_TEMPERATURE > BMS_DATA[current_ic].temperature[i]) && !((ALLOWED_TEMPERATURE_ERROR[current_ic]>>(11-i))&0b1) && (BMS_DATA[current_ic].temperature[i] > 0)) {
          error_flag = true;
          error_code[current_ic * 7 + i] += 2;
          error_temp_state[current_ic * 7 + i] = -1;
          error_temp_h = true;
        } else if ((BMS_DATA[current_ic].temperature[i] > OVER_TEMPERATURE)&& !((ALLOWED_TEMPERATURE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true;
          error_code[current_ic * 7 + i] += 2;
          error_temp_state[current_ic * 7 + i] = 1;
          error_temp_l = true;
        }
      }
    }
    /*
    for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
      for (int i = 0; i < 12; i++) {
        if (error_flag) {
          break;
        } else if ((UNDER_VOLTAGE > BMS_DATA[current_ic].cell_voltage[i])&& !((ALLOWED_VOLTAGE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true; //has error
          error_type = false; //volt error
          error_bound = false; //under
          error_ic = current_ic + 1;
          error_cell = i + 1;
        } else if ((BMS_DATA[current_ic].cell_voltage[i] > OVER_VOLTAGE)&& !((ALLOWED_VOLTAGE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true; //has error
          error_type = false; //volt error
          error_bound = true; //over
          error_ic = current_ic + 1;
          error_cell = i + 1;
        }
      }
      for (int i = 0; i < 12; i++) {
        if (error_flag) {
          break;
        } else if ((UNDER_TEMPERATURE > BMS_DATA[current_ic].temperature[i]) && !((ALLOWED_TEMPERATURE_ERROR[current_ic]>>(11-i))&0b1) && (BMS_DATA[current_ic].temperature[i] > 0)) {
          error_flag = true; //has error
          error_type = true; //temperature error
          error_bound = false; //under
          error_ic = current_ic + 1;
          error_cell = i + 1;
        } else if ((BMS_DATA[current_ic].temperature[i] > OVER_TEMPERATURE)&& !((ALLOWED_TEMPERATURE_ERROR[current_ic]>>(11-i))&0b1)) {
          error_flag = true; //has error
          error_type = true; //temperature error
          error_bound = true; //over
          error_ic = current_ic + 1;
          error_cell = i + 1;
        }
      }
    }
    */

    Serial.println("$ERROR$");
    if (error_flag) {
      Serial.println("Error found");
      for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
        for (int i = 0; i < 12; i++) {
          if (error_vol_state[current_ic * 7 + i] != 0) {
            Serial.print("Voltage error at Seg");
            Serial.print(current_ic + 1);
            Serial.print(" Cell ");
            Serial.println(i + 1);
            Serial.print("Type: ");
            if (error_vol_state[current_ic * 7 + i] < 0) {
              Serial.print("Under");
            } else {
              Serial.print("Over");
            }
          }

          if (error_temp_state[current_ic * 7 + i] != 0) {
            Serial.print("Temperature error at Seg");
            Serial.print(current_ic + 1);
            Serial.print(" Cell ");
            Serial.println(i + 1);
            Serial.print("Type: ");
            if (error_temp_state[current_ic * 7 + i] < 0) {
              Serial.print("Under");
            } else {
              Serial.print("Over");
            }
          }
        }
      }
      /*
      if (!error_bound) {
        Serial.print("Under"); //false for under
      } else {
        Serial.print("Over"); //true for over
      }
      if (!error_type) { //volt error
        Serial.print("Voltage error at IC "); //false for volt
      } else { //vtemperature error
        Serial.print("Temperature error at IC "); //true for temp
      }
      Serial.print(error_ic);
      Serial.print(" Cell ");
      Serial.println(error_cell);
      */
      digitalWrite(STATUS_PIN, LOW);
    } else {
      Serial.println("No error found");
      digitalWrite(STATUS_PIN, HIGH);
    }
    Serial.println("$ENDERROR$");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    digitalWrite(53, HIGH);
    digitalWrite(48, LOW);

    /*
    cantmp.data[0] = 0xff;
    cantmp.data[1] = 0xff;
    cantmp.data[2] = 0xff;
    mcp2515.sendMessage(&cantmp);
    */

    double tmp;
    for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
      for (int i = 0; i < 12; i++) {
        //can_data[current_ic * 4 + i / 3].data[(i % 3) * 2 + 1] = (uint8_t)((BMS_DATA[current_ic].cell_voltage[i] - 2) / 0.01); // 2 ~ 4.55, 0.01/tick
        //can_data[current_ic * 4 + i / 3].data[((i % 3) + 1) * 2] = (uint8_t)((BMS_DATA[current_ic].temperature[i] + 20) / 0.3125)

        tmp = (BMS_DATA[current_ic].cell_voltage[i] - 2) / 0.01;
        can_data.data[(i % 3) * 2 + 1] = (uint8_t)tmp; // 2 ~ 4.55, 0.01/tick

        if (BMS_DATA[current_ic].temperature[i] < -20) {
          tmp = 0;
        } else {
          tmp = ((BMS_DATA[current_ic].temperature[i] + 20) / 0.3125);
        }
        can_data.data[((i % 3) + 1) * 2] = (uint8_t)tmp;

        if (i % 3 == 2) {
          can_data.data[0] = current_ic + ((i / 3) << 4);
          can_data.data[7] = 0;
          for (int cell = 0; cell < 3; cell++) {
            can_data.data[7] = (error_code[current_ic * 7 + i] & 0b11) << cell * 2; 
          }
          mcp2515.sendMessage(&can_data);
          delay(5);
        }
      }
    }

    if (error_vol_h) {
      error_status.data[0] = 1 << 0;
    }
    if (error_vol_l) {
      error_status.data[0] = 1 << 1;
    }
    if (error_temp_h) {
      error_status.data[0] = 1 << 2;
    }
    if (error_temp_l) {
      error_status.data[0] = 1 << 3;
    }
    mcp2515.sendMessage(&error_status);
    /*
    for (int i = 0; i < frame_num; i++) {
      mcp2515.sendMessage(&can_data[i]);
      delay(1);
      //mcp2515.sendMessage(&canTemp[i]);
    }
    */
    

    digitalWrite(48, HIGH);
    digitalWrite(53, LOW);
    spi_enable(SPI_CLOCK_DIV16);

    if(DISCHARGE_MODE){
      double highest_voltage = 0.0;
      double lowest_voltage = 1000.0;

      /* Move to 265*/
      for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
        for (int i = 0; i < 12; i++) { //scan every cell
          if(!((ALLOWED_VOLTAGE_ERROR[current_ic]>>(11-i))&0b1)){ //if the correct voltage is needed
            if (BMS_DATA[current_ic].cell_voltage[i] > highest_voltage){
              highest_voltage = BMS_DATA[current_ic].cell_voltage[i];
            } else if(BMS_DATA[current_ic].cell_voltage[i] < lowest_voltage && discharge[current_ic * 12 + i] == 0){
              lowest_voltage = BMS_DATA[current_ic].cell_voltage[i];
            }
          } 
        }
      }

      double diff = 0.0;
      for (int current_ic = (BEGIN_IC - 1) ; current_ic < END_IC; current_ic++) {
        for (int i = 0; i < 12; i++) { //scan every cell
          diff = BMS_DATA[current_ic].cell_voltage[i] - lowest_voltage;
          if (diff > 0.03 && BMS_DATA[current_ic].cell_voltage[i] > 4.1 && discharge[current_ic * 12 + i] == 0) {
            LTC6811_single_cell_discharge(i+1, current_ic, BMS_IC, 1); //discharge it
            discharge[current_ic * 12 + i] = 1;
            discharge_vol[current_ic * 12 + i] = BMS_DATA[current_ic].cell_voltage[i];
          } 
          /*else {
            LTC6811_single_cell_discharge(i+1, current_ic, BMS_IC, 0); // stop dischargin it
          }
          */
          /*
          if (BMS_DATA[current_ic].cell_voltage[i] > lowest_voltage){
            LTC6811_single_cell_discharge(i+1, current_ic, BMS_IC, 1); //discharge it
          }else{
            LTC6811_single_cell_discharge(i+1, current_ic, BMS_IC, 0); // stop dischargin it
          }
          */
        }
      }
      discharge_last = millis();
      if(highest_voltage == lowest_voltage){
        Serial.println("Discharge Complete");
      }
    }else{
      
    }

    t2 = millis();
    if (t2 - discharge_last > 300) {
      turn_off_discharge();
    }

    //delay(MEASURE_INTERVAL);
  } //end racing function
}

void turn_off_discharge()
{
  for (int i = 0; i < 7 * 12; i++) {
    if (discharge[i] == 1) {
      discharge[i] = 0;
      LTC6811_single_cell_discharge((i % 12) + 1, i / 12, BMS_IC, 0);
    }
  }
}

/*!*****************************************
  \brief Executes the user command
  @return void
*******************************************/
void run_command(uint32_t cmd)
{
  uint8_t streg = 0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read = 0;

  switch (cmd)
  {
    case 1: // Write and Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC); // Write into Configuration Register
#if DEBUG_MODE
      print_wrconfig();
#endif
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC); // Read Configuration Register
#if DEBUG_MODE
      check_error(error);
      print_rxconfig();
#endif
      break;

    case 2: // Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);

      check_error(error);
      print_rxconfig();
      break;

    case 3: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
#if DEBUG_MODE
      print_conv_time(conv_time);
#endif
      break;

    case 4: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
#if DEBUG_MODE
      check_error(error);
      print_cells(DATALOG_DISABLED);
#endif
      break;

    case 5: // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
#if DEBUG_MODE
      print_conv_time(conv_time);
#endif
      break;

    case 6: // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
#if DEBUG_MODE
      check_error(error);
      print_aux(DATALOG_DISABLED);
#endif
      break;

    case 7: // Start Status ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      break;

    case 8: // Read Status registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all stat registers
      check_error(error);
      print_stat();
      break;

    case 9:// Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcvax(ADC_CONVERSION_MODE, ADC_DCP);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdaux(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read back aux registers A
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;

    case 10: //Start Combined Cell Voltage and Sum of cells
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdstat(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read stat registers A
      check_error(error);
      print_sumofcells();
      break;

    case 11: // Loop Measurements of configuration register or cell voltages or auxiliary register or status register without data-log output
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      measurement_loop(DATALOG_DISABLED);
      print_menu();
      break;

    case 12: //Data-log print option Loop Measurements of configuration register or cell voltages or auxiliary register or status register
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      measurement_loop(DATALOG_ENABLED);
      print_menu();
      break;

    case 13: // Clear all ADC measurement registers
      wakeup_sleep(TOTAL_IC);
      LTC6811_clrcell();
      LTC6811_clraux();
      LTC6811_clrstat();
      wakeup_idle(TOTAL_IC);
      LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all cell voltage registers
      print_cells(DATALOG_DISABLED);

      LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all aux registers
      print_aux(DATALOG_DISABLED);

      LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all stat
      print_stat();
      break;

    case 14: //Read CV,AUX and ADSTAT Voltages
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);

      wakeup_sleep(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_aux(DATALOG_DISABLED);

      wakeup_sleep(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all status registers
      check_error(error);
      print_stat();
      break;

    case 15: // Run the Mux Decoder Self Test
      wakeup_sleep(TOTAL_IC);
      LTC6811_diagn();
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      error = LTC6811_rdstat(SEL_REG_B, TOTAL_IC, BMS_IC); // Set to read back status register B
      check_error(error);
      check_mux_fail();
      break;

    case 16:  // Run the ADC/Memory Self Test
      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(CELL, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
      print_selftest_errors(CELL, error);

      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(AUX, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
      print_selftest_errors(AUX, error);

      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(STAT, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
      print_selftest_errors(STAT, error);
      print_menu();
      break;

    case 17: // Run ADC Overlap self test
      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = (int8_t)LTC6811_run_adc_overlap(TOTAL_IC, BMS_IC);
      print_overlap_results(error);
      break;

    case 18: // Run ADC Digital Redundancy self test
      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, AUX, TOTAL_IC, BMS_IC);
      print_digital_redundancy_errors(AUX, error);

      error = 0;
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, STAT, TOTAL_IC, BMS_IC);
      print_digital_redundancy_errors(STAT, error);
      break;

    case 19: // Open Wire test for single cell detection
      wakeup_sleep(TOTAL_IC);
      LTC6811_run_openwire_single(TOTAL_IC, BMS_IC);
      print_open_wires();
      break;

    case 20: // Open Wire test for multiple cell and two consecutive cells detection
      wakeup_sleep(TOTAL_IC);
      LTC6811_run_openwire_multi(TOTAL_IC, BMS_IC);
      break;

    case 21:// PEC Errors Detected
      print_pec_error_count();
      break;

    case 22: // Reset PEC Counter
      LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
      print_pec_error_count();
      break;

    case 23: // Enable a discharge transistor
      s_pin_read = select_s_pin();

      wakeup_sleep(TOTAL_IC);
      LTC6811_set_discharge(s_pin_read, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      print_wrconfig();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      break;

    case 24: // Clear all discharge transistors
      wakeup_sleep(TOTAL_IC);
      LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      print_wrconfig();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      break;

    case 25:// Write read pwm configuration
      /*****************************************************
         PWM configuration data.
         1)Set the corresponding DCC bit (discharge cell switch DCCBITS[12]) to one for pwm operation.
         2)Set the DCTO bits to the required discharge time.
         3)Choose the value to be configured depending on the
          required duty cycle.
         Refer to the data sheet.
      *******************************************************/
      wakeup_sleep(TOTAL_IC);
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        BMS_IC[current_ic].pwm.tx_data[0] = 0x88; // Duty cycle for S pin 2 and 1
        BMS_IC[current_ic].pwm.tx_data[1] = 0x88; // Duty cycle for S pin 4 and 3
        BMS_IC[current_ic].pwm.tx_data[2] = 0x88; // Duty cycle for S pin 6 and 5
        BMS_IC[current_ic].pwm.tx_data[3] = 0x88; // Duty cycle for S pin 8 and 7
        BMS_IC[current_ic].pwm.tx_data[4] = 0x88; // Duty cycle for S pin 10 and 9
        BMS_IC[current_ic].pwm.tx_data[5] = 0x88; // Duty cycle for S pin 12 and 11
      }
      LTC6811_wrpwm(TOTAL_IC, 0, BMS_IC);
      print_wrpwm();

      wakeup_idle(TOTAL_IC);
      LTC6811_rdpwm(TOTAL_IC, 0, BMS_IC);
      print_rxpwm();
      break;

    case 26: // Write and read S Control Register Group
      wakeup_sleep(TOTAL_IC);
      /**************************************************************************************
         S pin control.
         1)Ensure that the pwm is set according to the requirement using the previous case.
         2)Choose the value depending on the required number of pulses on S pin.
         Refer to the data sheet.
      ***************************************************************************************/
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        BMS_IC[current_ic].sctrl.tx_data[0] = 0x77; // No. of high pulses on S pin 2 and 1
        BMS_IC[current_ic].sctrl.tx_data[1] = 0x77; // No. of high pulses on S pin 4 and 3
        BMS_IC[current_ic].sctrl.tx_data[2] = 0x77; // No. of high pulses on S pin 6 and 5
        BMS_IC[current_ic].sctrl.tx_data[3] = 0x77; // No. of high pulses on S pin 8 and 7
        BMS_IC[current_ic].sctrl.tx_data[4] = 0x77; // No. of high pulses on S pin 10 and 9
        BMS_IC[current_ic].sctrl.tx_data[5] = 0x77; // No. of high pulses on S pin 12 and 11
      }
      LTC6811_wrsctrl(TOTAL_IC, streg, BMS_IC);
      print_wrsctrl();

      // Start S Control pulsing
      wakeup_idle(TOTAL_IC);
      LTC6811_stsctrl();

      // Read S Control Register Group
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdsctrl(TOTAL_IC, streg, BMS_IC);
      check_error(error);
      print_rxsctrl();
      break;

    case 27: // Clear S Control Register Group
      wakeup_sleep(TOTAL_IC);
      LTC6811_clrsctrl();

      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdsctrl(TOTAL_IC, streg, BMS_IC); // Read S Control Register Group
      check_error(error);
      print_rxsctrl();
      break;

    case 28://SPI Communication
      /*************************************************************
         Ensure to set the GPIO bits to 1 in the CFG register group.
      *************************************************************/
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        //Communication control bits and communication data bytes. Refer to the data sheet.
        BMS_IC[current_ic].com.tx_data[0] = 0x81; // Icom CSBM Low(8) + data D0 (0x11)
        BMS_IC[current_ic].com.tx_data[1] = 0x10; // Fcom CSBM Low(0)
        BMS_IC[current_ic].com.tx_data[2] = 0xA2; // Icom CSBM Falling Edge (A) +  D1 (0x25)
        BMS_IC[current_ic].com.tx_data[3] = 0x50; // Fcom CSBM Low(0)
        BMS_IC[current_ic].com.tx_data[4] = 0xA1; // Icom CSBM Falling Edge (A) +  D2 (0x17)
        BMS_IC[current_ic].com.tx_data[5] = 0x79; // Fcom CSBM High(9)
      }
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
      print_wrcomm(); // print data in the comm register

      wakeup_idle(TOTAL_IC);
      LTC6811_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
      check_error(error);
      print_rxcomm();  // print received data into the comm register
      break;

    case 29: // write byte I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
      /************************************************************
        Ensure to set the GPIO bits to 1 in the CFG register group.
      *************************************************************/
      //Adress=00的ADG728
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        //Communication control bits and communication data bytes. Refer to the data sheet.
        //pull-up resistor to 3V = 4.7k ohm

        BMS_IC[current_ic].com.tx_data[0] = 0x69; // Icom Start(6) + I2C_address D0 (98) addr=00
        //BMS_IC[current_ic].com.tx_data[0] = 0x6E; // Icom Start(6) + I2C_address D0 (9E) addr=11
        BMS_IC[current_ic].com.tx_data[1] = 0x88; // Fcom master NACK(8)
        BMS_IC[current_ic].com.tx_data[2] = 0x00; // Icom Blank (0) + D1 (0000 0010) <- S8~S1
        BMS_IC[current_ic].com.tx_data[3] = 0x29; // Fcom master NACK + Stop(9)
        BMS_IC[current_ic].com.tx_data[4] = 0x7F; // Icom No Transmit (7) + data D2 (FF)
        BMS_IC[current_ic].com.tx_data[5] = 0xF9; // Fcom master NACK + Stop(9)

      }
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
      print_wrcomm(); // print transmitted data from the comm register

      wakeup_idle(TOTAL_IC);
      LTC6811_stcomm(3); // data length=2 // initiates communication between master and the I2C slave

      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
      check_error(error);
      print_rxcomm(); // print received data into the comm register

      break;

    case 30: // Read byte data I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
      /************************************************************
         Ensure to set the GPIO bits to 1 in the CFG register group.
      *************************************************************/

      wakeup_sleep(TOTAL_IC);          for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        //Communication control bits and communication data bytes. Refer to the data sheet.

        BMS_IC[current_ic].com.tx_data[0] = 0x69; // Icom Start (6) + I2C_address D0 (99)
        //BMS_IC[current_ic].com.tx_data[0] = 0x6F; // Icom Start (6) + I2C_address D0 (9F)
        BMS_IC[current_ic].com.tx_data[1] = 0x98; // Fcom master NACK(8)
        BMS_IC[current_ic].com.tx_data[2] = 0x01; // Icom Blank (0) + D1 (11) <-random read byte
        BMS_IC[current_ic].com.tx_data[3] = 0x19; // Fcom master NACK + Stop(9)
        BMS_IC[current_ic].com.tx_data[4] = 0x7F; // Icom No Transmit (7) + data D2 (FF)
        BMS_IC[current_ic].com.tx_data[5] = 0xF9; // Fcom master NACK + Stop(9)

      }
      LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register

      wakeup_idle(TOTAL_IC);
      LTC6811_stcomm(3); // data length=2// initiates communication between master and the I2C slave

      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
      check_error(error);
      print_rxcomm(); // print received data from the comm register
      break;

    case 31: // Set or reset the gpio pins(to drive output on gpio pins)
      /***********************************************************************
        Please ensure you have set the GPIO bits according to your requirement
        in the configuration register.( check the global variable GPIOBITS_A )
      ************************************************************************/
      wakeup_sleep(TOTAL_IC);
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
      {
        LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
      }
      wakeup_idle(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
#if DEBUG_MODE
      print_wrconfig();
#endif
      break;

    case 'd': //start discharging
      
      break;

    case 'm': //prints menu
      print_menu();
      break;

    default:
      char str_error[] = "Incorrect Option \n";
      serial_print_text(str_error);
      break;
  }
}

/*!**********************************************************************************************************************************************
  \brief For writing/reading configuration data or measuring cell voltages or reading aux register or reading status register in a continuous loop
  @return void
*************************************************************************************************************************************************/
void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;

  Serial.println(F("Transmit 'm' to quit"));

  while (input != 'm')
  {
    if (Serial.available() > 0)
    {
      input = read_char();
    }
    if (WRITE_CONFIG == ENABLED)
    {
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      print_wrconfig();
    }

    if (READ_CONFIG == ENABLED)
    {
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
    }

    if (MEASURE_CELL == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
      check_error(error);
      print_cells(datalog_en);
    }

    if (MEASURE_AUX == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_aux(datalog_en);
    }

    if (MEASURE_STAT == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_stat();
    }

    if (PRINT_PEC == ENABLED)
    {
      print_pec_error_count();
    }

    delay(MEASUREMENT_LOOP_TIME);
  }
}

/*!*********************************
  \brief Prints the main menu
  @return void
***********************************/
void print_menu(void)
{
  Serial.println(F("List of 6811 Commands: "));
  Serial.println(F("Write and Read Configuration: 1                            |Loop measurements with data-log output: 12     |Set Discharge: 23"));
  Serial.println(F("Read Configuration: 2                                      |Clear Registers: 13                            |Clear Discharge: 24"));
  Serial.println(F("Start Cell Voltage Conversion: 3                           |Read CV,AUX and ADSTAT Voltages: 14            |Write and Read of PWM: 25"));
  Serial.println(F("Read Cell Voltages: 4                                      |Run Mux Self Test: 15                          |Write and Read of S control: 26"));
  Serial.println(F("Start Aux Voltage Conversion: 5                            |Run ADC Self Test: 16                          |Clear S control register: 27"));
  Serial.println(F("Read Aux Voltages: 6                                       |ADC overlap Test : 17                          |SPI Communication: 28"));
  Serial.println(F("Start Stat Voltage Conversion: 7                           |Run Digital Redundancy Test: 18                |I2C Communication Write to Slave: 29"));
  Serial.println(F("Read Stat Voltages: 8                                      |Open Wire Test for single cell detection: 19   |I2C Communication Read from Slave:30"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 9 |Open Wire Test for multiple cell detection: 20 |Set or Reset the GPIO pins: 31 "));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 10                  |Print PEC Counter: 21                          |Call MUX+ADC: 32"));
  Serial.println(F("Loop Measurements: 11                                      |Reset PEC Counter: 22                          | \n "));

  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: \n"));
}

/*!******************************************************************************
  \brief Prints the configuration data that is going to be written to the LTC6811
  to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void)
{
  int cfg_pec;

  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
  \brief Prints the configuration data that was read back from the
  LTC6811 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfig(void)
{
  Serial.println(F("Received Configuration "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints cell voltage to the serial port
   @return void
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(": ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print(" Cells :");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(":");

      for (int i = 0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].aux.a_codes[5] * 0.0001, 4);
      Serial.println();
    }
    else
    {
      Serial.print("AUX ");
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(": ");

      for (int i = 0; i < 6; i++)
      {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_stat(void)
{
  double itmp;
  for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 20, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0075)) - 273);   //Internal Die Temperature(°C) = itmp • (100 µV / 7.5mV)°C - 273°C
    Serial.print(itmp, 4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3] * 0.0001, 4);
    Serial.println();
    Serial.print(F(" Flags:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
    Serial.print(F("   Mux fail flag:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
    Serial.print(F("   THSD:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
  @return void
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 20, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

/*!****************************************************************
  \brief Function to check the MUX fail bit in the Status Register
   @return void
*******************************************************************/
void check_mux_fail(void)
{
  int8_t error = 0;
  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    Serial.print(" IC ");
    Serial.println(ic + 1, DEC);
    if (BMS_IC[ic].stat.mux_fail[0] != 0) error++;

    if (error == 0) Serial.println(F("Mux Test: PASS \n"));
    else Serial.println(F("Mux Test: FAIL \n"));
  }
}

/*!************************************************************
  \brief Prints Errors Detected during self test
   @return void
*************************************************************/
void print_selftest_errors(uint8_t adc_reg , int8_t error)
{
  if (adc_reg == 1)
  {
    Serial.println("Cell ");
  }
  else if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }
  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Digital Filter and Memory \n"));
}

/*!************************************************************
  \brief Prints the output of  the ADC overlap test
   @return void
*************************************************************/
void print_overlap_results(int8_t error)
{
  if (error == 0) Serial.println(F("Overlap Test: PASS \n"));
  else Serial.println(F("Overlap Test: FAIL \n"));
}

/*!************************************************************
  \brief Prints Errors Detected during Digital Redundancy test
   @return void
*************************************************************/
void print_digital_redundancy_errors(uint8_t adc_reg , int8_t error)
{
  if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }

  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Measurement \n"));
}

/*****************************************************************************
  \brief Prints Open wire test results to the serial port
  @return void
 *****************************************************************************/
void print_open_wires(void)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (BMS_IC[current_ic].system_open_wire == 65535)
    {
      Serial.print("No Opens Detected on IC ");
      Serial.println(current_ic + 1, DEC);
    }
    else
    {
      Serial.print(F("There is an open wire on IC "));
      Serial.print(current_ic + 1, DEC);
      Serial.print(F(" Channel: "));
      Serial.println(BMS_IC[current_ic].system_open_wire);
    }
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints the PEC errors detected to the serial port
  @return void
 *************************************************************/
void print_pec_error_count(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.println("");
    Serial.print(BMS_IC[current_ic].crc_count.pec_count, DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic + 1, DEC);
  }
  Serial.println("\n");
}

/*!****************************************************
  \brief Function to select the S pin for discharge
  @return void
 ******************************************************/
int8_t select_s_pin(void)
{
  int8_t read_s_pin = 0;

  Serial.print(F("Please enter the Spin number: "));
  read_s_pin = (int8_t)read_int();
  Serial.println(read_s_pin);
  return (read_s_pin);
}

/*!******************************************************************************
  \brief Prints  PWM the configuration data that is going to be written to the LTC6811
  to the serial port.
  @return void
 ********************************************************************************/
void print_wrpwm(void)
{
  int pwm_pec;

  Serial.println(F("Written PWM Configuration: "));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6, &BMS_IC[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
  \brief Prints the PWM configuration data that was read back from the
  LTC6811 to the serial port.
  @return void
 *******************************************************************/
void print_rxpwm(void)
{
  Serial.println(F("Received pwm Configuration:"));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints S control register data to the serial port
  @return void
 *************************************************************/
void print_wrsctrl(void)
{
  int sctrl_pec;

  Serial.println(F("Written Data in Sctrl register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC: "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" Sctrl register group:"));
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6, &BMS_IC[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints s control register data that was read back from the
  LTC6811 to the serial port.
  @return void
 *************************************************************/
void print_rxsctrl(void)
{
  Serial.println(F("Received Data:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[i]);
    }

    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints comm register data to the serial port
  @return void
 *************************************************************/
void print_wrcomm(void)
{
  int comm_pec;

  Serial.println(F("Written Data in COMM Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6, &BMS_IC[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints comm register data that was read back from the
  LTC6811 to the serial port.
  @return void
 *************************************************************/
void print_rxcomm(void)
{
  Serial.println(F("Received Data in COMM register:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Function to print the Conversion Time
  @return void
 *****************************************************************************/
void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor = 1000; // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}

/*!************************************************************
  \brief Function to check error flag and print PEC error message
  @return void
 *************************************************************/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!************************************************************
  \brief Function to print text on serial monitor
  @return void
*************************************************************/
void serial_print_text(char data[])
{
  Serial.println(data);
}

/*!************************************************************
  \brief Function to print in HEX form
  @return void
 *************************************************************/
void serial_print_hex(uint8_t data)
{
  if (data < 16)
  {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}

/*!************************************************************
  \brief Hex conversion constants
 *************************************************************/
char hex_digits[16] =
{
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

/*!************************************************************
  \brief Global Variables
 *************************************************************/
char hex_to_byte_buffer[5] =
{
  '0', 'x', '0', '0', '\0'
};

/*!************************************************************
  \brief Buffer for ASCII hex to byte conversion
 *************************************************************/
char byte_to_hex_buffer[3] =
{
  '\0', '\0', '\0'
};

/*!************************************************************
  \brief Read 2 hex characters from the serial buffer and convert them to a byte
  @return char data Read Data
 *************************************************************/
char read_hex(void) {
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}

/*!************************************************************
  \brief Read a command from the serial port
  @return char
 *************************************************************/
char get_char(void) {
  while (Serial.available() <= 0);
  return (Serial.read());
}


double VOLT_READING_TO_TEMP(double v_read, double _r_pull_up) {
  double v_ref = 3.0; //Pulled up to 3Volt
  double r_pull_up = _r_pull_up; //Pulled up by a 4.7kOhm resistor
  double r_ntc = v_read * r_pull_up / (v_ref - v_read);
  double r_25 = 10000.0; //10kOhm at 25 degree Celcius
  double b_const = 3380.0; //B-Constant (25/50℃): 3380k
  double t = 1 / (log((r_ntc / r_25 > 0 ? r_ntc / r_25 : 0)) / b_const + 1 / 298.15) - 273.15;

  return t; //return temperature in celcius
}

void LTC6811_single_cell_discharge(int Cell, //The cell to be discharged
               uint8_t the_IC, //The specific IC in the system
               cell_asic *ic, //A two dimensional array that will store the data
               bool discharging //0 for not dischargin, 1 for discharging
               )
{
  //remember, the_IC starts with 0 while Cell starts with 1
  int8_t error = 0;
  wakeup_sleep(TOTAL_IC);
  if(discharging){
    if ((Cell<9)&& (Cell!=0) ){
      ic[the_IC].config.tx_data[4] |= (1<<(Cell-1));
    }else if (Cell < 13){
      ic[the_IC].config.tx_data[5] |= (1<<(Cell-9));
    }
  }else{
    if ((Cell<9)&& (Cell!=0) ){
      ic[the_IC].config.tx_data[4] &= (~(1<<(Cell-1)));
    }else if (Cell < 13){
      ic[the_IC].config.tx_data[5] &= (~(1<<(Cell-9)));
    }
  }
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
#if DEBUG_MODE
  print_wrconfig();
#endif
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
#if DEBUG_MODE
  check_error(error);
  print_rxconfig(); 
#endif
}
