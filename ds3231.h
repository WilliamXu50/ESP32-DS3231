#ifndef _DS3231_H_
#define _DS3231_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driver/i2c.h"

//DS3231 slave address for I2C communication
#define DS3231_SLAVE_ADDR           0x68

// I2C pin configurations
#define I2C_MASTER_SCK      19
#define I2C_MASTER_SDA      18
#define DS3231_INTR_PIN     21

//DS3231 I2C clock frequencies
#define DS3231_MIN_FREQ     100000
#define DS3231_MAX_FREQ     400000

/*  Register addresses for DS3231 time-keeping registers, 
 *  alarm registers and control & status registers
 */
#define DS3231_SECONDS_REG_ADDR     0x00
#define DS3231_MINUTES_REG_ADDR     0x01
#define DS3231_HOURS_REG_ADDR       0x02
#define DS3231_DAY_REG_ADDR         0x03
#define DS3231_DATE_REG_ADDR        0x04
#define DS3231_MONTH_REG_ADDR       0x05
#define DS3231_YEAR_REG_ADDR        0x06

#define DS3231_ALARM_1_SECONDS_REG_ADDR     0x07
#define DS3231_ALARM_1_MINUTES_REG_ADDR     0x08
#define DS3231_ALARM_1_HOURS_REG_ADDR       0x09
#define DS3231_ALARM_1_DAY_DATE_REG_ADDR    0x0A

#define DS3231_ALARM_2_MINUTES_REG_ADDR     0x0B
#define DS3231_ALARM_2_HOURS_REG_ADDR       0x0C
#define DS3231_ALARM_2_DAY_DATE_REG_ADDR    0x0D

#define DS3231_CONTROL_REG_ADDR     0x0E
#define DS3231_STATUS_REG_ADDR      0x0F

#define RECEIVED_ACK        0x00
#define RECEIVED_NOT_ACK    0x01

//Control register bit masks
#define DS3231_EN_OSC        7
#define DS3231_BBSQW         6
#define DS3231_CONV_TEMP     5
#define DS3231_RS2           4
#define DS3231_RS1           3
#define DS3231_INTCN         2
#define DS3231_A2IE          1
#define DS3231_A1IE          0

//Status register bit masks
#define DS3231_STATUS_OSF          7
#define DS3231_STATUS_EN_32KHZ     3
#define DS3231_STATUS_BSY          2
#define DS3231_STATUS_AF1          0
#define DS3231_STATUS_AF2          1

//Square wave frequencies bitmasks
#define DS3231_CONTROL_RATE_1HZ         0b00000000
#define DS3231_CONTROL_RATE_1024HZ      0b00001000
#define DS3231_CONTROL_RATE_4096HZ      0b00010000
#define DS3231_CONTROL_RATE_8192HZ      0b00011000

//Alarm bit masks
#define DS3231_A1M1     7
#define DS3231_A1M2     7
#define DS3231_A1M3     7
#define DS3231_A1M4     7

#define DS3231_A2M2     7
#define DS3231_A2M3     7
#define DS3231_A2M4     7
#define DS3231_DYDT     6

//Struct for holding the I2C port number & DS3231 slave address
typedef struct{
    i2c_port_t i2c_port_num;
    uint8_t i2c_dev_addr;
}ds3231_t;

//Time registers read from DS3231 are stored here
typedef struct{
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
}ds3231_timestamp_t;

//Status register flags are stored in here
typedef struct{
    bool osf;
    bool en_32khz;
    bool bsy;
    bool alarm1;
    bool alarm2;
}ds3231_status_reg_t;

/*  Enum values for selecting a square wave output frequency
*   These enum values will be used as indexes to access the different frequency
*   bitmasks in sq_wave_rate_sel[4] array;
*/
typedef enum{
    DS3231_SQ_WAVE_1HZ=0,
    DS3231_SQ_WAVE_1024HZ=1,
    DS3231_SQ_WAVE_4096HZ=2,
    DS3231_SQ_WAVE_8192HZ=3
}ds3231_sq_wave_rate;

static uint8_t sq_wave_rate_sel[4]={
    DS3231_CONTROL_RATE_1HZ,
    DS3231_CONTROL_RATE_1024HZ,
    DS3231_CONTROL_RATE_4096HZ,
    DS3231_CONTROL_RATE_8192HZ
};

typedef enum{
    alarm1 = 1,
    alarm2 = 2
}alarm_ID;

//Enum values for alarm 1
typedef enum{
    Alarm1EverySec = 0x0F,
    Alarm1MatchSecs = 0x0E,
    Alarm1MatchMins = 0x0C,
    Alarm1MatchHrs = 0x08,
    Alarm1MatchDay = 0x10,
    Alarm1MatchDate = 0x00 
}ds3231_alarm1_whence;

//Enum values for alarm 2
typedef enum{
    Alarm2EveryMin = 0x0E,
    Alarm2MatchMins = 0x0C,
    Alarm2MatchHrs = 0x08,
    Alarm2MatchDay = 0x10,
    Alarm2MatchDate = 0x00
}ds3231_alarm2_whence;

/*  Converts a BCD value to a decimal value */
uint8_t bcd_to_dec(uint8_t bcd);

/*  Converts a decimal value to a BCD value */
uint8_t dec_to_bcd(uint8_t dec);

void ds3231_dev_init(ds3231_t* dev, i2c_port_t i2c_port_num, uint8_t i2c_dev_addr);

esp_err_t ds3231_i2c_init(ds3231_t* dev, gpio_num_t sda, gpio_num_t sck, int freq);


/* Writes a data buffer to DS3231, starting at base address 'reg_base_addr'
*
*  To send a byte to DS3231 via I2C, the master needs to send a chain commands to DS3231 as follows:
*
*  | START CMD | -> | 7-bit slave address + R/W bit + ACK bit| -> 
*  | Register address + ACK bit | -> | Data byte 1 + ACK bit | -> | Data byte 2 + ACK bit | -> .... -> 
*  | Data byte n + ACK bit | ->| STOP CMD |
*
*  These commands are to be queued into the command handle (i2c_cmd_handle_t), which is really just a void ptr
*
*  START CMD: This command is queued using i2c_master_start()
*             This command signals the DS3231 that an I2C data transfer is to be commenced
*
*  7-Bit Slave Addr, R/W Bit & ACK Bit: The 7-bit slave address and the read/write bit is 
*                                       queued together as 1-byte using i2c_master_write_byte()
*                                       and by enabling 'ack_en' arg to check for acknowledge bit
*
*  Register Address: The specific register address which we wish to write data to.
*                    Queued using i2c_master_write_byte() and by enabling 'ack_en' arg 
*                    to check for acknowledge bit
*
*  Data buffer: Actual data which we wish to write to a DS3231 register. 
*               Queued using i2c_master_write() to transfer an entire buffer,
*               and by enable checking the acknowledge bit after each byte sent
*
*  STOP CMD: This command signals DS3231 that the I2C data transfer is finished
*            This command is queued using i2c_master_stop
*
*  After queuing the above chain of data & signals to the command list (i2c_cmd_handle_t),
*  i2c_master_cmd_begin() will beginning sending the command list to the I2C bus -> DS3231
*
*
*   PARAMETERS:
*   - i2c_port_num: The I2C port number that is configured for DS3231
*   - slave_addr: The DS3231 slave address
*   - reg_base_addr: The starting register address of DS3231 that data is written to 
*      (after each byte written, the DS3231 will automatically increment the register pointer)
*   - data_buf: Pointer to a data buffer in which the data is to be written to DS3231
*   - size: The number of bytes to be written
*/
esp_err_t ds3231_i2c_write(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* data_buff, size_t num_bytes);


/*  Read a stream of bytes, of size 'num_bytes', from DS3231 starting at the base addr 'reg_base_addr', 
    and store it in 'data_buff'
*
*  To initiate I2C master read, the master has to do the following:
*   - Send START command
*   - Send DS3231 slave address + WR bit + ACK
*   - Send starting register address + ACK:
*
*   - Send a repeated START command
*   - Send the DS3231 slave address again + RD bit +ACK
*   - Start reading data from DS3231 byte by byte (each byte, except the last) will be 
*     acknowledged by the master
*     (last byte doesn't need to be acknowledged)
*   - Send the STOP command (end of communication)
*
*   Data Reception Diagram:
*
*   |START| -> |DS3231 SLAVE ADDR + WRITE + ACK| -> |REGISTER ADDR + ACK| ->
*   |REPEATED START| -> |DS3231 SLAVE ADDR + READ + ACK| -> 
*   |DATA1 + ACK| -> |DATA2 + ACK| -> .... -> |LAST DATA + NACK| -> |STOP|
*
*   PARAMETERS:
*   - i2c_port_num: The I2C port number that is configured for DS3231
*   - slave_addr: The DS3231 I2C slave address
*   - reg_base_addr: The starting register address of DS3231 that is to be read 
*     (after each byte read, the DS3231 will automatically increment the register pointer)
*   - read_buf: Pointer to a data buffer in which the data read from I2C will be stored
*   - size: The number of bytes to be read
*/
esp_err_t ds3231_i2c_read(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* read_buf, size_t size);


/*  Set the seconds, minutes & hours values of the DS3231 time registers 
*
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - Seconds: 0 -> 59
*   - Minutes: 0 -> 59
*   - Hours: 0 -> 23
*
*   NOTE: The seconds, minutes and hours values will wrap around if exceeded their boundaries
*/
esp_err_t ds3231_set_time(ds3231_t* dev, uint8_t secs, uint8_t mins, uint8_t hrs);


/*  Read the DS3231 time registers (seconds, minutes & hours)
*
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - time: a pointer to a ds3231_timestamp_t struct to which the (seconds, minutes & hours)
*           values read will be stored.
*/
esp_err_t ds3231_get_time(ds3231_t* dev, ds3231_timestamp_t* time);


/*  Read the DS3231 status register
*
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - status: pointer to an uint8_t variable where the DS3231 status register value will be stored 
*/
esp_err_t ds3231_get_status_reg(ds3231_t* dev, uint8_t* status);


/*  Write to DS3231 status register
*   
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - status_reg: a pointer to a ds3231_status_reg_t which contains the boolean values,
*                 with each boolean corresponding to a specific flag bit. The individual
*                 boolean values will be converted to an 8-bit unsigned int by bitwise
*                 OR operation
*
*   NOTE: This function will overwrite the entire register. 
*/
esp_err_t ds3231_set_status(ds3231_t* dev, ds3231_status_reg_t* status_reg);


/*  Set the DS3231 calender registers (day of the week, day of the month, month & year)
*
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - day: 1 -> 7 or 0 -> 6 (user preference)
*   - date: 1 -> 31
*   - month: 1 -> 12
*   - year: 0 -> 99
*
*   NOTE: This function will also check for the validity of the date entered (the day of the month and the month)
*/
esp_err_t ds3231_set_calendar(ds3231_t* dev, uint8_t day, uint8_t date, uint8_t month, uint8_t year);


/*  Read the DS3231 calender registers (day of the week, day of the month, month & year) 
*   
*   PARAMETERS
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - time: a pointer to a ds3231_timestamp_t struct in which the (day of the week, 
            day of the month, month & year) values read from DS3231 will be stored
*
*/
esp_err_t ds3231_get_calendar(ds3231_t* dev, ds3231_timestamp_t* time);


/*  Set the square-wave output frequency 
*   rate: this enum value will select the bit-mask for the desired frequency
*/
esp_err_t ds3231_set_square_wave(ds3231_t* dev, ds3231_sq_wave_rate rate);


/*  Set DS3231 alarm 1 time setting registers
*
*   This function will set the time registers for alarm 1 as well as the alarm trigger frequency.
*   For each alarm trigger frequency, when the values set for the alarm 1 time registers settings match
*   the actual RTC time-registers, the INT/SQW pin will be pulled low to generate an active low pulse.
*
*   IMPORTANT:
*   When the alarm 1 time settings match the RTC time registers, the AF1 interrupt flag is set in the
*   DS3231 status register. This bit must be cleared after each alarm interrupt trigger.
*
*   PARAMETERS:
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - alarm1_whence: an enumeration value from the typedef enum 'ds3231_alarm1_whence' that selects
*                    the alarm 1 trigger frequency
*   - secs: alarm 1 seconds register time setting
*   - mins: alarm 1 minutes register time setting
*   - hrs: alarm 1 hours register time setting
*   - day_date:alarm 1 hours register time setting
*
*   NOTE: When the alarm trigger frequency is set to trigger every 1 second (Alarm1EverySec), 
*         the values entered into the alarm 1 time settings will be ignored.
*/
esp_err_t ds3231_set_alarm1(ds3231_t* dev, ds3231_alarm1_whence alarm1_whence, uint8_t secs, uint8_t mins, uint8_t hrs, uint8_t day_date);


/*  Set alarm 2 time setting registers
*   
*   This function will set the time registers for alarm 2 as well as the alarm trigger frequency.
*   For each alarm trigger frequency, when the values set for the alarm 2 time registers settings match
*   the actual time-registers, the INT/SQW pin will be pulled low to generate an active low pulse.
*
*   IMPORTANT:
*   When the alarm 2 time settings match the RTC time registers, the AF2 interrupt flag is set in the
*   DS3231 status register. This bit must be cleared after each alarm interrupt trigger.
*
*   PARAMETERS: 
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - alarm2_whence: an enumeration value from the typedef enum 'ds3231_alarm2_whence' that selects
*                    the alarm 2 trigger frequency
*   - mins: alarm 2 minutes register time setting
*   - hrs: alarm 2 hours register time setting
*   - day_date: alarm 2 hours register time setting
*
*   NOTE: When the alarm trigger frequency is set to trigger every 1 minutee (Alarm2EveryMin), 
*         the values entered into the alarm 2 time settings will be ignored.
*/
esp_err_t ds3231_set_alarm2(ds3231_t* dev, ds3231_alarm2_whence alarm2_whence, uint8_t mins, uint8_t hrs, uint8_t day_date);


/*  Enable alarm interrupts 
*
*   This function sets the INTCN bit in the DS3231 control register as well as the corresponding
*   alarm interrupt enable bit (AxIE) based on the 'alarm' value
*
*   PARAMETERS
*   - dev: pointer to a ds3231_t struct that contains the valid I2C port number 
*          and device address
*   - alarm: an enumeration value from typedef denum 'alarm_ID', specifying which alarm
*/
esp_err_t ds3231_enable_alarm_int(ds3231_t* dev, alarm_ID alarm);

esp_err_t ds3231_reg_clear_bit(ds3231_t* dev, uint8_t reg_addr, int bit_num);

esp_err_t ds3231_reg_set_bit(ds3231_t* dev, uint8_t reg_addr, int bit_num);

esp_err_t ds3231_clear_alarm_int_flag(ds3231_t* dev, alarm_ID alarm);
#endif