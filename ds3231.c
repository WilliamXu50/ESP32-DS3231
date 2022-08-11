#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "driver/i2c.h"
#include "ds3231.h"

uint8_t bcd_to_dec(uint8_t bcd){
    uint8_t bcd_hi=(bcd & 0xF0) >> 4;
    uint8_t bcd_lo=(bcd & 0x0F);

    if(bcd_hi>9 && bcd_lo>9){
        return bcd;
    }

    uint8_t res=(bcd_hi * 10)+bcd_lo;
    
    return res;
}

uint8_t dec_to_bcd(uint8_t dec){
    uint8_t bcd_lo=dec % 10;
    uint8_t bcd_hi=(dec-bcd_lo)/10;

    uint8_t res=(bcd_hi << 4)|bcd_lo;

    return res;
}

void ds3231_dev_init(ds3231_t* dev, i2c_port_t i2c_port_num, uint8_t i2c_dev_addr){
    dev->i2c_port_num=i2c_port_num;
    dev->i2c_dev_addr=i2c_dev_addr;
}

esp_err_t ds3231_i2c_init(ds3231_t* dev, gpio_num_t sda, gpio_num_t sck, int freq){
    i2c_config_t i2c_master_config={
        .mode=I2C_MODE_MASTER,
        .sda_io_num=sda,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_io_num=sck,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=freq
    };

    i2c_param_config(dev->i2c_port_num, &i2c_master_config);
   
    esp_err_t ret=i2c_driver_install(dev->i2c_port_num, i2c_master_config.mode, 0, 0, 0);
    
    return ret;
}

esp_err_t ds3231_i2c_write(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* data_buff, size_t num_bytes){
    //Create a command link
    i2c_cmd_handle_t cmd_handle=i2c_cmd_link_create();
    esp_err_t ret;

    //Queue START command
    i2c_master_start(cmd_handle);   
    //Queue DS3231 slave address
    i2c_master_write_byte(cmd_handle, (slave_addr << 1)|I2C_MASTER_WRITE, true);
    //Quene the desired DS3231 register address
    i2c_master_write_byte(cmd_handle, reg_base_addr, true);
    //Queue the actual data buffer
    i2c_master_write(cmd_handle, data_buff, num_bytes, true);
    //Queue the STOP command
    i2c_master_stop(cmd_handle);

    //Send the queued data and signals to I2C bus for transfer
    ret=i2c_master_cmd_begin(i2c_port_num, cmd_handle, 1000/portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}


esp_err_t ds3231_i2c_read(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* read_buf, size_t size){
    if(size==0){
        return ESP_FAIL;
    } 
    
    i2c_cmd_handle_t cmd_handle=i2c_cmd_link_create();
    esp_err_t ret;

    //First send the DS3231 slave address and the register address to start reading from
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (slave_addr << 1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_base_addr, true);

    //Then send a repeated START, DS3231 slave address, and start reading
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (slave_addr << 1)|I2C_MASTER_READ, true);

    if(size>1){ //Multi-byte read
        i2c_master_read(cmd_handle, read_buf, size, I2C_MASTER_LAST_NACK);
    }
    else{   //Single byte read
        i2c_master_read_byte(cmd_handle, read_buf, RECEIVED_NOT_ACK);
    }

    i2c_master_stop(cmd_handle);

    ret=i2c_master_cmd_begin(i2c_port_num, cmd_handle, 1000/portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

/*******************************************************************************************/

/*                                      USER FUNCTIONS                                     */

/*******************************************************************************************/
/*  Set a single bit in a register  */
esp_err_t ds3231_reg_set_bit(ds3231_t* dev, uint8_t reg_addr, int bit_num){
    uint8_t reg_read;
    ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, reg_addr, &reg_read, 1);

    reg_read |=(1<<bit_num);

    esp_err_t ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, reg_addr, &reg_read, 1);
    
    return ret;
}

/*  Clear a single bit in a register  */
esp_err_t ds3231_reg_clear_bit(ds3231_t* dev, uint8_t reg_addr, int bit_num){
    uint8_t reg_read;
    ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, reg_addr, &reg_read, 1);

    reg_read &= ~(1<<bit_num);

    esp_err_t ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, reg_addr, &reg_read, 1);
    
    return ret;
}

/*  Set the DS3231 time registers */
esp_err_t ds3231_set_time(ds3231_t* dev, uint8_t secs, uint8_t mins, uint8_t hrs){
    //Confine the values of secs, mins & hrs within their boundaries
    secs = secs % 60;
    mins = mins % 60;
    hrs = hrs % 24;

    //Convert the decimal to BCD format
    uint8_t bcd_time[3];
    bcd_time[0]=dec_to_bcd(secs);
    bcd_time[1]=dec_to_bcd(mins);
    bcd_time[2]=dec_to_bcd(hrs) & 0x3F;
    
    esp_err_t ret;

    ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_SECONDS_REG_ADDR, bcd_time, sizeof(bcd_time));

    return ret;
}

/*  Read the DS3231 time registers */
esp_err_t ds3231_get_time(ds3231_t* dev, ds3231_timestamp_t* time){
    //Store the secs, mins & hrs in the first 3 bytes of 'ds3231_timestamp_t' struct
    esp_err_t ret=ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_SECONDS_REG_ADDR, (uint8_t*)time, 3);
    
    if(ret==ESP_OK){
        time->secs=bcd_to_dec(time->secs);
        time->mins=bcd_to_dec(time->mins);
        time->hours=bcd_to_dec(time->hours & 0x3F);
    }

    return ret;
}

esp_err_t ds3231_set_calendar(ds3231_t* dev, uint8_t day, uint8_t date, uint8_t month, uint8_t year){
    if(day>7 || date>31 || month>12){
        return -1;
    }

    //Check months and their number of days
    if(month==2 && date>28){
        return -2;
    }

    if(month==4 || month==6 || month==9 || month==11){
        if(date>30){
            return -2;
        }
    }

    year = year % 100;
    
    uint8_t bcd_date[4];
    bcd_date[0]=dec_to_bcd(day);
    bcd_date[1]=dec_to_bcd(date);
    bcd_date[2]=dec_to_bcd(month);
    bcd_date[3]=dec_to_bcd(year);

    esp_err_t ret;
    ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_DAY_REG_ADDR, bcd_date, 4);

    return ret;
}

esp_err_t ds3231_get_calendar(ds3231_t* dev, ds3231_timestamp_t* time){
    esp_err_t ret=ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_DAY_REG_ADDR, (uint8_t*)&time->day, 4);

    if(ret==ESP_OK){
        time->day=bcd_to_dec(time->day);
        time->date=bcd_to_dec(time->date);
        time->month=bcd_to_dec(time->month & 0x1F);
        time->year=bcd_to_dec(time->year);
    }

    return ret;
}

/*
esp_err_t ds3231_get_status(ds3231_t* dev, ds3231_status_reg_t* status_reg){
    uint8_t status;
    memset(status_reg, 0, sizeof(ds3231_status_reg_t));

    esp_err_t ret=ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_STATUS_REG_ADDR, &status, 1);

    if(ret==ESP_OK){
        status_reg->osf=status & (1 << DS3231_STATUS_OSF);
        status_reg->en_32khz=status & (1 << DS3231_STATUS_EN_32KHZ);
        status_reg->bsy=status & (1 << DS3231_STATUS_BSY);
        status_reg->alarm1=status & (1 << DS3231_STATUS_AF1);
        status_reg->alarm2=status & (1 << DS3231_STATUS_AF2);
    }

    return ret;
}*/

esp_err_t ds3231_get_status_reg(ds3231_t* dev, uint8_t* status){
    return ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_STATUS_REG_ADDR, status, 1);
}

esp_err_t ds3231_get_control_reg(ds3231_t* dev, uint8_t* control){
    return ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_CONTROL_REG_ADDR, control, 1);
}

esp_err_t ds3231_set_status(ds3231_t* dev, ds3231_status_reg_t* status_reg){
    uint8_t write_status=0;

    write_status |=((uint8_t)status_reg->osf)<<7 |
                   ((uint8_t)status_reg->en_32khz)<<3 |
                   ((uint8_t)status_reg->bsy)<<2 |
                   ((uint8_t)status_reg->alarm2)<<1 |
                   ((uint8_t)status_reg->alarm1)<<0;

    esp_err_t ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_STATUS_REG_ADDR, &write_status, 1);

    return ret;
}

esp_err_t ds3231_set_control_reg(ds3231_t* dev, uint8_t flag_bitmask){
    uint8_t control;
    ds3231_get_control_reg(dev, &control);

    control |= flag_bitmask;

    return ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_CONTROL_REG_ADDR, &control, 1);
}

esp_err_t ds3231_set_square_wave(ds3231_t* dev, ds3231_sq_wave_rate rate){
    uint8_t control;

    if(ds3231_i2c_read(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_CONTROL_REG_ADDR, &control, 1)!=ESP_OK){
        return -1;
    }

    if(rate > DS3231_SQ_WAVE_8192HZ){return -2;}

    //Clear the INTCN bit (as well as the current RS1 & RS2 bits)
    control &=~((1 << DS3231_INTCN)|
                (1 << DS3231_RS2)|
                (1 << DS3231_RS1));

    //Select the frequency bit-mask
    control |= (1 << DS3231_BBSQW) | sq_wave_rate_sel[rate];

    esp_err_t ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_CONTROL_REG_ADDR, &control, 1);

    return ret;
}

esp_err_t ds3231_set_alarm1(ds3231_t* dev, ds3231_alarm1_whence alarm1_whence, uint8_t secs, uint8_t mins, uint8_t hrs, uint8_t day_date){
    //Confine the secs, mins & hrs within their limits
    secs = secs % 60;
    mins = mins % 60;
    hrs = hrs % 24;
    day_date = day_date % 31;

    uint8_t buf[4];

    //Store alarm 1 time 
    buf[0]=dec_to_bcd(secs);
    buf[1]=dec_to_bcd(mins);
    buf[2]=dec_to_bcd(hrs);
    buf[3]=dec_to_bcd(day_date);

    if(alarm1_whence & 0x01){buf[0] |= (1 << DS3231_A1M1);}
    if(alarm1_whence & 0x02){buf[1] |= (1 << DS3231_A1M2);}
    if(alarm1_whence & 0x04){buf[2] |= (1 << DS3231_A1M3);}
    if(alarm1_whence & 0x08){buf[3] |= (1 << DS3231_A1M4);}
    if(alarm1_whence & 0x10){buf[3] |= (1 << DS3231_DYDT);}
    
    esp_err_t ret;
    ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_ALARM_1_SECONDS_REG_ADDR, buf, 4);

    //Clear alarm 1 interrupt flag
    ds3231_reg_clear_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_AF1);
    return ret;
}

esp_err_t ds3231_set_alarm2(ds3231_t* dev, ds3231_alarm2_whence alarm2_whence, uint8_t mins, uint8_t hrs, uint8_t day_date){
    //Confine the values within their boundaries
    mins = mins % 60;
    hrs = hrs % 24;
    day_date = day_date % 31;

    uint8_t buf[3];

    buf[0]=dec_to_bcd(mins);
    buf[1]=dec_to_bcd(hrs);
    buf[2]=dec_to_bcd(day_date);

    if(alarm2_whence & 0x02){buf[0] |= (1 << DS3231_A2M2);}
    if(alarm2_whence & 0x04){buf[1] |= (1 << DS3231_A2M3);}
    if(alarm2_whence & 0x08){buf[2] |= (1 << DS3231_A2M4);}
    if(alarm2_whence & 0x10){buf[2] |= (1 << DS3231_DYDT);}

    esp_err_t ret;
    ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_ALARM_2_MINUTES_REG_ADDR, buf, 3);

    //Clear alarm 2 interrupt flag
    ds3231_reg_clear_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_AF2);
    return ret;
}

esp_err_t ds3231_enable_alarm_int(ds3231_t* dev, alarm_ID alarm){
    /*if(alarm!=alarm1 && alarm!=alarm2){
        return -10;
    }*/
    //Read the control register
    //uint8_t control_reg;
    //ds3231_get_control_reg(dev, &control_reg);

    //Check if the INTCN flag is set
    //Set the interrupt control flag (INTCN): this will disable the square-wave output and allow alarm interrupts
    /*if(!(control_reg & (1 << DS3231_INTCN))){
        control_reg |= (1 << DS3231_INTCN);
    }*/

    //Set the appropriate alarm interrupt flag
    /*if(alarm == alarm1){
        control_reg |= (1 << DS3231_A1IE);
    }*/
    /*else{
        control_reg |= (1 << DS3231_A2IE);
    }*/

    //Write back to control register
    //esp_err_t ret=ds3231_i2c_write(dev->i2c_port_num, dev->i2c_dev_addr, DS3231_CONTROL_REG_ADDR, &control_reg, 1);

    if(alarm==alarm1){
        return ds3231_set_control_reg(dev, (1<<DS3231_INTCN)|(1<<DS3231_A1IE));
    }
    else if(alarm==alarm2){
        return ds3231_set_control_reg(dev, (1<<DS3231_INTCN)|(1<<DS3231_A2IE));
    }
    else{return -10;}
}

esp_err_t ds3231_disable_alarm(ds3231_t* dev, alarm_ID alarm){
    if(alarm==alarm1){
        return ds3231_reg_clear_bit(dev, DS3231_CONTROL_REG_ADDR, DS3231_A1IE);
    }
    else if(alarm==alarm2){
        return ds3231_reg_clear_bit(dev, DS3231_CONTROL_REG_ADDR, DS3231_A2IE);
    }
    else{return -10;}
}

esp_err_t ds3231_disable_int(ds3231_t* dev){
    return ds3231_reg_clear_bit(dev, DS3231_CONTROL_REG_ADDR, DS3231_INTCN);
}

esp_err_t ds3231_clear_alarm_int_flag(ds3231_t* dev, alarm_ID alarm){
    if(alarm==alarm1){
        return ds3231_reg_clear_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_AF1);
    }
    else if(alarm==alarm2){
        return ds3231_reg_clear_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_AF2);
    }
    else{return -10;}
}

esp_err_t ds3231_32khz_out(ds3231_t* dev, bool enable){
    if(enable==true){
        return ds3231_reg_set_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_EN_32KHZ);
    }
    else{
        return ds3231_reg_clear_bit(dev, DS3231_STATUS_REG_ADDR, DS3231_STATUS_EN_32KHZ);
    }
}