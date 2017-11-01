/* * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>

//#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL_Linux/GPIO.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_ADIS16405.h"

/* Registers and bits definitions. The indented ones are the bits for the upper
 * register. */

#define ADIS16405_FLASH_CNT 0x00
#define ADIS16405_SUPPLY_OUT 0x02
#define ADIS16405_XGYRO_OUT 0x04
#define ADIS16405_YGYRO_OUT 0x06
#define ADIS16405_ZGYRO_OUT 0x08
#define ADIS16405_XACCL_OUT 0x0A
#define ADIS16405_YACCL_OUT 0x0C
#define ADIS16405_ZACCL_OUT 0x0E
#define ADIS16405_TEMP_OUT  0x16
#define ADIS16405_AUX_ADC   0x18
#define ADIS16405_XGYRO_OFF 0x1A
#define ADIS16405_YGYRO_OFF 0x1C
#define ADIS16405_ZGYRO_OFF 0x1E
#define ADIS16405 XACCL_OFF 0x20
#define ADIS16405_YACCL_OFF 0x22
#define ADIS16405_ZACCL_OFF 0x24
#define ADIS16405_XMAGN_HIF 0x26
#define ADIS16405_YMAGN_HIF 0x28
#define ADIS16405_ZMAGN_HIF 0x2A
#define ADIS16405_GPIO_CTRL 0x2E
#define ADIS16405_MSC_CTRL  0x30
#define ADIS16405_SMPL_PRD 0x36
#define ADIS16405_SENS_AVG 0x38
#define ADIS16405_SLP_CNT 0x3A
#define ADIS16405_DIAG_START 0x3C
#define ADIS16405_GLOB_CMD 0x3E
#define ADIS16405_ALM_MAG1 0x40
#define ADIS16405_ALM_MAG2 0x42
#define ADIS16405_ALM_SML1 0x44
#define ADIS16405_ALM_SML2 0x46 
#define ADIS16405_ALM_CTRL 0x48
#define ADIS16405_AUX_DAC 0x4A
#define ADIS16405_PRODUCT_ID 0x56

#define ADIS16405_READ_FLAG 0x80  
#define ADIS16405_HARDWARE_INIT_MAX_TRIES 5
#define ADIS16405_CMD_SOFTRESET 0x80
#define ADIS16405_CHIP_ID 0x4015
#define ADIS16405_RAW_SAMPLING_HZ 819
/* Datasheet says that the device powers up in less than 10ms, so waiting for
 * 220 ms before initialization is enough. */
#define ADIS16405_POWERUP_DELAY_MSEC  220

#define ADIS16405_INT1_GPIO -1



struct PACKED RawData
{
    struct 
    {
        le16_t x;
        le16_t y;
        le16_t z; 
    }gyro;
    struct 
    {
        le16_t x;
        le16_t y;
        le16_t z; 
    }accel;
    struct 
    {
        le16_t x;
        le16_t y;
        le16_t z; 
    }magn;
    le16_t tmptr; 
};

extern const AP_HAL::HAL& hal;

AP_InertialSensor_ADIS16405::AP_InertialSensor_ADIS16405(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS16405::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_ADIS16405(imu, std::move(dev));

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16405::start()
{
    bool r;

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    r = _configure_register();
    if (!r) {
        AP_HAL::panic("ADIS16405: Unable to configure accelerometer");
    }


    if (ADIS16405_INT1_GPIO >= 0) {
        r = _configure_data_rdy_pin();
        if (!r) {
            AP_HAL::panic("ADIS16405: unable to configure INT1 pin");
        }
    }

    _dev->get_semaphore()->give();

    _accel_instance = _imu.register_accel(ADIS16405_RAW_SAMPLING_HZ, _dev->get_bus_id_devtype(DEVTYPE_ADIS16405));
    _gyro_instance = _imu.register_gyro(ADIS16405_RAW_SAMPLING_HZ,_dev->get_bus_id_devtype(DEVTYPE_ADIS16405));

    printf("adis16405 start is called by probe\n");
    /* Call _poll_data() at 1kHz */
    _dev->register_periodic_callback(50,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16405::_poll_data, void));
}

bool AP_InertialSensor_ADIS16405::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

void AP_InertialSensor_ADIS16405::_check_err_reg()
{
#ifdef BMI160_DEBUG
    uint8_t v;
    bool r;

    r = _dev->read_registers(BMI160_REG_ERR_REG, &v, 1);
    if (!r) {
        AP_HAL::panic("BMI160: couldn't read ERR_REG\n");
    }
    if (v) {
        AP_HAL::panic("BMI160: error detected on ERR_REG\n");
    }
#endif
}

bool AP_InertialSensor_ADIS16405::_configure_register()
{
//    bool r;
//
//    r = _dev->write_register(BMI160_REG_ACC_CONF, BMI160_OSR | BMI160_ODR);
//    if (!r) {
//        return false;
//    }
//    hal.scheduler->delay(1);
//
//    _check_err_reg();
//
//    r = _dev->write_register(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_BITS);
//    if (!r) {
//        return false;
//    }
//    hal.scheduler->delay(1);
//
//    /* The sensitivity in LSb/g for an accel range i (as defined by the macros
//     * BMI160_ACC_RANGE_*G) can be calculated with:
//     *     2 ^ 16 / (2 * 2 ^ (i + 1)) = 2 ^(14 - i)
//     * That matches the typical values in the datasheet. */
//    _accel_scale = GRAVITY_MSS / (1 << (14 - BMI160_ACC_RANGE));
//
    return true;
}

bool AP_InertialSensor_ADIS16405::_configure_data_rdy_pin()
{
    _drdy_pin = hal.gpio->channel(ADIS16405_INT1_GPIO);
    if ( _drdy_pin == nullptr) {
        hal.console->printf("ADIS16405: Couldn't request data ready GPIO channel\n");
        return false;
    }
    _drdy_pin->mode(HAL_GPIO_INPUT);

    return true;
}


bool AP_InertialSensor_ADIS16405::_imu_data_ready()
{
    return true;
    //if(_drdy_pin !=nullptr)
    //{  
    //    return _drdy_pin->read()!= 0;
    //}
     
}

void AP_InertialSensor_ADIS16405::_poll_data()
{
    
    struct RawData raw;
    bool r = true;
    uint16_t num_bytes;
    
    if(_imu_data_ready())
    {
        // reset signal pin to prepare receive next sampling     
        //get the raw data 
        num_bytes = sizeof(struct RawData);
        memset((void*)&raw,0,sizeof(struct RawData));
        //for(uint8_t tries = 0; tries < 5;tries++)
        //{
            
            if(!_dev->set_chip_select(true))
            {
                printf("cs_force is false");
            }
            r = _dev->read_registers(ADIS16405_XGYRO_OUT,
                                (uint8_t*)&raw,         
                                num_bytes);
            //printf("adis1405 read register and return the status = %02x",r);
            if(r)
            {
                //Vector3f accel{(float)(int16_t)le16toh(raw.accel.x),
                //               (float)(int16_t)le16toh(raw.accel.y),
                //               (float)(int16_t)le16toh(raw.accel.z)};
                //Vector3f gyro{(float)(int16_t)le16toh(raw.gyro.x),
                //               (float)(int16_t)le16toh(raw.gyro.y),
                //               (float)(int16_t)le16toh(raw.gyro.z)};
                //Vector3f magn{(float)(int16_t)le16toh(raw.magn.x),
                //               (float)(int16_t)le16toh(raw.magn.y),
                //               (float)(int16_t)le16toh(raw.magn.z)};
                //printf("send num_bytes = %d",num_bytes);
                // printf("accle.x=%f,acclel.y=%f,accel.z=%f\n",(float)(raw.accel.x),(float)(raw.accel.y),(float)(raw.accel.z));
                printf("gyro.x=%f,gyro=%f,gyro.z=%f\n",(float)(raw.gyro.x),(float)(raw.gyro.y),(float)(raw.gyro.z));

                raw.tmptr = (float)(int16_t)le16toh(raw.tmptr);
        	//_rotate_and_correct_accel(_accel_instance, accel);
        	//_rotate_and_correct_gyro(_gyro_instance, gyro);
        	//_notify_new_accel_raw_sample(_accel_instance, accel);
        	//_notify_new_gyro_raw_sample(_gyro_instance, gyro);
                            }
            else
            {
                if(!r)
                {
                    hal.console->printf("ADIS16405:error on reading data from registers\n");
                    printf("adis16405 read data from register failed!\n");
                    return; 
                }
            }
        //}
        
       }   
}

bool AP_InertialSensor_ADIS16405::_hardware_init()
{
    bool ret = false;

    hal.scheduler->delay(ADIS16405_POWERUP_DELAY_MSEC);

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    //for (unsigned i = 0; i < ADIS16405_HARDWARE_INIT_MAX_TRIES; i++) {
        for(;;){
        uint16_t v;
        ret = _dev->write_register(ADIS16405_GLOB_CMD,
                                   ADIS16405_CMD_SOFTRESET);

        if (!ret) {
            continue;
        }
        hal.scheduler->delay(ADIS16405_POWERUP_DELAY_MSEC);

        /* The datasheet recommends doing a read operation on the register 0x7F
         * in order to guarantee the sensor works using the SPI protocol. This
         * shouldn't have side effects for I2C. */

        //ret = _dev->read_registers(0x7F, &v, 1);
        //if (!ret) {
        //    continue;
        //}

        ret = _dev->read_registers(ADIS16405_PRODUCT_ID, (uint8_t*)&v, 1);
        if (!ret) {
            continue;
        }
        printf("the adis16405 product id is %02x\n",v);
        if (v != ADIS16405_CHIP_ID) {
            ret = false;
            continue;
        }
        else
        {
            break;
        }

        //ret = _dev->write_register(BMI160_REG_CMD,
        //                           BMI160_CMD_ACCEL_NORMAL_POWER_MODE);
        //if (!ret) {
        //    continue;
        //}
        //hal.scheduler->delay(BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC);

        //ret = _dev->write_register(BMI160_REG_CMD,
        //                           BMI160_CMD_GYRO_NORMAL_POWER_MODE);
        //if (!ret) {
        //    continue;
        //}
        //hal.scheduler->delay(BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC);

        break;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();
    //return ret;
    return true;
}

bool AP_InertialSensor_ADIS16405::_init()
{
    bool ret = false;
    _dev->set_read_flag(ADIS16405_READ_FLAG);

    ret = _hardware_init();
    if (!ret) {
        hal.console->printf("ADIS16405: failed to init\n");
    }

    return ret;
}

