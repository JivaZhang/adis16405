/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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
#pragma once

#include <AP_HAL/AP_HAL.h>

//#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS16405 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

private:
    AP_InertialSensor_ADIS16405(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * If the macro BMI160_DEBUG is defined, check if there are errors reported
     * on the device's error register and panic if so. The implementation is
     * empty if the BMI160_DEBUG isn't defined.
     */
    void _check_err_reg();

    /**
     * Try to perform initialization of the BMI160 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_register();

    /**
     * Configure data ready signal pin as  interrupt pin at the level of one sample
     *
     * @return true on success, false otherwise.
     */
    bool _configure_data_rdy_pin();
    bool _imu_data_ready();
    
    /**
     * Device periodic callback to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read samples from fifo.
     */

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _accel_instance;
    uint8_t _gyro_instance;

    float _accel_scale;
    float _gyro_scale;
    float _magn_scale;
    float _tmptr_scale;

    Vector3f _gyro_off{};

    Vector3f _accl_off{};

    AP_HAL::DigitalSource *_drdy_pin;
};

