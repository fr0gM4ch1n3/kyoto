'use strict';

var i2c = require('i2c-bus');
var mpu9250 = require('./mpu9250');

var i2c1 = i2c.open(1, function (err) {
    // Instantiate and initialize.
    var mpu = new mpu9250({
        i2c: i2c1,
        DEBUG: true,
        GYRO_FS: 0,
        ACCEL_FS: 0,
        scaleValues: true,
        UpMagneto: false
    });

    if (mpu.initialize() !== false) {

        mpu.reset();

    }
});
