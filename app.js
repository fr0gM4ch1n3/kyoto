'use strict';

var async = require('async');
var i2c = require('i2c-bus');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var Pca9685Driver = require('pca9685').Pca9685Driver;
var Mpu9250Driver = require('./mpu9250');

var MPU9250_ADDR = 0x68;
var ACCEL_XOUT_H = 0x3B;
var ACCEL_XOUT_L = 0x3C;
var ACCEL_YOUT_H = 0x3D;
var ACCEL_YOUT_L = 0x3E;
var ACCEL_ZOUT_H = 0x3F;
var ACCEL_ZOUT_L = 0x40;
var TEMP_OUT_H = 0x41;
var TEMP_OUT_L = 0x42;
var GYRO_XOUT_H = 0x43;
var GYRO_XOUT_L = 0x44;
var GYRO_YOUT_H = 0x45;
var GYRO_YOUT_L = 0x46;
var GYRO_ZOUT_H = 0x47;
var GYRO_ZOUT_L = 0x48;

var RA_GYRO_CONFIG = 0x1B;

var G_GAIN = 0.07; // 2000dps
// var G_GAIN = 0.00875; // 250dps

var AK8963_ADDR = 0x0C;
var XOUT_L = 0x03;
var XOUT_H = 0x04;
var YOUT_L = 0x05;
var YOUT_H = 0x06;
var ZOUT_L = 0x07;
var ZOUT_H = 0x08;

var BMP180_ADDR = 0x77;

var VL53L0X_ADDR = 0x29;
var VL53L0X_REG_SYSRANGE_START= 0x00;
var VL53L0X_REG_RESULT_RANGE_STATUS = 0x14;

var PCA9685_ADDR = 0x40;
var PCA9685_FREQ = 490;
var PCA9685_MIN = 1450;
var PCA9685_MAX = 1650;

var DRONE_STATE_DISARMED = 0;
var DRONE_STATE_ARMED = 1;

// \033[0G

var pwm;
var droneState = 0;

http.listen(1337, function(){
  console.log('listening on *:' + 1337);
});

app.get('', function (req, res, next) {
  res.sendFile('index.html', {
    root: __dirname + '/',
    dotfiles: 'deny',
    headers: {}
  }, function (err) {
    if (err) {
      next(err);
    } else {
      console.log('Sent:', 'index.html');
    }
  });
});

// set-up CTRL-C with graceful shutdown
process.on("SIGINT", function () {
    console.log("\nGracefully shutting down from SIGINT (Ctrl-C)");

    pwm.setPulseLength(0, 0);
    pwm.setPulseLength(5, 0);
    pwm.setPulseLength(6, 0);
    pwm.setPulseLength(7, 0);

    pwm.dispose();
});

var i2c1 = i2c.open(1, function (err) {
  if (err) throw err;

  var data = {
    c: 0,
    t: 0,
    a: {
      x: 0,
      y: 0,
      z: 0,
      avg: {
        x: 0,
        y: 0,
        z: 0
      },
      raw: {
        x: 0,
        y: 0,
        z: 0
      }
    },
    g: {
      x: 0,
      y: 0,
      z: 0,
      avg: {
        x: 0,
        y: 0,
        z: 0,
        x2: 0,
        y2: 0
      },
      raw: {
        x: 0,
        y: 0,
        z: 0
      }
    },
    p: {
      x: 0,
      y: 0,
      z: 0
    },
    d: {
      x: 0,
      y: 0,
      x1: 0,
      y1: 0,
      z1: 0,
      x2: 0,
      y2: 0
    },
    pwm: {},
    tof: {
      a: 0,
      s: 0,
      d: 0,
      r: 0
    }
  };

  var statData = {};
  var stats = function(f, k, e, l) {
      if (!statData[f]) {statData[f] = {};}
      switch (f) {
          case "avg":
              if (!statData[f][k]) {statData[f][k] = [];}
              statData[f][k].push(e);
              statData[f][k].splice(0, Math.max(0, statData[f][k].length - l));
              return statData[f][k].reduce(function (a, b) {
                  return a + b;
              }) / statData[f][k].length;
              break;
          case "min":
              return (statData[f][k] = Math.min(statData[f][k] || e, e));
              break;
          case "max":
              return (statData[f][k] = Math.max(statData[f][k] || e, e));
              break;
      }
  }


  var cD = 300;
  var preCalibrated = true;
  var DT;
  function readA() {
    i2c1.readI2cBlock(MPU9250_ADDR, ACCEL_XOUT_H, 14, new Buffer(14) , function (err, bytesRead, buffer) {
      if (err) throw err;
      data.a.raw.x = buffer.readInt16BE(0);
      data.a.raw.y = buffer.readInt16BE(2);
      data.a.raw.z = buffer.readInt16BE(4);
      
      data.t = buffer.readInt16BE(6);

      data.g.raw.x = buffer.readInt16BE(8);
      data.g.raw.y = buffer.readInt16BE(10);
      data.g.raw.z = buffer.readInt16BE(12);

      if (data.c < cD) {
        if (preCalibrated) {
          console.log('use stored calibration data');
          data.c = cD;
          data.cs = 1;

          data.a.avg = {
            x: 52.28,
            y: 612.1866666666666,
            z: 17637.226666666666
          };
          data.g.avg = {
            x: 0.9533333333333334,
            y: 10.113333333333333,
            z: -40.95,
            x2: 181.98795022366795,
            y2: 269.83018193269277
          }
          preCalibrated = false;
        } else {
          data.cs = ++data.c / cD;

          data.a.avg.x = stats('avg', 'a.x', data.a.raw.x, cD);
          data.a.avg.y = stats('avg', 'a.y', data.a.raw.y, cD);
          data.a.avg.z = stats('avg', 'a.z', data.a.raw.z, cD);
          
          data.g.avg.x = stats('avg', 'g.x', data.g.raw.x, cD);
          data.g.avg.y = stats('avg', 'g.y', data.g.raw.y, cD);
          data.g.avg.z = stats('avg', 'g.z', data.g.raw.z, cD);

          data.g.avg.x2 = stats('avg', 'g.x2', (Math.atan2(data.a.raw.y, data.a.raw.z) + Math.PI) * (180 / Math.PI), cD);
          data.g.avg.y2 = stats('avg', 'g.y2', (Math.atan2(data.a.raw.z, data.a.raw.x) + Math.PI) * (180 / Math.PI), cD);


          if (data.c === 1) {
            console.log('calibration started');
          }
          if (data.c === cD) {
            data.p.x = 0;
            data.p.y = 0;
            data.p.z = 0;
            data.d.x = 0;
            data.d.y = 0;
            data.d.z = 0;
            data.d.x1 = 0;
            data.d.y1 = 0;
            data.d.z1 = 0;
            statData = {};
            console.log('calibration done');
            console.log('ACCEL_OUT:', data.a.avg);
            console.log('GYRO_OUT:', data.g.avg);
          }
        }
      }



      data.a.x = data.a.raw.x - data.a.avg.x;
      data.a.y = data.a.raw.y - data.a.avg.y;
      data.a.z = data.a.raw.z - data.a.avg.z;

      data.g.x = (data.g.raw.x - data.g.avg.x) * G_GAIN;
      data.g.y = (data.g.raw.y - data.g.avg.y) * G_GAIN;
      data.g.z = (data.g.raw.z - data.g.avg.z) * G_GAIN;
      
      data.d.x2 = (Math.atan2(data.a.raw.y, data.a.raw.z) + Math.PI) * (180 / Math.PI) - data.g.avg.x2;
      data.d.y2 = (Math.atan2(data.a.raw.z, data.a.raw.x) + Math.PI) * (180 / Math.PI) - data.g.avg.y2;

      if (DT) {
        var G_DT = process.hrtime(DT);
        DT = process.hrtime();
        G_DT = G_DT[0] + G_DT[1] / 1e9;
        var AA = 0.98;
        
        if (data.c === cD) {
          data.d.x1 += data.g.x * G_DT;
          data.d.y1 += data.g.y * G_DT;
          data.d.z1 += data.g.z * G_DT;
        }
      

        data.d.x = AA * (data.d.x + data.d.x1) + (1 - AA) * data.d.x2;
        data.d.y = AA * (data.d.y + data.d.y1) + (1 - AA) * data.d.y2;
      } else {
        DT = process.hrtime();
      }




      readB();
    });
  };

  function makeuint16(lsb, msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
  }

  function readB() {

    i2c1.readI2cBlock(VL53L0X_ADDR, VL53L0X_REG_RESULT_RANGE_STATUS, 12, new Buffer(12) , function (err, bytesRead, buffer) {
      if ((buffer[0] & 0x01) === 0x01) {
        i2c1.writeI2cBlock(VL53L0X_ADDR, VL53L0X_REG_SYSRANGE_START, 1, Buffer.from([0x01]), () => {});

        if ((data.tof.r = ((buffer[0] & 0x78) >> 3)) === 11) {
          data.tof.a = (makeuint16(buffer[7], buffer[6]));
          data.tof.s = (makeuint16(buffer[9], buffer[8]));
          data.tof.d = (makeuint16(buffer[11], buffer[10]));
        }

        print();
      } else {
        print();
      }
    });
  };
  
  function print() {
    try {
      io.emit('x', data);
    } catch(err) {
      console.error(err);
    }
    /*process.stdout.write('' + 
    'TEMP_OUT:' + ((data.t / 333.87) + 21.0) + '\n' +
    'ACCEL_XOUT:' + (data.a.x) + '\n' +
    'ACCEL_YOUT:' + (data.a.y) + '\n' +
    'ACCEL_ZOUT:' + (data.a.z) + '\n' +
    'GYRO_XOUT:' + (data.g.x) + '\n' +
    'GYRO_YOUT:' + (data.g.y) + '\n' +
    'GYRO_ZOUT:' + (data.g.z) + '\n');*/
    // flightControl();
  };

  var pwmPL = {};
  function setPulseLength(channel, pulseLength, callback) {
    if (pwmPL[channel] !== typeof pulseLength === 'undefined') {
      pwm.setPulseLength(channel, 0, 0, callback);
    }
    pulseLength = Math.max(PCA9685_MIN, Math.min(PCA9685_MAX, Math.round(pulseLength)));
    if (pwmPL[channel] !== pulseLength) {
      // data.pwm[channel] = pwmPL[channel] = pulseLength;
      pwm.setPulseLength(channel, data.pwm[channel] = pwmPL[channel] = pulseLength, 0, callback);
    }

    /*

                    y+

                0       5
                  \   /
                    B
         x-         X         x+
                    P
                  /   \
                7       6

                    y-

    */
  }

  var diffAX = 0;
  var diffAY = 0;
  function flightControl() {
    if (droneState === DRONE_STATE_ARMED) {

      diffAX = Math.pow((data.d.x2),  2) * 0.05 * (data.d.x2 / Math.abs(data.d.x2));
      diffAY = 0; // (data.d.y2 / 360) * 10;
      // var diffAX = 0; // (data.d.x2 / 360) * (PCA9685_MAX - PCA9685_MIN);
      // var diffAY = 0; // (-data.d.y2 / 360) * (PCA9685_MAX - PCA9685_MIN);
      var diffAZ = 0; // 1.0 * data.a.z * (PCA9685_MAX - PCA9685_MIN);

/*
      data.g.x === 999
      data.d.x2 === 0



      data.g.x === 0
      data.d.x2 === 180 / 180  === 1
*/

      var rd = Math.round(data.tof.d / 10) * 10;
      var TARGET = 300;

      // diffAZ = Math.min(Math.max((TARGET - rd), -100), 100);

      diffAZ = 300;


      var tl = -diffAX +diffAY +diffAZ;
      var tr = +diffAX +diffAY +diffAZ;
      var br = +diffAX -diffAY +diffAZ;
      var bl = -diffAX -diffAY +diffAZ;

      var doff = Math.max(0, PCA9685_MIN - Math.min(tl, tr, br, bl));

      async.parallel([
        callback => setPulseLength(0, tl + doff, callback),     // L - T
        callback => setPulseLength(5, tr + doff, callback),     // R - T
        callback => setPulseLength(6, br + doff, callback),     // R - B
        callback => setPulseLength(7, bl + doff, callback),     // L - B
      ], (err, results) => {

      });
    } else {
      setPulseLength(0);
      setPulseLength(5);
      setPulseLength(6);
      setPulseLength(7);
    }
  }

  var rB = function (b, o, l, v) {
      b[0] &= ~((0xFF >> (o + l + 1)) << l);
      b[0] |= (v << (o - 1));

      return b;
  }

  i2c1.readI2cBlock(MPU9250_ADDR, RA_GYRO_CONFIG, 1, new Buffer(1), (err, bytesRead, buffer) => {

    var byte = buffer[0];
    byte = byte & 0x18;
    byte = byte >> 3;

    rB(buffer, 4, 2, 0x03);


    console.log(buffer, byte);

    i2c1.writeI2cBlock(MPU9250_ADDR, RA_GYRO_CONFIG, 1, buffer, () => {

      setInterval(flightControl, 20);
      setInterval(readA, 20);
    });
  });
  




  io.on('connection', function (socket) {
    console.log('a user connected');

    socket.on('disconnect', function () {
      console.log('user disconnected');
    });

    socket.on('calibrate', function() {
      data.c = 0;
    });
    
    socket.on('droneState', function(state) {
      console.log('set droneState to', state);
      droneState = state;
    });
  });


  var mpu = new Mpu9250Driver({
    i2c: i2c1
  });


  // initialize PCA9685 and start loop once initialized
  pwm = new Pca9685Driver({
    i2c: i2c1,
    address: PCA9685_ADDR,
    frequency: PCA9685_FREQ,
    debug: false
  }, function startLoop(err) {
    if (err) {
      console.error("Error initializing PCA9685");
      process.exit(-1);
    }

    pwm.setPulseLength(0, 1000);
    pwm.setPulseLength(5, 1000);
    pwm.setPulseLength(6, 1000);
    pwm.setPulseLength(7, 1000);
  });
  

});
