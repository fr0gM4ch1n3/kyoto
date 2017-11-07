'use strict';

const async = require('async');
const i2c = require('i2c-bus');
const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const Pca9685Driver = require('pca9685').Pca9685Driver;
const Mpu9250Driver = require('./mpu9250');
const PIDController = require('./pidc');


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

    pwm.setPulseLength(4, 0);
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




  var ACCEL_NAME = 'Accel (g)';
  var GYRO_NAME = 'Gyro (°/sec)';
  var MAG_NAME = 'Mag (uT)';
  var HEADING_NAME = 'Heading (°)';
  var KAL_NAME = 'Kalman';
  var stats = new Stats([ACCEL_NAME, GYRO_NAME, MAG_NAME, HEADING_NAME, KAL_NAME], 1000);

  var cnt = 0;
  var lastMag = [0, 0, 0];

  let dr = 0
  let dp = 0

  function getSensorData() {
                var m9;
                // Only get the magnetometer values every 100Hz
                var getMag = cnt++ % 2;
                if (getMag) {
                    m9 = mpu.getMotion6().concat(lastMag);
                } else {
                    m9 = mpu.getMotion9();
                    lastMag = [m9[6], m9[7], m9[8]];
                }

				var dt = (micros() - timer) / 1000000;
				timer = micros();

				pitch = mpu.getPitch(m9);
				roll = mpu.getRoll(m9);
				yaw = mpu.getYaw(values);

				var gyroXrate = values[3] / 131.0;
				var gyroYrate = values[4] / 131.0;
				var gyroZrate = values[5] / 131.0;

				if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
					kalmanX.setAngle(roll);
					compAngleX = roll;
					kalAngleX = roll;
					gyroXangle = roll;
				} else {
					kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
				}

				if (Math.abs(kalAngleX) > 90) {
					gyroYrate = -gyroYrate;
				}
				kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

				gyroXangle += gyroXrate * dt;
				gyroYangle += gyroYrate * dt;
				compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
				compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

				if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
				if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;


                stats.add(ACCEL_NAME, m9[0], m9[1], m9[2]);
                stats.add(GYRO_NAME, m9[3], m9[4], m9[5]);

                if (getMag) {
                    stats.add(MAG_NAME, m9[6], m9[7], m9[8]);
                    stats.addValue(HEADING_NAME, calcHeading(m9[6], m9[7]));
                }
                stats.add(KAL_NAME, compAngleX, compAngleY);

                dr = ctrRoll.update(compAngleX - 2.55280);
                dp = ctrPitch.update(compAngleY - -6.83516);











      // Make the numbers pretty
      var str = '';
      for (var i = 0; i < m9.length; i++) {
          str += p(m9[i]);
      }

      // process.stdout.write(p(dt) + str + p(mpu.getTemperatureCelsiusDigital()) + p(calcHeading(m9[6], m9[7])) + p(compAngleX - 2.55280) + p(compAngleY - -6.83516) + p(dr) + p(dp) + '  \r');

      readB();
  }




    function p(num, pre) {
        if (num === undefined) {
            return '       ';
        }
        var str = num.toFixed(pre || 3);
        while (str.length <= 7) {
            str = ' ' + str;
        }
        return str + ' ';
    }

    function calcHeading(x, y) {
        var heading = Math.atan2(y, x) * 180 / Math.PI;

        if (heading < -180) {
            heading += 360;
        } else if (heading > 180) {
            heading -= 360;
        }

        return heading;
    }


    /**
     * Calculate Statistics
     * @param {[string]} names The names of the vectors.
     */
    function Stats(vectorNames, numStats) {

        this.vectorNames = vectorNames;
        this.numStats = numStats;
        this.vectors = {};
        this.done = false;

        for (var i = 0; i < vectorNames.length; i += 1) {
            var name = vectorNames[i];
            this.vectors[name] = {
                x: [],
                y: [],
                z: [],
                pos: 0
            };
        }

        function exitHandler(options, err) {
            if (err) {
                console.log(err.stack);
            } else {
                this.printStats();
            }
            if (options.exit) {
                var exit = process.exit;
                exit();
            }
        }

        // do something when app is closing
        process.on('exit', exitHandler.bind(this, {cleanup: true}));

        // catches ctrl+c event
        process.on('SIGINT', exitHandler.bind(this, {exit: true}));

        // catches uncaught exceptions
        process.on('uncaughtException', exitHandler.bind(this, {exit: true}));
    }
    Stats.prototype.add = function(vectorName, x, y, z) {
        var v = this.vectors[vectorName];
        var len = v.x.length;
        if (v.pos >= this.numStats) {
            v.pos = 0;
        } else {
            v.pos += 1;
        }
        v.x[v.pos] = x;
        v.y[v.pos] = y;
        v.z[v.pos] = z;
    };
    Stats.prototype.addValue = function(vectorName, x) {
        var v = this.vectors[vectorName];
        v.isValue = true;
        if (v.pos >= this.numStats) {
            v.pos = 0;
        } else {
            v.pos += 1;
        }
        v.x[v.pos] = x;
    };
    Stats.prototype.printStats = function () {
        if (this.done) return;
        this.done = true;

        console.log('\n\n\nStatistics:');
        console.log('           average   std.dev.  num.same.values');
        for (var i = 0; i < this.vectorNames.length; i += 1) {
            var name = this.vectorNames[i];
            var v = this.vectors[name];
            console.log(name + ':');
            console.log(' -> x: ', average(v.x).toFixed(5), standardDeviation(v.x).toFixed(5), numSameValues(v.x));
            if (!v.isValue) {
                console.log(' -> y: ', average(v.y).toFixed(5), standardDeviation(v.y).toFixed(5), numSameValues(v.y));
                console.log(' -> z: ', average(v.z).toFixed(5), standardDeviation(v.z).toFixed(5), numSameValues(v.z));
            }
            console.log(' -> num samples: ', v.x.length);
            console.log();
        }

        function standardDeviation(values) {
            var avg = average(values);

            var squareDiffs = values.map(function (value) {
                var diff = value - avg;
                var sqrDiff = diff * diff;
                return sqrDiff;
            });

            var avgSquareDiff = average(squareDiffs);

            var stdDev = Math.sqrt(avgSquareDiff);
            return stdDev;
        }

        function average(values) {
            var sumData = values.reduce(function (sum, value) {
                return sum + value;
            }, 0);

            var avg = sumData / values.length;
            return avg;
        }

        function numSameValues(values) {
            var same = 0;
            var lastVal = NaN;
            values.forEach(function(val) {
                if (val === lastVal) {
                    same += 1;
                }
                lastVal = val;
            });
            return same;
        }
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

               0/4      5
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

      diffAZ = 0;
      diffAY = dp * 2.0;
      diffAX = dr * 2.0;


      var fl = +diffAX -diffAY +diffAZ;
      var fr = -diffAX -diffAY +diffAZ;
      var br = -diffAX +diffAY +diffAZ;
      var bl = +diffAX +diffAY +diffAZ;

      var doff = Math.max(0, PCA9685_MIN - Math.min(fl, fr, br, bl));

      async.parallel([
        callback => setPulseLength(4, fl + doff, callback),     // L - F
        callback => setPulseLength(5, fr + doff, callback),     // R - F
        callback => setPulseLength(6, br + doff, callback),     // R - B
        callback => setPulseLength(7, bl + doff, callback),     // L - B
      ], (err, results) => {

      });
    } else {
      setPulseLength(4);
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


  // These values were generated using calibrate_mag.js - you will want to create your own.
  var MAG_CALIBRATION = {
      min: { x: -106.171875, y: -56.8125, z: -14.828125 },
      max: { x: 71.9609375, y: 117.17578125, z: 164.25 },
      offset: { x: -17.10546875, y: 30.181640625, z: 74.7109375 },
      scale: {
          x: 1.491020130696022,
          y: 1.5265373476123123,
          z: 1.483149376145188
      }
  };

    // These values were generated using calibrate_gyro.js - you will want to create your own.
    // NOTE: These are temperature dependent.
    var GYRO_OFFSET = {
        x: -0.12351145038167934,
        y: -0.5848702290076341,
        z: 2.429755725190839
    }

    // These values were generated using calibrate_accel.js - you will want to create your own.
    var ACCEL_CALIBRATION = {
        offset: {
            x: 0.09908772786458334,
            y: 0,
            z: 0.24810709635416667
        },
        scale: {
            x: [ -3.9217220052083333, 4.1018294270833335 ],
            y: [ -4, 4 ],
            z: [ -3.790582682291667, 4.328382161458333 ]
        }
    }


  var mpu = new Mpu9250Driver({
      i2c: i2c1,
      // i2c path (default is '/dev/i2c-1')
      // device: '/dev/i2c-2',
      // Enable/Disable debug mode (default false)
      DEBUG: true,
      // Set the Gyroscope sensitivity (default 0), where:
      //      0 => 250 degrees / second
      //      1 => 500 degrees / second
      //      2 => 1000 degrees / second
      //      3 => 2000 degrees / second
      GYRO_FS: 0,
      // Set the Accelerometer sensitivity (default 2), where:
      //      0 => +/- 2 g
      //      1 => +/- 4 g
      //      2 => +/- 8 g
      //      3 => +/- 16 g
      ACCEL_FS: 0,
      scaleValues: true,
      UpMagneto: true,
      magCalibration: MAG_CALIBRATION,
      gyroBiasOffset: GYRO_OFFSET,
      accelCalibration: ACCEL_CALIBRATION
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
  



  var timer = 0;
  const kalmanX = new mpu.Kalman_filter();
  const kalmanY = new mpu.Kalman_filter();
  const ctrRoll = new PIDController({
    k_p: 0.25,
    k_i: 0.01,
    k_d: 0.01
  });
  const ctrPitch = new PIDController({
    k_p: 0.25,
    k_i: 0.01,
    k_d: 0.01
  });
  if (mpu.initialize() !== false) {
	console.log('MPU VALUE : ', mpu.getMotion9());
	console.log('Temperature : ' + mpu.getTemperatureCelsius());
	var values = mpu.getMotion9();
	var pitch = mpu.getPitch(values);
	var roll = mpu.getRoll(values);
	var yaw = mpu.getYaw(values);
	console.log('pitch value : ', pitch);
	console.log('roll value : ', roll);
	console.log('yaw value : ', yaw);
	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);

    ctrRoll.setTarget(0);
    ctrPitch.setTarget(0);




	var micros = function() {
		return new Date().getTime();
	};
	var dt = 0;

	timer = micros();

	var interval;

	var kalAngleX = 0,
		kalAngleY = 0,
		kalAngleZ = 0,
		gyroXangle = roll,
		gyroYangle = pitch,
		gyroZangle = yaw,
		gyroXrate = 0,
		gyroYrate = 0,
		gyroZrate = 0,
		compAngleX = roll,
		compAngleY = pitch,
		compAngleZ = yaw;


    setInterval(flightControl, 50);
    console.log('\n   Time     Accel.x  Accel.y  Accel.z  Gyro.x   Gyro.y   Gyro.z   Mag.x   Mag.y   Mag.z    Temp(°C) head(°)     Roll    Pitch');
    setInterval(getSensorData, 20);
  }




});
