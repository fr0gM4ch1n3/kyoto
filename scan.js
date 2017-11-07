'use strict';


var fs = require('fs'),
  i2c = require('i2c-bus'),
  i2c1 = i2c.openSync(1);

var EBUSY = 16; /* Device or resource busy */

function scan(first, last) {
  var found = [];
  var addr;

  for (addr = 0; addr <= 127; addr += 1) {
    if (addr < first || addr > last) {
    } else {
      try {
        i2c1.writeQuickSync(addr, 0);
        found.push(addr);
      } catch (e) {}
    }
  }

  return found;
}

if (!i2c1.i2cFuncsSync().smbusQuick) {
  console.err(new Error('Can\'t use SMBus Quick Write command on this bus'));
} else {
  var addr = scan(0x3, 0x77);
  console.log('found:', addr.join());
}

