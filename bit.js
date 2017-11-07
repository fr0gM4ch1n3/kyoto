




var b = Buffer.from([0xB5]); // 1011 0101




var rB = function (b, o, l, v) {
    b[0] &= ~((0xFF >> (o + l + 1)) << l);
    b[0] |= (v << (o - 1));

    return b;
}


console.log( rB(b, 3, 2, 0xffff).toString('hex') ); // 1011 1101