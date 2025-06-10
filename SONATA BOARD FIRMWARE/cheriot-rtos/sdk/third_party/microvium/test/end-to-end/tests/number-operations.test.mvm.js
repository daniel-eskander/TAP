/*---
description: >
  Tests various operations that should classify as vm_TeNumberOp operations
runExportedFunction: 0
assertionCount: 153
---*/
vmExport(0, run);

function run() {
  testNegate();
  testUnaryPlus();
  testAddition();
  testSubtraction();
  testMultiplication();
  testDivision();
  testLessThan();
  testGreaterThan();
  testRemainder();
  testPower();
  testIncrDecr();
  testStringToInt();
  testIntToString();
  testFloatToString();
}

function testNegate() {
  assertEqual(-1, 2 - 3);
  assertEqual(-Infinity, -1.1 / 0);
  assertEqual(-(-0x80000000), overflowChecks ? 0x80000000 : -0x80000000);
}

function testUnaryPlus() {
  assertEqual(+(1 + 1), 2);
  assertEqual(+(1.1 + 2), 3.1);
}

function testAddition() {
  assertEqual(3 + 2, 5);
  assertEqual(3_000 + 2_000, 5_000); // out of 8 bit range
  assertEqual(3_000 + 3_500, 6_500); // 12 bit addition (should not overflow, but should take the fast path still)
  assertEqual(6_000 + 500, 6_500); // 13 bit addition. Does not technically overflow but should take the slow path
  assertEqual(500 + 6_500, 7_000); // 13 bit addition. Does not technically overflow but should take the slow path
  assertEqual(10_000 + 8_000, 18_000); // out of 14 bit signed range
  assertEqual(80_000 + 70_000, 150_000); // out of 16 bit range
  assertEqual(7_500 + 7_000, 14_500); // overflow 14-bit range
  assertEqual(2_000_000_000 + 2_000_000_000, overflowChecks ? 4_000_000_000 : -294967296); // overflow signed 32-bit range
  assertEqual(-1.5 + 1, -0.5);
  assertEqual(-2 + 0.5, -1.5);
  assertEqual(-5_000_000_000 + 4_999_999_000, -1_000);
}

function testSubtraction() {
  assertEqual(3 - 2, 1);
  assertEqual(3_000 - 2_000, 1_000); // out of 8 bit range
  assertEqual(10_000 - 8_000, 2_000); // out of 14 bit signed range
  assertEqual(80_000 - 70_000, 10_000); // out of 16 bit range
  assertEqual(-7_500 - 7_000, -14_500); // underflow 14-bit range
  assertEqual(-2_000_000_000 - 2_000_000_000, overflowChecks ? -4_000_000_000 : 294967296); // underflow signed 32-bit range
  assertEqual(1.5 - 1, 0.5);
  assertEqual(2 - 0.5, 1.5);
  assertEqual(5_000_000_000 - 4_999_999_000, 1_000);
}

function testMultiplication() {
  assertEqual(5 * 6, 30);
  assertEqual(5.5 * 6, 33);
  assertEqual((-5) * (-6), 30);
  assertEqual(5 * (-6), -30);

  assertEqual(5_000 * 5_000, 25_000_000); // Overflow 14-bit range
  assertEqual(17_000 * 2, 34_000);
  assertEqual(5_000_000 * 5_000_000, overflowChecks ? 25_000_000_000_000 : -1004630016); // Overflow 32-bit range
  assertEqual(25_000_000_000_000 * 1, 25_000_000_000_000);
}

function testDivision() {
  // Floating point division (the normal)
  assertEqual(6 / 3, 2);
  assertEqual(7 / 2, 3.5);
  assertEqual(8.5 / 2.5, 3.4);
  assertEqual(8 / 0, Infinity);
  assertEqual(8 / -0, -Infinity);
  // Without overflow checks enabled, the negation of integer zero is integer zero
  assertEqual(8 / -(1-1), overflowChecks ? -Infinity : Infinity);
  assertEqual(-8 / 0, -Infinity);
  assertEqual(-8 / -0, Infinity);
  assert(Number.isNaN(Infinity / Infinity));

  // Integer division
  assertEqual(6 / 3 | 0, 2);
  assertEqual(7 / 2 | 0, 3);
  assertEqual((8.5 / 2.5) | 0, 3);
  assertEqual(-6 / -3 | 0, 2);
  assertEqual(-7 / -2 | 0, 3);
  assertEqual((-8.5 / -2.5) | 0, 3);
  assertEqual(-6 / 3 | 0, -2);
  assertEqual(-7 / 2 | 0, -3);
  assertEqual((-8.5 / 2.5) | 0, -3);
  assertEqual(8 / 0 | 0, 0);
  assertEqual(8 / -0 | 0, 0);
  assertEqual(-8 / 0 | 0, 0);
  assertEqual(-8 / -0 | 0, 0);
  assertEqual(NaN / NaN | 0, 0);
  assertEqual(Infinity / Infinity | 0, 0);
}

function testLessThan() {
  // Integers
  assertEqual(1 < 2, true);
  assertEqual(2 < 1, false);
  assertEqual(2 < 2, false);

  // Negative integers
  assertEqual(-1 < -2, false);
  assertEqual(-2 < -1, true);
  assertEqual(-2 < -2, false);

  // Floating point
  assertEqual(1.1 < 2.1, true);
  assertEqual(2.1 < 1.1, false);
  assertEqual(2.1 < 2.1, false);

  // Integers
  assertEqual(1 <= 2, true);
  assertEqual(2 <= 1, false);
  assertEqual(2 <= 2, true);

  // Negative integers
  assertEqual(-1 <= -2, false);
  assertEqual(-2 <= -1, true);
  assertEqual(-2 <= -2, true);

  // Floating point
  assertEqual(1.1 <= 2.1, true);
  assertEqual(2.1 <= 1.1, false);
  assertEqual(2.1 <= 2.1, true);
}

function testGreaterThan() {
  // Integers
  assertEqual(1 > 2, false);
  assertEqual(2 > 1, true);
  assertEqual(2 > 2, false);

  // Negative integers
  assertEqual(-1 > -2, true);
  assertEqual(-2 > -1, false);
  assertEqual(-2 > -2, false);

  // Floating point
  assertEqual(1.1 > 2.1, false);
  assertEqual(2.1 > 1.1, true);
  assertEqual(2.1 > 2.1, false);

  // Integers
  assertEqual(1 >= 2, false);
  assertEqual(2 >= 1, true);
  assertEqual(2 >= 2, true);

  // Negative integers
  assertEqual(-1 >= -2, true);
  assertEqual(-2 >= -1, false);
  assertEqual(-2 >= -2, true);

  // Floating point
  assertEqual(1.1 >= 2.1, false);
  assertEqual(2.1 >= 1.1, true);
  assertEqual(2.1 >= 2.1, true);
}

function testRemainder() {
  assertEqual(2 % 1, 0);
  assertEqual(5 % 2, 1);
  assertEqual(550 % 100, 50);

  assertEqual(-8 % 3, -2);
  assertEqual(8 % -3, 2);
  assertEqual(-8 % -3, -2);

  assertEqual(2.25 % 1, 0.25);
  assertEqual(5.25 % 2, 1.25);
  assertEqual(550.25 % 100, 50.25);

  assertEqual(-7.25 % 4, -3.25);
  assertEqual(7.25 % -4, 3.25);
  assertEqual(-7.25 % -4, -3.25);

  assert(Number.isNaN(5 % 0));
  assert(Number.isNaN(5.1 % 0));
}

function testPower() {
  assertEqual(2 ** 3, 8);
  assertEqual(2 ** 0, 1);
  assertEqual(2.5 ** 1, 2.5);
  assert(Number.isNaN(1 ** Infinity));
}

function testIncrDecr() {
  let x = 1;
  assertEqual(x++, 1);
  assertEqual(x, 2);
  assertEqual(++x, 3);
  assertEqual(x, 3);
  assertEqual(x--, 3);
  assertEqual(x, 2);
  assertEqual(--x, 1);
  assertEqual(x, 1);

  x = 1.5;
  assertEqual(++x, 2.5);
  assertEqual(--x, 1.5);
}

function testStringToInt() {
  assert(Number.isNaN(+"x"));
  assert(Number.isNaN(+"length"));
  assert(Number.isNaN(+"__proto__"));
  assert(Number.isNaN(+"1a"));
  assert(Number.isNaN(+"1.1.1"));
  assert(Number.isNaN(+"123456789123456789.1.1"));
  assert(Number.isNaN(+"123\0"));

  // Empty string
  assertEqual(+"", 0);

  // Whitespace
  assertEqual(+"  ", 0);

  // Small integers
  assertEqual(+"123", 123);
  assertEqual(+"-123", -123);

  // Leading and trailing whitespace
  assertEqual(+"  123   ", 123);
  assertEqual(+"  -123   ", -123);

  // Int32
  assertEqual(+"12345678", 12345678);
  assertEqual(+"-12345678", -12345678);

  // Multiply
  assertEqual(1 * "123", 123);
}

function testIntToString() {
  assertEqual('' + 0, '0');
  assertEqual('' + 1, '1');
  assertEqual('' + -1, '-1');
  assertEqual('' + (0x7FFFFFFF), '2147483647');
  assertEqual('' + (-0x80000000), '-2147483648');
}

function testFloatToString() {
  assertEqual('' + NaN, 'NaN');
  assertEqual('' + Infinity, 'Infinity');
  assertEqual('' + (-Infinity), '-Infinity');
  assertEqual('' + (-0.0), '0');
  assertEqual('' + 0.1, '0.1');
  assertEqual('' + (-0.1), '-0.1');
  assertEqual('' + 1e30, '1e+30');
  assertEqual('' + (-1e30), '-1e+30');
  assertEqual('' + 1e-30, '1e-30');
  assertEqual('' + (-1e-30), '-1e-30');
}