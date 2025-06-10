/*---
description: >
  Tests various operations that should classify as vm_TeBitwiseOp operations
runExportedFunction: 0
assertionCount: 25
---*/
vmExport(0, run);

function run() {
  assertEqual(3 << 0, 3);
  assertEqual(3 << 2, 12);
  // Wrap around
  assertEqual(3 << 34, 12);
  assertEqual(3 << 32, 3);
  assertEqual(2 << 31, 0);

  // Left shift negative numbers
  assertEqual(-2 << 2, -8);

  assertEqual(8 >> 1, 4);
  assertEqual(8 >>> 1, 4);
  assertEqual(-8 >> 1, -4);
  assertEqual(-8 >>> 1, 0x7ffffffc);

  // Shifting by zero
  assertEqual(4 >> 0, 4);
  assertEqual(4 >>> 0, 4);
  assertEqual(-4 >> 0, -4);
  assertEqual(-4 >>> 0, overflowChecks ? 0xfffffffc : -4);

  // Shifting by negative numbers
  assertEqual(8 >> -30, 8 >> 2);
  assertEqual(8 >>> -30, 8 >>> 2);
  assertEqual(8 << -30, 8 << 2);

  // Other operators. These are much simpler
  assertEqual(3 | 6, 7);
  assertEqual(3 & 6, 2);
  assertEqual(3 ^ 6, 5);
  assertEqual(~3, -4);
  assertEqual(-3 | -6, -1);
  assertEqual(-3 & -6, -8);
  assertEqual(-3 ^ -6, 7);
  assertEqual(~-3, 2);
}

