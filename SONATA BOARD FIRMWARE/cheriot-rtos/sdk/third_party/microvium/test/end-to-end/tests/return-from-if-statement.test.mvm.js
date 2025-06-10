/*---
runExportedFunction: 0
expectedPrintout: |
  Consequent path
  Alternate path
---*/
vmExport(0, run);

function run() {
  assertEqual(foo(true), 1);
  assertEqual(foo(false), 2);
}

function foo(b) {
  if (b) {
    console.log('Consequent path')
    return 1;
  } else {
    console.log('Alternate path')
    return 2;
  }
  console.log("Shouldn't get here");
  return 3;
}
