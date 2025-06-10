/*---
description: >
  Exports a function that tests both branches of an if-else statement
runExportedFunction: 0
expectedPrintout: |
  #1: This is the alternate
  #2: This is the consequent
---*/
function run() {
  if (false) {
    print("#1: This is the consequent");
  } else {
    print("#1: This is the alternate");
  }

  if (true) {
    print("#2: This is the consequent");
  } else {
    print("#2: This is the alternate");
  }
}

vmExport(0, run);