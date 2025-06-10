/*---
description: >
  Exports a function with ID 42 that prints "hello-world".
runExportedFunction: 42
expectedPrintout: Hello, World!
---*/
function run() {
  print('Hello, World!');
}

vmExport(42, run);