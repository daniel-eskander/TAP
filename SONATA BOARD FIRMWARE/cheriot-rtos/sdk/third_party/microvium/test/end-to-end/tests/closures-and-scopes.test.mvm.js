/*---
runExportedFunction: 0
assertionCount: 35
---*/

vmExport(0, run);

function run() {
  basics();
  nestedLexicalScopes();
  differentVariableTypes();
  closureOperations();
  thisCapturing();
  rootScope();
}

function basics() {
  const incrementor1 = makeIncrementorA();
  const incrementor2 = makeIncrementorA();
  assertEqual(incrementor1(), 1);
  assertEqual(incrementor1(), 2);
  assertEqual(incrementor2(), 1);
  assertEqual(incrementor2(), 2);

  const incrementor3 = makeIncrementorA2();
  assertEqual(incrementor3(), 1);
  assertEqual(incrementor3(), 2);

  const incrementor4 = makeIncrementorB();
  assertEqual(incrementor4(), 1);
  assertEqual(incrementor4(), 2);

  const incrementorC = makeIncrementorC();
  assertEqual(incrementorC()(), 1);
  assertEqual(incrementorC()(), 2);
}

function makeIncrementorA() {
  let x = 0;
  // Arrow function
  return () => ++x;
}

function makeIncrementorA2() {
  let x = 0;
  // Function expression (note that we do not support named function expressions, yet)
  return function() { return ++x; }
}

function makeIncrementorB() {
  let x = 0;
  return increment;
  // Function declaration
  function increment() {
    return ++x;
  }
}

// Double-nested functions
function makeIncrementorC() {
  let x = 0;
  // The inner-most function doesn't access its direct outer scope. It accesses
  // its grandparent scope.
  return () => () => ++x;
}

function nestedLexicalScopes() {
  let x = 1;
  let f1;
  let f2;
  let f3;
  {
    let x = 50;
    f1 = () => x++;
  }
  {
    let x = 100;
    f2 = () => x++;
  }
  f3 = () => x++;
  assertEqual(f1(), 50);
  assertEqual(f1(), 51);
  assertEqual(f2(), 100);
  assertEqual(f2(), 101);
  assertEqual(f3(), 1);
  assertEqual(f3(), 2);
}

function differentVariableTypes() {
  const foo = (
    x1, // constant parameter
    x2, // unused parameter
    x3, // mutated parameter
  ) => {
    const x4 = 5; // const
    var x5 = 6; // var
    let x6 = 7; // let
    // New block scope
    {
      const x7 = 5; // const
      var x8 = 6; // var
      let x9 = 7; // let
      return /* bar */ (
        x10, // local constant parameter
        x11, // local unused parameter
        x12, // local mutated parameter
      ) => {
        const x13 = 5; // const
        var x14 = 6; // var
        let x15 = 7; // let
        // New block scope
        {
          const x16 = 5; // const
          var x17 = 6; // var
          let x18 = 7; // let
          return /* baz */ (
            x19, // local constant parameter
            x20, // local unused parameter
            x21, // local mutated parameter
          ) =>
            0
              + x1 * 2
              + x3++ * 3
              + x4 * 5
              + x5++ * 7
              + x6++ * 11
              + x7 * 13
              + x8++ * 17
              + x9++ * 19
              + x10 * 23
              + x12++ * 29
              + x13 * 31
              + x14++ * 37
              + x15++ * 41
              + x16 * 43
              + x17++ * 47
              + x18++ * 53
              + x19
              + x21++ * 59
        }
      }
    }
  };
  const f1 = foo(1, 2, 3);
  const f2 = f1(4, 5, 6);
  const f3 = f1(7, 8, 9);
  assertEqual(f2(10, 11, 12), 2971);
  assertEqual(f2(13, 14, 15), 3415);
  assertEqual(f2(16, 17, 19), 3918);
  assertEqual(f3(19, 20, 21), 3838);
  assertEqual(f3(22, 23, 24), 4282);
  assertEqual(f3(25, 26, 27), 4726);
}

function closureOperations() {
  const f1 = () => {};
  const f2 = () => {};

  // Check equality operator
  assertEqual(f1 === f1, true);
  assertEqual(f1 === f2, false);
  assertEqual(f1 !== f1, false);
  assertEqual(f1 !== f2, true);

  // Other operations
  assertEqual(f1 | 0, 0);
  assert(Number.isNaN(+f1));

  // Note: we don't support conversion to string at this time
}

function thisCapturing() {
  function printMe1() {
    return this.me;
  }

  function getObj2() {
    return {
      printMe1,
      printMe2: () => /*obj1*/ this.me,
      printMe3: function() { return /*obj2*/ this.me },
      me: 'obj2',
    }
  }

  const obj1 = {
    printMe1,
    getObj2,
    me: 'obj1'
  }

  const obj2 = obj1.getObj2();

  assertEqual(obj1.printMe1(), 'obj1');
  assertEqual(obj2.printMe1(), 'obj2');
  assertEqual(obj2.printMe2(), 'obj1');
  assertEqual(obj2.printMe3(), 'obj2');
}

let f;
var v1 = 1;
let v2 = 2;
// A block
{
  let v3 = 3;
  var v4 = 4;
  // Variables at the root level are compiled differently to variables at the
  // block level, so this is just checking that both work ok. `v1` and `v2` will
  // be promoted to global variables because they're accessed by a nested
  // function. `v4` will also be a global, because it's hoisted into the same
  // scope. `v3` will be a closure slot in the context of the entry function.
  f = () =>
    1000 * v1++ +
    100 * v2++ +
    10 * v3++ +
    1 * v4++;
}

function rootScope() {
  assertEqual(f(), 1234);
  assertEqual(f(), 2345);
  assertEqual(f(), 3456);
}


// TODO: TDZ
// TODO: Closures capturing block-scoped variables at the root level
// TODO: function declarations nested in blocks
// TODO: all the same "closure" tests but for module-scoped variables
// TODO: local variables in module entry function

// TODO: test that closure state serializes in the snapshot