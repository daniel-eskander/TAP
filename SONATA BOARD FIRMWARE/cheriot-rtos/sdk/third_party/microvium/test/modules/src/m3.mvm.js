import * as fs from 'fs';
print('importing m3');
print('Writing to a file');
fs.writeFileSync('./test/modules/output.txt', 'This is some output');