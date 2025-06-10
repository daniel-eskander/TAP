import * as fs from 'fs-extra';

export function htmlPageTemplate(contents: string) {
  return `
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      </head>
      <body>
      ${contents}
      <style>${fs.readFileSync('./lib/snapshot-binary-html-styles.css', 'utf8')}</style>
      </body>
      </html>`
    }
    // <script>setInterval(() => location.reload(), 1000);</script>