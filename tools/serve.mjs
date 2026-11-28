#!/usr/bin/env node
// Minimal static file server for the repo root (no dependencies).
// Usage:  node tools/serve.mjs [port]   (default 8000; or set PORT)
// Also importable: serve(port, {quiet}) resolves to the listening server (port 0 = any free port).
import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { dirname, extname, join, normalize, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { argv } from 'node:process';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const types = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
  '.json': 'application/json', '.md': 'text/markdown; charset=utf-8', '.svg': 'image/svg+xml',
  '.png': 'image/png', '.ico': 'image/x-icon',
};

export const serve = (port, { quiet = false } = {}) => new Promise(ok => { const srv = createServer(async (req, res) => {
  let path;
  try { path = decodeURIComponent(new URL(req.url, 'http://x').pathname); }
  catch { res.writeHead(400).end('Bad request'); return; }
  let file = normalize(join(ROOT, path));
  if (file !== ROOT && !file.startsWith(ROOT + sep)) { res.writeHead(403).end('Forbidden'); return; }
  try {
    if ((await stat(file)).isDirectory()) file = join(file, 'index.html');
    const body = await readFile(file);
    res.writeHead(200, { 'Content-Type': types[extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-store' });
    res.end(body);
  } catch { res.writeHead(404).end('Not found'); }
  if (!quiet) console.log(req.method, req.url, res.statusCode);
}); srv.listen(port, '127.0.0.1', () => ok(srv)); });

if (argv[1] && resolve(argv[1]) === fileURLToPath(import.meta.url)) {
  const port = +(argv[2] || process.env.PORT || 8000);
  serve(port).then(() => console.log(`Serving ${ROOT} at http://localhost:${port}/`));
}
