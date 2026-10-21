// Glass — bundle the page into one self-contained HTML file: the ES modules
// under src/ (each in its own scope), the core's loader, avd.wasm (base64) and
// the glass-profile aircraft.
// SPDX-License-Identifier: MIT
// Usage: node scripts/bundle.mjs [dist-dir]   (after scripts/build_core.sh) -> dist/glass.html
import { readFile, writeFile, readdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const dist = path.resolve(process.argv[2] ?? path.join(root, 'dist'));
const core = path.join(dist, 'core');

// A minimal module linker for this repository's own sources: named imports
// and exports only (no default exports, no re-exports, no dynamic import).
const IMPORT = /^import\s*\{([^}]*)\}\s*from\s*'([^']+)';?\s*$/gm;
const EXPORT_DECL = /^export\s+(?:async\s+)?(?:const|let|class|function\*?)\s+([A-Za-z_$][\w$]*)/gm;
async function link(entry) {
  const order = [], seen = new Map();
  const visit = async (file) => {
    if (seen.has(file)) return seen.get(file);
    const id = '__m' + seen.size;
    seen.set(file, id);
    const src = await readFile(file, 'utf8');
    const deps = [];
    for (const [, names, spec] of src.matchAll(IMPORT)) {
      const dep = await visit(path.resolve(path.dirname(file), spec));
      deps.push(`const { ${names.split(',').map((n) => n.trim()).filter(Boolean).map((n) => n.replace(/\s+as\s+/, ': ')).join(', ')} } = ${dep};`);
    }
    if (/^export\s+default|^export\s*\{|^export\s*\*|import\(/m.test(src)) throw new Error(`${file}: only named declarations may be exported`);
    const exported = [...src.matchAll(EXPORT_DECL)].map((m) => m[1]);
    const body = src.replace(IMPORT, '').replace(/^export\s+/gm, '');
    order.push(`// ${path.relative(root, file)}\nconst ${id} = (() => {\n${deps.join('\n')}\n${body}\nreturn { ${exported.join(', ')} };\n})();`);
    return id;
  };
  const top = await visit(entry);
  return { code: order.join('\n'), top };
}

const [html, wasm, loader, files] = await Promise.all([
  readFile(path.join(root, 'src', 'glass.html'), 'utf8'),
  readFile(path.join(core, 'avd.wasm')),
  readFile(path.join(core, 'avd.mjs'), 'utf8'),
  readdir(core),
]);
const aircraft = {};
for (const f of files.filter((f) => f.endsWith('.json')).sort()) {
  const text = await readFile(path.join(core, f), 'utf8');
  if (JSON.parse(text).panel?.profile === 'business_jet_glass') aircraft[f.replace(/\.json$/, '')] = text;
}
if (!Object.keys(aircraft).length) throw new Error(`no glass-profile aircraft in ${core}`);
const app = await link(path.join(root, 'src', 'app.js'));
const coreLoader = loader.replace(/^export\s+default\s+/m, '').replace(/^export\s+/gm, '');
const safe = (s) => s.replaceAll('</script', '<\\/script');
const scripts = [
  `<script type="application/json" id="glass-aircraft">${safe(JSON.stringify(aircraft))}</script>`,
  `<script type="application/octet-stream" id="glass-wasm">${wasm.toString('base64')}</script>`,
  `<script type="module">
${safe(coreLoader)}
${safe(app.code)}
const bytes = Uint8Array.from(atob(document.getElementById('glass-wasm').textContent), (c) => c.charCodeAt(0));
${app.top}.boot({ createAvdModule, wasmBytes: bytes, aircraft: JSON.parse(document.getElementById('glass-aircraft').textContent) });
</script>`,
].join('\n');
const out = path.join(dist, 'glass.html');
const page = html.replace('<!--GLASS_SCRIPTS-->', scripts);
await writeFile(out, page);
console.log(`${out}: ${(Buffer.byteLength(page) / 1024).toFixed(0)} KiB, aircraft ${Object.keys(aircraft).join(', ')}`);
