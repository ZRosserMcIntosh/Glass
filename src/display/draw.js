// Glass — shared drawing helpers and the display palette.
// SPDX-License-Identifier: MIT
//
// The displays are black in either page theme, as real flight displays are;
// the colours are the conventional ones (white scales, green active, cyan
// selected, magenta FMS/GPS, amber caution, red warning).

export const P = {
  bg: '#000000', white: '#f2f2f2', grey: '#8a8f98', dim: '#3a3f48', green: '#2ee65a', cyan: '#33d6ff',
  magenta: '#ff52ff', amber: '#ffb31a', red: '#ff3b30', sky: '#2f73c9', earth: '#7a4f24', tape: 'rgba(80,88,100,0.55)',
};
export const FONT = '"Roboto Condensed", "Arial Narrow", "Helvetica Neue", Arial, sans-serif';
export const MONO = '"Roboto Mono", ui-monospace, "SFMono-Regular", Menlo, monospace';
export const D2R = Math.PI / 180;

export const colourOf = (name) => ({ white: P.white, amber: P.amber, red: P.red, green: P.green, cyan: P.cyan, magenta: P.magenta, off: P.dim }[name] ?? P.white);

// Size a canvas for its CSS box and the device pixel ratio, and map a logical
// w x h drawing space onto it (letterboxed). Returns the 2D context.
export function prepare(canvas, w, h) {
  const dpr = Math.min(globalThis.devicePixelRatio || 1, 3);
  const cw = Math.max(1, Math.round(canvas.clientWidth * dpr)), ch = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== cw || canvas.height !== ch) { canvas.width = cw; canvas.height = ch; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = P.bg;
  ctx.fillRect(0, 0, cw, ch);
  const s = Math.min(cw / w, ch / h);
  ctx.setTransform(s, 0, 0, s, (cw - w * s) / 2, (ch - h * s) / 2);
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  return ctx;
}

export function text(ctx, s, x, y, { size = 20, colour = P.white, align = 'center', base = 'middle', font = FONT, weight = 500 } = {}) {
  ctx.font = `${weight} ${size}px ${font}`;
  ctx.fillStyle = colour;
  ctx.textAlign = align;
  ctx.textBaseline = base;
  ctx.fillText(s, x, y);
}

export function line(ctx, x0, y0, x1, y1, colour = P.white, width = 2) {
  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
}

export function box(ctx, x, y, w, h, { stroke = P.white, fill = null, width = 2 } = {}) {
  if (fill) { ctx.fillStyle = fill; ctx.fillRect(x, y, w, h); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.strokeRect(x, y, w, h); }
}

// Alternating stripes (barber pole / checkerboard bands) in a rectangle.
export function stripes(ctx, x, y, w, h, a, b, period = 14) {
  ctx.save();
  ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip();
  ctx.fillStyle = b; ctx.fillRect(x, y, w, h);
  ctx.fillStyle = a;
  for (let yy = y - w; yy < y + h + w; yy += period) {
    ctx.beginPath(); ctx.moveTo(x, yy); ctx.lineTo(x + w, yy - w * 0.6); ctx.lineTo(x + w, yy - w * 0.6 + period / 2); ctx.lineTo(x, yy + period / 2); ctx.fill();
  }
  ctx.restore();
}

export const fmt = (v, digits = 0) => (Number.isFinite(v) ? v.toFixed(digits) : '---');
export const pad3 = (deg) => String(Math.round(((deg % 360) + 360) % 360) || 360).padStart(3, '0');
