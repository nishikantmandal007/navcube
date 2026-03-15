/**
 * pyside-navicube — interactive browser demo
 *
 * Faithful HTML5 Canvas port of NaviCubeOverlay.
 * Same geometry (chamfer, edge/corner/main faces), same orthographic
 * projection, same dark-theme palette, same SLERP snap animation,
 * same quad-to-quad label mapping — no Three.js, no WebGL.
 */
(function () {
  'use strict';

  // ── Math helpers ──────────────────────────────────────────────────────────

  function norm(v) {
    const n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return n > 1e-10 ? [v[0]/n, v[1]/n, v[2]/n] : v.slice();
  }
  function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
  function cross(a, b) {
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  }
  function add(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
  function sub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
  function scl(v, s) { return [v[0]*s, v[1]*s, v[2]*s]; }

  /** Rodrigues rotation of v around unit-axis by angle radians. */
  function rodrigues(v, axis, angle) {
    const a = norm(axis), c = Math.cos(angle), s = Math.sin(angle);
    const d = dot(a, v), cx = cross(a, v);
    return [v[0]*c+cx[0]*s+a[0]*d*(1-c),
            v[1]*c+cx[1]*s+a[1]*d*(1-c),
            v[2]*c+cx[2]*s+a[2]*d*(1-c)];
  }

  /** Vector SLERP — handles antipodal (dot≈−1) case. */
  function vslerp(v0, v1, t) {
    v0 = norm(v0); v1 = norm(v1);
    let d = Math.max(-1, Math.min(1, dot(v0, v1)));
    if (d > 0.9999) return norm(add(v0, scl(sub(v1, v0), t)));
    if (d < -0.9999) {
      const cand = Math.abs(v0[0]) < 0.9 ? [1,0,0] : [0,1,0];
      const mid = norm(cross(v0, cand));
      return t < 0.5 ? vslerp(v0, mid, t*2) : vslerp(mid, v1, (t-0.5)*2);
    }
    const om = Math.acos(d), so = Math.sin(om);
    return [(Math.sin((1-t)*om)/so)*v0[0]+(Math.sin(t*om)/so)*v1[0],
            (Math.sin((1-t)*om)/so)*v0[1]+(Math.sin(t*om)/so)*v1[1],
            (Math.sin((1-t)*om)/so)*v0[2]+(Math.sin(t*om)/so)*v1[2]];
  }

  function smoothstep(t) {
    t = Math.max(0, Math.min(1, t));
    return t*t*t*(t*(t*6-15)+10);
  }

  /** Returns { r, u } — right and up vectors for the camera basis. */
  function cameraBasis(d, u) {
    d = norm(d); u = norm(u);
    let r = cross(d, u);
    if (r[0]*r[0]+r[1]*r[1]+r[2]*r[2] < 1e-12)
      r = cross(d, Math.abs(d[0]) < 0.9 ? [1,0,0] : [0,1,0]);
    r = norm(r);
    u = norm(cross(r, d));
    return { r, u };
  }

  // ── Geometry (exact Python port) ─────────────────────────────────────────

  const CHAMFER = 0.12;

  function buildFaces() {
    const faces = {};

    function addFace(name, xv, zv, type, label) {
      const yv = cross(xv, scl(zv, -1));
      const C = CHAMFER;
      let pts, labelPts = null;

      if (type === 'corner') {
        const xc = scl(xv, C), yc = scl(yv, C), zc = scl(zv, 1-2*C);
        pts = [
          sub(zc, scl(xv, 2*C)),
          sub(sub(zc, xc), yc),
          sub(add(zc, xc), yc),
          add(zc, scl(xv, 2*C)),
          add(add(zc, xc), yc),
          add(sub(zc, xc), yc),
        ];
      } else if (type === 'edge') {
        const x4 = scl(xv, 1-C*4), ye = scl(yv, C), ze = scl(zv, 1-C);
        pts = [sub(sub(ze,x4),ye), sub(add(ze,x4),ye),
               add(add(ze,x4),ye), add(sub(ze,x4),ye)];
      } else {  // main
        const x2=scl(xv,1-C*2), y2=scl(yv,1-C*2),
              x4=scl(xv,1-C*4), y4=scl(yv,1-C*4);
        pts = [
          sub(sub(zv,x2),y4), sub(sub(zv,x4),y2), sub(add(zv,x4),y2), sub(add(zv,x2),y4),
          add(add(zv,x2),y4), add(add(zv,x4),y2), add(sub(zv,x4),y2), add(sub(zv,x2),y4),
        ];
        labelPts = [
          sub(sub(zv,x2),y2), sub(add(zv,x2),y2),
          add(add(zv,x2),y2), add(sub(zv,x2),y2),
        ];
      }

      const sum = pts.reduce((a,p) => add(a,p), [0,0,0]);
      const ctr = scl(sum, 1/pts.length);
      faces[name] = { pts, normal: norm(ctr), ctr, label: label||null, type, labelPts };
    }

    // 6 main faces
    addFace('TOP',    [1,0,0],  [0,0,1],  'main', 'TOP');
    addFace('FRONT',  [1,0,0],  [0,-1,0], 'main', 'FRONT');
    addFace('LEFT',   [0,-1,0], [-1,0,0], 'main', 'LEFT');
    addFace('BACK',   [-1,0,0], [0,1,0],  'main', 'BACK');
    addFace('RIGHT',  [0,1,0],  [1,0,0],  'main', 'RIGHT');
    addFace('BOTTOM', [1,0,0],  [0,0,-1], 'main', 'BOTTOM');
    // 8 corners
    addFace('FTR',[-1,-1,0],[1,-1,1],'corner'); addFace('FTL',[-1,1,0],[-1,-1,1],'corner');
    addFace('FBR',[1,1,0],[1,-1,-1],'corner');  addFace('FBL',[1,-1,0],[-1,-1,-1],'corner');
    addFace('RTR',[1,-1,0],[1,1,1],'corner');   addFace('RTL',[1,1,0],[-1,1,1],'corner');
    addFace('RBR',[-1,1,0],[1,1,-1],'corner');  addFace('RBL',[-1,-1,0],[-1,1,-1],'corner');
    // 12 edges
    addFace('FRONT_TOP',[1,0,0],[0,-1,1],'edge');  addFace('FRONT_BOTTOM',[1,0,0],[0,-1,-1],'edge');
    addFace('REAR_BOTTOM',[1,0,0],[0,1,-1],'edge'); addFace('REAR_TOP',[1,0,0],[0,1,1],'edge');
    addFace('REAR_RIGHT',[0,0,1],[1,1,0],'edge');   addFace('FRONT_RIGHT',[0,0,1],[1,-1,0],'edge');
    addFace('FRONT_LEFT',[0,0,1],[-1,-1,0],'edge'); addFace('REAR_LEFT',[0,0,1],[-1,1,0],'edge');
    addFace('TOP_LEFT',[0,1,0],[0,1,1],'edge');     addFace('TOP_RIGHT',[0,1,0],[1,0,1],'edge');
    addFace('BOTTOM_RIGHT',[0,1,0],[1,0,-1],'edge'); addFace('BOTTOM_LEFT',[0,1,0],[-1,0,-1],'edge');

    return faces;
  }

  // Snap targets: inward direction (eye→scene) + up vector
  const SNAP = {
    TOP:    { d:[0,0,-1], u:[0,-1,0] },
    BOTTOM: { d:[0,0,1],  u:[0,1,0]  },
    FRONT:  { d:[0,1,0],  u:[0,0,1]  },
    BACK:   { d:[0,-1,0], u:[0,0,1]  },
    LEFT:   { d:[1,0,0],  u:[0,0,1]  },
    RIGHT:  { d:[-1,0,0], u:[0,0,1]  },
  };

  // ── Affine quad-to-quad transform ─────────────────────────────────────────
  // Equivalent to Qt's QTransform::quadToQuad (affine portion).
  // Maps virtual 200×200 canvas → projected screen quad.
  function quadToAffine(src, dst) {
    const s0=src[0], s1=src[1], s2=src[2];
    const d0=dst[0], d1=dst[1], d2=dst[2];
    const sx=s1.x-s0.x, sy=s2.x-s0.x, tx=s1.y-s0.y, ty=s2.y-s0.y;
    const det = sx*ty - sy*tx;
    if (Math.abs(det) < 1e-10) return null;
    const idx=ty/det, idy=-sy/det, itx=-tx/det, ity=sx/det;
    const dx=d1.x-d0.x, dy=d2.x-d0.x, ex=d1.y-d0.y, ey=d2.y-d0.y;
    const a=dx*idx+dy*itx, c=dx*idy+dy*ity;
    const b=ex*idx+ey*itx, dd=ex*idy+ey*ity;
    return { a, b, c, d:dd, e:d0.x-a*s0.x-c*s0.y, f:d0.y-b*s0.x-dd*s0.y };
  }

  // ── Color helpers ─────────────────────────────────────────────────────────

  function shadeHex(hex, s) {
    const r=parseInt(hex.slice(1,3),16),
          g=parseInt(hex.slice(3,5),16),
          b=parseInt(hex.slice(5,7),16);
    return `rgb(${Math.round(r*s)},${Math.round(g*s)},${Math.round(b*s)})`;
  }

  // ── NaviCube Canvas renderer ──────────────────────────────────────────────

  class NaviCube {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx    = canvas.getContext('2d');
      this.faces  = buildFaces();

      // Dark theme palette — exact NaviCubeStyle dark defaults
      this.pal = {
        fMain:  '#9ba0b2',   // (155,160,178)
        fEdge:  '#767a8a',   // (118,122,138)
        fCorn:  '#606371',   // (96,99,113)
        text:   '#eeeeee',
        hover:  'rgba(0,148,255,0.92)',
        hoverTx:'#ffffff',
        border: 'rgba(10,10,12,1)',
        borderS:'rgba(20,20,22,1)',
        shadow: 'rgba(0,0,0,0.31)',
      };

      // Light direction — same as Python default
      this.light = norm([-0.8, -1.0, -1.8]);

      // Camera (inward: eye→scene, Z-up)  — default ISO view
      this.dir = norm([-1, 1, -1]);
      this.up  = [0, 0, 1];

      // Projection scale — calibrated on resize
      this.dpr   = 1;
      this.scale = 38;

      // Snap animation
      this.anim      = false;
      this.animStart = 0;
      this.animDur   = 240;
      this.animD0 = this.dir.slice();
      this.animU0 = this.up.slice();
      this.animD1 = null;
      this.animU1 = null;

      // Interaction
      this.hoveredId    = null;
      this.dragging     = false;
      this.lastMouse    = null;
      this.idleSince    = performance.now();
      this.autoRotating = true;

      this._bindEvents();
      this.resize();
      requestAnimationFrame(t => this._frame(t));
    }

    resize() {
      this.dpr = window.devicePixelRatio || 1;
      const rect = this.canvas.getBoundingClientRect();
      this.canvas.width  = Math.round(rect.width  * this.dpr);
      this.canvas.height = Math.round(rect.height * this.dpr);
      // Scale so cube fills ~35% of the shorter side
      const shorter = Math.min(this.canvas.width, this.canvas.height);
      this.scale = shorter * 0.195;
    }

    get cx() { return this.canvas.width  / 2; }
    get cy() { return this.canvas.height / 2; }

    _proj(pt, basis) {
      return {
        x: this.cx + dot(pt, basis.r) * this.scale,
        y: this.cy - dot(pt, basis.u) * this.scale,
      };
    }

    _visibleFaces(basis) {
      const out = [];
      for (const [name, f] of Object.entries(this.faces)) {
        if (-dot(f.normal, this.dir) <= 0.10) continue;
        out.push({
          name,
          face:  f,
          pts2d: f.pts.map(p => this._proj(p, basis)),
          depth: dot(f.ctr, this.dir),
        });
      }
      out.sort((a, b) => b.depth - a.depth);  // back-to-front
      return out;
    }

    _shade(normal) {
      return 0.55 + 0.45 * Math.max(0, -dot(normal, this.light));
    }

    _poly(ctx, pts) {
      ctx.beginPath();
      ctx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
      ctx.closePath();
    }

    _drawLabel(ctx, face, basis, textCol) {
      if (!face.labelPts) return;
      const lp = face.labelPts.map(p => this._proj(p, basis));

      // Python's direct quad mapping: dst = [lp[3], lp[2], lp[1], lp[0]]
      const dst = [lp[3], lp[2], lp[1], lp[0]];
      const src = [{x:0,y:0}, {x:200,y:0}, {x:200,y:200}];
      const tf  = quadToAffine(src, [dst[0], dst[1], dst[2]]);
      if (!tf) return;

      const fs = Math.round(56 * this.dpr);
      ctx.save();
      ctx.transform(tf.a, tf.b, tf.c, tf.d, tf.e, tf.f);
      ctx.fillStyle = textCol;
      ctx.font = `bold ${fs}px Arial, Helvetica, sans-serif`;
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(face.label, 100, 100);
      ctx.restore();
    }

    _draw() {
      const ctx = this.ctx;
      const w = this.canvas.width, h = this.canvas.height, dpr = this.dpr;

      ctx.clearRect(0, 0, w, h);

      // Dark background
      ctx.fillStyle = '#1a1d23';
      ctx.fillRect(0, 0, w, h);
      this._drawDotGrid(ctx, w, h, dpr);

      const basis = cameraBasis(this.dir, this.up);
      const vis   = this._visibleFaces(basis);

      for (const { name, face, pts2d } of vis) {
        const hov = name === this.hoveredId;
        const s   = this._shade(face.normal);

        // Drop shadow
        const sdx = 1.8 * dpr, sdy = 2.3 * dpr;
        this._poly(ctx, pts2d.map(p => ({x:p.x+sdx, y:p.y+sdy})));
        ctx.fillStyle = this.pal.shadow;
        ctx.fill();

        // Face fill
        this._poly(ctx, pts2d);
        if (hov) {
          ctx.fillStyle = this.pal.hover;
        } else {
          const base = face.type==='main' ? this.pal.fMain
                     : face.type==='edge' ? this.pal.fEdge : this.pal.fCorn;
          ctx.fillStyle = shadeHex(base, s);
        }
        ctx.fill();

        // Border
        ctx.strokeStyle = face.type === 'main' ? this.pal.border : this.pal.borderS;
        ctx.lineWidth   = face.type === 'main' ? 2.0*dpr : 1.2*dpr;
        ctx.stroke();

        // Label (main faces only)
        if (face.label) {
          this._drawLabel(ctx, face, basis, hov ? this.pal.hoverTx : this.pal.text);
        }
      }

      this._drawGizmo(ctx, basis, dpr);
      this._drawHint(ctx, w, h, dpr);
    }

    _drawDotGrid(ctx, w, h, dpr) {
      ctx.fillStyle = 'rgba(255,255,255,0.045)';
      const step = 32 * dpr;
      for (let x = step/2; x < w; x += step)
        for (let y = step/2; y < h; y += step) {
          ctx.beginPath();
          ctx.arc(x, y, 1.1*dpr, 0, Math.PI*2);
          ctx.fill();
        }
    }

    _drawGizmo(ctx, basis, dpr) {
      const pad = 30*dpr, gl = 18*dpr;
      const gx = pad, gy = this.canvas.height - pad;
      const axes = [
        { dir:[1,0,0], col:'#d73434', lbl:'X' },
        { dir:[0,1,0], col:'#34c334', lbl:'Y' },
        { dir:[0,0,1], col:'#3773ff', lbl:'Z' },
      ];
      for (const { dir, col, lbl } of axes) {
        const ex = gx + dot(dir, basis.r) * gl;
        const ey = gy - dot(dir, basis.u) * gl;
        ctx.strokeStyle = col; ctx.lineWidth = 1.8*dpr;
        ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(ex, ey); ctx.stroke();
        ctx.fillStyle = col;
        ctx.font = `bold ${9*dpr}px Arial`;
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(lbl, ex+(ex-gx)*0.3, ey+(ey-gy)*0.3);
      }
    }

    _drawHint(ctx, w, h, dpr) {
      ctx.fillStyle = 'rgba(255,255,255,0.25)';
      ctx.font = `${11*dpr}px Arial, sans-serif`;
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'bottom';
      ctx.fillText('Drag to orbit · Click a face to snap to that view', w/2, h - 10*dpr);
    }

    // ── Hit test ────────────────────────────────────────────────────────────
    _hitTest(x, y) {
      const basis = cameraBasis(this.dir, this.up);
      const vis   = this._visibleFaces(basis);
      for (let i = vis.length-1; i >= 0; i--)
        if (this._pip(x, y, vis[i].pts2d)) return vis[i].name;
      return null;
    }

    _pip(x, y, pts) {
      let inside = false;
      for (let i=0, j=pts.length-1; i<pts.length; j=i++) {
        const xi=pts[i].x, yi=pts[i].y, xj=pts[j].x, yj=pts[j].y;
        if (((yi>y)!==(yj>y)) && x<(xj-xi)*(y-yi)/(yj-yi)+xi) inside=!inside;
      }
      return inside;
    }

    // ── Snap ────────────────────────────────────────────────────────────────
    snapTo(faceId) {
      const t = SNAP[faceId];
      if (t) {
        this._startSnap(t.d, t.u);
      } else {
        const f = this.faces[faceId];
        if (!f) return;
        const up = Math.abs(dot(f.normal, [0,0,1])) > 0.9 ? [0,-1,0] : [0,0,1];
        this._startSnap(scl(f.normal, -1), up);
      }
    }

    _startSnap(newDir, newUp) {
      this.animD0    = this.dir.slice();
      this.animU0    = this.up.slice();
      this.animD1    = norm(newDir);
      this.animU1    = norm(newUp);
      this.animStart = performance.now();
      this.anim      = true;
    }

    // ── Orbit drag ──────────────────────────────────────────────────────────
    orbit(dx, dy) {
      const b = cameraBasis(this.dir, this.up);
      this.dir = norm(rodrigues(this.dir, b.u, dx));
      this.dir = norm(rodrigues(this.dir, b.r, dy));
      this.up  = norm(rodrigues(this.up,  b.u, dx));
      this.up  = norm(rodrigues(this.up,  b.r, dy));
    }

    // ── Events ──────────────────────────────────────────────────────────────
    _xy(e) {
      const r = this.canvas.getBoundingClientRect(), dpr = this.dpr;
      return { x:(e.clientX-r.left)*dpr, y:(e.clientY-r.top)*dpr };
    }

    _bindEvents() {
      const c = this.canvas;

      c.addEventListener('mousemove', e => {
        const p = this._xy(e);
        if (this.dragging && this.lastMouse) {
          this.orbit((p.x-this.lastMouse.x)/this.dpr * 0.009,
                     (p.y-this.lastMouse.y)/this.dpr * 0.009);
          this.autoRotating = false;
        } else {
          this.hoveredId = this._hitTest(p.x, p.y);
        }
        this.lastMouse = p;
      });

      c.addEventListener('mousedown', e => {
        this.dragging = true;
        this.lastMouse = this._xy(e);
        this.autoRotating = false;
        c.style.cursor = 'grabbing';
        e.preventDefault();
      });

      c.addEventListener('mouseup', () => {
        this.dragging = false;
        c.style.cursor = 'grab';
        this.idleSince = performance.now();
      });

      c.addEventListener('mouseleave', () => {
        this.dragging  = false;
        this.hoveredId = null;
        c.style.cursor = 'grab';
        this.idleSince = performance.now();
      });

      c.addEventListener('click', e => {
        if (!this.wasDragging) {
          const id = this._hitTest(this._xy(e).x, this._xy(e).y);
          if (id) { this.snapTo(id); this.autoRotating = false; }
        }
      });

      let touchMoved = false;
      c.addEventListener('touchstart', e => {
        this.dragging  = true;
        touchMoved     = false;
        this.lastMouse = this._xy(e.touches[0]);
        this.autoRotating = false;
      }, { passive:true });

      c.addEventListener('touchmove', e => {
        e.preventDefault();
        touchMoved = true;
        const p = this._xy(e.touches[0]);
        if (this.lastMouse)
          this.orbit((p.x-this.lastMouse.x)/this.dpr * 0.009,
                     (p.y-this.lastMouse.y)/this.dpr * 0.009);
        this.lastMouse = p;
      }, { passive:false });

      c.addEventListener('touchend', e => {
        this.dragging = false;
        if (!touchMoved && e.changedTouches.length) {
          const p  = this._xy(e.changedTouches[0]);
          const id = this._hitTest(p.x, p.y);
          if (id) { this.snapTo(id); this.autoRotating = false; }
        }
        this.idleSince = performance.now();
      });

      window.addEventListener('resize', () => this.resize());
    }

    // ── Render loop ──────────────────────────────────────────────────────────
    _frame(now) {
      // Snap animation
      if (this.anim) {
        const t  = Math.min(1, (now - this.animStart) / this.animDur);
        const st = smoothstep(t);
        this.dir = vslerp(this.animD0, this.animD1, st);
        this.up  = vslerp(this.animU0, this.animU1, st);
        if (t >= 1) {
          this.anim = false;
          this.dir  = this.animD1.slice();
          this.up   = this.animU1.slice();
        }
      }

      // Auto-rotate after 3 s of idle
      if (!this.dragging && !this.anim) {
        if (now - this.idleSince > 3000) this.autoRotating = true;
        if (this.autoRotating) this.orbit(0.003, 0.0004);
      }

      this._draw();
      requestAnimationFrame(t => this._frame(t));
    }
  }

  // ── Bootstrap ─────────────────────────────────────────────────────────────

  function init() {
    const div = document.getElementById('demo-canvas');
    if (!div) return;
    const canvas = document.createElement('canvas');
    canvas.style.cssText = 'width:100%;height:100%;display:block;cursor:grab;touch-action:none;';
    div.innerHTML = '';
    div.appendChild(canvas);
    new NaviCube(canvas);
  }

  if (document.readyState === 'loading')
    document.addEventListener('DOMContentLoaded', init);
  else
    init();

})();
