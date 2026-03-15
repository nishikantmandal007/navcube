/**
 * pyside-navicube — interactive browser demo
 *
 * Simulates a real CAD viewport:
 *  • Light "macOS" background with a 3D structural model
 *  • NaviCubeOverlay in top-right corner (exact port from Python)
 *  • Orbit arrow buttons around the cube (like Osdag)
 *  • Shared camera — dragging the viewport or clicking the cube
 *    updates the same view
 *
 * No Three.js. Pure HTML5 Canvas + the same math as NaviCubeOverlay.
 */
(function () {
  'use strict';

  // ── Math helpers (exact Python port) ─────────────────────────────────────

  function norm(v) {
    const n = Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    return n > 1e-10 ? [v[0]/n,v[1]/n,v[2]/n] : v.slice();
  }
  function dot(a,b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
  function cross(a,b) {
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  }
  function add(a,b) { return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]; }
  function sub(a,b) { return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]; }
  function scl(v,s) { return [v[0]*s,v[1]*s,v[2]*s]; }

  function rodrigues(v, axis, angle) {
    const a=norm(axis), c=Math.cos(angle), s=Math.sin(angle), d=dot(a,v), cx=cross(a,v);
    return [v[0]*c+cx[0]*s+a[0]*d*(1-c),
            v[1]*c+cx[1]*s+a[1]*d*(1-c),
            v[2]*c+cx[2]*s+a[2]*d*(1-c)];
  }

  function vslerp(v0, v1, t) {
    v0=norm(v0); v1=norm(v1);
    let d=Math.max(-1,Math.min(1,dot(v0,v1)));
    if (d>0.9999) return norm(add(v0,scl(sub(v1,v0),t)));
    if (d<-0.9999) {
      const mid=norm(cross(v0,Math.abs(v0[0])<0.9?[1,0,0]:[0,1,0]));
      return t<0.5?vslerp(v0,mid,t*2):vslerp(mid,v1,(t-0.5)*2);
    }
    const om=Math.acos(d), so=Math.sin(om);
    return [(Math.sin((1-t)*om)/so)*v0[0]+(Math.sin(t*om)/so)*v1[0],
            (Math.sin((1-t)*om)/so)*v0[1]+(Math.sin(t*om)/so)*v1[1],
            (Math.sin((1-t)*om)/so)*v0[2]+(Math.sin(t*om)/so)*v1[2]];
  }

  function smoothstep(t) { t=Math.max(0,Math.min(1,t)); return t*t*t*(t*(t*6-15)+10); }

  function cameraBasis(d, u) {
    d=norm(d); u=norm(u);
    let r=cross(d,u);
    if (r[0]*r[0]+r[1]*r[1]+r[2]*r[2]<1e-12) r=cross(d,Math.abs(d[0])<0.9?[1,0,0]:[0,1,0]);
    r=norm(r); u=norm(cross(r,d));
    return {r,u};
  }

  // ── NaviCube face geometry (exact Python port, CHAMFER=0.12) ──────────────

  function buildNaviFaces() {
    const faces={}, C=0.12;
    function add1(name,xv,zv,type,label) {
      const yv=cross(xv,scl(zv,-1));
      let pts,labelPts=null;
      if (type==='corner') {
        const xc=scl(xv,C),yc=scl(yv,C),zc=scl(zv,1-2*C);
        pts=[sub(zc,scl(xv,2*C)),sub(sub(zc,xc),yc),sub(add(zc,xc),yc),
             add(zc,scl(xv,2*C)),add(add(zc,xc),yc),add(sub(zc,xc),yc)];
      } else if (type==='edge') {
        const x4=scl(xv,1-C*4),ye=scl(yv,C),ze=scl(zv,1-C);
        pts=[sub(sub(ze,x4),ye),sub(add(ze,x4),ye),add(add(ze,x4),ye),add(sub(ze,x4),ye)];
      } else {
        const x2=scl(xv,1-C*2),y2=scl(yv,1-C*2),x4=scl(xv,1-C*4),y4=scl(yv,1-C*4);
        pts=[sub(sub(zv,x2),y4),sub(sub(zv,x4),y2),sub(add(zv,x4),y2),sub(add(zv,x2),y4),
             add(add(zv,x2),y4),add(add(zv,x4),y2),add(sub(zv,x4),y2),add(sub(zv,x2),y4)];
        labelPts=[sub(sub(zv,x2),y2),sub(add(zv,x2),y2),add(add(zv,x2),y2),add(sub(zv,x2),y2)];
      }
      const sum=pts.reduce((a,p)=>add(a,p),[0,0,0]);
      const ctr=scl(sum,1/pts.length);
      faces[name]={pts,normal:norm(ctr),ctr,label:label||null,type,labelPts};
    }
    add1('TOP',   [1,0,0],[0,0,1], 'main','TOP');
    add1('FRONT', [1,0,0],[0,-1,0],'main','FRONT');
    add1('LEFT',  [0,-1,0],[-1,0,0],'main','LEFT');
    add1('BACK',  [-1,0,0],[0,1,0],'main','BACK');
    add1('RIGHT', [0,1,0],[1,0,0], 'main','RIGHT');
    add1('BOTTOM',[1,0,0],[0,0,-1],'main','BOTTOM');
    add1('FTR',[-1,-1,0],[1,-1,1],'corner');  add1('FTL',[-1,1,0],[-1,-1,1],'corner');
    add1('FBR',[1,1,0],[1,-1,-1],'corner');   add1('FBL',[1,-1,0],[-1,-1,-1],'corner');
    add1('RTR',[1,-1,0],[1,1,1],'corner');    add1('RTL',[1,1,0],[-1,1,1],'corner');
    add1('RBR',[-1,1,0],[1,1,-1],'corner');   add1('RBL',[-1,-1,0],[-1,1,-1],'corner');
    add1('FRONT_TOP',[1,0,0],[0,-1,1],'edge');    add1('FRONT_BOTTOM',[1,0,0],[0,-1,-1],'edge');
    add1('REAR_BOTTOM',[1,0,0],[0,1,-1],'edge');  add1('REAR_TOP',[1,0,0],[0,1,1],'edge');
    add1('REAR_RIGHT',[0,0,1],[1,1,0],'edge');    add1('FRONT_RIGHT',[0,0,1],[1,-1,0],'edge');
    add1('FRONT_LEFT',[0,0,1],[-1,-1,0],'edge');  add1('REAR_LEFT',[0,0,1],[-1,1,0],'edge');
    add1('TOP_LEFT',[0,1,0],[0,1,1],'edge');      add1('TOP_RIGHT',[0,1,0],[1,0,1],'edge');
    add1('BOTTOM_RIGHT',[0,1,0],[1,0,-1],'edge'); add1('BOTTOM_LEFT',[0,1,0],[-1,0,-1],'edge');
    return faces;
  }

  const SNAP = {
    TOP:   {d:[0,0,-1],u:[0,-1,0]}, BOTTOM:{d:[0,0,1], u:[0,1,0]},
    FRONT: {d:[0,1,0], u:[0,0,1]},  BACK:  {d:[0,-1,0],u:[0,0,1]},
    LEFT:  {d:[1,0,0], u:[0,0,1]},  RIGHT: {d:[-1,0,0],u:[0,0,1]},
  };

  // ── Affine quad mapping (= Qt's QTransform::quadToQuad) ──────────────────
  function quadToAffine(src, dst) {
    const [s0,s1,s2]=[src[0],src[1],src[2]], [d0,d1,d2]=[dst[0],dst[1],dst[2]];
    const sx=s1.x-s0.x, sy=s2.x-s0.x, tx=s1.y-s0.y, ty=s2.y-s0.y;
    const det=sx*ty-sy*tx; if (Math.abs(det)<1e-10) return null;
    const idx=ty/det, idy=-sy/det, itx=-tx/det, ity=sx/det;
    const dx=d1.x-d0.x, dy=d2.x-d0.x, ex=d1.y-d0.y, ey=d2.y-d0.y;
    const a=dx*idx+dy*itx, c=dx*idy+dy*ity, b=ex*idx+ey*itx, dd=ex*idy+ey*ity;
    return {a, b, c, d:dd, e:d0.x-a*s0.x-c*s0.y, f:d0.y-b*s0.x-dd*s0.y};
  }

  // ── Color ─────────────────────────────────────────────────────────────────
  function shadeHex(hex, s) {
    const r=parseInt(hex.slice(1,3),16), g=parseInt(hex.slice(3,5),16), b=parseInt(hex.slice(5,7),16);
    return `rgb(${Math.round(r*s)},${Math.round(g*s)},${Math.round(b*s)})`;
  }

  // ── 3D scene model: structural portal frame ───────────────────────────────
  // Two columns + one beam — looks like a real structural element in Osdag

  function boxMesh(ox, oy, oz, hw, hd, hh) {
    const v = [
      [ox-hw,oy-hd,oz-hh],[ox+hw,oy-hd,oz-hh],[ox+hw,oy+hd,oz-hh],[ox-hw,oy+hd,oz-hh],
      [ox-hw,oy-hd,oz+hh],[ox+hw,oy-hd,oz+hh],[ox+hw,oy+hd,oz+hh],[ox-hw,oy+hd,oz+hh],
    ];
    return {
      verts: v,
      faces: [
        {vi:[4,5,6,7],n:[0,0, 1]}, {vi:[0,3,2,1],n:[0,0,-1]},
        {vi:[0,1,5,4],n:[0,-1,0]}, {vi:[2,3,7,6],n:[0,1, 0]},
        {vi:[1,2,6,5],n:[1, 0,0]}, {vi:[3,0,4,7],n:[-1,0,0]},
      ],
    };
  }

  // Portal frame: 2 columns (Z-up, feet at z=−0.85) + 1 horizontal beam
  const FRAME = [
    boxMesh(-0.50,0,0, 0.09,0.09,0.85),   // left column  (center at z=0)
    boxMesh( 0.50,0,0, 0.09,0.09,0.85),   // right column
    boxMesh( 0.00,0,0.94, 0.59,0.09,0.09),// top beam
  ];

  // ── Main demo ─────────────────────────────────────────────────────────────

  class Demo {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx    = canvas.getContext('2d');

      this.naviFaces = buildNaviFaces();

      // Shared camera — inward direction (eye→scene), Z-up
      this.dir = norm([-1, 1, -1]);   // ISO view
      this.up  = [0, 0, 1];

      // Lambertian light (same as Python)
      this.LIGHT = norm([-0.8, -1.0, -1.8]);

      // Snap animation
      this.anim=false; this.animStart=0; this.animDur=240;
      this.animD0=this.dir.slice(); this.animU0=this.up.slice();
      this.animD1=null; this.animU1=null;

      // Hover / drag
      this.hovNavi   = null;   // hovered navicube face id
      this.hovBtn    = null;   // hovered button id
      this.dragging  = false;
      this.lastMouse = null;
      this.idleSince = performance.now();
      this.autoRot   = true;

      this.dpr = 1;
      this._bindEvents();
      this.resize();
      requestAnimationFrame(t => this._frame(t));
    }

    // ── Layout ──────────────────────────────────────────────────────────────

    resize() {
      this.dpr = window.devicePixelRatio || 1;
      const rect = this.canvas.getBoundingClientRect();
      this.W = Math.round(rect.width  * this.dpr);
      this.H = Math.round(rect.height * this.dpr);
      this.canvas.width  = this.W;
      this.canvas.height = this.H;

      // Scene projection center + scale
      this.sCx = this.W * 0.46;
      this.sCy = this.H * 0.52;
      this.sS  = this.H * 0.26;     // scene scale: frame ~2 units tall → 52% of canvas

      // NaviCube scale (cube "radius" in device px)
      this.nS = this.H * 0.095;     // navi scale: cube spans ±1 unit
      // NaviCube center (top-right area, with room for arrows)
      const aOff = this.nS * 1.55;  // arrow offset from cube center
      this.nCx = this.W - aOff - this.nS*0.6 - 14*this.dpr;
      this.nCy = aOff + this.nS*0.6 + 14*this.dpr;

      this._buildButtons();
    }

    _buildButtons() {
      const cx=this.nCx, cy=this.nCy, aOff=this.nS*1.55, aR=this.nS*0.38;
      // 4 directional orbit arrows + home circle
      this.btns = [
        {id:'up',    x:cx,       y:cy-aOff, ax: 0, ay:-1, ang:-Math.PI/2, r:aR},
        {id:'down',  x:cx,       y:cy+aOff, ax: 0, ay: 1, ang: Math.PI/2, r:aR},
        {id:'left',  x:cx-aOff,  y:cy,      ax:-1, ay: 0, ang: Math.PI,   r:aR},
        {id:'right', x:cx+aOff,  y:cy,      ax: 1, ay: 0, ang: 0,         r:aR},
        {id:'home',  x:cx+aOff*0.78, y:cy-aOff*0.78, r:aR*0.82, isHome:true},
      ];
    }

    // ── Projection ───────────────────────────────────────────────────────────

    _pS(pt, b) { // scene projection
      return {x: this.sCx+dot(pt,b.r)*this.sS, y: this.sCy-dot(pt,b.u)*this.sS};
    }
    _pN(pt, b) { // navicube projection
      return {x: this.nCx+dot(pt,b.r)*this.nS, y: this.nCy-dot(pt,b.u)*this.nS};
    }

    // ── Draw ─────────────────────────────────────────────────────────────────

    _draw() {
      const ctx=this.ctx, W=this.W, H=this.H, dpr=this.dpr;
      ctx.clearRect(0,0,W,H);

      const basis = cameraBasis(this.dir, this.up);

      // ── Background: light "apple" viewport ──
      const bg = ctx.createLinearGradient(0,0,0,H);
      bg.addColorStop(0,'#f5f5f7'); bg.addColorStop(1,'#e8e8ec');
      ctx.fillStyle = bg; ctx.fillRect(0,0,W,H);

      // ── Ground grid (Z=0 XY plane) ──
      ctx.strokeStyle='rgba(0,0,0,0.065)'; ctx.lineWidth=1;
      for (let i=-3; i<=3.01; i+=0.5) {
        const p1=this._pS([i,-3,0],basis), p2=this._pS([i,3,0],basis);
        ctx.beginPath(); ctx.moveTo(p1.x,p1.y); ctx.lineTo(p2.x,p2.y); ctx.stroke();
        const p3=this._pS([-3,i,0],basis), p4=this._pS([3,i,0],basis);
        ctx.beginPath(); ctx.moveTo(p3.x,p3.y); ctx.lineTo(p4.x,p4.y); ctx.stroke();
      }

      // ── Portal frame ──
      this._drawFrame(ctx, basis, dpr);

      // ── Scene gizmo (bottom-left) ──
      this._drawSceneGizmo(ctx, basis, dpr);

      // ── NaviCube panel + cube ──
      this._drawNaviPanel(ctx, basis, dpr);

      // ── Hint ──
      ctx.fillStyle='rgba(0,0,0,0.28)';
      ctx.font=`${11*dpr}px -apple-system,Arial,sans-serif`;
      ctx.textAlign='center'; ctx.textBaseline='bottom';
      ctx.fillText('Drag to orbit · Click a face on the NaviCube to snap', W/2, H-10*dpr);
    }

    _drawFrame(ctx, basis, dpr) {
      // Collect visible faces from all 3 meshes
      const all = [];
      for (const mesh of FRAME) {
        for (const face of mesh.faces) {
          const vis = -dot(face.n, this.dir);
          if (vis <= 0.0) continue;
          const pts2d = face.vi.map(i => this._pS(mesh.verts[i], basis));
          const depth = face.vi.reduce((s,i)=>s+dot(mesh.verts[i],this.dir),0)/face.vi.length;
          const shade = 0.45 + 0.55 * Math.max(0, -dot(face.n, this.LIGHT));
          all.push({pts2d, depth, shade});
        }
      }
      all.sort((a,b)=>b.depth-a.depth);

      for (const {pts2d, shade} of all) {
        // Face fill — steel blue-gray matching Osdag's structural steel look
        const r=Math.round(162*shade), g=Math.round(176*shade), b=Math.round(195*shade);
        this._poly(ctx, pts2d);
        ctx.fillStyle = `rgb(${r},${g},${b})`;
        ctx.fill();
        ctx.strokeStyle = 'rgba(35,45,65,0.55)';
        ctx.lineWidth = 1.2*dpr;
        ctx.stroke();
      }
    }

    _drawSceneGizmo(ctx, basis, dpr) {
      const gx=32*dpr, gy=this.H-32*dpr, gl=20*dpr;
      for (const [dir,col,lbl] of [[[1,0,0],'#c0392b','X'],[[0,1,0],'#27ae60','Y'],[[0,0,1],'#2980b9','Z']]) {
        const ex=gx+dot(dir,basis.r)*gl, ey=gy-dot(dir,basis.u)*gl;
        ctx.strokeStyle=col; ctx.lineWidth=2*dpr;
        ctx.beginPath(); ctx.moveTo(gx,gy); ctx.lineTo(ex,ey); ctx.stroke();
        ctx.fillStyle=col;
        ctx.font=`bold ${9*dpr}px Arial`; ctx.textAlign='center'; ctx.textBaseline='middle';
        ctx.fillText(lbl, ex+(ex-gx)*0.28, ey+(ey-gy)*0.28);
      }
    }

    _drawNaviPanel(ctx, basis, dpr) {
      // ── Panel background ──
      const aOff=this.nS*1.55, pad=10*dpr;
      const px=this.nCx-aOff-pad, py=this.nCy-aOff-pad;
      const pw=aOff*2+pad*2, ph=aOff*2+pad*2;

      ctx.fillStyle='rgba(255,255,255,0.92)';
      ctx.strokeStyle='rgba(0,0,0,0.12)';
      ctx.lineWidth=dpr;
      this._rRect(ctx, px, py, pw, ph, 10*dpr);
      ctx.fill(); ctx.stroke();

      // ── NaviCube faces (light palette — Python's NaviCubeStyle light defaults) ──
      const pal = {
        fMain:'#f8f8fc', fEdge:'#d2d2d7', fCorn:'#b9b9be',
        text:'#121212',  border:'rgba(28,28,32,1)', borderS:'rgba(50,50,55,1)',
        hover:'rgba(0,148,255,0.92)', hoverTx:'#ffffff', shadow:'rgba(0,0,0,0.14)',
      };

      const vis=[];
      for (const [name,f] of Object.entries(this.naviFaces)) {
        if (-dot(f.normal,this.dir)<=0.10) continue;
        vis.push({name,face:f,pts2d:f.pts.map(p=>this._pN(p,basis)),depth:dot(f.ctr,this.dir)});
      }
      vis.sort((a,b)=>b.depth-a.depth);

      for (const {name,face,pts2d} of vis) {
        const hov=name===this.hovNavi;
        const s=0.55+0.45*Math.max(0,-dot(face.normal,this.LIGHT));

        // Shadow
        this._poly(ctx, pts2d.map(p=>({x:p.x+1.4*dpr,y:p.y+1.8*dpr})));
        ctx.fillStyle=pal.shadow; ctx.fill();

        // Face
        this._poly(ctx, pts2d);
        ctx.fillStyle = hov ? pal.hover
          : shadeHex(face.type==='main'?pal.fMain:face.type==='edge'?pal.fEdge:pal.fCorn, s);
        ctx.fill();

        ctx.strokeStyle=face.type==='main'?pal.border:pal.borderS;
        ctx.lineWidth=face.type==='main'?2*dpr:1.2*dpr;
        ctx.stroke();

        if (face.label)
          this._drawLabel(ctx, face, basis, hov?pal.hoverTx:pal.text);
      }

      // ── Orbit arrow buttons ──
      for (const btn of this.btns) {
        const hov = this.hovBtn===btn.id;
        if (btn.isHome) {
          // Home: circle with ⌂
          ctx.beginPath(); ctx.arc(btn.x,btn.y,btn.r,0,Math.PI*2);
          ctx.fillStyle = hov?'rgba(0,148,255,0.9)':'rgba(0,0,0,0.13)';
          ctx.fill();
          ctx.fillStyle = hov?'#fff':'rgba(0,0,0,0.45)';
          ctx.font=`${btn.r*1.1}px Arial`; ctx.textAlign='center'; ctx.textBaseline='middle';
          ctx.fillText('⌂', btn.x, btn.y+dpr*0.5);
        } else {
          // Arrow: filled triangle
          ctx.save(); ctx.translate(btn.x,btn.y); ctx.rotate(btn.ang);
          const s=btn.r;
          ctx.beginPath();
          ctx.moveTo(s,0); ctx.lineTo(-s*0.7,s*0.65); ctx.lineTo(-s*0.7,-s*0.65);
          ctx.closePath();
          ctx.fillStyle = hov?'rgba(0,148,255,0.9)':'rgba(0,0,0,0.18)';
          ctx.fill();
          ctx.restore();
        }
      }
    }

    _drawLabel(ctx, face, basis, col) {
      if (!face.labelPts) return;
      const lp=face.labelPts.map(p=>this._pN(p,basis));
      // Python's reversed winding: dst = [lp[3], lp[2], lp[1], lp[0]]
      const dst=[lp[3],lp[2],lp[1],lp[0]];
      const src=[{x:0,y:0},{x:200,y:0},{x:200,y:200}];
      const tf=quadToAffine(src,[dst[0],dst[1],dst[2]]);
      if (!tf) return;
      const fs=Math.round(54*this.dpr);
      ctx.save();
      ctx.transform(tf.a,tf.b,tf.c,tf.d,tf.e,tf.f);
      ctx.fillStyle=col;
      ctx.font=`bold ${fs}px Arial,Helvetica,sans-serif`;
      ctx.textAlign='center'; ctx.textBaseline='middle';
      ctx.fillText(face.label,100,100);
      ctx.restore();
    }

    // ── Hit test ─────────────────────────────────────────────────────────────

    _hitNavi(x, y) {
      const basis=cameraBasis(this.dir,this.up);
      const vis=[];
      for (const [name,f] of Object.entries(this.naviFaces)) {
        if (-dot(f.normal,this.dir)<=0.10) continue;
        vis.push({name,pts2d:f.pts.map(p=>this._pN(p,basis)),depth:dot(f.ctr,this.dir)});
      }
      vis.sort((a,b)=>b.depth-a.depth);
      for (let i=vis.length-1;i>=0;i--)
        if (this._pip(x,y,vis[i].pts2d)) return vis[i].name;
      return null;
    }

    _hitBtn(x, y) {
      for (const btn of this.btns) {
        const dx=x-btn.x, dy=y-btn.y;
        if (dx*dx+dy*dy <= btn.r*btn.r*1.8) return btn.id;
      }
      return null;
    }

    _pip(x, y, pts) {
      let inside=false;
      for (let i=0,j=pts.length-1;i<pts.length;j=i++) {
        const xi=pts[i].x,yi=pts[i].y,xj=pts[j].x,yj=pts[j].y;
        if (((yi>y)!==(yj>y))&&x<(xj-xi)*(y-yi)/(yj-yi)+xi) inside=!inside;
      }
      return inside;
    }

    // Is (x,y) inside the navicube panel area?
    _inPanel(x, y) {
      const aOff=this.nS*1.55, pad=12*this.dpr;
      return x>=this.nCx-aOff-pad && x<=this.nCx+aOff+pad &&
             y>=this.nCy-aOff-pad && y<=this.nCy+aOff+pad;
    }

    // ── Snap + orbit ─────────────────────────────────────────────────────────

    snapTo(faceId) {
      const t=SNAP[faceId];
      if (t) {
        this._snap(t.d, t.u);
      } else {
        const f=this.naviFaces[faceId]; if (!f) return;
        const up=Math.abs(dot(f.normal,[0,0,1]))>0.9?[0,-1,0]:[0,0,1];
        this._snap(scl(f.normal,-1), up);
      }
    }

    _snap(d, u) {
      this.animD0=this.dir.slice(); this.animU0=this.up.slice();
      this.animD1=norm(d); this.animU1=norm(u);
      this.animStart=performance.now(); this.anim=true;
    }

    orbit(dx, dy) {
      const b=cameraBasis(this.dir,this.up);
      this.dir=norm(rodrigues(this.dir,b.u,dx)); this.dir=norm(rodrigues(this.dir,b.r,dy));
      this.up =norm(rodrigues(this.up, b.u,dx)); this.up =norm(rodrigues(this.up, b.r,dy));
    }

    stepOrbit(btn) {
      if (btn.isHome) { this._snap(norm([-1,1,-1]),[0,0,1]); return; }
      const step=Math.PI/12;  // 15°
      const b=cameraBasis(this.dir,this.up);
      const axis = btn.ax!==0 ? b.u : b.r;
      const sign = (btn.ax>0||btn.ay>0) ? -1 : 1;
      const d=rodrigues(this.dir,axis,sign*step);
      const u=rodrigues(this.up, axis,sign*step);
      this._snap(d, u);
    }

    // ── Utilities ─────────────────────────────────────────────────────────────

    _poly(ctx, pts) {
      ctx.beginPath(); ctx.moveTo(pts[0].x,pts[0].y);
      for (let i=1;i<pts.length;i++) ctx.lineTo(pts[i].x,pts[i].y);
      ctx.closePath();
    }

    _rRect(ctx, x, y, w, h, r) {
      ctx.beginPath();
      ctx.moveTo(x+r,y); ctx.lineTo(x+w-r,y); ctx.arcTo(x+w,y,x+w,y+r,r);
      ctx.lineTo(x+w,y+h-r); ctx.arcTo(x+w,y+h,x+w-r,y+h,r);
      ctx.lineTo(x+r,y+h); ctx.arcTo(x,y+h,x,y+h-r,r);
      ctx.lineTo(x,y+r); ctx.arcTo(x,y,x+r,y,r); ctx.closePath();
    }

    // ── Events ────────────────────────────────────────────────────────────────

    _xy(e) {
      const r=this.canvas.getBoundingClientRect(), dpr=this.dpr;
      return {x:(e.clientX-r.left)*dpr, y:(e.clientY-r.top)*dpr};
    }

    _bindEvents() {
      const c=this.canvas;

      c.addEventListener('mousemove', e => {
        const p=this._xy(e);
        if (this.dragging && this.lastMouse && !this._inPanel(p.x,p.y)) {
          this.orbit((p.x-this.lastMouse.x)/this.dpr*0.008,
                     (p.y-this.lastMouse.y)/this.dpr*0.008);
          this.autoRot=false; this.idleSince=performance.now();
        } else if (!this.dragging) {
          if (this._inPanel(p.x,p.y)) {
            this.hovNavi = this._hitNavi(p.x,p.y);
            this.hovBtn  = this._hitBtn(p.x,p.y);
            if (this.hovNavi||this.hovBtn) c.style.cursor='pointer';
            else c.style.cursor='default';
          } else {
            this.hovNavi=null; this.hovBtn=null; c.style.cursor='grab';
          }
        }
        this.lastMouse=p;
      });

      c.addEventListener('mousedown', e => {
        const p=this._xy(e);
        if (!this._inPanel(p.x,p.y)) {
          this.dragging=true; this.lastMouse=p;
          this.autoRot=false; c.style.cursor='grabbing';
        }
        e.preventDefault();
      });

      c.addEventListener('mouseup', () => {
        this.dragging=false; c.style.cursor='grab';
        this.idleSince=performance.now();
      });

      c.addEventListener('mouseleave', () => {
        this.dragging=false; this.hovNavi=null; this.hovBtn=null;
        c.style.cursor='grab'; this.idleSince=performance.now();
      });

      c.addEventListener('click', e => {
        const p=this._xy(e);
        if (this._inPanel(p.x,p.y)) {
          const btn=this.btns.find(b=>{ const dx=p.x-b.x,dy=p.y-b.y; return dx*dx+dy*dy<=b.r*b.r*1.8;});
          if (btn) { this.stepOrbit(btn); this.autoRot=false; return; }
          const fid=this._hitNavi(p.x,p.y);
          if (fid) { this.snapTo(fid); this.autoRot=false; }
        }
      });

      let touchMoved=false;
      c.addEventListener('touchstart', e=>{
        this.dragging=true; touchMoved=false;
        this.lastMouse=this._xy(e.touches[0]); this.autoRot=false;
      },{passive:true});
      c.addEventListener('touchmove', e=>{
        e.preventDefault(); touchMoved=true;
        const p=this._xy(e.touches[0]);
        if (this.lastMouse&&!this._inPanel(p.x,p.y))
          this.orbit((p.x-this.lastMouse.x)/this.dpr*0.008,
                     (p.y-this.lastMouse.y)/this.dpr*0.008);
        this.lastMouse=p;
      },{passive:false});
      c.addEventListener('touchend', e=>{
        this.dragging=false;
        if (!touchMoved&&e.changedTouches.length) {
          const p=this._xy(e.changedTouches[0]);
          if (this._inPanel(p.x,p.y)) {
            const fid=this._hitNavi(p.x,p.y);
            if (fid) { this.snapTo(fid); this.autoRot=false; }
          }
        }
        this.idleSince=performance.now();
      });

      window.addEventListener('resize', ()=>this.resize());
    }

    // ── Render loop ───────────────────────────────────────────────────────────

    _frame(now) {
      if (this.anim) {
        const t=Math.min(1,(now-this.animStart)/this.animDur);
        const st=smoothstep(t);
        this.dir=vslerp(this.animD0,this.animD1,st);
        this.up =vslerp(this.animU0,this.animU1,st);
        if (t>=1) { this.anim=false; this.dir=this.animD1.slice(); this.up=this.animU1.slice(); }
      }
      if (!this.dragging&&!this.anim) {
        if (now-this.idleSince>4000) this.autoRot=true;
        if (this.autoRot) this.orbit(0.003, 0.0003);
      }
      this._draw();
      requestAnimationFrame(t=>this._frame(t));
    }
  }

  // ── Bootstrap ─────────────────────────────────────────────────────────────

  function init() {
    const div=document.getElementById('demo-canvas');
    if (!div) return;
    const canvas=document.createElement('canvas');
    canvas.style.cssText='width:100%;height:100%;display:block;cursor:grab;touch-action:none;';
    div.innerHTML=''; div.appendChild(canvas);
    new Demo(canvas);
  }

  if (document.readyState==='loading') document.addEventListener('DOMContentLoaded',init);
  else init();
})();
