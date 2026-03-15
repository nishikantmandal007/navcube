/**
 * navcube — interactive browser demo
 *
 * Renders an OCC-style dark 3D viewport with:
 *   • Gradient background (dark blue-gray, matches OCC defaults)
 *   • Multi-bay structural steel portal frame
 *   • XYZ gizmo (bottom-left)
 *   • Full NaviCube overlay (top-right) — pixel-faithful to Python widget
 *   • Shared orbit camera — drag viewport OR use NaviCube controls
 *   • Status bar integration
 */
(function () {
  'use strict';

  // ── Vector math ──────────────────────────────────────────────────────────

  function norm(v){const n=Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);return n>1e-10?[v[0]/n,v[1]/n,v[2]/n]:v.slice();}
  function dot(a,b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
  function cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
  function add(a,b){return[a[0]+b[0],a[1]+b[1],a[2]+b[2]];}
  function sub(a,b){return[a[0]-b[0],a[1]-b[1],a[2]-b[2]];}
  function scl(v,s){return[v[0]*s,v[1]*s,v[2]*s];}

  function rodrigues(v,axis,angle){
    const a=norm(axis),c=Math.cos(angle),s=Math.sin(angle),d=dot(a,v),cx=cross(a,v);
    return[v[0]*c+cx[0]*s+a[0]*d*(1-c),v[1]*c+cx[1]*s+a[1]*d*(1-c),v[2]*c+cx[2]*s+a[2]*d*(1-c)];
  }
  function vslerp(v0,v1,t){
    v0=norm(v0);v1=norm(v1);
    let d=Math.max(-1,Math.min(1,dot(v0,v1)));
    if(d>0.9999)return norm(add(v0,scl(sub(v1,v0),t)));
    if(d<-0.9999){const mid=norm(cross(v0,Math.abs(v0[0])<0.9?[1,0,0]:[0,1,0]));return t<0.5?vslerp(v0,mid,t*2):vslerp(mid,v1,(t-0.5)*2);}
    const om=Math.acos(d),so=Math.sin(om);
    return[(Math.sin((1-t)*om)/so)*v0[0]+(Math.sin(t*om)/so)*v1[0],
           (Math.sin((1-t)*om)/so)*v0[1]+(Math.sin(t*om)/so)*v1[1],
           (Math.sin((1-t)*om)/so)*v0[2]+(Math.sin(t*om)/so)*v1[2]];
  }
  function smoothstep(t){t=Math.max(0,Math.min(1,t));return t*t*t*(t*(t*6-15)+10);}
  function cameraBasis(d,u){
    d=norm(d);u=norm(u);
    let r=cross(d,u);
    if(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]<1e-12)r=cross(d,Math.abs(d[0])<0.9?[1,0,0]:[0,1,0]);
    r=norm(r);u=norm(cross(r,d));return{r,u};
  }

  // ── NaviCube face geometry (chamfer = 0.12, matches Python) ──────────────

  function buildNaviFaces(){
    const faces={},C=0.12;
    function af(name,xv,zv,type,label){
      const yv=cross(xv,scl(zv,-1));
      let pts,labelPts=null;
      if(type==='corner'){
        const xc=scl(xv,C),yc=scl(yv,C),zc=scl(zv,1-2*C);
        pts=[sub(zc,scl(xv,2*C)),sub(sub(zc,xc),yc),sub(add(zc,xc),yc),
             add(zc,scl(xv,2*C)),add(add(zc,xc),yc),add(sub(zc,xc),yc)];
      }else if(type==='edge'){
        const x4=scl(xv,1-C*4),ye=scl(yv,C),ze=scl(zv,1-C);
        pts=[sub(sub(ze,x4),ye),sub(add(ze,x4),ye),add(add(ze,x4),ye),add(sub(ze,x4),ye)];
      }else{
        const x2=scl(xv,1-C*2),y2=scl(yv,1-C*2),x4=scl(xv,1-C*4),y4=scl(yv,1-C*4);
        pts=[sub(sub(zv,x2),y4),sub(sub(zv,x4),y2),sub(add(zv,x4),y2),sub(add(zv,x2),y4),
             add(add(zv,x2),y4),add(add(zv,x4),y2),add(sub(zv,x4),y2),add(sub(zv,x2),y4)];
        labelPts=[sub(sub(zv,x2),y2),sub(add(zv,x2),y2),add(add(zv,x2),y2),add(sub(zv,x2),y2)];
      }
      const ctr=scl(pts.reduce((a,p)=>add(a,p),[0,0,0]),1/pts.length);
      faces[name]={pts,normal:norm(ctr),ctr,label:label||null,type,labelPts};
    }
    af('TOP',   [1,0,0],[0,0,1], 'main','TOP');
    af('FRONT', [1,0,0],[0,-1,0],'main','FRONT');
    af('LEFT',  [0,-1,0],[-1,0,0],'main','LEFT');
    af('BACK',  [-1,0,0],[0,1,0],'main','BACK');
    af('RIGHT', [0,1,0],[1,0,0], 'main','RIGHT');
    af('BOTTOM',[1,0,0],[0,0,-1],'main','BOTTOM');
    af('FTR',[-1,-1,0],[1,-1,1],'corner');af('FTL',[-1,1,0],[-1,-1,1],'corner');
    af('FBR',[1,1,0],[1,-1,-1],'corner'); af('FBL',[1,-1,0],[-1,-1,-1],'corner');
    af('RTR',[1,-1,0],[1,1,1],'corner');  af('RTL',[1,1,0],[-1,1,1],'corner');
    af('RBR',[-1,1,0],[1,1,-1],'corner'); af('RBL',[-1,-1,0],[-1,1,-1],'corner');
    af('FRONT_TOP',[1,0,0],[0,-1,1],'edge');    af('FRONT_BOTTOM',[1,0,0],[0,-1,-1],'edge');
    af('REAR_BOTTOM',[1,0,0],[0,1,-1],'edge');  af('REAR_TOP',[1,0,0],[0,1,1],'edge');
    af('REAR_RIGHT',[0,0,1],[1,1,0],'edge');    af('FRONT_RIGHT',[0,0,1],[1,-1,0],'edge');
    af('FRONT_LEFT',[0,0,1],[-1,-1,0],'edge');  af('REAR_LEFT',[0,0,1],[-1,1,0],'edge');
    af('TOP_LEFT',[0,1,0],[0,1,1],'edge');      af('TOP_RIGHT',[0,1,0],[1,0,1],'edge');
    af('BOTTOM_RIGHT',[0,1,0],[1,0,-1],'edge'); af('BOTTOM_LEFT',[0,1,0],[-1,0,-1],'edge');
    return faces;
  }

  const SNAP={
    TOP:{d:[0,0,-1],u:[0,-1,0]},BOTTOM:{d:[0,0,1],u:[0,1,0]},
    FRONT:{d:[0,1,0],u:[0,0,1]},BACK:{d:[0,-1,0],u:[0,0,1]},
    LEFT:{d:[1,0,0],u:[0,0,1]},RIGHT:{d:[-1,0,0],u:[0,0,1]},
  };

  // ── NaviCube button layout ────────────────────────────────────────────────

  function buildNaviButtons(S){
    const btns={};
    function tri(pts,act){return{type:'poly',pts,act};}
    function circ(cx,cy,r,act){return{type:'circ',cx:cx*S,cy:cy*S,r:r*S,act};}
    btns.ArrowEast  =tri([{x:S,y:.5*S},{x:.9*S,y:.41*S},{x:.9*S,y:.59*S}],'orbit_r');
    btns.ArrowWest  =tri([{x:0,y:.5*S},{x:.1*S,y:.41*S},{x:.1*S,y:.59*S}],'orbit_l');
    btns.ArrowNorth =tri([{x:.5*S,y:0},{x:.41*S,y:.1*S},{x:.59*S,y:.1*S}],'orbit_u');
    btns.ArrowSouth =tri([{x:.5*S,y:S},{x:.41*S,y:.9*S},{x:.59*S,y:.9*S}],'orbit_d');
    const rollRaw=[66.6,-66.6,58.3,-74,49.2,-80.3,39.4,-85.5,29,-89.5,25.3,-78.1,34.3,-74.3,42.8,-69.9,50.8,-64.4,58.1,-58.1,53.8,-53.8,74.7,-46.8,70.7,-70.4];
    const rollR=[],rollL=[];
    for(let i=0;i<rollRaw.length/2;i++){
      const rx=(rollRaw[i*2]*0.005+0.5)*S,ry=(rollRaw[i*2+1]*0.005+0.5)*S;
      rollR.push({x:rx,y:ry});rollL.push({x:S-rx,y:ry});
    }
    btns.ArrowRight=tri(rollR,'roll_cw');btns.ArrowLeft=tri(rollL,'roll_ccw');
    btns.DotBackside=circ(0.935,0.065,0.055,'backside');
    const menuRaw=[0,0,15,-6,0,-12,-15,-6,0,0,-15,-6,-15,12,0,18,0,0,0,18,15,12,15,-6];
    const menuPts=menuRaw.reduce((arr,_,i,a)=>{if(i%2===0)arr.push({x:(a[i]*0.005+0.84)*S,y:(a[i+1]*0.005+0.84)*S});return arr;},[]);
    btns.ViewMenu={type:'poly',pts:menuPts,act:'home',hitCirc:{cx:0.84*S,cy:0.84*S,r:0.1*S}};
    return btns;
  }

  // ── Affine transform for label quad-mapping ───────────────────────────────
  function quadToAffine(src,dst){
    const[s0,s1,s2]=[src[0],src[1],src[2]],[d0,d1,d2]=[dst[0],dst[1],dst[2]];
    const sx=s1.x-s0.x,sy=s2.x-s0.x,tx=s1.y-s0.y,ty=s2.y-s0.y;
    const det=sx*ty-sy*tx;if(Math.abs(det)<1e-10)return null;
    const idx=ty/det,idy=-sy/det,itx=-tx/det,ity=sx/det;
    const dx=d1.x-d0.x,dy=d2.x-d0.x,ex=d1.y-d0.y,ey=d2.y-d0.y;
    const a=dx*idx+dy*itx,c=dx*idy+dy*ity,b=ex*idx+ey*itx,dd=ex*idy+ey*ity;
    return{a,b,c,d:dd,e:d0.x-a*s0.x-c*s0.y,f:d0.y-b*s0.x-dd*s0.y};
  }

  function shadeRGB(r,g,b,s){return`rgb(${Math.round(r*s)},${Math.round(g*s)},${Math.round(b*s)})`;}

  // ── Structural steel model ────────────────────────────────────────────────
  // Two-bay portal frame building section with purlins and base plates

  function boxMesh(ox,oy,oz,hw,hd,hh,cr,cg,cb){
    const v=[
      [ox-hw,oy-hd,oz-hh],[ox+hw,oy-hd,oz-hh],[ox+hw,oy+hd,oz-hh],[ox-hw,oy+hd,oz-hh],
      [ox-hw,oy-hd,oz+hh],[ox+hw,oy-hd,oz+hh],[ox+hw,oy+hd,oz+hh],[ox-hw,oy+hd,oz+hh]
    ];
    return{verts:v,faces:[
      {vi:[4,5,6,7],n:[0,0,1]},{vi:[0,3,2,1],n:[0,0,-1]},
      {vi:[0,1,5,4],n:[0,-1,0]},{vi:[2,3,7,6],n:[0,1,0]},
      {vi:[1,2,6,5],n:[1,0,0]},{vi:[3,0,4,7],n:[-1,0,0]},
    ],cr:cr||155,cg:cg||163,cb:cb||178};
  }

  const STEEL=[
    // Frame A (y = +0.48)
    boxMesh(-0.72, 0.48, 0,    0.065,0.065,0.88),
    boxMesh( 0.72, 0.48, 0,    0.065,0.065,0.88),
    boxMesh( 0.00, 0.48, 0.945,0.785,0.065,0.065),
    // Frame B (y = -0.48)
    boxMesh(-0.72,-0.48, 0,    0.065,0.065,0.88),
    boxMesh( 0.72,-0.48, 0,    0.065,0.065,0.88),
    boxMesh( 0.00,-0.48, 0.945,0.785,0.065,0.065),
    // Purlins
    boxMesh(-0.60, 0.00,1.00,  0.045,0.46,0.035),
    boxMesh(-0.22, 0.00,1.00,  0.045,0.46,0.035),
    boxMesh( 0.22, 0.00,1.00,  0.045,0.46,0.035),
    boxMesh( 0.60, 0.00,1.00,  0.045,0.46,0.035),
    // Eave strut
    boxMesh( 0.00, 0.00,0.88,  0.72,0.045,0.032),
    // Base plates
    boxMesh(-0.72, 0.48,-0.045,0.115,0.115,0.045,118,122,138),
    boxMesh( 0.72, 0.48,-0.045,0.115,0.115,0.045,118,122,138),
    boxMesh(-0.72,-0.48,-0.045,0.115,0.115,0.045,118,122,138),
    boxMesh( 0.72,-0.48,-0.045,0.115,0.115,0.045,118,122,138),
  ];

  // ── Demo class ───────────────────────────────────────────────────────────

  class Demo {
    constructor(canvas){
      this.canvas=canvas; this.ctx=canvas.getContext('2d');
      this.naviFaces=buildNaviFaces();
      this.LIGHT=norm([-0.8,-1.0,-1.8]);

      this.dir=norm([-1,1,-1]); this.up=[0,0,1];

      this.anim=false; this.animStart=0; this.animDur=240;
      this.animD0=this.dir.slice(); this.animU0=this.up.slice();
      this.animD1=null; this.animU1=null;

      this.hovNavi=null; this.hovBtn=null;
      this.dragging=false; this.lastMouse=null;
      this.dpr=1;

      this._statusEl=document.getElementById('demo-status-left');

      this._bindEvents();
      this.resize();
      requestAnimationFrame(t=>this._frame(t));
    }

    _setStatus(txt){if(this._statusEl)this._statusEl.textContent=txt;}

    resize(){
      this.dpr=window.devicePixelRatio||1;
      const rect=this.canvas.getBoundingClientRect();
      this.W=Math.round(rect.width*this.dpr);
      this.H=Math.round(rect.height*this.dpr);
      this.canvas.width=this.W; this.canvas.height=this.H;

      this.sCx=this.W*0.44; this.sCy=this.H*0.53;
      this.sS=this.H*0.265;

      this.nS=Math.min(this.H*0.24,this.W*0.18,148*this.dpr);
      this.nL=this.W-this.nS-14*this.dpr;
      this.nT=14*this.dpr;
      this.nCx=this.nL+this.nS*0.5;
      this.nCy=this.nT+this.nS*0.5;
      this.nScale=this.nS*0.225;

      this.naviBtns=buildNaviButtons(this.nS);
    }

    _pS(pt,b){return{x:this.sCx+dot(pt,b.r)*this.sS, y:this.sCy-dot(pt,b.u)*this.sS};}
    _pN(pt,b){return{x:this.nCx+dot(pt,b.r)*this.nScale, y:this.nCy-dot(pt,b.u)*this.nScale};}

    _draw(){
      const ctx=this.ctx,W=this.W,H=this.H,dpr=this.dpr;
      ctx.clearRect(0,0,W,H);
      const basis=cameraBasis(this.dir,this.up);

      // OCC-style dark gradient background
      const bg=ctx.createLinearGradient(0,0,0,H);
      bg.addColorStop(0,'#4f5568');
      bg.addColorStop(1,'#272b36');
      ctx.fillStyle=bg; ctx.fillRect(0,0,W,H);

      // Ground grid
      ctx.lineWidth=0.7;
      for(let i=-4;i<=4.01;i+=0.5){
        if(Math.abs(i)<0.01)continue;
        ctx.strokeStyle='rgba(255,255,255,0.055)';
        const p1=this._pS([i,-4,0],basis),p2=this._pS([i,4,0],basis);
        ctx.beginPath();ctx.moveTo(p1.x,p1.y);ctx.lineTo(p2.x,p2.y);ctx.stroke();
        const p3=this._pS([-4,i,0],basis),p4=this._pS([4,i,0],basis);
        ctx.beginPath();ctx.moveTo(p3.x,p3.y);ctx.lineTo(p4.x,p4.y);ctx.stroke();
      }
      // Colored axis lines on ground
      ctx.lineWidth=1.4;
      ctx.strokeStyle='rgba(220,70,70,0.55)';
      {const p1=this._pS([-4,0,0],basis),p2=this._pS([4,0,0],basis);ctx.beginPath();ctx.moveTo(p1.x,p1.y);ctx.lineTo(p2.x,p2.y);ctx.stroke();}
      ctx.strokeStyle='rgba(70,200,70,0.55)';
      {const p1=this._pS([0,-4,0],basis),p2=this._pS([0,4,0],basis);ctx.beginPath();ctx.moveTo(p1.x,p1.y);ctx.lineTo(p2.x,p2.y);ctx.stroke();}

      this._drawSteel(ctx,basis,dpr);
      this._drawGizmo(ctx,basis,dpr);
      this._drawNaviWidget(ctx,basis,dpr);
    }

    _drawSteel(ctx,basis,dpr){
      const all=[];
      for(const mesh of STEEL){
        for(const face of mesh.faces){
          if(-dot(face.n,this.dir)<=0.0)continue;
          const pts2d=face.vi.map(i=>this._pS(mesh.verts[i],basis));
          const depth=face.vi.reduce((s,i)=>s+dot(mesh.verts[i],this.dir),0)/face.vi.length;
          const shade=0.35+0.65*Math.max(0,-dot(face.n,this.LIGHT));
          all.push({pts2d,depth,shade,cr:mesh.cr,cg:mesh.cg,cb:mesh.cb});
        }
      }
      all.sort((a,b)=>b.depth-a.depth);
      for(const{pts2d,shade,cr,cg,cb}of all){
        this._poly(ctx,pts2d);
        ctx.fillStyle=shadeRGB(cr,cg,cb,shade);
        ctx.fill();
        ctx.strokeStyle='rgba(15,18,28,0.65)';
        ctx.lineWidth=1.0*dpr;
        ctx.stroke();
      }
    }

    _drawGizmo(ctx,basis,dpr){
      const gx=36*dpr,gy=this.H-36*dpr,gl=26*dpr;
      ctx.beginPath();ctx.arc(gx,gy,gl*0.72,0,Math.PI*2);
      ctx.fillStyle='rgba(0,0,0,0.25)';ctx.fill();
      for(const[dir,r,g,b,lbl]of[
        [[1,0,0],220,65,65,'X'],
        [[0,1,0],65,200,65,'Y'],
        [[0,0,1],65,110,255,'Z']
      ]){
        const ex=gx+dot(dir,basis.r)*gl,ey=gy-dot(dir,basis.u)*gl;
        ctx.strokeStyle=`rgb(${r},${g},${b})`;ctx.lineWidth=2.0*dpr;
        ctx.beginPath();ctx.moveTo(gx,gy);ctx.lineTo(ex,ey);ctx.stroke();
        ctx.beginPath();ctx.arc(ex,ey,3.5*dpr,0,Math.PI*2);
        ctx.fillStyle=`rgb(${r},${g},${b})`;ctx.fill();
        ctx.font=`bold ${8.5*dpr}px Arial`;ctx.textAlign='center';ctx.textBaseline='middle';
        ctx.fillText(lbl,ex+(ex-gx)*0.38,ey+(ey-gy)*0.38);
      }
    }

    _drawNaviWidget(ctx,basis,dpr){
      const S=this.nS,L=this.nL,T=this.nT;

      // Dark palette matching OCC dark theme
      const vis=[];
      for(const[name,f]of Object.entries(this.naviFaces)){
        if(-dot(f.normal,this.dir)<=0.10)continue;
        vis.push({name,face:f,pts2d:f.pts.map(p=>this._pN(p,basis)),depth:dot(f.ctr,this.dir)});
      }
      vis.sort((a,b)=>b.depth-a.depth);

      // Panel bg
      ctx.fillStyle='rgba(0,0,0,0.20)';
      this._rRect(ctx,L-4*dpr,T-4*dpr,S+8*dpr,S+8*dpr,8*dpr);
      ctx.fill();

      for(const{name,face,pts2d}of vis){
        const hov=name===this.hovNavi;
        const shade=0.55+0.45*Math.max(0,-dot(face.normal,this.LIGHT));
        // Shadow
        this._poly(ctx,pts2d.map(p=>({x:p.x+1.5*dpr,y:p.y+2.0*dpr})));
        ctx.fillStyle='rgba(0,0,0,0.50)';ctx.fill();
        // Face
        this._poly(ctx,pts2d);
        if(hov){ctx.fillStyle='rgba(0,148,255,0.92)';}
        else{
          const base=face.type==='main'?[155,160,178]:face.type==='edge'?[118,122,138]:[96,99,113];
          ctx.fillStyle=shadeRGB(base[0],base[1],base[2],shade);
        }
        ctx.fill();
        ctx.strokeStyle=face.type==='main'?'rgba(10,10,12,1)':'rgba(20,20,22,1)';
        ctx.lineWidth=face.type==='main'?1.8*dpr:1.1*dpr;
        ctx.stroke();
        if(face.label)this._drawLabel(ctx,face,basis,hov?'#ffffff':'#eeeeee',dpr);
      }
      this._drawNaviBtns(ctx,dpr);
    }

    _drawNaviBtns(ctx,dpr){
      const L=this.nL,T=this.nT,S=this.nS;
      ctx.save();ctx.translate(L,T);
      for(const[bname,btn]of Object.entries(this.naviBtns)){
        const hov=this.hovBtn===bname;
        const fill=hov?'rgba(0,148,255,0.85)':'rgba(78,78,82,0.55)';
        const stroke='rgba(0,0,0,0.35)';
        if(btn.type==='circ'){
          ctx.beginPath();ctx.arc(btn.cx,btn.cy,btn.r,0,Math.PI*2);
          ctx.fillStyle=fill;ctx.fill();
          ctx.strokeStyle=stroke;ctx.lineWidth=dpr;ctx.stroke();
        }else if(bname==='ViewMenu'){
          const raw=[0,0,15,-6,0,-12,-15,-6,0,0,-15,-6,-15,12,0,18,0,0,0,18,15,12,15,-6];
          const ox=0.84*S,oy=0.84*S,sc=0.005*S;
          const v3=[];for(let i=0;i<raw.length/2;i++)v3.push({x:ox+raw[i*2]*sc,y:oy+raw[i*2+1]*sc});
          [[0,1,2,3,'#767a8a'],[3,4,5,6,'#606371'],[0,6,7,8,'#9ba0b2']].forEach(([a,b,c,d,fc])=>{
            ctx.beginPath();ctx.moveTo(v3[a].x,v3[a].y);ctx.lineTo(v3[b].x,v3[b].y);
            ctx.lineTo(v3[c].x,v3[c].y);ctx.lineTo(v3[d].x,v3[d].y);ctx.closePath();
            ctx.fillStyle=hov?'rgba(0,148,255,0.6)':fc;ctx.fill();
          });
          ctx.strokeStyle=hov?'rgba(0,148,255,0.9)':stroke;ctx.lineWidth=dpr*0.7;
          ctx.beginPath();[v3[0],v3[1],v3[2],v3[3]].forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.stroke();
        }else{
          ctx.beginPath();ctx.moveTo(btn.pts[0].x,btn.pts[0].y);
          for(let i=1;i<btn.pts.length;i++)ctx.lineTo(btn.pts[i].x,btn.pts[i].y);
          ctx.closePath();
          ctx.fillStyle=fill;ctx.fill();
          ctx.strokeStyle=stroke;ctx.lineWidth=dpr*0.7;ctx.stroke();
        }
      }
      ctx.restore();
    }

    _drawLabel(ctx,face,basis,col,dpr){
      if(!face.labelPts)return;
      const lp=face.labelPts.map(p=>this._pN(p,basis));
      const dst=[lp[3],lp[2],lp[1],lp[0]];
      const tf=quadToAffine([{x:0,y:0},{x:200,y:0},{x:200,y:200}],[dst[0],dst[1],dst[2]]);
      if(!tf)return;
      ctx.save();
      ctx.transform(tf.a,tf.b,tf.c,tf.d,tf.e,tf.f);
      ctx.fillStyle=col;
      ctx.font=`bold ${Math.round(52*dpr)}px Arial,Helvetica,sans-serif`;
      ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillText(face.label,100,100);
      ctx.restore();
    }

    _hitNavi(x,y){
      const basis=cameraBasis(this.dir,this.up),vis=[];
      for(const[name,f]of Object.entries(this.naviFaces)){
        if(-dot(f.normal,this.dir)<=0.10)continue;
        vis.push({name,pts2d:f.pts.map(p=>this._pN(p,basis)),depth:dot(f.ctr,this.dir)});
      }
      vis.sort((a,b)=>b.depth-a.depth);
      for(let i=vis.length-1;i>=0;i--)
        if(this._pip(x,y,vis[i].pts2d))return vis[i].name;
      return null;
    }
    _hitBtn(x,y){
      const lx=x-this.nL,ly=y-this.nT;
      for(const[bname,btn]of Object.entries(this.naviBtns)){
        if(btn.type==='circ'){const dx=lx-btn.cx,dy=ly-btn.cy;if(dx*dx+dy*dy<=btn.r*btn.r*1.5)return bname;}
        else if(bname==='ViewMenu'){const hc=btn.hitCirc,dx=lx-hc.cx,dy=ly-hc.cy;if(dx*dx+dy*dy<=hc.r*hc.r*2)return bname;}
        else if(this._pip(lx,ly,btn.pts))return bname;
      }
      return null;
    }
    _inWidget(x,y){return x>=this.nL&&x<=this.nL+this.nS&&y>=this.nT&&y<=this.nT+this.nS;}
    _pip(x,y,pts){
      let inside=false;
      for(let i=0,j=pts.length-1;i<pts.length;j=i++){
        const xi=pts[i].x,yi=pts[i].y,xj=pts[j].x,yj=pts[j].y;
        if(((yi>y)!==(yj>y))&&x<(xj-xi)*(y-yi)/(yj-yi)+xi)inside=!inside;
      }
      return inside;
    }

    _snap(d,u,label){
      this.animD0=this.dir.slice();this.animU0=this.up.slice();
      this.animD1=norm(d);this.animU1=norm(u);
      this.animStart=performance.now();this.anim=true;
      if(label)this._setStatus(label+' \u00b7 Z-up');
    }
    snapTo(id){
      const labels={TOP:'Top view',BOTTOM:'Bottom view',FRONT:'Front view',BACK:'Back view',LEFT:'Left view',RIGHT:'Right view'};
      const t=SNAP[id];
      if(t)this._snap(t.d,t.u,labels[id]||id);
      else{const f=this.naviFaces[id];if(!f)return;this._snap(scl(f.normal,-1),Math.abs(dot(f.normal,[0,0,1]))>0.9?[0,-1,0]:[0,0,1],id.replace(/_/g,' '));}
    }
    orbit(dx,dy){
      const b=cameraBasis(this.dir,this.up);
      this.dir=norm(rodrigues(this.dir,b.u,dx));this.dir=norm(rodrigues(this.dir,b.r,dy));
      this.up =norm(rodrigues(this.up, b.u,dx));this.up =norm(rodrigues(this.up, b.r,dy));
    }
    execBtn(bname){
      const act=this.naviBtns[bname]?.act;if(!act)return;
      const step=Math.PI/12,b=cameraBasis(this.dir,this.up);
      if(act==='home')this._snap(norm([-1,1,-1]),[0,0,1],'ISO view');
      else if(act==='orbit_r')this._snap(rodrigues(this.dir,b.u,-step),rodrigues(this.up,b.u,-step),'Orbit right');
      else if(act==='orbit_l')this._snap(rodrigues(this.dir,b.u, step),rodrigues(this.up,b.u, step),'Orbit left');
      else if(act==='orbit_u')this._snap(rodrigues(this.dir,b.r,-step),rodrigues(this.up,b.r,-step),'Orbit up');
      else if(act==='orbit_d')this._snap(rodrigues(this.dir,b.r, step),rodrigues(this.up,b.r, step),'Orbit down');
      else if(act==='roll_cw') this._snap(this.dir,rodrigues(this.up,this.dir, step),'Roll CW');
      else if(act==='roll_ccw')this._snap(this.dir,rodrigues(this.up,this.dir,-step),'Roll CCW');
      else if(act==='backside')this._snap(scl(this.dir,-1),this.up,'Backside');
    }

    _poly(ctx,pts){ctx.beginPath();ctx.moveTo(pts[0].x,pts[0].y);for(let i=1;i<pts.length;i++)ctx.lineTo(pts[i].x,pts[i].y);ctx.closePath();}
    _rRect(ctx,x,y,w,h,r){
      ctx.beginPath();ctx.moveTo(x+r,y);ctx.lineTo(x+w-r,y);ctx.arcTo(x+w,y,x+w,y+r,r);
      ctx.lineTo(x+w,y+h-r);ctx.arcTo(x+w,y+h,x+w-r,y+h,r);
      ctx.lineTo(x+r,y+h);ctx.arcTo(x,y+h,x,y+h-r,r);
      ctx.lineTo(x,y+r);ctx.arcTo(x,y,x+r,y,r);ctx.closePath();
    }

    _xy(e){const r=this.canvas.getBoundingClientRect(),dpr=this.dpr;return{x:(e.clientX-r.left)*dpr,y:(e.clientY-r.top)*dpr};}

    _bindEvents(){
      const c=this.canvas;
      c.addEventListener('mousemove',e=>{
        const p=this._xy(e);
        if(this.dragging&&this.lastMouse&&!this._inWidget(p.x,p.y)){
          this.orbit((p.x-this.lastMouse.x)/this.dpr*0.008,(p.y-this.lastMouse.y)/this.dpr*0.008);
          this.autoRot=false;this.idleSince=performance.now();
        }else if(!this.dragging){
          if(this._inWidget(p.x,p.y)){
            this.hovNavi=this._hitNavi(p.x,p.y);
            this.hovBtn =this._hitBtn(p.x,p.y);
            const lbl=this.hovNavi||this.hovBtn;
            c.style.cursor=lbl?'pointer':'default';
            if(lbl)this._setStatus((lbl.replace(/_/g,' '))+' \u00b7 click to snap');
          }else{this.hovNavi=null;this.hovBtn=null;c.style.cursor='grab';}
        }
        this.lastMouse=p;
      });
      c.addEventListener('mousedown',e=>{
        const p=this._xy(e);
        if(!this._inWidget(p.x,p.y)){this.dragging=true;this.lastMouse=p;this.autoRot=false;c.style.cursor='grabbing';}
        e.preventDefault();
      });
      c.addEventListener('mouseup',()=>{this.dragging=false;c.style.cursor='grab';this.idleSince=performance.now();});
      c.addEventListener('mouseleave',()=>{this.dragging=false;this.hovNavi=null;this.hovBtn=null;c.style.cursor='grab';this.idleSince=performance.now();});
      c.addEventListener('click',e=>{
        const p=this._xy(e);
        if(this._inWidget(p.x,p.y)){
          const bn=this._hitBtn(p.x,p.y);if(bn){this.execBtn(bn);this.autoRot=false;return;}
          const fn=this._hitNavi(p.x,p.y);if(fn){this.snapTo(fn);this.autoRot=false;}
        }
      });
      let tmov=false;
      c.addEventListener('touchstart',e=>{this.dragging=true;tmov=false;this.lastMouse=this._xy(e.touches[0]);this.autoRot=false;},{passive:true});
      c.addEventListener('touchmove',e=>{
        e.preventDefault();tmov=true;const p=this._xy(e.touches[0]);
        if(this.lastMouse&&!this._inWidget(p.x,p.y))this.orbit((p.x-this.lastMouse.x)/this.dpr*0.008,(p.y-this.lastMouse.y)/this.dpr*0.008);
        this.lastMouse=p;
      },{passive:false});
      c.addEventListener('touchend',e=>{
        this.dragging=false;
        if(!tmov&&e.changedTouches.length){const p=this._xy(e.changedTouches[0]);if(this._inWidget(p.x,p.y)){const fn=this._hitNavi(p.x,p.y);if(fn){this.snapTo(fn);this.autoRot=false;}}}
        this.idleSince=performance.now();
      });
      window.addEventListener('resize',()=>this.resize());
    }

    _frame(now){
      if(this.anim){
        const t=Math.min(1,(now-this.animStart)/this.animDur),st=smoothstep(t);
        this.dir=vslerp(this.animD0,this.animD1,st);this.up=vslerp(this.animU0,this.animU1,st);
        if(t>=1){this.anim=false;this.dir=this.animD1.slice();this.up=this.animU1.slice();}
      }
      // no auto-rotation — user interacts directly
      this._draw();
      requestAnimationFrame(t=>this._frame(t));
    }
  }

  function init(){
    const div=document.getElementById('demo-canvas');if(!div)return;
    const canvas=document.createElement('canvas');
    canvas.style.cssText='width:100%;height:100%;display:block;cursor:grab;touch-action:none;background:#272b36;';
    div.innerHTML='';div.appendChild(canvas);
    new Demo(canvas);
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);
  else init();
})();
