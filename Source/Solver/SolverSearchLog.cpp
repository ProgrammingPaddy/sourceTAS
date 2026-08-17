#include "SolverSearchLog.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <algorithm>

#include "SolverRoute.h"
#include "SolverWorld.h"

namespace Solver {
namespace SearchLog {

	Sink* g_sink = nullptr;

	int Sink::BeginStage(const std::string& name) {
		CommitPending();
		for (size_t i = 0; i < stages_.size(); ++i)
			if (stages_[i].name == name) {
				cur_stage_ = static_cast<int>(i);
				return cur_stage_;
			}
		Stage st;
		st.name = name;
		stages_.push_back(st);
		pool_.push_back(std::vector<int>());
		pool_seen_.push_back(0);
		cur_stage_ = static_cast<int>(stages_.size()) - 1;
		return cur_stage_;
	}

	void Sink::SetContext(const std::string& ctx) {
		if (ctx.empty()) {
			cur_ctx_ = -1;
			return;
		}
		for (size_t i = 0; i < contexts_.size(); ++i)
			if (contexts_[i] == ctx) {
				cur_ctx_ = static_cast<int>(i);
				return;
			}
		contexts_.push_back(ctx);
		cur_ctx_ = static_cast<int>(contexts_.size()) - 1;
	}

	void Sink::StartTraj(const Vec3& p0) {
		CommitPending();
		if (cur_stage_ < 0)
			return;
		open_ = true;
		tick_ = 0;
		pending_ = Traj();
		pending_.stage = cur_stage_;
		pending_.ctx = cur_ctx_;
		pending_.pts.push_back(p0);
	}

	void Sink::Point(const Vec3& p) {
		if (!open_)
			return;
		tick_++;
		if (tick_ % keep_every_ == 0)
			pending_.pts.push_back(p);
	}

	void Sink::EndTraj(int outcome) {
		if (!open_)
			return;
		open_ = false;
		pending_.outcome = static_cast<unsigned char>(outcome);
		pending_.eval = eval_counter_++;
		stages_[pending_.stage].total++;
		have_pending_ = true;
	}

	void Sink::Score(float sc) {
		if (have_pending_)
			pending_.score = sc;
	}

	void Sink::MarkBest() {
		if (have_pending_)
			pending_.notable = true;
	}

	void Sink::AddRef(const std::string& name,
	                  const std::vector<Vec3>& pts) {
		CommitPending();
		const int st = BeginStage(name);
		Traj t;
		t.stage = st;
		t.eval = eval_counter_++;
		t.outcome = kRef;
		t.notable = true;
		t.pts = pts;
		stages_[st].total++;
		trajs_.push_back(t);
		cur_stage_ = -1;
	}

	void Sink::AddHeat(const Vec3& a, const Vec3& b, const Vec3& c,
	                   float v01) {
		HeatTri t;
		t.a = a;
		t.b = b;
		t.c = c;
		t.v = v01 < 0.f ? 0.f : (v01 > 1.f ? 1.f : v01);
		heat_.push_back(t);
	}

	void Sink::Flush() {
		CommitPending();
	}

	void Sink::CommitPending() {
		if (!have_pending_) {
			open_ = false;
			return;
		}
		have_pending_ = false;
		const int st = pending_.stage;
		if (pending_.notable) {
			// Milestones always kept.
			trajs_.push_back(pending_);
			return;
		}
		// Uniform reservoir over the stage's non-notable candidates.
		pool_seen_[st]++;
		if (static_cast<int>(pool_[st].size()) < cap_) {
			pool_[st].push_back(static_cast<int>(trajs_.size()));
			trajs_.push_back(pending_);
			return;
		}
		rng_ = rng_ * 1664525u + 1013904223u;
		const int r = static_cast<int>((rng_ >> 8)
			% static_cast<unsigned>(pool_seen_[st]));
		if (r < cap_)
			trajs_[pool_[st][r]] = pending_;
	}

	// ---- HTML writer ----------------------------------------------------

	namespace {

		void AppendB64(std::string* out, const unsigned char* p, size_t n) {
			static const char* k =
				"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
				"0123456789+/";
			size_t i = 0;
			for (; i + 2 < n; i += 3) {
				const unsigned v = (p[i] << 16) | (p[i + 1] << 8) | p[i + 2];
				out->push_back(k[(v >> 18) & 63]);
				out->push_back(k[(v >> 12) & 63]);
				out->push_back(k[(v >> 6) & 63]);
				out->push_back(k[v & 63]);
			}
			if (i + 1 == n) {
				const unsigned v = p[i] << 16;
				out->push_back(k[(v >> 18) & 63]);
				out->push_back(k[(v >> 12) & 63]);
				out->push_back('=');
				out->push_back('=');
			} else if (i + 2 == n) {
				const unsigned v = (p[i] << 16) | (p[i + 1] << 8);
				out->push_back(k[(v >> 18) & 63]);
				out->push_back(k[(v >> 12) & 63]);
				out->push_back(k[(v >> 6) & 63]);
				out->push_back('=');
			}
		}

		void AppendF(std::string* s, const char* fmt, ...) {
			char buf[512];
			va_list ap;
			va_start(ap, fmt);
			vsnprintf(buf, sizeof(buf), fmt, ap);
			va_end(ap);
			*s += buf;
		}

		std::string JsonEscape(const std::string& in) {
			std::string o;
			for (char c : in) {
				if (c == '"' || c == '\\') {
					o.push_back('\\');
					o.push_back(c);
				} else if (c == '\n') {
					o += "\\n";
				} else {
					o.push_back(c);
				}
			}
			return o;
		}

		const char* kHtmlTemplate = R"HTMLEOF(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>__TITLE__</title>
<style>
html,body{margin:0;height:100%;background:#0b0e14;color:#c8d0dc;
font:12px/1.45 Consolas,Menlo,monospace;overflow:hidden}
#c{position:absolute;inset:0;width:100%;height:100%}
#ui{position:absolute;top:0;left:0;bottom:0;width:295px;overflow-y:auto;
background:rgba(10,13,20,.92);border-right:1px solid #232a38;padding:10px 12px;
box-sizing:border-box}
#ui h1{font-size:13px;margin:0 0 2px;color:#e8eef8}
#ui .sub{color:#5b667a;margin-bottom:8px}
.sec{margin:10px 0 4px;color:#8fa1bd;text-transform:uppercase;font-size:10px;
letter-spacing:.08em;border-top:1px solid #1d2432;padding-top:8px}
label{display:flex;align-items:center;gap:6px;cursor:pointer;padding:1px 0}
label span.n{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
label span.k{color:#5b667a}
input[type=range]{width:100%}
.sw{display:inline-block;width:10px;height:10px;border-radius:2px}
#stats{position:absolute;right:10px;top:8px;text-align:right;color:#8fa1bd;
background:rgba(10,13,20,.75);padding:6px 10px;border-radius:4px}
#pick{position:absolute;right:10px;bottom:10px;max-width:430px;
background:rgba(10,13,20,.92);border:1px solid #2c3549;border-radius:5px;
padding:8px 12px;display:none;color:#c8d0dc}
#pick b{color:#e8eef8}
#pick .ctx{color:#9fe8b0;margin-top:4px;word-break:break-word}
button{background:#1a2230;color:#c8d0dc;border:1px solid #2c3549;
border-radius:3px;cursor:pointer;font:inherit;padding:2px 10px}
button:hover{background:#232d40}
.row{display:flex;gap:6px;align-items:center;margin:3px 0}
.hint{color:#4c576c;margin-top:10px}
</style></head><body>
<canvas id="c"></canvas>
<div id="ui">
<h1>__TITLE__</h1>
<div class="sub" id="gen"></div>
<div class="sec">Search replay</div>
<div class="row"><button id="play">&#9654; play</button>
<span id="evlab" style="flex:1;text-align:right"></span></div>
<input type="range" id="ev" min="0" max="1000" value="1000">
<div class="sec">Density</div>
<div class="row"><span style="width:70px">alpha</span>
<input type="range" id="al" min="1" max="100" value="22"></div>
<div class="row"><span style="width:70px">best %</span>
<input type="range" id="pct" min="1" max="100" value="100"></div>
<div class="sec">Outcomes</div>
<div id="outs"></div>
<div class="sec">Overlays</div>
<div id="ovl"></div>
<div class="sec">Stages</div>
<div id="stages"></div>
<div class="hint">drag rotate &middot; right-drag / shift-drag pan &middot;
wheel zoom &middot; click a line to inspect it and the route that fed it
&middot; the scrub replays the search in eval order</div>
</div>
<div id="stats"></div>
<div id="pick"></div>
<script>
"use strict";
const D=/*__DATA__*/null;
function decodeB64(s){const bin=atob(s);const n=bin.length;
const u=new Uint8Array(n);for(let i=0;i<n;i++)u[i]=bin.charCodeAt(i);
return new Int16Array(u.buffer);}
const Q=decodeB64(D.pts);
const NP=Q.length/3;
const P=new Float32Array(NP*3);
for(let i=0;i<NP*3;i++)P[i]=Q[i]*0.25;
const T=D.trajs; // [stage,outcome,notable,score,eval,off,npts,ctx]
const NT=T.length;
const OUTNAMES=["miss","grounded","struck","HIT","exited","ZONED","ref"];
const OUTTIPS=[
"ran to the horizon without reaching anything",
"landed on walkable ground (the ride/flight died there)",
"ended on a strike that was NOT the target face",
"board/tap strike ON the target face (a transfer)",
"clean-air exit (carve handed off to a flight)",
"entered the end-zone volume (a finish)",
"reference overlay"];
const OUTON=[true,true,true,true,true,true,true];
// per-stage score percentile ranks
const rank=new Float32Array(NT);
{const by={};for(let i=0;i<NT;i++){const s=T[i][0];(by[s]=by[s]||[]).push(i);}
for(const s in by){const a=by[s];a.sort((x,y)=>T[x][3]-T[y][3]);
for(let j=0;j<a.length;j++)rank[a[j]]=a.length>1?j/(a.length-1):0;}}
let maxEval=1;for(let i=0;i<NT;i++)if(T[i][4]>maxEval)maxEval=T[i][4];
// stage colors: golden-angle hues; refs get fixed identities
function hsl(h,s,l){const a=s*Math.min(l,1-l);
const f=n=>{const k=(n+h/30)%12;return l-a*Math.max(Math.min(k-3,9-k,1),-1);};
return [f(0),f(8),f(4)];}
const stageColor=[],isRef=[];
{let hue=210,ri=0;const refc=[[0,1,.85],[1,.62,.1],[.3,1,.4],[1,.3,.9]];
for(let s=0;s<D.stages.length;s++){
const nm=D.stages[s].name;
if(nm.indexOf("RESULT")>=0){stageColor.push([.25,1,.35]);isRef.push(true);}
else if(nm.indexOf("REF")>=0){stageColor.push(refc[(ri++)%refc.length]);isRef.push(true);}
else{stageColor.push(hsl(hue%360,.75,.62));hue+=47;isRef.push(false);}}}
const stageOn=D.stages.map(()=>true);
// GL
const cv=document.getElementById("c");
const gl=cv.getContext("webgl",{antialias:true,alpha:false});
const vs=`attribute vec3 p;attribute vec4 c;uniform mat4 m;varying vec4 vc;
void main(){gl_Position=m*vec4(p,1.0);vc=c;}`;
const fs=`precision mediump float;varying vec4 vc;uniform float ga;
void main(){gl_FragColor=vec4(vc.rgb,vc.a*ga);}`;
function sh(t,src){const s=gl.createShader(t);gl.shaderSource(s,src);
gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))
throw gl.getShaderInfoLog(s);return s;}
const prog=gl.createProgram();
gl.attachShader(prog,sh(gl.VERTEX_SHADER,vs));
gl.attachShader(prog,sh(gl.FRAGMENT_SHADER,fs));
gl.linkProgram(prog);gl.useProgram(prog);
const aP=gl.getAttribLocation(prog,"p"),aC=gl.getAttribLocation(prog,"c");
const uM=gl.getUniformLocation(prog,"m"),uA=gl.getUniformLocation(prog,"ga");
gl.enableVertexAttribArray(aP);gl.enableVertexAttribArray(aC);
gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE);
gl.disable(gl.DEPTH_TEST);
// buffers: dynamic (filtered trajs) + static (geometry)
let maxSegs=0;for(let i=0;i<NT;i++)maxSegs+=Math.max(0,T[i][6]-1);
const dynP=new Float32Array(maxSegs*2*3);
const dynC=new Uint8Array(maxSegs*2*4);
const bP=gl.createBuffer(),bC=gl.createBuffer();
let dynN=0;
// geometry static
const geoP=[],geoC=[];
function pushSeg(arrP,arrC,x1,y1,z1,x2,y2,z2,r,g2,b,a){
arrP.push(x1,y1,z1,x2,y2,z2);
arrC.push(r,g2,b,a,r,g2,b,a);}
for(const bx of D.geo.boxes){
const [x0,y0,z0,x1,y1,z1]=bx;const cE=[60,70,88,140];
const e=[[x0,y0,z0,x1,y0,z0],[x0,y1,z0,x1,y1,z0],[x0,y0,z1,x1,y0,z1],
[x0,y1,z1,x1,y1,z1],[x0,y0,z0,x0,y1,z0],[x1,y0,z0,x1,y1,z0],
[x0,y0,z1,x0,y1,z1],[x1,y0,z1,x1,y1,z1],[x0,y0,z0,x0,y0,z1],
[x1,y0,z0,x1,y0,z1],[x0,y1,z0,x0,y1,z1],[x1,y1,z0,x1,y1,z1]];
for(const s of e)pushSeg(geoP,geoC,s[0],s[1],s[2],s[3],s[4],s[5],
cE[0],cE[1],cE[2],cE[3]);}
for(const f of D.geo.faces){
for(let i=0;i<f.length;i+=3){const j=(i+3)%f.length;
pushSeg(geoP,geoC,f[i],f[i+1],f[i+2],f[j],f[j+1],f[j+2],120,150,190,220);}}
if(D.geo.zone){const z=D.geo.zone;const c=[255,60,60,255];
const e=[[z[0],z[1],z[2],z[3],z[1],z[2]],[z[0],z[4],z[2],z[3],z[4],z[2]],
[z[0],z[1],z[5],z[3],z[1],z[5]],[z[0],z[4],z[5],z[3],z[4],z[5]],
[z[0],z[1],z[2],z[0],z[4],z[2]],[z[3],z[1],z[2],z[3],z[4],z[2]],
[z[0],z[1],z[5],z[0],z[4],z[5]],[z[3],z[1],z[5],z[3],z[4],z[5]],
[z[0],z[1],z[2],z[0],z[1],z[5]],[z[3],z[1],z[2],z[3],z[1],z[5]],
[z[0],z[4],z[2],z[0],z[4],z[5]],[z[3],z[4],z[2],z[3],z[4],z[5]]];
for(const s of e)pushSeg(geoP,geoC,s[0],s[1],s[2],s[3],s[4],s[5],
c[0],c[1],c[2],c[3]);}
const gP=gl.createBuffer(),gC=gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER,gP);
gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(geoP),gl.STATIC_DRAW);
gl.bindBuffer(gl.ARRAY_BUFFER,gC);
gl.bufferData(gl.ARRAY_BUFFER,new Uint8Array(geoC),gl.STATIC_DRAW);
const geoN=geoP.length/3;
let showGeo=true,showHeat=true;
// Energy heat triangles: cold blue -> yellow -> hot red.
function heatColor(v){
if(v<0.5){const t=v*2;return [40+40*t,60+150*t,200-90*t];}
const t=(v-0.5)*2;return [80+175*t,210-140*t,110-80*t];}
const heatP=[],heatC=[];
for(const h of (D.heat||[])){
const col=heatColor(h[9]);
const a=Math.round(90+120*h[9]);
for(let k=0;k<3;k++){
heatP.push(h[k*3],h[k*3+1],h[k*3+2]);
heatC.push(col[0],col[1],col[2],a);}}
const hP=gl.createBuffer(),hC=gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER,hP);
gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(heatP),gl.STATIC_DRAW);
gl.bindBuffer(gl.ARRAY_BUFFER,hC);
gl.bufferData(gl.ARRAY_BUFFER,new Uint8Array(heatC),gl.STATIC_DRAW);
const heatN=heatP.length/3;
// UI state
let evCut=maxEval,pctCut=1.0,alpha=0.22,selId=-1,selDesc=null;
// Selection lineage: contexts are chain prefixes, so descendants of
// the selected line's committed chain = contexts that start with it.
function computeSel(){
selDesc=null;
if(selId<0)return;
const sc=T[selId][7];
if(sc<0)return;
const pref=D.ctx[sc];
selDesc=D.ctx.map(c=>c===pref||c.indexOf(pref)===0);}
function visT(i){
const t=T[i];const st=t[0],oc=t[1],nb=t[2];
if(!stageOn[st])return false;
if(!OUTON[oc])return false;
if(oc!==6){
if(t[4]>evCut)return false;
if(rank[i]>pctCut&&!nb)return false;}
return true;}
// rebuild filtered buffer
let visKept=0,visTotal=0;
function rebuild(){
let o=0;visKept=0;
for(let i=0;i<NT;i++){
if(!visT(i))continue;
const t=T[i];const st=t[0],oc=t[1],nb=t[2];
const ref=oc===6;
visKept++;
const col=stageColor[st];
let r=col[0]*255,g2=col[1]*255,b=col[2]*255,a=ref?255:(nb?235:90);
if(nb&&!ref){r=Math.min(255,r*1.35+40);g2=Math.min(255,g2*1.35+40);
b=Math.min(255,b*1.35+40);}
else if(!ref){const q=1.0-0.65*rank[i];r*=q;g2*=q;b*=q;}
if(selId>=0){
if(i===selId){r=255;g2=255;b=255;a=255;}
else{
const fam=t[7]>=0&&selDesc&&selDesc[t[7]];
if(fam||ref){a=255;}
else{const gy=(r+g2+b)/3*0.3+26;r=gy;g2=gy;b=gy+6;a=Math.min(a,24);}}}
const off=t[5],n=t[6];
for(let k2=0;k2<n-1;k2++){
const i1=(off+k2)*3,i2=(off+k2+1)*3;
dynP[o*3]=P[i1];dynP[o*3+1]=P[i1+1];dynP[o*3+2]=P[i1+2];
dynC[o*4]=r;dynC[o*4+1]=g2;dynC[o*4+2]=b;dynC[o*4+3]=a;o++;
dynP[o*3]=P[i2];dynP[o*3+1]=P[i2+1];dynP[o*3+2]=P[i2+2];
dynC[o*4]=r;dynC[o*4+1]=g2;dynC[o*4+2]=b;dynC[o*4+3]=a;o++;}}
dynN=o;
gl.bindBuffer(gl.ARRAY_BUFFER,bP);
gl.bufferData(gl.ARRAY_BUFFER,dynP.subarray(0,dynN*3),gl.DYNAMIC_DRAW);
gl.bindBuffer(gl.ARRAY_BUFFER,bC);
gl.bufferData(gl.ARRAY_BUFFER,dynC.subarray(0,dynN*4),gl.DYNAMIC_DRAW);
document.getElementById("stats").innerHTML=
visKept+" / "+NT+" kept lines &middot; sampled from "+D.total_evals+
" evaluated candidates";}
// camera
const ctr=D.geo.center;let yaw=-1.1,pitch=0.5,dist=D.geo.diag*1.1;
let tgt=[ctr[0],ctr[1],ctr[2]];
function mat(){
const w=cv.width,h=cv.height,asp=w/h,f=1/Math.tan(30*Math.PI/180);
const zn=8,zf=60000;
const pr=[f/asp,0,0,0, 0,f,0,0, 0,0,(zf+zn)/(zn-zf),-1,
0,0,2*zf*zn/(zn-zf),0];
const cp=Math.cos(pitch),sp2=Math.sin(pitch);
const ey=[tgt[0]+dist*cp*Math.cos(yaw),tgt[1]+dist*cp*Math.sin(yaw),
tgt[2]+dist*sp2];
let zx=ey[0]-tgt[0],zy=ey[1]-tgt[1],zz=ey[2]-tgt[2];
let zl=Math.hypot(zx,zy,zz);zx/=zl;zy/=zl;zz/=zl;
let xx=-zy,xy=zx,xz=0;const xl=Math.hypot(xx,xy,xz)||1;xx/=xl;xy/=xl;
let yx=zy*xz-zz*xy,yy=zz*xx-zx*xz,yz=zx*xy-zy*xx;
const vw=[xx,yx,zx,0, xy,yy,zy,0, xz,yz,zz,0,
-(xx*ey[0]+xy*ey[1]+xz*ey[2]),
-(yx*ey[0]+yy*ey[1]+yz*ey[2]),
-(zx*ey[0]+zy*ey[1]+zz*ey[2]),1];
const m=new Float32Array(16);
for(let i2=0;i2<4;i2++)for(let j=0;j<4;j++){let s2=0;
for(let k2=0;k2<4;k2++)s2+=pr[k2*4+j]*vw[i2*4+k2];m[i2*4+j]=s2;}
return m;}
function draw(){
const dpr=window.devicePixelRatio||1;
const w=cv.clientWidth*dpr,h=cv.clientHeight*dpr;
if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h;}
gl.viewport(0,0,w,h);
gl.clearColor(0.043,0.055,0.078,1);gl.clear(gl.COLOR_BUFFER_BIT);
const m=mat();gl.uniformMatrix4fv(uM,false,m);
if(showHeat&&heatN){gl.uniform1f(uA,0.55);
gl.bindBuffer(gl.ARRAY_BUFFER,hP);
gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
gl.bindBuffer(gl.ARRAY_BUFFER,hC);
gl.vertexAttribPointer(aC,4,gl.UNSIGNED_BYTE,true,0,0);
gl.drawArrays(gl.TRIANGLES,0,heatN);}
if(showGeo&&geoN){gl.uniform1f(uA,1.0);
gl.bindBuffer(gl.ARRAY_BUFFER,gP);
gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
gl.bindBuffer(gl.ARRAY_BUFFER,gC);
gl.vertexAttribPointer(aC,4,gl.UNSIGNED_BYTE,true,0,0);
gl.drawArrays(gl.LINES,0,geoN);}
if(dynN){gl.uniform1f(uA,alpha*3.0);
gl.bindBuffer(gl.ARRAY_BUFFER,bP);
gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
gl.bindBuffer(gl.ARRAY_BUFFER,bC);
gl.vertexAttribPointer(aC,4,gl.UNSIGNED_BYTE,true,0,0);
gl.drawArrays(gl.LINES,0,dynN);}
requestAnimationFrame(draw);}
// input: L-drag rotate, R/middle/shift-drag PAN (grab the world),
// small L-click picks a line.
let drag=0,px=0,py=0,moved=0;
cv.addEventListener("mousedown",e=>{
drag=(e.shiftKey||e.button===2||e.button===1)?2:1;
px=e.clientX;py=e.clientY;moved=0;});
window.addEventListener("mouseup",e=>{
if(drag===1&&moved<5&&e.button===0)pick(e.clientX,e.clientY);
drag=0;});
window.addEventListener("mousemove",e=>{
if(!drag)return;const dx=e.clientX-px,dy=e.clientY-py;px=e.clientX;py=e.clientY;
moved+=Math.abs(dx)+Math.abs(dy);
if(drag===1){yaw-=dx*0.006;pitch=Math.max(-1.5,Math.min(1.5,pitch+dy*0.006));}
else{const sp2=Math.sin(pitch),cp=Math.cos(pitch);
const rx=-Math.sin(yaw),ry=Math.cos(yaw);
const ux=-sp2*Math.cos(yaw),uy=-sp2*Math.sin(yaw),uz=cp;
const s2=dist*0.0014;
tgt[0]+=(-rx*dx+ux*dy)*s2;tgt[1]+=(-ry*dx+uy*dy)*s2;tgt[2]+=uz*dy*s2;}});
cv.addEventListener("wheel",e=>{e.preventDefault();
dist*=Math.pow(1.0016,e.deltaY);dist=Math.max(60,Math.min(40000,dist));},
{passive:false});
cv.addEventListener("contextmenu",e=>e.preventDefault());
window.addEventListener("keydown",e=>{
if(e.key==="Escape"&&selId>=0){selId=-1;computeSel();showPick();
queueRebuild();}});
// picking: nearest visible line to the click, in screen space
function pick(mx,my){
const m=mat();const dpr=window.devicePixelRatio||1;
const w=cv.width,h=cv.height,sx=mx*dpr,sy=my*dpr;
let best=-1,bd=(14*dpr)*(14*dpr);
for(let i=0;i<NT;i++){
if(!visT(i))continue;
const t=T[i];const off=t[5],n=t[6];
const step=n>24?Math.ceil(n/24):1;
for(let k=0;k<n;k+=step){
const j=(off+k)*3;
const x=P[j],y=P[j+1],z=P[j+2];
const pw=m[3]*x+m[7]*y+m[11]*z+m[15];
if(pw<=0)continue;
const qx=((m[0]*x+m[4]*y+m[8]*z+m[12])/pw*0.5+0.5)*w;
const qy=(1-((m[1]*x+m[5]*y+m[9]*z+m[13])/pw*0.5+0.5))*h;
const d=(qx-sx)*(qx-sx)+(qy-sy)*(qy-sy);
if(d<bd){bd=d;best=i;}}}
selId=best;computeSel();showPick();queueRebuild();}
function showPick(){
const el=document.getElementById("pick");
if(selId<0){el.style.display="none";return;}
const t=T[selId];
el.innerHTML="<b>"+D.stages[t[0]].name+"</b> &middot; "+OUTNAMES[t[1]]+
" &middot; score "+t[3]+" &middot; eval #"+t[4]+
(t[2]?" &middot; <b>best-so-far</b>":"")+
(t[7]>=0&&D.ctx[t[7]]?'<div class="ctx">fed by: '+D.ctx[t[7]]+"</div>":"")+
'<div style="color:#5b667a">esc to deselect</div>';
el.style.display="block";}
// UI build
document.getElementById("gen").textContent=D.generated;
const evS=document.getElementById("ev");evS.max=maxEval;evS.value=maxEval;
const evlab=document.getElementById("evlab");
function evTxt(){evlab.textContent=evCut+" / "+maxEval+" evals";}
evTxt();
let rebuildQueued=false;
function queueRebuild(){if(rebuildQueued)return;rebuildQueued=true;
requestAnimationFrame(()=>{rebuildQueued=false;rebuild();});}
evS.addEventListener("input",()=>{evCut=+evS.value;evTxt();queueRebuild();});
document.getElementById("al").addEventListener("input",
e=>{alpha=+e.target.value/100;});
document.getElementById("pct").addEventListener("input",
e=>{pctCut=+e.target.value/100;queueRebuild();});
let playing=false;const playBtn=document.getElementById("play");
playBtn.addEventListener("click",()=>{playing=!playing;
playBtn.innerHTML=playing?"&#10074;&#10074; pause":"&#9654; play";
if(playing&&evCut>=maxEval){evCut=0;}});
setInterval(()=>{if(!playing)return;
evCut=Math.min(maxEval,evCut+Math.max(1,Math.round(maxEval/400)));
evS.value=evCut;evTxt();queueRebuild();
if(evCut>=maxEval){playing=false;playBtn.innerHTML="&#9654; play";}},33);
const outsDiv=document.getElementById("outs");
for(let i=0;i<7;i++){if(i===6)continue;
const l=document.createElement("label");
l.title=OUTTIPS[i];
l.innerHTML='<input type="checkbox" checked><span class="n">'+
OUTNAMES[i]+'</span>';
l.firstChild.addEventListener("change",e=>{OUTON[i]=e.target.checked;
queueRebuild();});
outsDiv.appendChild(l);}
const ovlDiv=document.getElementById("ovl");
{const l=document.createElement("label");
l.innerHTML='<input type="checkbox" checked><span class="n">map geometry'+
'</span>';
l.firstChild.addEventListener("change",e=>{showGeo=e.target.checked;});
ovlDiv.appendChild(l);}
if(heatN){const l=document.createElement("label");
l.title="hotspot field: effective energy of the best possible board at "+
"each landing point (cold blue = poor, red = hot)";
l.innerHTML='<input type="checkbox" checked><span class="n">energy heat'+
'</span>';
l.firstChild.addEventListener("change",e=>{showHeat=e.target.checked;});
ovlDiv.appendChild(l);}
const stDiv=document.getElementById("stages");
const kept=D.stages.map(()=>0);
for(let i=0;i<NT;i++)kept[T[i][0]]++;
for(let s=0;s<D.stages.length;s++){
const c=stageColor[s];
const l=document.createElement("label");
const swc="rgb("+Math.round(c[0]*255)+","+Math.round(c[1]*255)+","+
Math.round(c[2]*255)+")";
l.innerHTML='<input type="checkbox" checked>'+
'<span class="sw" style="background:'+swc+'"></span>'+
'<span class="n">'+D.stages[s].name+'</span>'+
'<span class="k">'+kept[s]+"/"+D.stages[s].total+'</span>';
l.firstChild.addEventListener("change",e=>{stageOn[s]=e.target.checked;
queueRebuild();});
(isRef[s]?ovlDiv:stDiv).appendChild(l);}
rebuild();draw();
</script></body></html>
)HTMLEOF";

	} // namespace

	bool WriteHtml(const std::string& path, const World& w,
	               const Route::Graph& g, const Sink& sink,
	               const std::string& title, std::string* err) {
		std::vector<Traj> trajs = sink.TrajsRef();
		std::sort(trajs.begin(), trajs.end(),
			[](const Traj& a, const Traj& b) { return a.eval < b.eval; });

		// Point blob (int16, 0.25u quantization).
		std::string b64;
		{
			std::vector<int16_t> q;
			for (const Traj& t : trajs)
				for (const Vec3& p : t.pts) {
					auto qz = [](float v) -> int16_t {
						float s = v * 4.f;
						if (s > 32767.f) s = 32767.f;
						if (s < -32768.f) s = -32768.f;
						return static_cast<int16_t>(s);
					};
					q.push_back(qz(p.X));
					q.push_back(qz(p.Y));
					q.push_back(qz(p.Z));
				}
			if (!q.empty())
				AppendB64(&b64,
					reinterpret_cast<const unsigned char*>(q.data()),
					q.size() * 2);
		}

		std::string js = "{";
		{
			time_t now = time(nullptr);
			struct tm tmv;
			localtime_s(&tmv, &now);
			char stamp[64];
			strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M", &tmv);
			long long total = 0;
			for (const Stage& st : sink.StagesRef())
				total += st.total;
			AppendF(&js, "\"generated\":\"%s\",\"total_evals\":%lld,",
				stamp, total);
		}
		js += "\"stages\":[";
		for (size_t i = 0; i < sink.StagesRef().size(); ++i) {
			const Stage& st = sink.StagesRef()[i];
			AppendF(&js, "%s{\"name\":\"%s\",\"total\":%d}",
				i ? "," : "", JsonEscape(st.name).c_str(), st.total);
		}
		js += "],\"ctx\":[";
		for (size_t i = 0; i < sink.ContextsRef().size(); ++i)
			AppendF(&js, "%s\"%s\"", i ? "," : "",
				JsonEscape(sink.ContextsRef()[i]).c_str());
		js += "],\"trajs\":[";
		{
			int off = 0;
			for (size_t i = 0; i < trajs.size(); ++i) {
				const Traj& t = trajs[i];
				AppendF(&js, "%s[%d,%d,%d,%.1f,%d,%d,%d,%d]",
					i ? "," : "", t.stage, t.outcome,
					t.notable ? 1 : 0, t.score, t.eval, off,
					static_cast<int>(t.pts.size()), t.ctx);
				off += static_cast<int>(t.pts.size());
			}
		}
		js += "],\"heat\":[";
		for (size_t i = 0; i < sink.HeatRef().size(); ++i) {
			const Sink::HeatTri& h = sink.HeatRef()[i];
			AppendF(&js, "%s[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,"
				"%.0f,%.0f,%.2f]", i ? "," : "", h.a.X, h.a.Y,
				h.a.Z, h.b.X, h.b.Y, h.b.Z, h.c.X, h.c.Y, h.c.Z,
				h.v);
		}
		js += "],\"pts\":\"" + b64 + "\",\"geo\":{";
		js += "\"boxes\":[";
		for (size_t i = 0; i < w.brushes.size(); ++i) {
			const WorldBrush& b = w.brushes[i];
			AppendF(&js, "%s[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
				i ? "," : "", b.bmin.X, b.bmin.Y, b.bmin.Z,
				b.bmax.X, b.bmax.Y, b.bmax.Z);
		}
		js += "],\"faces\":[";
		for (size_t i = 0; i < g.faces.size(); ++i) {
			js += i ? ",[" : "[";
			const Route::Face& f = g.faces[i];
			for (size_t v = 0; v < f.verts.size(); ++v)
				AppendF(&js, "%s%.0f,%.0f,%.0f", v ? "," : "",
					f.verts[v].X, f.verts[v].Y, f.verts[v].Z);
			js += "]";
		}
		js += "]";
		{
			std::vector<int> reds;
			w.FindZoneBrushes(nullptr, &reds);
			if (!reds.empty()) {
				const int zi = w.IndexOfBrushId(reds[0]);
				if (zi >= 0) {
					const WorldBrush& zb = w.brushes[zi];
					AppendF(&js, ",\"zone\":[%.0f,%.0f,%.0f,%.0f,"
						"%.0f,%.0f]", zb.bmin.X, zb.bmin.Y,
						zb.bmin.Z - 4.f, zb.bmax.X, zb.bmax.Y,
						zb.bmax.Z + 120.f);
				}
			}
		}
		{
			const Vec3 c = Scale(w.world_min + w.world_max, 0.5f);
			const Vec3 d = w.world_max - w.world_min;
			AppendF(&js, ",\"center\":[%.0f,%.0f,%.0f],\"diag\":%.0f",
				c.X, c.Y, c.Z, Len(d));
		}
		js += "}}";

		std::string html = kHtmlTemplate;
		{
			const std::string tt = "__TITLE__";
			size_t pos;
			while ((pos = html.find(tt)) != std::string::npos)
				html.replace(pos, tt.size(), title);
			const std::string dm = "/*__DATA__*/null";
			pos = html.find(dm);
			if (pos == std::string::npos) {
				if (err) *err = "template marker missing";
				return false;
			}
			html.replace(pos, dm.size(), js);
		}
		FILE* f = nullptr;
		if (fopen_s(&f, path.c_str(), "wb") != 0 || !f) {
			if (err) *err = "cannot open " + path;
			return false;
		}
		fwrite(html.data(), 1, html.size(), f);
		fclose(f);
		return true;
	}

} // namespace SearchLog
} // namespace Solver
