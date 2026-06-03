/* ══════════════════════════════════════════════════════════
   PCAN — Predictive Cone-based Avoidance Navigation
   TCA (Time-to-Closest-Approach) Remake
   ══════════════════════════════════════════════════════════ */

"use strict";

/* ── Constants ───────────────────────────────────────── */
const TAU  = Math.PI * 2;
const DEG  = Math.PI / 180;
const GOAL_RADIUS    = 18;
const DRONE_SIZE     = 12;
const OBS_RADIUS     = 10;
const TRAIL_MAX      = 400;
const ARRIVAL_DIST   = 22;
const STALL_THRESHOLD_SPEED = 0.25;

/* ── Canvas ──────────────────────────────────────────── */
const canvas = document.getElementById("sim-canvas");
const ctx    = canvas.getContext("2d");
let W = 0, H = 0;

function resize() {
  const header = document.getElementById("app-header");
  const side   = document.getElementById("side-panel");
  const hh = header ? header.offsetHeight : 56;
  const sw = (side && side.offsetWidth > 0) ? side.offsetWidth : 340;
  W = Math.max(window.innerWidth  - sw, 300);
  H = Math.max(window.innerHeight - hh, 200);
  canvas.width  = Math.floor(W * devicePixelRatio);
  canvas.height = Math.floor(H * devicePixelRatio);
  canvas.style.width  = W + "px";
  canvas.style.height = H + "px";
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
}
window.addEventListener("resize", () => { resize(); render(); });

/* ── Parameters ──────────────────────────────────────── */
const P = {
  kAtt:           1.5,   // goal attraction strength
  kRep:           200,   // avoidance force strength
  T:              1.5,   // prediction horizon (seconds)
  detectionRange: 250,   // max distance to check obstacles (px)
  safetyMargin:   35,    // extra clearance around obstacles (px)
  stallThreshold: 30,    // frames before escape heuristic
  escapeAngle:    45,    // escape vector angle offset (°)
  escapeStr:      2.5,   // escape vector strength
  maxSpeed:       3,
  obsCount:       6,
  showTrails:     true,
  showPred:       true,
  showVectors:    true,
};

/* ── Vec2 Helpers ────────────────────────────────────── */
function v2(x,y)     { return {x,y}; }
function vAdd(a,b)   { return v2(a.x+b.x, a.y+b.y); }
function vSub(a,b)   { return v2(a.x-b.x, a.y-b.y); }
function vScale(a,s) { return v2(a.x*s, a.y*s); }
function vLen(a)     { return Math.sqrt(a.x*a.x + a.y*a.y); }
function vNorm(a)    { const l=vLen(a); return l<1e-9?v2(0,0):v2(a.x/l,a.y/l); }
function vDot(a,b)   { return a.x*b.x + a.y*b.y; }
function vDist(a,b)  { return vLen(vSub(a,b)); }
function vAngle(a)   { return Math.atan2(a.y,a.x); }
function vRot(a,r)   { const c=Math.cos(r),s=Math.sin(r); return v2(a.x*c-a.y*s, a.x*s+a.y*c); }
function vPerp(a)    { return v2(-a.y, a.x); }  // 90° CCW

/* ── Simulation State ────────────────────────────────── */
let drone, goal, obstacles, trail;
let frame=0, stallCounter=0, running=false, arrived=false;
let lastForces  = { fGoal:v2(0,0), fRep:v2(0,0), fEscape:v2(0,0), fTotal:v2(0,0) };
let lastObsData = [];
let activeStep  = -1;

function randomPos(margin) {
  margin = margin||60;
  return v2(margin + Math.random()*(W-2*margin), margin + Math.random()*(H-2*margin));
}
function randomVel(speed) {
  const a=Math.random()*TAU; return v2(Math.cos(a)*speed, Math.sin(a)*speed);
}
function createObstacles(n) {
  const obs=[];
  for (let i=0;i<n;i++) {
    obs.push({ pos:randomPos(80), vel:randomVel(0.8+Math.random()*1.8),
               radius:OBS_RADIUS+Math.random()*6, id:i });
  }
  return obs;
}

function resetSim(autoPlay) {
  drone     = { pos:v2(80,H/2), vel:v2(0,0), heading:v2(1,0) };
  goal      = v2(W-80, H/2);
  obstacles = createObstacles(P.obsCount);
  trail=[]; frame=0; stallCounter=0; arrived=false;
  running   = !!autoPlay;
  lastForces  = { fGoal:v2(0,0), fRep:v2(0,0), fEscape:v2(0,0), fTotal:v2(0,0) };
  lastObsData = [];
  activeStep  = -1;
  updatePlayIcon();
  updateStatus(running?"Navigating":"Ready", running?"running":"");
}

/* ══════════════════════════════════════════════════════
   PCAN CORE — TCA-based avoidance
   ══════════════════════════════════════════════════════ */
function pcanStep() {
  if (arrived) return;
  frame++;

  /* ── STEP 1: Goal Attraction ────────────────────── */
  activeStep = 1;
  const toGoal = vSub(goal, drone.pos);
  const fGoal  = vScale(vNorm(toGoal), P.kAtt);

  /* ── STEP 2-5: Per-obstacle TCA avoidance ───────── */
  let fRepTotal = v2(0,0);
  const heading = vLen(drone.vel)>0.01 ? vNorm(drone.vel) : drone.heading;
  drone.heading = heading;
  const obsData = [];

  for (const obs of obstacles) {
    /* STEP 2: Relative motion */
    activeStep = 2;
    const relPos  = vSub(obs.pos, drone.pos);
    const dist    = vLen(relPos);
    const pPred   = vAdd(obs.pos, vScale(obs.vel, P.T));

    // Skip obstacles outside detection range
    if (dist > P.detectionRange) {
      obsData.push({ id:obs.id, inCone:false, dist:dist.toFixed(1),
                     alignment:"—", fRep:v2(0,0), pPred, urgency:0, tca:0,
                     collisionPt:null });
      continue;
    }

    /* STEP 3: Time to Closest Approach */
    activeStep = 3;
    const relVel   = vSub(obs.vel, drone.vel);
    const relVelSq = vDot(relVel, relVel);
    // tca = time at which |relPos + relVel*t| is minimised
    let tca = relVelSq > 1e-8 ? -vDot(relPos, relVel) / relVelSq : 0;
    tca = Math.max(0, Math.min(tca, P.T));

    const droneFuture = vAdd(drone.pos, vScale(drone.vel, tca));
    const obsFuture   = vAdd(obs.pos,   vScale(obs.vel,   tca));
    const closestDist = vDist(droneFuture, obsFuture);
    const safeRadius  = obs.radius + DRONE_SIZE + P.safetyMargin;

    /* STEP 4: Urgency */
    activeStep = 4;
    const willCollide = closestDist < safeRadius;
    // urgency 0→1: how deeply we're inside the collision envelope
    const urgency    = willCollide ? Math.max(0, 1 - closestDist/safeRadius) : 0;
    // timeFactor 0→1: sooner collision = more urgent
    const timeFactor = 1 - tca / Math.max(P.T, 0.01);

    /* STEP 5: Avoidance force */
    activeStep = 5;
    let fRep = v2(0,0);
    if (willCollide) {
      // Direction 1: push away from the predicted collision point
      const collisionVec = vSub(drone.pos, obsFuture);
      const awayDir = vLen(collisionVec)>0.01 ? vNorm(collisionVec) : vNorm(vScale(relPos,-1));

      // Direction 2: steer perpendicular to obstacle's velocity, biased toward goal
      const obsVelNorm = vLen(obs.vel)>0.01 ? vNorm(obs.vel) : heading;
      const perp    = vPerp(obsVelNorm);           // perpendicular to obs motion
      const perpAlt = vScale(perp, -1);            // other perpendicular direction
      const goalDir = vNorm(toGoal);
      // Pick the perpendicular that keeps us closer to the goal
      const steerPerp = vDot(perp,goalDir) >= vDot(perpAlt,goalDir) ? perp : perpAlt;

      // Blend: 45% away-from-collision, 55% goal-biased perpendicular steer
      const blended  = vAdd(vScale(awayDir,0.45), vScale(steerPerp,0.55));
      const steerDir = vNorm(blended);

      // Magnitude scales with urgency, time-to-collision, and inverse distance
      const mag = P.kRep * urgency * (1 + timeFactor) / Math.max(dist * 0.25, 1);
      fRep      = vScale(steerDir, mag);
      fRepTotal = vAdd(fRepTotal, fRep);
    }

    obsData.push({ id:obs.id, inCone:willCollide, dist:dist.toFixed(1),
                   alignment:urgency.toFixed(2), fRep, pPred, urgency,
                   tca, collisionPt: willCollide ? obsFuture : null });
  }

  /* ── STEP 6: Stall Escape ───────────────────────── */
  activeStep = 6;
  let fEscape = v2(0,0);
  if (vLen(drone.vel) < STALL_THRESHOLD_SPEED) stallCounter++;
  else stallCounter = Math.max(0, stallCounter-1);

  if (stallCounter >= P.stallThreshold) {
    const side = Math.random()>0.5 ? 1 : -1;
    const escDir = vRot(vNorm(toGoal), P.escapeAngle * DEG * side);
    fEscape      = vScale(escDir, P.escapeStr);
    stallCounter = 0;
    updateStatus("Escape!", "escaped");
  }

  /* ── Integration ────────────────────────────────── */
  const fTotal = vAdd(vAdd(fGoal, fRepTotal), fEscape);
  drone.vel    = vAdd(drone.vel, vScale(fTotal, 0.1));
  if (vLen(drone.vel) > P.maxSpeed) drone.vel = vScale(vNorm(drone.vel), P.maxSpeed);
  drone.vel    = vScale(drone.vel, 0.96);  // damping
  drone.pos    = vAdd(drone.pos, drone.vel);
  drone.pos.x  = Math.max(DRONE_SIZE, Math.min(W-DRONE_SIZE, drone.pos.x));
  drone.pos.y  = Math.max(DRONE_SIZE, Math.min(H-DRONE_SIZE, drone.pos.y));

  trail.push({...drone.pos});
  if (trail.length > TRAIL_MAX) trail.shift();

  /* ── Move obstacles (bounce) ────────────────────── */
  for (const obs of obstacles) {
    obs.pos = vAdd(obs.pos, obs.vel);
    if (obs.pos.x<obs.radius || obs.pos.x>W-obs.radius) obs.vel.x*=-1;
    if (obs.pos.y<obs.radius || obs.pos.y>H-obs.radius) obs.vel.y*=-1;
    obs.pos.x = Math.max(obs.radius, Math.min(W-obs.radius, obs.pos.x));
    obs.pos.y = Math.max(obs.radius, Math.min(H-obs.radius, obs.pos.y));
  }

  /* ── Arrival / status ───────────────────────────── */
  if (vDist(drone.pos,goal) < ARRIVAL_DIST) {
    arrived=true; running=false;
    updatePlayIcon(); updateStatus("Goal Reached!","reached");
  } else if (stallCounter > P.stallThreshold*0.6) {
    updateStatus("Stalling…","stalled");
  } else if (running) {
    updateStatus("Navigating","running");
  }

  lastForces  = { fGoal, fRep:fRepTotal, fEscape, fTotal };
  lastObsData = obsData;
}

/* ══════════════════════════════════════════════════════
   RENDERING
   ══════════════════════════════════════════════════════ */

function drawGrid() {
  ctx.strokeStyle="rgba(255,255,255,0.025)"; ctx.lineWidth=1;
  for (let x=0;x<W;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}
  for (let y=0;y<H;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
}

function drawTrail() {
  if (!P.showTrails||trail.length<2) return;
  for (let i=1;i<trail.length;i++) {
    ctx.beginPath();
    ctx.strokeStyle=`rgba(0,229,255,${(i/trail.length)*0.5})`;
    ctx.lineWidth=1.5;
    ctx.moveTo(trail[i-1].x,trail[i-1].y);
    ctx.lineTo(trail[i].x,trail[i].y);
    ctx.stroke();
  }
}

function drawDetectionRing() {
  // Subtle detection range circle around drone
  ctx.beginPath();
  ctx.arc(drone.pos.x, drone.pos.y, P.detectionRange, 0, TAU);
  ctx.strokeStyle = "rgba(0,229,255,0.07)";
  ctx.lineWidth = 1;
  ctx.setLineDash([4,8]);
  ctx.stroke();
  ctx.setLineDash([]);
  // inner glow
  const grad = ctx.createRadialGradient(
    drone.pos.x, drone.pos.y, 0,
    drone.pos.x, drone.pos.y, P.detectionRange
  );
  grad.addColorStop(0,   "rgba(0,229,255,0.015)");
  grad.addColorStop(0.6, "rgba(0,229,255,0.005)");
  grad.addColorStop(1,   "rgba(0,229,255,0)");
  ctx.beginPath();
  ctx.arc(drone.pos.x, drone.pos.y, P.detectionRange, 0, TAU);
  ctx.fillStyle = grad; ctx.fill();
}

function drawGoal() {
  const grad = ctx.createRadialGradient(goal.x,goal.y,0,goal.x,goal.y,GOAL_RADIUS*2.5);
  grad.addColorStop(0,"rgba(118,255,3,0.15)"); grad.addColorStop(1,"rgba(118,255,3,0)");
  ctx.fillStyle=grad; ctx.beginPath(); ctx.arc(goal.x,goal.y,GOAL_RADIUS*2.5,0,TAU); ctx.fill();
  const pr=GOAL_RADIUS+6+Math.sin(Date.now()*0.004)*4;
  ctx.beginPath(); ctx.arc(goal.x,goal.y,pr,0,TAU);
  ctx.strokeStyle="rgba(118,255,3,0.25)"; ctx.lineWidth=1; ctx.stroke();
  ctx.beginPath(); ctx.arc(goal.x,goal.y,GOAL_RADIUS,0,TAU);
  ctx.fillStyle="rgba(118,255,3,0.2)"; ctx.fill();
  ctx.strokeStyle="#76ff03"; ctx.lineWidth=2; ctx.stroke();
  ctx.strokeStyle="rgba(118,255,3,0.5)"; ctx.lineWidth=1;
  ctx.beginPath();
  ctx.moveTo(goal.x-8,goal.y); ctx.lineTo(goal.x+8,goal.y);
  ctx.moveTo(goal.x,goal.y-8); ctx.lineTo(goal.x,goal.y+8);
  ctx.stroke();
}

function drawArrowhead(tip, dir, size, color) {
  const l=vAdd(tip,vScale(vRot(dir, Math.PI*0.82),size));
  const r=vAdd(tip,vScale(vRot(dir,-Math.PI*0.82),size));
  ctx.beginPath(); ctx.moveTo(tip.x,tip.y);
  ctx.lineTo(l.x,l.y); ctx.lineTo(r.x,r.y);
  ctx.closePath(); ctx.fillStyle=color; ctx.fill();
}

function drawObstacles() {
  for (const data of lastObsData) {
    const obs = obstacles[data.id];
    const threat = data.inCone;

    // Predicted position ghost
    if (P.showPred) {
      ctx.beginPath(); ctx.arc(data.pPred.x, data.pPred.y, obs.radius, 0, TAU);
      ctx.fillStyle   = threat ? "rgba(255,23,68,0.10)" : "rgba(255,23,68,0.03)"; ctx.fill();
      ctx.setLineDash([3,3]);
      ctx.strokeStyle = threat ? "rgba(255,23,68,0.30)" : "rgba(255,23,68,0.08)";
      ctx.lineWidth=1; ctx.stroke(); ctx.setLineDash([]);
      // line from current → predicted
      ctx.beginPath(); ctx.moveTo(obs.pos.x,obs.pos.y); ctx.lineTo(data.pPred.x,data.pPred.y);
      ctx.strokeStyle="rgba(255,23,68,0.12)"; ctx.lineWidth=1;
      ctx.setLineDash([2,4]); ctx.stroke(); ctx.setLineDash([]);
    }

    // Collision point marker (where TCA says they'd meet)
    if (P.showPred && data.collisionPt) {
      ctx.beginPath(); ctx.arc(data.collisionPt.x, data.collisionPt.y, 5, 0, TAU);
      ctx.fillStyle="rgba(255,50,50,0.55)"; ctx.fill();
      ctx.strokeStyle="rgba(255,100,100,0.6)"; ctx.lineWidth=1.5; ctx.stroke();
      // line drone → collision point
      ctx.beginPath();
      ctx.moveTo(drone.pos.x,drone.pos.y); ctx.lineTo(data.collisionPt.x,data.collisionPt.y);
      ctx.strokeStyle="rgba(255,50,50,0.15)"; ctx.lineWidth=1;
      ctx.setLineDash([3,5]); ctx.stroke(); ctx.setLineDash([]);
    }

    // Obstacle glow
    const glow = ctx.createRadialGradient(obs.pos.x,obs.pos.y,0,obs.pos.x,obs.pos.y,obs.radius*2.5);
    glow.addColorStop(0, threat?"rgba(255,23,68,0.25)":"rgba(255,23,68,0.07)");
    glow.addColorStop(1, "rgba(255,23,68,0)");
    ctx.beginPath(); ctx.arc(obs.pos.x,obs.pos.y,obs.radius*2.5,0,TAU); ctx.fillStyle=glow; ctx.fill();

    // Obstacle body — urgency-tinted
    const alpha = 0.25 + data.urgency*0.5;
    ctx.beginPath(); ctx.arc(obs.pos.x,obs.pos.y,obs.radius,0,TAU);
    ctx.fillStyle=`rgba(255,23,68,${alpha})`; ctx.fill();
    ctx.strokeStyle = threat ? "#ff1744" : "rgba(255,23,68,0.35)";
    ctx.lineWidth = threat ? 2 : 1; ctx.stroke();

    // Velocity arrow
    if (vLen(obs.vel)>0.1) {
      const tip=vAdd(obs.pos,vScale(vNorm(obs.vel),obs.radius+14));
      ctx.beginPath(); ctx.moveTo(obs.pos.x,obs.pos.y); ctx.lineTo(tip.x,tip.y);
      ctx.strokeStyle="rgba(255,23,68,0.45)"; ctx.lineWidth=1.5; ctx.stroke();
      drawArrowhead(tip,vNorm(obs.vel),5,"rgba(255,23,68,0.45)");
    }

    // Safety radius ring (when threatening)
    if (threat) {
      const safeR = obs.radius + DRONE_SIZE + P.safetyMargin;
      ctx.beginPath(); ctx.arc(obs.pos.x,obs.pos.y,safeR,0,TAU);
      ctx.strokeStyle=`rgba(255,100,0,${0.1+data.urgency*0.3})`;
      ctx.lineWidth=1; ctx.setLineDash([3,4]); ctx.stroke(); ctx.setLineDash([]);
    }

    // ID label
    ctx.fillStyle=threat?"rgba(255,255,255,0.7)":"rgba(255,255,255,0.25)";
    ctx.font="bold 8px 'Inter'"; ctx.textAlign="center";
    ctx.fillText(`#${data.id}`,obs.pos.x,obs.pos.y-obs.radius-5);
  }

  // Fallback if no data yet
  if (lastObsData.length===0) {
    for (const obs of obstacles) {
      ctx.beginPath(); ctx.arc(obs.pos.x,obs.pos.y,obs.radius,0,TAU);
      ctx.fillStyle="rgba(255,23,68,0.2)"; ctx.fill();
      ctx.strokeStyle="rgba(255,23,68,0.4)"; ctx.lineWidth=1; ctx.stroke();
    }
  }
}

function drawDrone() {
  const p=drone.pos, angle=vAngle(drone.heading);
  const glow=ctx.createRadialGradient(p.x,p.y,0,p.x,p.y,DRONE_SIZE*3);
  glow.addColorStop(0,"rgba(0,229,255,0.18)"); glow.addColorStop(1,"rgba(0,229,255,0)");
  ctx.beginPath(); ctx.arc(p.x,p.y,DRONE_SIZE*3,0,TAU); ctx.fillStyle=glow; ctx.fill();
  ctx.save(); ctx.translate(p.x,p.y); ctx.rotate(angle);
  ctx.beginPath();
  ctx.moveTo(DRONE_SIZE,0);
  ctx.lineTo(-DRONE_SIZE*0.5,-DRONE_SIZE*0.6);
  ctx.lineTo(-DRONE_SIZE*0.3,0);
  ctx.lineTo(-DRONE_SIZE*0.5, DRONE_SIZE*0.6);
  ctx.closePath();
  ctx.fillStyle="rgba(0,229,255,0.25)"; ctx.fill();
  ctx.strokeStyle="#00e5ff"; ctx.lineWidth=1.5; ctx.stroke();
  ctx.beginPath(); ctx.arc(0,0,3,0,TAU); ctx.fillStyle="#00e5ff"; ctx.fill();
  ctx.restore();
}

function drawVector(origin, vec, scale, color, label) {
  if (vLen(vec)<0.001) return;
  const tip=vAdd(origin,vScale(vec,scale));
  ctx.beginPath(); ctx.moveTo(origin.x,origin.y); ctx.lineTo(tip.x,tip.y);
  ctx.strokeStyle=color; ctx.lineWidth=2; ctx.stroke();
  drawArrowhead(tip,vNorm(vec),7,color);
  if (label) {
    ctx.fillStyle=color; ctx.font="bold 9px 'Inter'";
    ctx.textAlign="left"; ctx.fillText(label,tip.x+6,tip.y-4);
  }
}

function drawForceVectors() {
  if (!P.showVectors) return;
  const p=drone.pos;
  drawVector(p, lastForces.fGoal,  25, "#00e5ff", "F_goal");
  if (vLen(lastForces.fRep)    > 0.01) drawVector(p, lastForces.fRep,    22, "#ff9100", "F_rep");
  if (vLen(lastForces.fEscape) > 0.01) drawVector(p, lastForces.fEscape, 25, "#e040fb", "F_esc");
  drawVector(p, lastForces.fTotal, 28, "rgba(255,255,255,0.75)", "");
}

function render() {
  ctx.clearRect(0,0,W,H);
  const bg=ctx.createRadialGradient(W/2,H/2,0,W/2,H/2,Math.max(W,H)*0.7);
  bg.addColorStop(0,"#0f1520"); bg.addColorStop(1,"#080c14");
  ctx.fillStyle=bg; ctx.fillRect(0,0,W,H);
  drawGrid(); drawTrail(); drawDetectionRing(); drawGoal();
  drawObstacles(); drawForceVectors(); drawDrone();
  // goal line
  ctx.beginPath(); ctx.moveTo(drone.pos.x,drone.pos.y); ctx.lineTo(goal.x,goal.y);
  ctx.strokeStyle="rgba(118,255,3,0.05)"; ctx.lineWidth=1;
  ctx.setLineDash([6,8]); ctx.stroke(); ctx.setLineDash([]);
}

/* ── HUD ─────────────────────────────────────────────── */
const hudFrameEl   = document.getElementById("hud-frame-val");
const hudStallEl   = document.getElementById("hud-stall-val");
const hudDistEl    = document.getElementById("hud-dist-val");
const hudSpeedEl   = document.getElementById("hud-speed-val");
const hudStallItem = document.getElementById("hud-stall");

function updateHUD() {
  hudFrameEl.textContent = frame;
  hudStallEl.textContent = stallCounter;
  hudDistEl.textContent  = drone ? vDist(drone.pos,goal).toFixed(0)+"px" : "—";
  hudSpeedEl.textContent = drone ? vLen(drone.vel).toFixed(2) : "—";
  const ratio = stallCounter / P.stallThreshold;
  hudStallItem.classList.toggle("warning",  ratio>0.4 && ratio<0.8);
  hudStallItem.classList.toggle("critical", ratio>=0.8);
}

/* ── Vectors Panel ───────────────────────────────────── */
const vecGoalEl     = document.getElementById("vec-goal");
const vecGoalMagEl  = document.getElementById("vec-goal-mag");
const vecRepEl      = document.getElementById("vec-rep");
const vecRepMagEl   = document.getElementById("vec-rep-mag");
const vecEscEl      = document.getElementById("vec-escape");
const vecEscMagEl   = document.getElementById("vec-escape-mag");
const vecTotalEl    = document.getElementById("vec-total");
const vecTotalMagEl = document.getElementById("vec-total-mag");
const obsBreakdownEl= document.getElementById("obs-breakdown");

function updateVectorsPanel() {
  const fmt=v=>`(${v.x.toFixed(2)}, ${v.y.toFixed(2)})`;
  const mag=v=>`|${vLen(v).toFixed(3)}|`;
  vecGoalEl.textContent     = fmt(lastForces.fGoal);
  vecGoalMagEl.textContent  = mag(lastForces.fGoal);
  vecRepEl.textContent      = fmt(lastForces.fRep);
  vecRepMagEl.textContent   = mag(lastForces.fRep);
  vecEscEl.textContent      = fmt(lastForces.fEscape);
  vecEscMagEl.textContent   = mag(lastForces.fEscape);
  vecTotalEl.textContent    = fmt(lastForces.fTotal);
  vecTotalMagEl.textContent = mag(lastForces.fTotal);
  let html="";
  for (const d of lastObsData) {
    html+=`<div class="obs-card ${d.inCone?'in-cone':''}">
      <div class="obs-header">
        <span class="obs-id">Obstacle #${d.id}</span>
        <span class="obs-status ${d.inCone?'in':'out'}">${d.inCone?'THREAT':'SAFE'}</span>
      </div>
      <div class="obs-detail">
        <span>d=${d.dist}</span>
        <span>urg=${d.alignment}</span>
        <span>|F|=${vLen(d.fRep).toFixed(3)}</span>
      </div></div>`;
  }
  obsBreakdownEl.innerHTML=html;
}

/* ── Step Highlight ──────────────────────────────────── */
const stepCards=[1,2,3,4,5,6].map(i=>document.getElementById(`step-${i}`));
function updateStepHighlight() {
  stepCards.forEach((c,i)=>c.classList.toggle("active",i+1===activeStep));
}

/* ── Status Badge ────────────────────────────────────── */
const statusBadge=document.getElementById("status-badge");
function updateStatus(text,cls) {
  statusBadge.querySelector(".status-text").textContent=text;
  statusBadge.className="status-badge";
  if(cls) statusBadge.classList.add(cls);
}

/* ── Play Icon ───────────────────────────────────────── */
const iconPlay  = document.getElementById("icon-play");
const iconPause = document.getElementById("icon-pause");
function updatePlayIcon() {
  iconPlay.style.display  = running?"none":"block";
  iconPause.style.display = running?"block":"none";
}

/* ── Main Loop ───────────────────────────────────────── */
function loop() {
  if (running && !arrived) pcanStep();
  render(); updateHUD();
  if (frame%3===0) { updateVectorsPanel(); updateStepHighlight(); }
  requestAnimationFrame(loop);
}

/* ── Controls ────────────────────────────────────────── */
document.getElementById("btn-play").addEventListener("click",()=>{
  if (arrived) { resize(); resetSim(true); return; }
  running=!running; updatePlayIcon();
  updateStatus(running?"Navigating":"Paused", running?"running":"");
});
document.getElementById("btn-step").addEventListener("click",()=>{
  if (arrived) return;
  running=false; updatePlayIcon();
  pcanStep(); updateVectorsPanel(); updateStepHighlight();
  updateStatus("Stepped","");
});
document.getElementById("btn-reset").addEventListener("click",()=>{ resize(); resetSim(true); });
document.getElementById("btn-add-obs").addEventListener("click",()=>{
  obstacles.push({ pos:randomPos(80), vel:randomVel(0.8+Math.random()*1.8),
                   radius:OBS_RADIUS+Math.random()*6, id:obstacles.length });
});
document.getElementById("btn-clear-obs").addEventListener("click",()=>{ obstacles=[]; lastObsData=[]; });

/* ── Canvas interaction ──────────────────────────────── */
let dragging=null;
canvas.addEventListener("mousedown",e=>{
  const r=canvas.getBoundingClientRect();
  const mx=e.clientX-r.left, my=e.clientY-r.top;
  if (vDist(v2(mx,my),drone.pos)<20) { dragging="drone"; return; }
  if (vDist(v2(mx,my),goal)<25)      { dragging="goal";  return; }
  goal=v2(mx,my); arrived=false; trail=[];
});
canvas.addEventListener("mousemove",e=>{
  if (!dragging) return;
  const r=canvas.getBoundingClientRect();
  const mx=e.clientX-r.left, my=e.clientY-r.top;
  if (dragging==="drone") { drone.pos=v2(mx,my); trail=[]; }
  else                    { goal=v2(mx,my); arrived=false; }
});
canvas.addEventListener("mouseup",   ()=>{ dragging=null; });
canvas.addEventListener("mouseleave",()=>{ dragging=null; });

/* ── Slider wiring ───────────────────────────────────── */
[
  ["param-k-att",        "val-k-att",        "kAtt",           parseFloat],
  ["param-k-rep",        "val-k-rep",        "kRep",           parseFloat],
  ["param-T",            "val-T",            "T",              parseFloat],
  ["param-det-range",    "val-det-range",    "detectionRange", parseFloat],
  ["param-safety",       "val-safety",       "safetyMargin",   parseFloat],
  ["param-stall",        "val-stall",        "stallThreshold", parseInt],
  ["param-escape-angle", "val-escape-angle", "escapeAngle",    parseFloat],
  ["param-escape-str",   "val-escape-str",   "escapeStr",      parseFloat],
  ["param-max-speed",    "val-max-speed",    "maxSpeed",       parseFloat],
  ["param-obs-count",    "val-obs-count",    "obsCount",       parseInt],
].forEach(([sid,vid,key,parser])=>{
  const sl=document.getElementById(sid), vl=document.getElementById(vid);
  if (!sl||!vl) return;
  sl.addEventListener("input",()=>{ P[key]=parser(sl.value); vl.textContent=sl.value; });
});
document.getElementById("param-show-trails").addEventListener("change",e=>P.showTrails=e.target.checked);
document.getElementById("param-show-pred").addEventListener("change",  e=>P.showPred=e.target.checked);
document.getElementById("param-show-vectors").addEventListener("change",e=>P.showVectors=e.target.checked);
document.getElementById("param-obs-count").addEventListener("change",()=>{
  obstacles=createObstacles(P.obsCount); lastObsData=[];
});

/* ── Tabs ────────────────────────────────────────────── */
document.querySelectorAll(".tab").forEach(tab=>{
  tab.addEventListener("click",()=>{
    document.querySelectorAll(".tab").forEach(t=>t.classList.remove("active"));
    document.querySelectorAll(".tab-content").forEach(tc=>tc.classList.remove("active"));
    tab.classList.add("active");
    document.getElementById(`tab-${tab.dataset.tab}`).classList.add("active");
  });
});

/* ── Help Modal ──────────────────────────────────────── */
const modal=document.getElementById("help-modal");
document.getElementById("btn-help").addEventListener("click",      ()=>modal.classList.remove("hidden"));
document.getElementById("btn-close-help").addEventListener("click",()=>modal.classList.add("hidden"));
modal.addEventListener("click",e=>{ if(e.target===modal) modal.classList.add("hidden"); });

/* ── Keyboard shortcuts ──────────────────────────────── */
document.addEventListener("keydown",e=>{
  if (e.target.tagName==="INPUT") return;
  if      (e.key===" ")      { e.preventDefault(); document.getElementById("btn-play").click(); }
  else if (e.key==="s")      { document.getElementById("btn-step").click(); }
  else if (e.key==="r")      { document.getElementById("btn-reset").click(); }
  else if (e.key==="Escape") { modal.classList.add("hidden"); }
});

/* ── Startup ─────────────────────────────────────────── */
window.addEventListener("load",()=>{
  resize();
  resetSim(true);
  loop();
});
