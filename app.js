(function () {
  const DATA = window.RACE_DATA;
  const meta = DATA.meta;
  const trackData = DATA.track;
  const scenarios = DATA.scenarios;

  const css = getComputedStyle(document.documentElement);
  const COLOR = {
    accent: css.getPropertyValue('--accent').trim() || '#4fd1c5',
    good: css.getPropertyValue('--good').trim() || '#57d38c',
    warn: css.getPropertyValue('--warn').trim() || '#e8b34d',
    crit: css.getPropertyValue('--crit').trim() || '#e5566b',
    thrust: css.getPropertyValue('--thrust').trim() || '#ff8a5c',
    line: css.getPropertyValue('--line').trim() || '#22303f',
    lineSoft: css.getPropertyValue('--line-soft').trim() || '#1a2432',
    panel2: css.getPropertyValue('--panel-2').trim() || '#131b29',
    text: css.getPropertyValue('--text').trim() || '#dfe6ee',
    textDim: css.getPropertyValue('--text-dim').trim() || '#7c8aa0',
    textFaint: css.getPropertyValue('--text-faint').trim() || '#4d5b70',
  };

  const sV0 = document.getElementById('sV0');
  const sObs = document.getElementById('sObs');
  sV0.max = meta.v_targets.length - 1;
  sObs.max = meta.obstacle_names.length - 1;

  function keyFor(vi, oi) {
    return meta.v_targets[vi].toFixed(0) + '|' + meta.obstacle_names[oi];
  }

  function fmtSigned(v, digits) {
    const s = v.toFixed(digits);
    return (v > 0 ? '+' : '') + s;
  }

  function obsLabel(name) {
    return { none: 'None', bottom_straight: 'Bottom straight', top_straight: 'Top straight' }[name] || name;
  }

  function updateLabels() {
    document.getElementById('lblV0').textContent = meta.v_targets[+sV0.value].toFixed(0) + ' m/s';
    document.getElementById('lblObs').textContent = obsLabel(meta.obstacle_names[+sObs.value]);
  }

  let traj = null;
  let playing = false;
  let playT = 0;
  let lastFrameTime = null;

  const scrub = document.getElementById('scrub');
  const playBtn = document.getElementById('playBtn');
  const metLabel = document.getElementById('metLabel');
  const statusDot = document.getElementById('statusDot');
  const statusText = document.getElementById('statusText');

  function loadScenario() {
    traj = scenarios[keyFor(+sV0.value, +sObs.value)];
    updateLabels();
    document.getElementById('rLapTime').textContent = traj.lap_time.toFixed(1) + ' s';
    const avgSpeed = meta.lap_length / traj.lap_time;
    document.getElementById('rAvgSpeed').textContent = avgSpeed.toFixed(1) + ' m/s';
    playT = 0;
    scrub.value = 0;
    render();
  }

  function sampleAt(frac) {
    const n = traj.t.length;
    const pos = frac * (n - 1);
    const i0 = Math.max(0, Math.min(n - 2, Math.floor(pos)));
    const f = pos - i0;
    const i1 = i0 + 1;
    function lerp(arr) { return arr[i0] + (arr[i1] - arr[i0]) * f; }
    function lerpAngle(arr) {
      const a = arr[i0], b = arr[i1];
      let d = b - a;
      while (d > Math.PI) d -= 2 * Math.PI;
      while (d < -Math.PI) d += 2 * Math.PI;
      return a + d * f;
    }
    return {
      idx: i0, frac: f,
      t: lerp(traj.t), s: lerp(traj.s), ey: lerp(traj.ey), epsi: lerpAngle(traj.epsi),
      v: lerp(traj.v), delta: lerp(traj.delta), a: lerp(traj.a),
      x: lerp(traj.x), y: lerp(traj.y), heading: lerpAngle(traj.heading),
    };
  }

  // ---- scene ----
  const scene = document.getElementById('scene');
  const sctx = scene.getContext('2d');
  const WORLD = { xMin: -26, xMax: 66, yMin: -12, yMax: 42 };
  const PAD = { left: 40, right: 40, top: 30, bottom: 30 };

  function worldToScreen(wx, wy) {
    const availW = scene.width - PAD.left - PAD.right;
    const availH = scene.height - PAD.top - PAD.bottom;
    const scale = Math.min(availW / (WORLD.xMax - WORLD.xMin), availH / (WORLD.yMax - WORLD.yMin));
    const cx = PAD.left + availW / 2, cy = PAD.top + availH / 2;
    const wcx = (WORLD.xMin + WORLD.xMax) / 2, wcy = (WORLD.yMin + WORLD.yMax) / 2;
    return { x: cx + (wx - wcx) * scale, y: cy - (wy - wcy) * scale, scale };
  }

  function pathScreen(xs, ys) {
    return xs.map((x, i) => worldToScreen(x, ys[i]));
  }

  function drawScene(sample) {
    const ctx = sctx;
    ctx.clearRect(0, 0, scene.width, scene.height);

    const left = pathScreen(trackData.left_edge.x, trackData.left_edge.y);
    const right = pathScreen(trackData.right_edge.x, trackData.right_edge.y);
    const center = pathScreen(trackData.centerline.x, trackData.centerline.y);

    // corridor fill
    ctx.fillStyle = COLOR.panel2;
    ctx.beginPath();
    left.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
    for (let i = right.length - 1; i >= 0; i--) ctx.lineTo(right[i].x, right[i].y);
    ctx.closePath();
    ctx.fill();

    // edges
    ctx.strokeStyle = COLOR.line;
    ctx.lineWidth = 2;
    [left, right].forEach((edge) => {
      ctx.beginPath();
      edge.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
      ctx.closePath();
      ctx.stroke();
    });

    // centerline dashed
    ctx.save();
    ctx.setLineDash([8, 8]);
    ctx.strokeStyle = COLOR.lineSoft;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    center.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
    ctx.closePath();
    ctx.stroke();
    ctx.restore();

    // obstacle
    const obsName = meta.obstacle_names[+sObs.value];
    const obs = trackData.obstacles[obsName];
    if (obs) {
      const p = worldToScreen(obs.x, obs.y);
      const r = obs.radius * (worldToScreen(0, 0).scale);
      ctx.fillStyle = 'rgba(229,86,107,0.25)';
      ctx.strokeStyle = COLOR.crit;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // flown trail
    const flown = pathScreen(traj.x.slice(0, sample.idx + 1), traj.y.slice(0, sample.idx + 1));
    ctx.strokeStyle = COLOR.accent;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    flown.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
    const cur = worldToScreen(sample.x, sample.y);
    ctx.lineTo(cur.x, cur.y);
    ctx.stroke();

    // car icon (top-down), heading: screen angle = -heading (world CCW -> canvas CW flip)
    ctx.save();
    ctx.translate(cur.x, cur.y);
    ctx.rotate(-sample.heading);
    ctx.fillStyle = COLOR.text;
    ctx.strokeStyle = COLOR.accent;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(11, 0);
    ctx.lineTo(-7, 6);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-7, -6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  // ---- strip charts (shared style with the lander project) ----
  function hexToRgba(hex, alpha) {
    const h = hex.replace('#', '');
    const r = parseInt(h.substring(0, 2), 16);
    const g = parseInt(h.substring(2, 4), 16);
    const b = parseInt(h.substring(4, 6), 16);
    return `rgba(${r},${g},${b},${alpha})`;
  }

  function drawStrip(canvas, series, opts) {
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const padL = 4, padR = 4, padT = 10, padB = 14;
    const min = opts.min !== undefined ? opts.min : Math.min(...series);
    const max = opts.max !== undefined ? opts.max : Math.max(...series);
    const range = (max - min) || 1;
    function xy(i) {
      const x = padL + (i / (series.length - 1)) * (w - padL - padR);
      const y = padT + (1 - (series[i] - min) / range) * (h - padT - padB);
      return [x, y];
    }
    if (opts.refLines) {
      ctx.strokeStyle = COLOR.lineSoft;
      ctx.lineWidth = 1;
      opts.refLines.forEach((v) => {
        const y = padT + (1 - (v - min) / range) * (h - padT - padB);
        ctx.beginPath();
        ctx.setLineDash([3, 4]);
        ctx.moveTo(padL, y);
        ctx.lineTo(w - padR, y);
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }
    ctx.fillStyle = hexToRgba(opts.color, 0.28);
    ctx.beginPath();
    ctx.moveTo(padL, h - padB);
    for (let i = 0; i < series.length; i++) { const [x, y] = xy(i); ctx.lineTo(x, y); }
    ctx.lineTo(w - padR, h - padB);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = opts.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < series.length; i++) { const [x, y] = xy(i); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }
    ctx.stroke();
    const frac = opts.playFrac;
    const px = padL + frac * (w - padL - padR);
    ctx.strokeStyle = COLOR.textFaint;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px, padT);
    ctx.lineTo(px, h - padB);
    ctx.stroke();
    const curY = padT + (1 - (opts.currentValue - min) / range) * (h - padT - padB);
    ctx.fillStyle = opts.color;
    ctx.beginPath();
    ctx.arc(px, curY, 3.2, 0, Math.PI * 2);
    ctx.fill();
  }

  function updateTelemetry(sample) {
    document.getElementById('tSpeed').innerHTML = sample.v.toFixed(1) + '<span class="unit">m/s</span>';
    const steerDeg = sample.delta * 180 / Math.PI;
    const steerEl = document.getElementById('tSteer');
    steerEl.innerHTML = fmtSigned(steerDeg, 1) + '<span class="unit">&deg;</span>';
    steerEl.className = 'tele-val ' + (Math.abs(steerDeg) > (meta.delta_max * 180 / Math.PI) * 0.9 ? 'warn' : 'good');
    document.getElementById('tAccel').innerHTML = fmtSigned(sample.a, 2) + '<span class="unit">m/s&sup2;</span>';
    const eyEl = document.getElementById('tEy');
    eyEl.innerHTML = fmtSigned(sample.ey, 2) + '<span class="unit">m</span>';
    eyEl.className = 'tele-val ' + (Math.abs(sample.ey) > meta.ey_lim * 0.85 ? 'warn' : 'good');
    document.getElementById('tEpsi').innerHTML = fmtSigned(sample.epsi * 180 / Math.PI, 1) + '<span class="unit">&deg;</span>';
    document.getElementById('tProg').innerHTML = (100 * sample.s / meta.lap_length).toFixed(0) + '<span class="unit">%</span>';

    const nearLimit = Math.abs(sample.ey) > meta.ey_lim * 0.85 || Math.abs(steerDeg) > (meta.delta_max * 180 / Math.PI) * 0.9;
    statusText.textContent = nearLimit ? 'NEAR LIMIT' : 'NOMINAL';
    statusDot.className = 'status-dot ' + (nearLimit ? 'warn' : '');

    metLabel.textContent = 'T+' + sample.t.toFixed(1) + 's / ' + traj.lap_time.toFixed(1) + 's';
  }

  function render() {
    const frac = playT;
    const sample = sampleAt(frac);
    drawScene(sample);
    updateTelemetry(sample);
    drawStrip(document.getElementById('stripSpeed'), traj.v, {
      color: COLOR.accent, min: 0, max: meta.v_max, playFrac: frac, currentValue: sample.v,
    });
    const steerSeries = traj.delta.map((d) => d * 180 / Math.PI);
    drawStrip(document.getElementById('stripSteer'), steerSeries, {
      color: COLOR.thrust,
      min: -meta.delta_max * 180 / Math.PI, max: meta.delta_max * 180 / Math.PI,
      refLines: [0],
      playFrac: frac, currentValue: sample.delta * 180 / Math.PI,
    });
  }

  scrub.addEventListener('input', () => {
    playing = false;
    playBtn.textContent = '▶ PLAY';
    playT = (+scrub.value) / 1000;
    render();
  });

  playBtn.addEventListener('click', () => {
    playing = !playing;
    playBtn.textContent = playing ? '⏸ PAUSE' : '▶ PLAY';
    if (playing) {
      if (playT >= 0.999) playT = 0;
      lastFrameTime = null;
      requestAnimationFrame(tick);
    }
  });

  function tick(now) {
    if (!playing) return;
    if (lastFrameTime == null) lastFrameTime = now;
    const dt = (now - lastFrameTime) / 1000;
    lastFrameTime = now;
    const speed = 1 / traj.lap_time;
    playT += dt * speed;
    if (playT >= 1) {
      playT = 1;
      playing = false;
      playBtn.textContent = '▶ PLAY';
    }
    scrub.value = Math.round(playT * 1000);
    render();
    if (playing) requestAnimationFrame(tick);
  }

  [sV0, sObs].forEach((el) => el.addEventListener('input', loadScenario));

  loadScenario();
})();
