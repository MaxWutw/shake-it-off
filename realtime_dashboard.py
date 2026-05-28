#!/usr/bin/env python3
"""
realtime_dashboard.py — Shake-It-Off 即時視覺化儀表板
=====================================================
Dependencies:  pip install pyserial websockets
Usage:
  python3 realtime_dashboard.py /dev/ttyACM0     # 實體硬體
  python3 realtime_dashboard.py --demo            # 模擬資料（不需硬體）

開啟瀏覽器: http://localhost:8080
"""
import asyncio, websockets, threading, json, queue, time, math, random, sys, re
from http.server import HTTPServer, BaseHTTPRequestHandler

try:
    import serial
except ImportError:
    serial = None

HTTP_PORT  = 8080
WS_PORT    = 8765
BAUD_RATE  = 115200
DECIMATION = 10   # 1000Hz → 100Hz 傳給瀏覽器

_q       = queue.Queue(maxsize=2000)
_clients = set()

# ── Parse ─────────────────────────────────────────────────────

def parse_line(line: str):
    if line.startswith('DATA,'):
        p = line.split(',')
        if len(p) < 16:
            return None
        try:
            ti, tf, tp, tg, ts = int(p[9]), int(p[10]), int(p[11]), int(p[12]), int(p[13])
            return {
                'type':    'data',
                'tick':    int(p[1]),
                'pitch':   float(p[2]),
                'roll':    float(p[3]),
                'tgt_p':   float(p[4]),
                'tgt_r':   float(p[5]),
                'gx':      float(p[6]),
                'gy':      float(p[7]),
                'dt_us':   int(p[8]),
                't_imu':   ti,
                't_total': ti + tf + tp + tg + ts,
                'miss':    int(p[14]),
                'state':   int(p[15].strip()),
            }
        except (ValueError, IndexError):
            return None
    if line.startswith('STEP_RESP,'):
        m = re.search(r'peak=([\d.]+),settle=([\d.]+)ms,resp#(\d+)', line)
        if m:
            return {'type': 'step', 'peak': float(m.group(1)),
                    'settle_ms': float(m.group(2)), 'resp_num': int(m.group(3))}
    return None

# ── Serial thread ──────────────────────────────────────────────

def serial_thread(port: str, baud: int):
    if serial is None:
        print('[serial] pyserial 未安裝: pip install pyserial'); return
    print(f'[serial] 開啟 {port} @ {baud}')
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print('[serial] 已連線')
            dec = 0
            while True:
                raw = ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw: continue
                msg = parse_line(raw)
                if msg is None: continue
                if msg['type'] == 'step':
                    try: _q.put_nowait(msg)
                    except queue.Full: pass
                else:
                    dec += 1
                    if dec >= DECIMATION:
                        dec = 0
                        try: _q.put_nowait(msg)
                        except queue.Full: pass
        except Exception as e:
            print(f'[serial] {e}，2s 後重試…')
            time.sleep(2)

# ── Demo thread ────────────────────────────────────────────────

def demo_thread():
    print('[demo] 產生模擬資料')
    rng = random.Random(42)
    t_ms = 5000
    pitch = roll = 0.0
    disturb_at = None
    disturb_pitch = disturb_roll = 0.0
    resp_count = miss = 0
    prev_state = 0
    peak_mag = 0.0

    while True:
        t_ms += 10
        # 每 ~14s 施加一次擾動
        if t_ms % 14000 < 11 and t_ms > 2000:
            disturb_pitch = rng.uniform(7, 12) * rng.choice([-1, 1])
            disturb_roll  = rng.uniform(3,  7) * rng.choice([-1, 1])
            disturb_at    = t_ms
            peak_mag      = 0.0
            resp_count   += 1

        if disturb_at:
            elapsed = (t_ms - disturb_at) / 1000.0
            zeta, wn = 0.45, 7.0
            wd  = wn * math.sqrt(max(0, 1 - zeta**2))
            env = math.exp(-zeta * wn * elapsed)
            pitch = disturb_pitch * env * math.cos(wd * elapsed)
            roll  = disturb_roll  * env * math.cos(wd * elapsed + 0.4)
        else:
            pitch *= 0.92; roll *= 0.92

        pn = pitch + rng.gauss(0, 0.07)
        rn = roll  + rng.gauss(0, 0.07)
        mag = math.sqrt(pn**2 + rn**2)
        if mag > peak_mag: peak_mag = mag

        if disturb_at:
            elapsed_ms = t_ms - disturb_at
            state = 1 if elapsed_ms < 60 else (2 if mag > 0.5 else 3)
        else:
            state = 0

        # Fire STEP_RESP when transitioning back to idle
        if prev_state == 3 and state == 0 and disturb_at:
            settle_ms = float(t_ms - disturb_at)
            try:
                _q.put_nowait({'type': 'step', 'peak': round(peak_mag, 2),
                               'settle_ms': round(settle_ms, 1), 'resp_num': resp_count})
            except queue.Full: pass
            disturb_at = None
        prev_state = state

        ti = max(200, int(rng.gauss(430, 30)))
        if rng.random() < 0.003:
            ti = int(rng.gauss(1100, 150)); miss += 1
        t_total = ti + int(rng.gauss(33, 5))

        try:
            _q.put_nowait({'type': 'data', 'tick': t_ms,
                           'pitch': round(pn, 3), 'roll': round(rn, 3),
                           'tgt_p': round(-pn*0.15, 3), 'tgt_r': round(-rn*0.15, 3),
                           'gx': round(rng.gauss(0, 2), 2), 'gy': round(rng.gauss(0, 2), 2),
                           'dt_us': 1000, 't_imu': ti, 't_total': t_total,
                           'miss': miss, 'state': state})
        except queue.Full: pass
        time.sleep(0.010)

# ── WebSocket ──────────────────────────────────────────────────

async def ws_handler(websocket):
    _clients.add(websocket)
    print(f'[ws] 連線（共 {len(_clients)}）')
    try:
        async for _ in websocket:   # drain; exits when client disconnects
            pass
    except Exception:
        pass
    finally:
        _clients.discard(websocket)

async def broadcast_loop():
    while True:
        try:
            msg = _q.get_nowait()
            if _clients:
                payload = json.dumps(msg)
                await asyncio.gather(
                    *[c.send(payload) for c in list(_clients)],
                    return_exceptions=True)
        except queue.Empty:
            await asyncio.sleep(0.004)

# ── HTML ───────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shake-It-Off — Live</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
*,*::before,*::after{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:#0f172a;color:#e2e8f0;font-family:'Segoe UI',system-ui,sans-serif;font-size:14px}
body{display:flex;flex-direction:column}
header{display:flex;justify-content:space-between;align-items:center;padding:10px 18px;border-bottom:1px solid #1e293b;flex-shrink:0}
header h1{font-size:.95rem;font-weight:700;color:#f8fafc}
header p{font-size:.65rem;color:#64748b;margin-top:2px}
.badge{display:flex;align-items:center;gap:7px;padding:5px 12px;background:#1e293b;border-radius:999px;font-size:.75rem;font-weight:600}
.dot{width:8px;height:8px;border-radius:50%;background:#475569;flex-shrink:0}
.dot.ok{background:#22c55e;animation:pulse 2s infinite}
.dot.warn{background:#f97316;animation:pulse .5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
main{flex:1;display:grid;grid-template-rows:auto 1fr auto;gap:10px;padding:10px 14px;min-height:0}
.metrics{display:grid;grid-template-columns:1fr 1fr 1.3fr;gap:10px}
.card{background:#1e293b;border-radius:10px;padding:12px 16px}
.lbl{font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:#64748b;margin-bottom:6px}
.big{font-size:2.6rem;font-weight:800;font-family:'Courier New',monospace;line-height:1}
.big.p{color:#60a5fa}.big.r{color:#a78bfa}
.unit{font-size:.68rem;color:#475569;margin-top:3px}
.chip{display:inline-flex;align-items:center;gap:5px;padding:4px 10px;border-radius:999px;font-size:.72rem;font-weight:700;border:1px solid transparent;margin-bottom:5px}
.s0{background:#14532d18;color:#4ade80;border-color:#4ade8030}
.s1{background:#7f1d1d18;color:#f87171;border-color:#f8717130;animation:pulse .4s infinite}
.s2{background:#7c2d1218;color:#fb923c;border-color:#fb923c30}
.s3{background:#0c4a6e18;color:#38bdf8;border-color:#38bdf830}
.note{font-size:.68rem;color:#64748b}
.chart-wrap{background:#1e293b;border-radius:10px;padding:10px 12px;min-height:0}
.bottom{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.panel{background:#1e293b;border-radius:10px;padding:12px 14px}
.ptitle{font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:#64748b;margin-bottom:8px}
.sr-item{display:flex;justify-content:space-between;align-items:center;padding:5px 8px;border-radius:6px;background:#0f172a;margin-bottom:4px;font-size:.78rem}
.sr-n{color:#475569}.sr-pk{color:#f87171;font-weight:600}.sr-st{color:#4ade80;font-weight:700;font-family:monospace}
.empty{color:#334155;font-style:italic;font-size:.75rem}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.stat{background:#0f172a;border-radius:6px;padding:7px 9px}
.sn{font-size:.58rem;color:#64748b;text-transform:uppercase;letter-spacing:.05em}
.sv{font-size:1rem;font-weight:700;font-family:monospace;margin-top:2px;color:#94a3b8}
.g{color:#4ade80}.w{color:#fb923c}.d{color:#f87171}
.bar-bg{height:4px;background:#334155;border-radius:2px;overflow:hidden;margin-top:3px}
.bar-fg{height:100%;border-radius:2px;transition:width .4s,background .4s}
</style>
</head>
<body>
<header>
  <div>
    <h1>🎯 Shake-It-Off — Real-Time Dashboard</h1>
    <p>Self-Stabilizing Platform &nbsp;·&nbsp; Complementary Filter + Velocity PID &nbsp;·&nbsp; 1000 Hz control loop</p>
  </div>
  <div class="badge"><span class="dot" id="dot"></span><span id="wst">Connecting…</span></div>
</header>
<main>
  <div class="metrics">
    <div class="card">
      <div class="lbl">Pitch</div>
      <div class="big p" id="vp">---</div>
      <div class="unit">degrees from level</div>
    </div>
    <div class="card">
      <div class="lbl">Roll</div>
      <div class="big r" id="vr">---</div>
      <div class="unit">degrees from level</div>
    </div>
    <div class="card">
      <div class="lbl">System State</div>
      <span class="chip s0" id="chip">● STABLE</span>
      <div class="note" id="snote">Waiting for data…</div>
    </div>
  </div>

  <div class="chart-wrap">
    <canvas id="cv"></canvas>
  </div>

  <div class="bottom">
    <div class="panel">
      <div class="ptitle">Step Response Events</div>
      <div id="srl"><span class="empty">Push the platform to record events…</span></div>
    </div>
    <div class="panel">
      <div class="ptitle">System Statistics</div>
      <div class="stats">
        <div class="stat"><div class="sn">Loop WCET</div><div class="sv" id="s-wcet">—</div></div>
        <div class="stat">
          <div class="sn">CPU Util</div>
          <div class="sv" id="s-util">—</div>
          <div class="bar-bg"><div class="bar-fg" id="ubar" style="width:0;background:#22c55e"></div></div>
        </div>
        <div class="stat"><div class="sn">Deadline Misses</div><div class="sv" id="s-miss">—</div></div>
        <div class="stat"><div class="sn">Avg t_IMU</div><div class="sv" id="s-imu">—</div></div>
      </div>
    </div>
  </div>
</main>

<script>
// ── Chart ────────────────────────────────────────────────────
const MAX = 600;
const tA=[], pA=[], rA=[];

Chart.register({
  id:'band',
  beforeDraw(ch){
    const {ctx,chartArea:{left,right},scales:{y}}=ch;
    if(!y)return;
    ctx.save();
    ctx.fillStyle='rgba(34,197,94,.07)';
    ctx.fillRect(left,y.getPixelForValue(.5),right-left,y.getPixelForValue(-.5)-y.getPixelForValue(.5));
    ctx.strokeStyle='rgba(34,197,94,.22)';ctx.setLineDash([5,5]);ctx.lineWidth=1;
    const y0=y.getPixelForValue(0);
    ctx.beginPath();ctx.moveTo(left,y0);ctx.lineTo(right,y0);ctx.stroke();
    ctx.restore();
  }
});

const chart = new Chart(document.getElementById('cv').getContext('2d'),{
  type:'line',
  data:{labels:tA,datasets:[
    {label:'Pitch',data:pA,borderColor:'#60a5fa',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'Roll', data:rA,borderColor:'#a78bfa',borderWidth:1.8,pointRadius:0,tension:.05},
  ]},
  options:{
    responsive:true,maintainAspectRatio:true,animation:false,
    plugins:{legend:{labels:{color:'#94a3b8',boxWidth:14,font:{size:11}}}},
    scales:{
      x:{ticks:{color:'#475569',maxTicksLimit:8,callback:(_,i)=>{const v=tA[i];return v!=null?(v/1000).toFixed(1)+'s':'';}},grid:{color:'#1e293b'}},
      y:{suggestedMin:-15,suggestedMax:15,ticks:{color:'#475569',callback:v=>v+'°'},grid:{color:'#1e293b55'}}
    }
  }
});

// ── State ─────────────────────────────────────────────────────
const SL=['● STABLE','⚡ DISTURBED','⟳ SETTLING','✓ SETTLED'];
const SC=['s0','s1','s2','s3'];
let wcet=0, miss=0, t0=null, dirty=false;
const imuBuf=[], srEvts=[];

// ── WebSocket ─────────────────────────────────────────────────
const WS=`ws://${location.hostname}:8765`;
let ws,retryT;
function connect(){
  ws=new WebSocket(WS);
  ws.onopen=()=>{
    document.getElementById('dot').className='dot ok';
    document.getElementById('wst').textContent='Connected';
    clearTimeout(retryT);
  };
  ws.onclose=()=>{
    document.getElementById('dot').className='dot warn';
    document.getElementById('wst').textContent='Reconnecting…';
    retryT=setTimeout(connect,2000);
  };
  ws.onerror=()=>ws.close();
  ws.onmessage=ev=>{
    const m=JSON.parse(ev.data);
    if(m.type==='data') onData(m);
    else if(m.type==='step') onStep(m);
  };
}
connect();

// ── Data handler ──────────────────────────────────────────────
function onData(d){
  if(t0===null)t0=d.tick;
  tA.push(d.tick-t0); pA.push(d.pitch); rA.push(d.roll);
  if(tA.length>MAX){tA.shift();pA.shift();rA.shift();}
  if(!dirty){dirty=true;requestAnimationFrame(()=>{chart.update('none');dirty=false;});}

  const f=v=>(v>=0?'+':'')+v.toFixed(2)+'°';
  document.getElementById('vp').textContent=f(d.pitch);
  document.getElementById('vr').textContent=f(d.roll);

  const chip=document.getElementById('chip');
  chip.textContent=SL[d.state]??'?';
  chip.className='chip '+(SC[d.state]??'s0');

  if(d.t_total>wcet)wcet=d.t_total;
  miss=d.miss;
  imuBuf.push(d.t_imu);if(imuBuf.length>200)imuBuf.shift();
  const avgImu=Math.round(imuBuf.reduce((a,b)=>a+b,0)/imuBuf.length);

  const dl=d.dt_us;
  const u=(wcet/dl*100);
  sv('s-wcet',wcet+'μs',wcet>dl?'d':'g');
  sv('s-util',u.toFixed(1)+'%',u>100?'d':u>75?'w':'g');
  const bar=document.getElementById('ubar');
  bar.style.width=Math.min(100,u)+'%';
  bar.style.background=u>100?'#f87171':u>75?'#fb923c':'#22c55e';
  sv('s-miss',String(miss),miss>0?'w':'g');
  sv('s-imu',avgImu+'μs','');
}

function sv(id,txt,cls){
  const el=document.getElementById(id);
  el.textContent=txt;
  el.className='sv'+(cls?' '+cls:'');
}

// ── Step response ─────────────────────────────────────────────
function onStep(d){
  srEvts.unshift(d);if(srEvts.length>6)srEvts.pop();
  document.getElementById('snote').textContent=`Last settling time: ${d.settle_ms.toFixed(0)} ms`;
  document.getElementById('srl').innerHTML=srEvts.map(e=>
    `<div class="sr-item"><span class="sr-n">#${e.resp_num}</span><span class="sr-pk">⚡ ${e.peak.toFixed(1)}°</span><span class="sr-st">→ ${e.settle_ms.toFixed(0)} ms</span></div>`
  ).join('');
}
</script>
</body>
</html>
"""

# ── HTTP server ────────────────────────────────────────────────

class _H(BaseHTTPRequestHandler):
    def do_GET(self):
        b = HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)
    def log_message(self, *_): pass

def _http(port):
    HTTPServer(('', port), _H).serve_forever()

# ── Main ───────────────────────────────────────────────────────

async def _amain(source: str, baud: int):
    if source == '--demo':
        threading.Thread(target=demo_thread, daemon=True).start()
    else:
        threading.Thread(target=serial_thread, args=(source, baud), daemon=True).start()

    threading.Thread(target=_http, args=(HTTP_PORT,), daemon=True).start()

    print(f'[*] 開啟瀏覽器: http://localhost:{HTTP_PORT}')
    print(f'[*] WebSocket:  ws://localhost:{WS_PORT}')
    print('[*] Ctrl+C 結束\n')

    async with websockets.serve(ws_handler, '', WS_PORT):
        await broadcast_loop()

def main():
    args = sys.argv[1:]
    if '--demo' in args or not args:
        source, baud = '--demo', BAUD_RATE
        if not args:
            print('[*] 未指定串口，使用 --demo 模式\n')
    else:
        source = next((a for a in args if not a.isdigit()), args[0])
        baud   = int(next((a for a in args if a.isdigit()), BAUD_RATE))

    try:
        asyncio.run(_amain(source, baud))
    except KeyboardInterrupt:
        print('\n[*] 已停止。')

if __name__ == '__main__':
    main()
