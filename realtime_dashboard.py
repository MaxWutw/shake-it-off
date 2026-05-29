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
                's1':      float(p[16]) if len(p) > 16 else 90.0,          # A0 physical
                's2':      float(p[17]) if len(p) > 17 else 90.0,          # A1 physical
                's3':      float(p[18]) if len(p) > 18 else 90.0,          # A2 physical
                's4':      float(p[19].strip()) if len(p) > 19 else 90.0,  # A3 physical
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

        tgt_p_v = round(-pn * 0.15, 3)
        tgt_r_v = round(-rn * 0.15, 3)
        # Physical servo angles (reversed servos already corrected):
        # s1(A0)/s4(A3) = pitch pair; s2(A1)/s3(A2) = roll pair
        sop = tgt_p_v * 1.8 + rng.gauss(0, 0.2)
        sor = tgt_r_v * 1.8 + rng.gauss(0, 0.2)
        try:
            _q.put_nowait({'type': 'data', 'tick': t_ms,
                           'pitch': round(pn, 3), 'roll': round(rn, 3),
                           'tgt_p': tgt_p_v, 'tgt_r': tgt_r_v,
                           'gx': round(rng.gauss(0, 2), 2), 'gy': round(rng.gauss(0, 2), 2),
                           'dt_us': 1000, 't_imu': ti, 't_total': t_total,
                           'miss': miss, 'state': state,
                           's1': round(90.0 + sop, 2),   # A0, pitch
                           's2': round(90.0 + sor, 2),   # A1, roll
                           's3': round(90.0 + sor, 2),   # A2, roll (physical)
                           's4': round(90.0 + sop, 2)})  # A3, pitch (physical)
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
main{flex:1;display:grid;grid-template-rows:auto 1fr auto;gap:10px;padding:10px 14px;min-height:0}
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.card{background:#1e293b;border-radius:10px;padding:12px 16px}
.lbl{font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:#64748b;margin-bottom:6px}
.big{font-size:2.6rem;font-weight:800;font-family:'Courier New',monospace;line-height:1}
.big.p{color:#60a5fa}.big.r{color:#a78bfa}
.unit{font-size:.68rem;color:#475569;margin-top:3px}
.chart-main{background:#1e293b;border-radius:10px;padding:10px 12px;display:flex;flex-direction:column;min-height:0}
.charts-sub{display:grid;grid-template-columns:1fr 1fr;gap:10px;height:265px}
.chart-sub{background:#1e293b;border-radius:10px;padding:10px 12px;display:flex;flex-direction:column;min-height:0}
.ctitle{font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:#64748b;margin-bottom:6px;flex-shrink:0}
.servo-vals{display:flex;gap:6px;margin-bottom:8px;flex-shrink:0;flex-wrap:wrap}
.sv-chip{display:inline-flex;align-items:baseline;gap:5px;padding:4px 10px;border-radius:6px;background:#0f172a;font-family:'Courier New',monospace;white-space:nowrap}
.sv-lbl{font-size:.65rem;font-weight:700;opacity:.55}
.sv-val{font-size:.95rem;font-weight:800}
.sv-s1{border:1px solid #3b82f650;color:#3b82f6}
.sv-s2{border:1px solid #22c55e50;color:#22c55e}
.sv-s3{border:1px solid #f9731650;color:#f97316}
.sv-s4{border:1px solid #ec489950;color:#ec4899}
canvas{flex:1;min-height:0;display:block;width:100%}
</style>
</head>
<body>
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
  </div>

  <div class="chart-main">
    <div class="ctitle">Step Response — Pitch &amp; Roll</div>
    <canvas id="cv"></canvas>
  </div>

  <div class="charts-sub">
    <div class="chart-sub">
      <div class="ctitle">Servo Motor Angles</div>
      <div class="servo-vals">
        <span class="sv-chip sv-s1"><span class="sv-lbl">S1 A0</span><span class="sv-val" id="sv1">—</span></span>
        <span class="sv-chip sv-s2"><span class="sv-lbl">S2 A1</span><span class="sv-val" id="sv2">—</span></span>
        <span class="sv-chip sv-s3"><span class="sv-lbl">S3 A2</span><span class="sv-val" id="sv3">—</span></span>
        <span class="sv-chip sv-s4"><span class="sv-lbl">S4 A3</span><span class="sv-val" id="sv4">—</span></span>
      </div>
      <canvas id="cv2"></canvas>
    </div>
    <div class="chart-sub">
      <div class="ctitle">Control Signal — Target Mech Angle</div>
      <canvas id="cv3"></canvas>
    </div>
  </div>
</main>

<script>
// ── Data buffers ──────────────────────────────────────────────
const MAX=600;
const tA=[],pA=[],rA=[];
const s1A=[],s4A=[],s2A=[],s3A=[];
const cPA=[],cRA=[];

// ── Plugins ───────────────────────────────────────────────────
Chart.register({
  id:'band',
  beforeDraw(ch){
    if(ch.canvas.id!=='cv')return;
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

Chart.register({
  id:'hline',
  beforeDraw(ch){
    const ref=ch.options._hline;
    if(ref==null)return;
    const {ctx,chartArea:{left,right},scales:{y}}=ch;
    if(!y)return;
    ctx.save();
    ctx.strokeStyle='rgba(100,116,139,.45)';ctx.setLineDash([4,4]);ctx.lineWidth=1;
    const yp=y.getPixelForValue(ref);
    ctx.beginPath();ctx.moveTo(left,yp);ctx.lineTo(right,yp);ctx.stroke();
    ctx.restore();
  }
});

const commonOpts={
  responsive:true,maintainAspectRatio:false,animation:false,
  plugins:{legend:{labels:{color:'#94a3b8',boxWidth:12,font:{size:10}}}},
  scales:{
    x:{ticks:{color:'#475569',maxTicksLimit:6,callback:(_,i)=>{const v=tA[i];return v!=null?(v/1000).toFixed(1)+'s':'';}},grid:{color:'#1e293b'}},
    y:{ticks:{color:'#475569'},grid:{color:'#1e293b55'}}
  }
};

// Chart 1 — Step Response
const chart=new Chart(document.getElementById('cv').getContext('2d'),{
  type:'line',
  data:{labels:tA,datasets:[
    {label:'Pitch',data:pA,borderColor:'#60a5fa',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'Roll', data:rA,borderColor:'#a78bfa',borderWidth:1.8,pointRadius:0,tension:.05},
  ]},
  options:{...commonOpts,
    scales:{...commonOpts.scales,
      y:{suggestedMin:-15,suggestedMax:15,ticks:{color:'#475569',callback:v=>v+'°'},grid:{color:'#1e293b55'}}
    }
  }
});

// Chart 2 — Servo Motor Angles (fixed 0–180°, physical angles, pin order A0-A3)
const chart2=new Chart(document.getElementById('cv2').getContext('2d'),{
  type:'line',
  data:{labels:tA,datasets:[
    {label:'S1 A0 pitch',data:s1A,borderColor:'#3b82f6',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'S2 A1 roll', data:s2A,borderColor:'#22c55e',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'S3 A2 roll', data:s3A,borderColor:'#f97316',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'S4 A3 pitch',data:s4A,borderColor:'#ec4899',borderWidth:1.8,pointRadius:0,tension:.05},
  ]},
  options:{...commonOpts,_hline:90,
    scales:{...commonOpts.scales,
      y:{min:0,max:180,ticks:{color:'#475569',stepSize:45,callback:v=>v+'°'},grid:{color:'#1e293b55'}}
    }
  }
});

// Chart 3 — Control Signal
const chart3=new Chart(document.getElementById('cv3').getContext('2d'),{
  type:'line',
  data:{labels:tA,datasets:[
    {label:'Target Pitch',data:cPA,borderColor:'#ef4444',borderWidth:1.8,pointRadius:0,tension:.05},
    {label:'Target Roll', data:cRA,borderColor:'#f97316',borderWidth:1.8,pointRadius:0,tension:.05},
  ]},
  options:{...commonOpts,_hline:0,
    scales:{...commonOpts.scales,
      y:{suggestedMin:-15,suggestedMax:15,ticks:{color:'#475569',callback:v=>v+'°'},grid:{color:'#1e293b55'}}
    }
  }
});

// ── WebSocket ─────────────────────────────────────────────────
let t0=null,dirty=false;
const WS=`ws://${location.hostname}:8765`;
let ws,retryT;
function connect(){
  ws=new WebSocket(WS);
  ws.onopen=()=>clearTimeout(retryT);
  ws.onclose=()=>{retryT=setTimeout(connect,2000);};
  ws.onerror=()=>ws.close();
  ws.onmessage=ev=>{
    const m=JSON.parse(ev.data);
    if(m.type==='data')onData(m);
  };
}
connect();

// ── Data handler ──────────────────────────────────────────────
function onData(d){
  if(t0===null)t0=d.tick;

  tA.push(d.tick-t0);pA.push(d.pitch);rA.push(d.roll);
  if(tA.length>MAX){tA.shift();pA.shift();rA.shift();}

  s1A.push(d.s1??90);s4A.push(d.s4??90);s2A.push(d.s2??90);s3A.push(d.s3??90);
  if(s1A.length>MAX){s1A.shift();s4A.shift();s2A.shift();s3A.shift();}

  cPA.push(d.tgt_p);cRA.push(d.tgt_r);
  if(cPA.length>MAX){cPA.shift();cRA.shift();}

  if(!dirty){
    dirty=true;
    requestAnimationFrame(()=>{
      chart.update('none');chart2.update('none');chart3.update('none');
      dirty=false;
    });
  }

  const f=v=>(v>=0?'+':'')+v.toFixed(2)+'°';
  document.getElementById('vp').textContent=f(d.pitch);
  document.getElementById('vr').textContent=f(d.roll);
  document.getElementById('sv1').textContent=(d.s1??90).toFixed(1)+'°';
  document.getElementById('sv4').textContent=(d.s4??90).toFixed(1)+'°';
  document.getElementById('sv2').textContent=(d.s2??90).toFixed(1)+'°';
  document.getElementById('sv3').textContent=(d.s3??90).toFixed(1)+'°';
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
