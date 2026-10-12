import asyncio
import numpy as np
import time
import ipywidgets as widgets
from IPython.display import display, HTML, Javascript
from bleak import BleakClient

# --- 1. CONFIGURATION & CALIBRATION ---
CALIBRATION_FACTOR = 1.0526 

# --- 2. BLE CONTROLLER (With Fragmentation Buffer) ---
class RobotBLE:
    def __init__(self):
        self.client = None
        self.write_char = None
        self.data_buffer = "" 
        self.telemetry = {
            "L": 0, "R": 0, "C": 0, "idx": 0, "path": "None", 
            "int_flag": 0, "task_ack": "None", "V_raw": 3380,
            "Magnitude": 0, "Angle": 0
        }
        self.is_connected = False

    async def connect(self):
        try:
            self.client = BleakClient("B0:B1:13:2D:55:D0")
            await self.client.connect()
            for service in self.client.services:
                for char in service.characteristics:
                    if "write" in char.properties: self.write_char = char.uuid
                    if "notify" in char.properties:
                        await self.client.start_notify(char.uuid, self.notification_handler)
            self.is_connected = True
        except: self.is_connected = False

    def notification_handler(self, sender, data):
        try:
            self.data_buffer += data.decode('utf-8', errors='ignore')
            if "\n" in self.data_buffer:
                lines = self.data_buffer.split("\n")
                for line in lines[:-1]:
                    self.parse_line(line)
                self.data_buffer = lines[-1]
        except: pass

    def parse_line(self, line):
        clean = line.strip()
        if ":" in clean:
            try:
                k, v = [x.strip() for x in clean.split(":", 1)]
                mapping = {
                    "Left":"L", "Right":"R", "Intersection":"C", 
                    "Index":"idx", "Intersection_Flag":"int_flag", 
                    "Task":"task_ack", "Battery":"V_raw", 
                    "magnitude": "Magnitude", "Angle": "Angle"
                }
                if k in mapping:
                    num_v = "".join(filter(lambda x: x.isdigit() or x=='.' or x=='-', v))
                    if num_v:
                        self.telemetry[mapping[k]] = float(num_v) if mapping[k] != "task_ack" else v
            except: pass

    async def send(self, cmd):
        if self.is_connected and self.write_char:
            try: await self.client.write_gatt_char(self.write_char, str(cmd).encode(), response=False)
            except: pass

bt_robot = RobotBLE()

# --- 3. PERSISTENT STATE ---
if 'dashboard_state' not in globals():
    dashboard_state = {
        "adc_history": {"L": [0]*20, "R": [0]*20, "C": [0]*20},
        "custom_cmds": [], "upload_status": "NO CSV",
        "display_idx": 0, "blink_until": 0, "smooth_soc": 0,
        "active_keys": set(), "prev_keys": set(),
        "toggles": {str(i):False for i in range(1,8)}
    }
    dashboard_state["toggles"]["claw"] = False

# --- 4. CSV PARSER ---
def on_upload_change(change):
    try:
        val = upload_widget.value
        raw_bytes = val[0]['content'] if isinstance(val, tuple) else val[next(iter(val))]['content']
        content = raw_bytes.tobytes().decode('utf-8-sig')
        import re
        parsed = [l.strip().upper() for l in re.split(r'[,\n\r]+', content) if l.strip().upper() in ['R', 'L', 'F']]
        if parsed:
            dashboard_state["custom_cmds"] = parsed
            dashboard_state["upload_status"] = f"LOADED ({len(parsed)})"
    except: dashboard_state["upload_status"] = "ERR"

upload_widget = widgets.FileUpload(accept='.csv', multiple=False)
upload_widget.observe(on_upload_change, names='value')

# --- 5. THE BRAIN (Keyboard Input) ---
def on_key_data_received(change):
    raw = change['new']
    if not raw or "_" not in raw: return
    current_keys = set(list(raw.split("_")[0]))
    dashboard_state["active_keys"] = current_keys
    now = time.time()
    new_presses = current_keys - dashboard_state.get("prev_keys", set())

    def trigger(k):
        if k in new_presses and (now - dashboard_state.get("last_toggle_time", {}).get(k, 0)) > 0.3:
            if "last_toggle_time" not in dashboard_state: dashboard_state["last_toggle_time"] = {}
            dashboard_state["last_toggle_time"][k] = now
            return True
        return False

    # Path triggers
    for k in ['1', '2', '3', '5']:
        if trigger(k):
            asyncio.create_task(bt_robot.send(k))
            dashboard_state["display_idx"] = 0
            bt_robot.telemetry["idx"] = 0
            for p in ['1','2','3','4','5']: dashboard_state["toggles"][p] = False
            dashboard_state["toggles"][k] = True

    # Custom Path trigger
    if trigger('4'):
        async def run_sequence():
            if not dashboard_state["custom_cmds"]: return
            dashboard_state["display_idx"] = 0
            bt_robot.telemetry["idx"] = 0
            await bt_robot.send('C'); await asyncio.sleep(0.2)
            for cmd in dashboard_state["custom_cmds"]:
                await bt_robot.send(cmd); await asyncio.sleep(0.1)
            await bt_robot.send('S'); await asyncio.sleep(0.2); await bt_robot.send('4')
            for p in ['1','2','3','4','5']: dashboard_state["toggles"][p] = False
            dashboard_state["toggles"]["4"] = True
            dashboard_state["upload_status"] = "RUNNING"
        asyncio.create_task(run_sequence())

    if trigger('q') or trigger('e'):
        dashboard_state["toggles"]["claw"] = not dashboard_state["toggles"]["claw"]
        asyncio.create_task(bt_robot.send('q'))

    dashboard_state["prev_keys"] = current_keys

bridge_input = widgets.Text(value='', layout=widgets.Layout(width='0px', height='0px', opacity='0'))
bridge_input.add_class("bridge-widget")
bridge_input.observe(on_key_data_received, names='value')

# --- 6. RENDERER ---
def spark(data):
    pts = " ".join([f"{(i/19)*100},{100-(float(v)/4095)*100}" for i,v in enumerate(data)])
    return f'''<div style="height:60px; width:100%; overflow:hidden; background:#0f172a; border:1px solid #334155; border-radius:6px;">
        <svg width="100%" height="100%" viewBox="0 -2 100 104" preserveAspectRatio="none" style="display:block;">
            <polyline points="{pts}" fill="none" stroke="#f1c40f" stroke-width="2" vector-effect="non-scaling-stroke"/>
        </svg></div>'''

def get_soc_table(v):
    if v >= 9.2: return 100
    elif v >= 8.9: return 90
    elif v >= 8.6: return 80
    elif v >= 8.3: return 70
    elif v >= 8.0: return 60
    elif v >= 7.7: return 50
    elif v >= 7.4: return 40
    elif v >= 7.1: return 30
    elif v >= 6.8: return 20
    elif v >= 6.5: return 10
    else: return 0

ui_output = widgets.HTML(value="")

def update_ui(bridge_raw):
    now = time.time()
    
    # 1. Battery Math
    v_raw = float(bt_robot.telemetry["V_raw"])
    v_batt = (v_raw * (3.3 / (4095 * 0.3125))) * CALIBRATION_FACTOR
    soc_step = get_soc_table(v_batt)
    dashboard_state["smooth_soc"] = (dashboard_state.get("smooth_soc", soc_step) * 0.9) + (soc_step * 0.1)

    # 2. Continuous Intersection update
    mcu_idx = int(bt_robot.telemetry["idx"])
    if mcu_idx > dashboard_state["display_idx"]:
        dashboard_state["blink_until"] = now + 1.0
        dashboard_state["display_idx"] = mcu_idx
    is_blinking = now < dashboard_state["blink_until"]

    # 3. Continuous Coils update
    for k, v in zip(["L", "C", "R"], [float(bt_robot.telemetry["L"]), float(bt_robot.telemetry["C"]), float(bt_robot.telemetry["R"])]):
        dashboard_state["adc_history"][k].append(v); dashboard_state["adc_history"][k].pop(0)

    # 4. Joystick & Keys
    real_angle = float(bt_robot.telemetry["Angle"])
    real_mag = float(bt_robot.telemetry["Magnitude"])
    rad = np.radians(90 - real_angle)
    joy_x, joy_y = 30 * np.cos(rad), -30 * np.sin(rad)
    keys = dashboard_state["active_keys"]
    
    bg="#0f172a"; card="#1e293b"; border="#334155"; accent="#38bdf8"; success="#10b981"; alert="#ef4444"; dim="#64748b"; text="#f8fafc"
    
    def btn(label, k_id, act_col):
        on = dashboard_state["toggles"].get(k_id)
        return f'<div style="background:{act_col if on else card}; color:{"white" if on else dim}; border:1px solid {border}; border-radius:8px; width:100%; height:40px; display:flex; align-items:center; justify-content:center; font-size:11px; font-weight:bold; margin-bottom:8px;">{label}</div>'

    def k_style(k):
        is_on = k in keys
        return f"width:32px; height:32px; border:1px solid {border}; border-radius:6px; display:flex; align-items:center; justify-content:center; font-weight:bold; font-size:12px; background:{accent if is_on else 'transparent'}; color:{'white' if is_on else dim}; transition:0.1s;"

    ui_output.value = f'''<div style="background:{bg}; width:100%; min-height:95vh; padding:35px; font-family:sans-serif; color:{text}; box-sizing:border-box;">
        <div style="display:flex; justify-content:space-between; align-items:center; border-bottom:2px solid {border}; padding-bottom:15px; margin-bottom:25px;">
            <div><div style="font-size:30px; font-weight:900;">ROBOT <span style="color:{accent}">CONTROLLER</span></div>
            <div style="font-size:11px; color:{dim};">ACK: {bt_robot.telemetry["task_ack"]}</div></div>
            <div style="display:flex; gap:25px; align-items:center;">
                <div style="text-align:center;"><div style="width:18px; height:18px; border-radius:50%; background:{alert if is_blinking else card}; border:2px solid {alert if is_blinking else border}; box-shadow:{'0 0 15px '+alert if is_blinking else 'none'}; transition:0.1s; margin:0 auto;"></div><div style="font-size:10px; font-weight:800; color:{dim}; margin-top:4px;">INT: {dashboard_state["display_idx"]}</div></div>
                <div style="font-size:13px; color:{success if bt_robot.is_connected else alert}; font-weight:800;">● {'LINKED' if bt_robot.is_connected else 'OFFLINE'}</div>
            </div>
        </div>
        <div style="display:flex; gap:20px; justify-content:center;">
            <div style="width:100px;">
                {btn("P1 (1)","1",accent)} {btn("P2 (2)","2",accent)} {btn("P3 (3)","3",accent)} {btn("P5 (5)","5",accent)}
                <div style="background:{card}; padding:8px; border-radius:8px; border:1px solid {border}; text-align:center; margin-bottom:10px;"><div style="font-size:8px; color:{dim};">CSV</div><div style="font-size:10px; font-weight:bold;">{dashboard_state["upload_status"]}</div></div>
                {btn("CUSTOM (4)","4",success)}
            </div>
            <div style="background:{card}; border-radius:20px; border:1px solid {border}; padding:20px; flex:0.6; height:450px; display:flex; flex-direction:column; align-items:center; justify-content:center;">
                <div style="font-size:11px; color:{accent}; font-weight:800; margin-bottom:30px;">POWER</div>
                <div style="text-align:center;"><div style="width:18px; height:250px; background:{bg}; border-radius:10px; position:relative;"><div style="position:absolute; bottom:0; width:100%; height:{dashboard_state["smooth_soc"]}%; background:{success}; border-radius:8px;"></div></div><div style="margin-top:15px; font-weight:bold; font-size:18px;">{int(soc_step)}%</div><div style="font-size:10px; color:{dim};">BATT</div></div>
            </div>
            <div style="background:{card}; border-radius:20px; border:1px solid {border}; padding:20px; flex:1.4; height:450px; display:flex; flex-direction:column;">
                <div style="font-size:11px; color:{accent}; font-weight:800; text-align:center; margin-bottom:15px;">COILS</div>
                <div style="display:flex; flex-direction:column; gap:12px; flex-grow:1;">
                    <div style="flex:1;"><div style="display:flex; justify-content:space-between; font-size:10px; color:{dim}; margin-bottom:4px;"><span>LEFT</span><span style="color:white">{int(bt_robot.telemetry["L"])}</span></div>{spark(dashboard_state["adc_history"]["L"])}</div>
                    <div style="flex:1;"><div style="display:flex; justify-content:space-between; font-size:10px; color:{dim}; margin-bottom:4px;"><span>CENTER</span><span style="color:white">{int(bt_robot.telemetry["C"])}</span></div>{spark(dashboard_state["adc_history"]["C"])}</div>
                    <div style="flex:1;"><div style="display:flex; justify-content:space-between; font-size:10px; color:{dim}; margin-bottom:4px;"><span>RIGHT</span><span style="color:white">{int(bt_robot.telemetry["R"])}</span></div>{spark(dashboard_state["adc_history"]["R"])}</div>
                </div>
            </div>
            <div style="background:{card}; padding:30px; border-radius:20px; border:1px solid {border}; flex:1.2; display:flex; flex-direction:column; align-items:center; justify-content:center;">
                <div style="font-size:11px; color:{accent}; font-weight:800; margin-bottom:30px;">CONTROLS</div>
                <div style="display:grid; grid-template-columns: repeat(3, 40px); gap:8px; margin-bottom:30px;">
                    <div style="{k_style('q')}">Q</div><div style="{k_style('w')}">W</div><div style="{k_style('e')}">E</div>
                    <div style="{k_style('a')}">A</div><div style="{k_style('s')}">S</div><div style="{k_style('d')}">D</div>
                </div>
                <div style="width:100px; height:100px; background:{bg}; border:2px solid {border}; border-radius:50%; position:relative;">
                    <div style="position:absolute; width:20px; height:20px; background:{text}; border-radius:50%; top:40px; left:40px; transform:translate({joy_x}px, {joy_y}px); box-shadow: 0 0 15px rgba(255,255,255,0.4);"></div>
                </div>
                <div style="font-size:14px; font-weight:800; color:{text}; margin-top:20px;">{int(real_angle)}°</div>
            </div>
        </div>
    </div>'''

js_poller = Javascript("""
(function() {
    let keys = new Set();
    window.addEventListener('keydown', (e) => {
        const k = e.key.toLowerCase();
        if(['w','a','s','d','q','e',' ','1','2','3','4','5','6'].includes(k)) {
            e.preventDefault(); keys.add(k);
        }
    }, true);
    window.addEventListener('keyup', (e) => { keys.delete(e.key.toLowerCase()); }, true);
    setInterval(() => {
        const bridge = document.querySelector('.bridge-widget input');
        if (bridge) {
            bridge.value = Array.from(keys).join("") + "_" + Date.now();
            bridge.dispatchEvent(new Event('input', { bubbles: true }));
        }
    }, 50);
})();
""")

out = widgets.interactive_output(update_ui, {'bridge_raw': bridge_input})

display(js_poller)
display(widgets.VBox([widgets.HBox([upload_widget, bridge_input]), ui_output]))
asyncio.create_task(bt_robot.connect())
