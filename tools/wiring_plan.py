#!/usr/bin/env python3
"""Draws docs/wiring_plan.svg: power, control and sensor wiring of the delivery robot (plan of 2026-10-27).
  python3 src/my_bot/tools/wiring_plan.py [output.svg]"""
import html
import sys

W, H = 1800, 1250
o = []
RED, ORA, PUR, BLU, BLK, GRN, GRY = '#d32f2f', '#ef6c00', '#7b1fa2', '#1565c0', '#222222', '#2e7d32', '#8a94a6'


def t(x, y, s, col='#3a4252', size=13.5, weight='400', anchor='start'):
    o.append(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{col}" text-anchor="{anchor}">'
             f'{html.escape(s)}</text>')


def box(x, y, w, h, title, lines=(), fill='#ffffff', new=False):
    dash = ' stroke-dasharray="8 5"' if new else ''
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" stroke="#1d2330" stroke-width="2.2"{dash}/>')
    t(x + w / 2, y + 24, title, '#1d2330', 16, '700', 'middle')
    for i, s in enumerate(lines):
        t(x + w / 2, y + 46 + i * 19, s, anchor='middle')


def wire(pts, col, w=3.2, label=None, lx=0, ly=0):
    o.append('<path d="M %s" fill="none" stroke="%s" stroke-width="%s" stroke-linejoin="round"/>'
             % (' L '.join('%s %s' % p for p in pts), col, w))
    if label:
        t(lx, ly, label, col, 12.5, '700')


o.append(f'<rect width="{W}" height="{H}" fill="#f6f7f9"/>')
t(30, 42, 'Delivery robot - wiring plan (power, control, sensors)', '#1d2330', 26, '700')
t(30, 68, 'Dashed boxes = parts to buy. The Pi keeps its own power bank. All grounds meet at ONE star point.', size=15)
for y, h, s in ((90, 380, 'POWER  (12 V side)'), (480, 330, 'CONTROL'), (820, 400, 'SENSORS & DISPLAY  (I2C on the Pi, 3.3 V)')):
    o.append(f'<rect x="15" y="{y}" width="{W - 30}" height="{h}" rx="14" fill="#ffffff" stroke="#dfe3ea"/>')
    t(30, y + 24, s, GRY, 15, '700')

# ---------- power
box(40, 130, 220, 130, '3S Li-ion pack', ['3 x 18650 in black holder', '(the "purple" cells)', '12.6 V full, 9 V empty', 'B-  B1  B2  B+'], '#f3e5f5')
box(300, 130, 200, 130, '3S BMS 20 A', ['Li-ion 12.6 V (NOT LiFePO4)', 'balance wires B1, B2', 'cuts off ~2.8 V per cell', 'outputs P+ / P-'], new=True)
wire([(260, 165), (300, 165)], RED); wire([(260, 195), (300, 195)], GRY, 2); wire([(260, 225), (300, 225)], BLK)
box(560, 120, 230, 150, 'ON-OFF-ON switch', ['SPDT, 10 A or more, centre OFF', 'middle pin = BMS P+', 'up = RUN (robot on)', 'down = CHARGE (robot off)', 'never both at once'], new=True)
wire([(500, 165), (560, 165)], RED, label='P+', lx=515, ly=158)
box(560, 300, 230, 120, 'Charge jack (panel DC)', ['5.5 x 2.1 mm', '+ to switch CHARGE pin', '- to star ground', 'charger: 3S Li-ion 12.6 V 2 A'], new=True)
wire([(675, 270), (675, 300)], RED, label='CHG', lx=682, ly=290)
box(830, 120, 150, 90, 'Fuse 5 A', ['blade fuse + holder', 'right after switch'], new=True)
wire([(790, 160), (830, 160)], RED, label='RUN', lx=793, ly=152)
box(1020, 120, 230, 120, 'INA219 (or INA226)', ['in the + wire: VIN+ to VIN-', 'measures V, A, W, Wh', 'I2C to the Pi (see below)', 'INA226: shunt must be R010'], new=True)
wire([(980, 160), (1020, 160)], RED)
wire([(1250, 160), (1290, 160), (1290, 220), (1760, 220)], RED, 5, '12 V BUS', 1560, 212)
box(1300, 260, 230, 150, 'L298N motor driver', ['12V in + GND', '1000 uF / 25 V on 12V-GND', '(+ to 12 V, - to GND!)', '+ 100 nF at the terminals', 'OUT1/2 left, OUT3/4 right'], '#fff8e1')
wire([(1340, 220), (1340, 260)], RED)
box(1560, 260, 210, 150, 'Buck 12 V to 5 V', ['LM2596 (your blue buck)', 'set 5.0 V BEFORE connecting', '470 uF on the 5 V out', '5 V BUS: servos and', 'ultrasonic sensors'], new=True)
wire([(1665, 220), (1665, 260)], RED)
o.append('<rect x="40" y="300" width="470" height="140" rx="10" fill="#eceff1" stroke="#222" stroke-width="2.2"/>')
t(60, 326, 'GND STAR POINT (terminal block / Wago)', BLK, 16, '700')
t(60, 350, 'BMS P-  |  charge jack -  |  L298N GND  |  buck GND', '#222')
t(60, 372, 'Uno GND  |  Nano GND  |  servo GND  |  sensor GND', '#222')
t(60, 394, 'Pi GND (header pin 6): one wire each, all to this point', '#222')
t(60, 420, 'Missing ground = motors dead (17 Oct lesson)', RED, 13.5, '600')
wire([(500, 225), (525, 225), (525, 300)], BLK)

# ---------- control
box(40, 520, 300, 260, 'Raspberry Pi 4', ['own power bank (USB-C), as now', 'USB 1-1.1 -> Arduino Uno', 'USB 1-1.4 -> Arduino Nano', 'USB 1-1.3 -> YDLIDAR X2', 'I2C: pin 1 3.3V, pin 3 SDA,', 'pin 5 SCL, pin 6 GND', 'WiFi -> laptop (Nav2, website)'], '#e3f2fd')
box(380, 520, 260, 260, 'Arduino Nano (lid, sonar)', ['D9 -> lid servo signal', 'D10 -> lock servo signal', 'D2/D3 -> HC-SR04 #1 trig/echo', 'D4/D7 -> HC-SR04 #2 trig/echo', '(firmware update: me)', 'servos and sonar get power', 'from the 5 V BUS, not Nano'], '#e8f5e9')
wire([(340, 560), (380, 560)], BLU, label='USB', lx=343, ly=552)
box(680, 520, 300, 260, 'Arduino Uno (motors)', ['ROSArduinoBridge, 57600 baud', 'D5, D6, D9, D10 -> L298 IN1-4', 'D12 -> ENA, D13 -> ENB', 'EACH of these 6 pins:', '10 k resistor to GND', 'D2, D3 <- left encoder A, B', 'A4, A5 <- right encoder A, B', '5V/GND -> encoder power'], '#e8f5e9')
wire([(340, 740), (360, 740), (360, 795), (830, 795), (830, 780)], BLU, label='USB', lx=560, ly=790)
box(1060, 520, 300, 260, '2 encoder motors', ['6 wires each:', 'M+ / M- -> L298 OUT', '100 nF ceramic soldered', 'ACROSS each motor', 'VCC 5V, GND -> Uno 5V/GND', 'A, B -> Uno encoder pins', '(4 motors: 100 nF on each)'], '#fff8e1')
wire([(930, 520), (930, 470), (1320, 470), (1320, 410)], BLU, 2.5, 'IN1-4, ENA, ENB', 1000, 462)
wire([(1350, 410), (1350, 520)], RED, 3, 'OUT', 1356, 470)
wire([(980, 700), (1060, 700)], BLU, 2.5, 'A/B', 1003, 692)
box(1580, 520, 190, 120, 'Lid servos 2 x SG90', ['signal <- Nano D9/D10', 'V+ <- 5 V BUS', 'GND -> star point'], '#fce4ec')
wire([(1665, 410), (1665, 520)], ORA, 3, '5 V BUS', 1672, 470)
box(1580, 660, 190, 120, 'YDLIDAR X2', ['USB -> Pi (as now)', 'top of robot, 22 cm', 'keep 360 deg clear'], '#e3f2fd')

# ---------- sensors
o.append('<rect x="40" y="870" width="430" height="90" rx="10" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2.2"/>')
t(60, 896, 'Pi I2C bus (3.3 V !)', PUR, 16, '700')
t(60, 920, 'all modules: VCC = Pi 3.3 V (pin 1), GND, SDA, SCL')
t(60, 942, 'addresses do not clash: 0x40, 0x68, 0x3C, 0x29')
wire([(190, 780), (190, 870)], GRN, 3, 'I2C', 198, 830)
wire([(470, 915), (1760, 915)], GRN, 4, 'SDA / SCL / 3.3 V / GND', 1480, 905)
for i, (s, ls) in enumerate((('INA219  0x40', ['battery V, A, W', 'sits in the power layer']),
                             ('MPU6050  0x68', ['gyro: real turning', 'on slippery floors']),
                             ('OLED 0.96"  0x3C', ['IP address, battery %,', 'robot state']),
                             ('VL53L1X  0x29 (option)', ['front, ~10 cm high', 'low obstacles']))):
    x = 520 + i * 310
    box(x, 960, 280, 100, s, ls, new=True)
    wire([(x + 140, 915), (x + 140, 960)], GRN, 2.5)
box(40, 1000, 430, 160, '2 x HC-SR04 (back corners)', ['VCC 5 V BUS, GND star point', 'trig/echo -> Nano D2/D3 and D4/D7', 'mounted ~10 cm high, angled outwards', '(the back swings out when turning)', 'a third one in front if no VL53L1X'], new=True)
t(520, 1100, 'Backup if no INA219: 10 k + 3.3 k divider from the 12 V BUS to Nano A0 = battery voltage only (12.6 V -> 3.1 V).')
t(520, 1126, 'Charging: switch to CHARGE, plug a 12.6 V Li-ion charger into the jack. Do NOT charge through the buck any more.', RED, 14, '600')
t(520, 1152, 'Before power-on: polarity of the 1000 uF capacitor, buck set to 5.0 V (multimeter), every cell >= 3.6 V.')
t(520, 1178, 'Colours: red 12 V, orange 5 V, purple/green 3.3 V + I2C, blue signals, black ground.')

out = sys.argv[1] if len(sys.argv) > 1 else 'wiring_plan.svg'
open(out, 'w').write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
                     f'font-family="DejaVu Sans, Arial, sans-serif">' + ''.join(o) + '</svg>')
print('written', out)
