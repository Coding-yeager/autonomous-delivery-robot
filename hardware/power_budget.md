# Power Budget & Battery Sizing Analysis

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

---

## 1. Electrical Subsystem Power Consumption

| Subsystem / Device | Operating Voltage | Nominal Current (A) | Peak Current (A) | Nominal Power (W) | Peak Power (W) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Left Drive Motor (Climbing 10° Slope)** | 12.0 V | 1.80 A | 4.50 A | 21.6 W | 54.0 W |
| **Right Drive Motor (Climbing 10° Slope)**| 12.0 V | 1.80 A | 4.50 A | 21.6 W | 54.0 W |
| **Raspberry Pi 4B (4 Cores Active + Fans)**| 5.1 V | 1.80 A | 2.50 A | 9.18 W | 12.75 W |
| **Slamtec RPLIDAR A1M8** | 5.0 V | 0.40 A | 0.60 A | 2.00 W | 3.00 W |
| **Holybro NEO-M9N GNSS + Active Antenna** | 5.0 V | 0.12 A | 0.18 A | 0.60 W | 0.90 W |
| **MPU-6050 IMU Breakout** | 3.3 V | 0.005 A | 0.01 A | 0.02 W | 0.03 W |
| **Relay Coil & Status Indicator LEDs** | 12.0 V | 0.08 A | 0.10 A | 0.96 W | 1.20 W |
| **DC-DC Buck Converter Losses ($\eta = 92\%$)**| 12.0 V | 0.10 A | 0.20 A | 1.20 W | 2.40 W |
| **TOTALS:** | | **4.70 A (at 12V eq)**| **10.60 A (at 12V eq)**| **57.16 W** | **128.28 W** |

---

## 2. Battery Sizing & Runtime Calculations

### Battery Specification
- **Chemistry**: Lithium Iron Phosphate ($\text{LiFePO}_4$ 4S1P)
- **Nominal Voltage**: $12.8\text{ V}$ ($3.2\text{ V} \times 4\text{ cells}$)
- **Nominal Capacity**: $10.0\text{ Ah}$
- **Total Energy**: $12.8\text{ V} \times 10.0\text{ Ah} = \mathbf{128.0\text{ Wh}}$
- **Safe Usable Depth of Discharge (DoD)**: $85\%$
$$\text{Usable Energy} = 128.0\text{ Wh} \times 0.85 = \mathbf{108.8\text{ Wh}}$$

### Operating Scenarios

#### Scenario A: Continuous Hill Climb with 10 kg Payload
- Continuous average power: $57.2\text{ W}$
$$\text{Run Time} = \frac{108.8\text{ Wh}}{57.2\text{ W}} \approx \mathbf{1.90\text{ hours (114 minutes)}}$$
- At $v = 0.40\text{ m/s}$ ($1.44\text{ km/h}$), this translates to a continuous range of:
$$\text{Range} = 1.90\text{ h} \times 1.44\text{ km/h} \approx \mathbf{2.74\text{ km}}$$

#### Scenario B: Realistic Campus Delivery Mission Profile
- Campus delivery missions consist of:
  - 40% active path travel ($57\text{ W}$),
  - 30% flat corridor gliding ($35\text{ W}$),
  - 30% stationary loading/unloading with motors disabled ($13\text{ W}$).
$$\text{Weighted Average Power} = (0.4 \times 57) + (0.3 \times 35) + (0.3 \times 13) = 22.8 + 10.5 + 3.9 = \mathbf{37.2\text{ W}}$$
$$\text{Operational Endurance} = \frac{108.8\text{ Wh}}{37.2\text{ W}} \approx \mathbf{2.92\text{ hours (175 minutes)}}$$

This endurance is more than sufficient for 6 to 8 round-trip delivery deliveries across the IIT Mandi North/South campuses on a single charge.
