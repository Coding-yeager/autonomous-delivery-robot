# Power Budget & Battery Sizing Analysis

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

---

## 1. Electrical Subsystem Power Consumption

| Subsystem / Device | Operating Voltage | Nominal Current (A) | Peak Current (A) | Nominal Power (W) | Peak Power (W) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Left Drive Motor (Climbing 10° Slope)** | 12.0 V | 2.00 A | 5.50 A | 24.0 W | 66.0 W |
| **Right Drive Motor (Climbing 10° Slope)**| 12.0 V | 2.00 A | 5.50 A | 24.0 W | 66.0 W |
| **Raspberry Pi 4B (8GB RAM + Active Cooling)**| 5.1 V | 1.90 A | 2.80 A | 9.69 W | 14.28 W |
| **Slamtec RPLIDAR A2M8** | 5.0 V | 0.45 A | 0.70 A | 2.25 W | 3.50 W |
| **Holybro NEO-M9N GNSS + Active Antenna** | 5.0 V | 0.12 A | 0.18 A | 0.60 W | 0.90 W |
| **Precision 6-DOF IMU Sensor** | 3.3 V | 0.010 A | 0.02 A | 0.03 W | 0.06 W |
| **Cliff / ToF Drop-Off Sensors (4x)** | 5.0 V | 0.15 A | 0.25 A | 0.75 W | 1.25 W |
| **Relay Coil, Solenoid Latch & LEDs** | 12.0 V | 0.12 A | 0.80 A | 1.44 W | 9.60 W |
| **Dual DC-DC Buck Converter Losses ($\eta = 92\%$)**| 12.0 V | 0.15 A | 0.30 A | 1.80 W | 3.60 W |
| **TOTALS:** | | **5.35 A (at 12V eq)**| **13.50 A (at 12V eq)**| **64.56 W** | **165.19 W** |

---

## 2. Battery Sizing & Runtime Calculations

### Battery Specification
- **Chemistry**: Lithium Iron Phosphate ($\text{LiFePO}_4$ 4S2P)
- **Nominal Voltage**: $12.8\text{ V}$ ($3.2\text{ V} \times 4\text{ cells}$)
- **Nominal Capacity**: $20.0\text{ Ah}$
- **Total Energy**: $12.8\text{ V} \times 20.0\text{ Ah} = \mathbf{256.0\text{ Wh}}$
- **Safe Usable Depth of Discharge (DoD)**: $85\%$
$$\text{Usable Energy} = 256.0\text{ Wh} \times 0.85 = \mathbf{217.6\text{ Wh}}$$

### Operating Scenarios

#### Scenario A: Continuous Heavy Incline Climb with 12 kg Payload
- Continuous average power: $64.6\text{ W}$
$$\text{Run Time} = \frac{217.6\text{ Wh}}{64.6\text{ W}} \approx \mathbf{3.37\text{ hours (202 minutes)}}$$
- At $v = 0.40\text{ m/s}$ ($1.44\text{ km/h}$), this translates to a continuous range of:
$$\text{Range} = 3.37\text{ h} \times 1.44\text{ km/h} \approx \mathbf{4.85\text{ km}}$$

#### Scenario B: Realistic Campus Delivery Mission Profile
- Campus delivery missions consist of:
  - 40% active path travel ($64.6\text{ W}$),
  - 30% flat corridor gliding ($38.0\text{ W}$),
  - 30% stationary loading/unloading with motors disabled ($15.0\text{ W}$).
$$\text{Weighted Average Power} = (0.4 \times 64.6) + (0.3 \times 38.0) + (0.3 \times 15.0) = 25.84 + 11.40 + 4.50 = \mathbf{41.74\text{ W}}$$
$$\text{Operational Endurance} = \frac{217.6\text{ Wh}}{41.74\text{ W}} \approx \mathbf{5.21\text{ hours (312 minutes)}}$$

This endurance easily accommodates a full shift of 12 to 16 autonomous round-trip deliveries across the IIT Mandi North and South campuses on a single charge.
