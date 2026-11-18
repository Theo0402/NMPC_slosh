# Elfin E03-Pro: overview

Built 22 Oct 2026 from the Elfin control attachments only (`机械臂_3.zip`; the second zip holds the same four manuals). Companion files: `elfin_commands.md`, `elfin_examples.md`.

**Correction, 12 Nov 2026.** The arm is an **Elfin E03-Pro**, not an E05-Pro as these files first said. It is the lab's arm, serial MH217340001, the one in the factory inspection report [QC p.1], and the owner reports that the manuals are for the 3 Pro. Facts marked [E03] therefore describe this arm. The model name is corrected throughout, and the notes that depended on it are updated.

**Markers.** **[E03]** from the lab's earlier records of this arm (inspection report, MATLAB scripts, ROS config), where it is called an E03; likely to hold, but confirm, since settings and firmware may have changed. **[UNCLEAR]** the sources are ambiguous or contradict each other. **[MISSING]** the attachments don't say. **[UNTESTED]** not tried on hardware.

**Sources.** Pages are the printed footer numbers; code is cited as file:line.

| Key | File | PDF page |
|---|---|---|
| PROTO | `说明书/HansRobotV5控制通信协议接口.pdf`: HansRobot V5 TCP protocol, HR-TCP-V1.0.0.3 (Chinese) | printed + 7; p.I–III are PDF 1–3 |
| NET | `说明书/HansRobot_网络配置.pdf`: network setup guide (Chinese, scanned) | printed + 2 |
| LINUX | `说明书/Linux system manual for Elfin cobot.pdf`: user manual (English) | printed + 1 |
| CN-SW | `说明书/Elfin软件操作说明书V2.pdf`: software manual V2 (Chinese) | printed + 1 |
| QC | `机械臂出货检验报告(1).pdf`: factory inspection report for the lab's arm | 3 pages, as printed |
| ROS | `Code/ROS/src/elfin_robot/`: Han's ROS driver (EtherCAT) | — |
| MATLAB | `Code/MATLAB/`: the lab's MATLAB scripts | — |

## What the attachments cover

- **The hardware record is for this arm.** The inspection report covers the lab's arm, called **E03** there (serial MH217340001), **Mini Cabinet HRC-E0610-A3** and pendant RTP10-0-0-E, inspected 21 Sep 2023 [QC p.1]. The CAD archive is also an E03: `mechanism/HansRobot_E03 3kg 590_总线末端-STEP.rar`. Its name suggests a 3 kg payload, 590 mm reach and the "bus end" (总线末端) variant. [E03] [UNCLEAR]
- **Missing for the E03-Pro:** rated payload and reach (only the CAD file name hints at them), mass, joint speed and torque ratings, and whether a force/torque sensor is fitted. [MISSING]
- **The manuals are generic.** The protocol is "common to all Elfin series robots" [PROTO p.I] and its limit ranges "depend on the model" [PROTO p.49–52]. LINUX and CN-SW give values for "the 5 kg Elfin" and refer to an attachment for other models that is not included [LINUX p.50; CN-SW p.39]. The owner reports that the manuals are the 3 Pro's; the "5 kg" wording stands as printed, so their limit tables may not be this arm's values. [UNCLEAR]
- **No SDK.** The protocol refers to `HansRobot_ErrorCode.docx` (error codes and the state-machine table) and to C++, C#, Java and Python SDK manuals [PROTO p.3, p.65, p.192]. None is attached [MISSING], so `elfin_examples.md` is written from the protocol.
- **Controller version.** This arm ran `5.51.5_EPro4ms.beta.20230522` at inspection [QC p.2–3]; it may have been updated since. The protocol's revision table lists controller versions 20220314.180 to 20220825.214 [PROTO p.II]. How the two numbering schemes relate is not stated. [MISSING] The pendant's About screen shows the version [LINUX p.19].

## Two ways to drive the arm

| | A. Commands to the Han's controller (these files) | B. ROS EtherCAT driver |
|---|---|---|
| Who runs the joints | The controller; the PC sends text commands over TCP [PROTO p.1] | The PC, talking EtherCAT (SOEM) to the arm's joint modules over a LAN cable [ROS README.md:20, :148, :159] |
| Rate | Commands on demand; state pushed at up to 20 Hz [PROTO p.7]; servo period set by `StartServo` [PROTO p.172] | 1 ms loop [ROS elfin_ros_control/src/elfin_hardware_interface.cpp:649] |
| Needs | A network cable | Ubuntu 20.04 and ROS noetic [ROS README.md:12], a PREEMPT_RT kernel [README.md:159], the vendor's `elfin_drivers.yaml` [README.md:146] |
| Controller safety functions | Reported over the protocol (e-stop, light curtain) [PROTO p.60] | Not stated whether any apply [MISSING]; treat as unprotected |

Path B flags:

- `elfin_drivers.yaml` holds the E03's encoder zero counts (equal to the origin counts on [QC p.3]) and the Elfin3 torque factors [ROS elfin_robot_bringup/config/elfin_drivers.yaml:23, :25]. Joint order is J2, J1, J3, J4, J5, J6 [yaml:3, :21]. [E03] Since this is that arm, the zero counts are likely this arm's; the file is still not to be used as it is until checked against the arm and confirmed by the vendor for the E03-Pro.
- The file also lists Elfin5 torque factors [yaml:11]; for this arm the Elfin3 factors are presumably the relevant ones. [UNCLEAR]
- The branch is for "bus end" arms; "485 end" arms differ [README.md:159, :191]. Which one this arm is: [MISSING]; the CAD archive's name says "bus end". [UNCLEAR]
- The torque command sent to the drives is computed from the measured `effort` (filled at cpp:574, :584), not from `effort_cmd` [cpp:617–618]. Check this before using a torque mode. [UNCLEAR]

## Network setup

1. Cable the PC to the cabinet port **LAN1**. **LAN1_ROBOT** is taken by the robot [NET p.2]. The guide does not name the cabinet in its drawing and photo; this arm has a Mini Cabinet HRC-E0610-A3 [QC p.1]. [MISSING] Check that your cabinet has ports labelled LAN1 and LAN1_ROBOT.
2. On the pendant, 管理 → 网络, interface `enp1s0`: set a fixed IP such as `192.168.0.10` with mask `255.255.255.0` and apply; the system restarts [NET p.3]. 192.168.0.0, .1 and .255 are not allowed [NET p.3].
3. Put the PC on the same subnet, e.g. `192.168.0.29` / `255.255.255.0` [NET p.4–5], and test with `ping 192.168.0.10` [NET p.5].
4. Optional web client: `http://<robot-ip>/dist`, default login admin / admin [NET p.6]. Use Chrome or Edge on a PC with more than 4 GB RAM and at least 4 CPUs [NET p.1].

- **One client at a time.** Only one client can use the robot console; a web login kicks the pendant off [NET p.1]. The wording leaves open whether a client on port 10003 counts. [UNCLEAR]
- The lab's scripts connect to `192.168.0.10:10003` [MATLAB Connect_robotarm.m:3]. [E03]
- The controller can also act as a TCP client that dials the PC when an IO signal triggers it [PROTO p.4]. These files assume server mode.
- Settings sent over the protocol are not saved and revert at controller restart; settings made on the pendant are saved [PROTO p.5].

## Ports [PROTO p.7]

| Port | Service |
|---|---|
| 10003 | Commands (`elfin_commands.md`) |
| 10004 / 10005 / 10006 | State ("Datasheet") pushed as JSON at 20 / 10 / 5 Hz |
| 10014 / 10015 / 10016 | The same as a binary struct at 20 / 10 / 5 Hz; layout [MISSING] |
| 10502 | Modbus TCP server, holding registers 0–99 |
| 30003 | Log server: sends log entries as they occur |

## Power and enable

- **Off, powered, enabled.** Powering on leaves the brakes on and the robot disabled. Enabling releases the brakes "with sound and slight movement", so keep the area clear [LINUX p.17–18]. Over TCP: `Electrify` powers on [PROTO p.10], `GrpEnable` enables [PROTO p.17].
- **Before enabling**, payload, mounting angle, TCP and origin must be set correctly. Otherwise "the robot arm and the electric cabinet will not work properly and will pose a hazard to people or equipment" [LINUX p.17].
- **The lab's scripts send only `GrpEnable`.** `Electrify` and `StartMaster` are commented out [MATLAB Connect_robotarm.m:5–9], so the arm was presumably powered on from the pendant. Whether a script ever needs `ConnectToBox`, `StartMaster` or `Electrify`: [MISSING]
- **E-stop** cuts the 48 V supply to the arm but not the cabinet's 220 V to 48 V supply [PROTO p.60]. After an e-stop reset the arm is unpowered and must be powered on again [LINUX p.122].
- **Light curtain** stops motion and rejects motion commands, but keeps power [PROTO p.60].
- **Shutting down.** No order is given. [MISSING] `GrpDisable` disables and `BlackOut` powers the arm off [PROTO p.18, p.11]. The lab's script ends with `GrpDisable`, with `CloseMaster` and `BlackOut` commented out [MATLAB Connect_robotarm.m:28–33]. [E03]
- **`OSCmd`** is titled "controller power-off (cut the robot's supply, shut the system down)". Type 1 is listed as stop, disable, cut 48 V; type 2 adds an operating-system restart [PROTO p.8]. Whether type 1 also shuts the controller down: [UNCLEAR]

## Motion limits

Joint positions are in degrees; Cartesian positions in mm and degrees. The default tool frame `TCP` and user frame `Base` are both 0,0,0,0,0,0 [PROTO p.2].

| Limit | Protocol range | Default or example | Source |
|---|---|---|---|
| Joint velocity | 0–200 °/s | read example 170 (J1–J4), 180 (J5–J6) | PROTO p.49, p.56 |
| Joint acceleration | 0–720 °/s² | read example 360 | PROTO p.50, p.57 |
| Joint jerk (read only) | 0–3600 °/s³ | read example 3600 | PROTO p.58 |
| Linear velocity | 0–3000 mm/s | default 2000 | PROTO p.51, p.59 |
| Linear acceleration | 0–2500 mm/s² | default 2500 | PROTO p.52, p.59 |
| Linear jerk (read only) | 0–10,000,000 mm/s³ | default 10,000,000 | PROTO p.59 |
| Speed override | 0.01–1; cannot change while moving | — | PROTO p.46 |
| Payload | 0 to the model's maximum, kg; centre of mass in mm | — | PROTO p.48 |

The velocity and acceleration ranges "depend on the model" [PROTO p.49–52], and the examples don't say which model they come from. [MISSING] Read the real values with `ReadJointMaxVel`, `ReadJointMaxAcc` and `ReadLinearMaxVel`.

Joint ranges (°) disagree between sources, and none is stated for the E03-Pro; the URDF in the table is the Elfin5's, and the attachments mention no E03 description. [UNCLEAR]

| Joint | LINUX p.50–51, CN-SW p.39 ("5 kg Elfin", factory setting) | PROTO p.53 (`SetMaxAcsRange` example) | ROS `elfin_description/urdf/elfin5.urdf.xacro`, rad (°) |
|---|---|---|---|
| J1 | ±360 | ±360 | ±3.14 (±180), lines 131–132 |
| J2 | ±135 | ±135 | ±2.35 (±135), lines 189–190 |
| J3 | ±153 | ±153 | ±2.61 (±150), lines 247–248 |
| J4 | ±360 | ±360 | ±3.14 (±180), lines 305–306 |
| J5 | ±360 | ±180 | ±2.56 (±147), lines 363–364 |
| J6 | ±360 | ±360 | ±3.14 (±180), lines 421–422 |

The URDF also sets 1.57 rad/s (90 °/s) for every joint and efforts of 420, 420, 200, 200, 69 and 69 [lines 133–134 to 423–424]. A Cartesian box limit is set with `SetMaxPcsRange` (mm; rotation limits "not enabled") [PROTO p.55].

## Safety functions

- **E-stops.** Buttons on the cabinet and the pendant comply with IEC 60947-5-5, and the cabinet has an input for an external e-stop. Reset by turning the button [LINUX p.11].
- **Normal and Reduced modes.** Reduced has stricter speed and momentum limits. It is triggered when the TCP crosses a safety boundary or by an input signal [LINUX p.48].
- **Six safety levels** limit tool force, speed, power, momentum, elbow speed and elbow force. Level 1 needs the largest force to stop the arm, level 6 the smallest [LINUX p.48–50]. Values for the lowest level are in the table below.
- **Collision stop** at the level's force threshold; reset to continue [LINUX p.123–124]. **Leaving the safe space** needs a "Restore" [LINUX p.125]. **At a singularity** while jogging, reset and then move joints; Cartesian moves stay invalid until then [LINUX p.126–127].
- **Force-sensor limit.** If the sensor exceeds the range set with `HRSetForceDataLimit`, the controller powers off [PROTO p.124].
- **Unpowered**, a joint can be back-driven with at least 500 N, at the risk of damage [LINUX p.12].
- **One motion at a time.** Send the next motion command only after the current one has finished, checked with `ReadRobotState` [PROTO p.1, p.189].
- Whether joint limits, collision detection and Reduced mode act on servo streams (`PushServoJ`) is not stated. [MISSING]

The manual's screenshot of the lowest safety level [LINUX p.49] shows these limits:

| Mode | Force | Speed | Power | Momentum | Elbow speed | Elbow force |
|---|---|---|---|---|---|---|
| Normal | 300 N | 3000 mm/s | 1000 W | 100 kg·m/s | 3000 | 300 |
| Reduced | 250 N | 2000 mm/s | 500 W | 35 kg·m/s | 2000 | 250 |

Both elbow units are printed as "kg m/s". [UNCLEAR] The screenshot does not name the robot model, and levels 2–6 are not given. [MISSING]

## Check on the E03-Pro before relying on these files

| Item | What the attachments give | How to check |
|---|---|---|
| Model | `ReadRobotModel` example reply `5KG` [PROTO p.16] | Send `ReadRobotModel,;` |
| Controller version | At inspection: 5.51.5_EPro4ms.beta.20230522 [QC p.2] | Pendant About screen [LINUX p.19] |
| Speed limits | The tables above | `ReadJointMaxVel`, `ReadJointMaxAcc`, `ReadLinearMaxVel` |
| Joint ranges | Three sources that disagree (above) | The protocol has a set command [PROTO p.53] but no read command [MISSING]; check the pendant's joint limits [LINUX p.50] |
| Cabinet and LAN port | This arm: Mini Cabinet [QC p.1]; the guide doesn't name its cabinet [NET p.2] | Look for LAN1 and LAN1_ROBOT |
| IP address | In the lab's scripts: 192.168.0.10 [MATLAB Connect_robotarm.m:3] | Pendant network page [NET p.3] |
| Force sensor | Nothing | Example 5 in `elfin_examples.md` |
| Servo timing | Only the example values 0.2 s and 0.02 s [PROTO p.172] | Example 3 in `elfin_examples.md` |
| ROS driver config | E03 zero counts and Elfin3 torque factors [ROS yaml:23, :25] | Check against the arm; ask the vendor to confirm it for the E03-Pro |
