# Elfin E03-Pro: commands (HansRobot V5 TCP protocol)

The commands needed to connect, enable, move, stream joint positions, read state and use a force sensor. Built 22 Oct 2026 from the Elfin control attachments only. Companion files: `elfin_overview.md`, `elfin_examples.md`.

**Correction, 12 Nov 2026.** The arm is an **Elfin E03-Pro**, not an E05-Pro as these files first said. It is the lab's arm, serial MH217340001, the one in the factory inspection report [QC p.1], and the owner reports that the manuals are for the 3 Pro. Facts marked [E03] therefore describe this arm. The model name is corrected throughout.

**Markers.** **[E03]** from the lab's earlier records of this arm (inspection report, MATLAB scripts, ROS config), where it is called an E03; likely to hold, but confirm, since settings and firmware may have changed. **[UNCLEAR]** the sources are ambiguous or contradict each other. **[MISSING]** the attachments don't say. **[UNTESTED]** not tried on hardware.

**Sources.** PROTO = `说明书/HansRobotV5控制通信协议接口.pdf` (HR-TCP-V1.0.0.3, Chinese); LINUX = `说明书/Linux system manual for Elfin cobot.pdf`; MATLAB = `Code/MATLAB/` (the lab's scripts). Pages are the printed footer numbers (PDF page = printed + 7 for PROTO, + 1 for LINUX); code is cited as file:line. Every command below is from PROTO unless marked otherwise. The protocol says it covers all Elfin models [PROTO p.I]; nothing in it is model-specific.

## Message rules

- Send `Name,p1,p2,…,pn,;` as ASCII text to port 10003. Fields are separated by commas, every message ends with `,;`, and unused parameters are sent as 0 [p.3, p.7].
- Success: `Name,OK,r1,…,rn,;`. Failure: `Name,Fail,ErrorCode,;` with an integer code [p.3]. The code list is in `HansRobot_ErrorCode.docx`, which is not attached. [MISSING]
- The controller handles one complete message at a time. If several arrive together, it processes the first valid one and drops the rest; an incomplete message gets an "invalid command" reply [p.1]. Wait for each reply before sending the next command.
- Motion commands reply as soon as the command is accepted, not when the motion ends. Poll `ReadRobotState` until it finishes [p.1, p.189].
- `nRbtID` is the robot ID, 0–5, default 0 [p.17]. Use 0.
- Settings sent this way revert when the controller restarts [p.5].

## Connect and power

| Command | Example (the format unless noted) | Reply and meaning | Page |
|---|---|---|---|
| ReadControllerState | `ReadControllerState,;` | `…,OK,nStarted,;` 1 = controller has finished starting | p.15 |
| ReadRobotModel | `ReadRobotModel,;` | `…,OK,strModel,;` example `5KG` | p.16 |
| IsSimulation | `IsSimulation,;` | 0 = real robot, 1 = simulated | p.14 |
| ConnectToBox | `ConnectToBox,0,;` | connect to the controller cabinet. The format line has no parameter but the example sends 0 [UNCLEAR] | p.9 |
| StartMaster, CloseMaster | `StartMaster,;` | connect to the controller; `CloseMaster` disconnects and shuts the master down | p.12–13 |
| Electrify, BlackOut | `Electrify,;` | power the arm on; `BlackOut` powers it off | p.10–11 |
| OSCmd | `OSCmd,nCmdType,;` | Titled "controller power-off (cut the robot's supply, shut the system down)". 1 = stop, disable, cut 48 V; 2 = the same, then restart the operating system. Whether 1 also shuts the controller down, and the example `OSCmd,1;` lacking a comma: [UNCLEAR] | p.8 |

The protocol gives no start-up order. [MISSING] The manuals' order is power on, then enable [LINUX p.17]. The lab's scripts send only `GrpEnable` [MATLAB Connect_robotarm.m:5–9]. [E03]

## Enable, reset, stop

All take `Name,nRbtID,;` and reply `Name,OK,;`.

| Command | Example | Meaning | Page |
|---|---|---|---|
| GrpEnable | `GrpEnable,0,;` | enable; this releases the brakes [LINUX p.17] | p.17 |
| GrpDisable | `GrpDisable,0,;` | disable | p.18 |
| GrpReset | `GrpReset,0,;` | clear errors; required after a motion error before the next motion [p.189] | p.19 |
| GrpStop | `GrpStop,0,;` | stop motion | p.20 |
| GrpInterrupt, GrpContinue | `GrpInterrupt,0,;` | pause and resume motion | p.21–22 |

## Settings and limits

| Command | Format | Units and range | Example | Page |
|---|---|---|---|---|
| SetOverride | `SetOverride,nRbtID,dOverride,;` | speed ratio 0.01–1; cannot change while moving | `SetOverride,0,0.5,;` | p.46 |
| SetPayload | `SetPayload,nRbtID,dMass,dX,dY,dZ,;` | kg; centre-of-mass offset in mm (range printed as ≥0) | `SetPayload,0,3,10,20,30,;` | p.48 |
| SetJointMaxVel | `SetJointMaxVel,nRbtID,dJ1,…,dJ6,;` | °/s, 0–200 | `SetJointMaxVel,0,180,180,180,180,200,200,;` | p.49 |
| SetJointMaxAcc | `SetJointMaxAcc,nRbtID,dJ1,…,dJ6,;` | °/s², 0–720 | `SetJointMaxAcc,0,360,360,360,360,360,360,;` | p.50 |
| SetLinearMaxVel | `SetLinearMaxVel,nRbtID,dMaxVel,;` | mm/s, 0–3000 | `SetLinearMaxVel,0,1000,;` | p.51 |
| SetLinearMaxAcc | `SetLinearMaxAcc,nRbtID,dMaxAcc,;` | mm/s², 0–2500 | `SetLinearMaxAcc,0,2500,;` | p.52 |
| ReadJointMaxVel, ReadJointMaxAcc, ReadJointMaxJerk | `ReadJointMaxVel,nRbtID,;` | 6 values in °/s, °/s², °/s³ | `ReadJointMaxVel,0,;` → `…,OK,170.000,170.000,170.000,170.000,180.000,180.000,;` | p.56–58 |
| ReadLinearMaxVel | `ReadLinearMaxVel,nRbtID,;` | 3 values: mm/s, mm/s², mm/s³ | → `…,OK,1000.000,2500.000,10000000.000,;` | p.59 |

The velocity and acceleration ranges "depend on the model" [p.49–52].

## Move to a point: WayPoint [p.137–139]

```
WayPoint,nRbtID,dX,dY,dZ,dRx,dRy,dRz,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,sTcpName,sUcsName,dVelocity,dAcc,dRadius,nMoveType,nIsUseJoint,nIsSeek,nIOBit,nIOState,strCmdID,;
```

| Parameter | Meaning |
|---|---|
| nMoveType | 0 = joint move, 1 = linear move |
| nIsUseJoint | Joint moves only. 1 = go to dJ1–dJ6. 0 = go to dX–dRz by inverse kinematics, using dJ1–dJ6 to choose the solution |
| dX–dRz | Target pose, mm and °. Used by linear moves and by joint moves with nIsUseJoint = 0 |
| dJ1–dJ6 | Target joints, °. Ignored by linear moves |
| sTcpName, sUcsName | Names of tool and user frames taught on the pendant; defaults `TCP` and `Base`. Ignored when nIsUseJoint = 1 |
| dVelocity, dAcc | Joint move: °/s and °/s². Linear move: mm/s and mm/s² (rotations in °/s and °/s²) |
| dRadius | Blend radius, mm |
| nIsSeek, nIOBit, nIOState | If nIsSeek = 1, stop when cabinet digital input nIOBit (0–7) equals nIOState |
| strCmdID | Your ID for this waypoint; read it back with `ReadCurWayPointID` [p.63] |

Examples [p.139]:

```
WayPoint,0,420,0,445,180,0,180,0,0,90,0,90,0,TCP,Base,50,360,0,0,1,0,0,0,0,;   joint move to J = 0,0,90,0,90,0 at 50 °/s
WayPoint,0,420,0,445,180,0,180,0,0,90,0,90,0,TCP,Base,50,360,0,1,0,0,0,0,0,;   linear move to 420,0,445 mm / 180,0,180° at 50 mm/s
```

- The Datasheet's pose is in the current user and tool frames [p.186]. `ReadActPos` does not say which frames its pose is in [MISSING], but it returns the current frames alongside it [p.66–67]. Before reusing a read pose as a linear target, pass matching frame names or check that both current frames are zero.
- `WayPointEx` takes numeric tool and user frames instead of names [p.134–136]. `WayPoint2` adds arcs (nMoveType 2). It can pass from a line into an arc without slowing; `WayPoint` does that only from line to line [p.140–142]. `MoveJ` and `MoveL` are deprecated [p.143, p.146].
- The lab's scripts use linear WayPoints at 50 mm/s and 360 mm/s² [MATLAB Connect_robotarm.m:15–25]. [E03]

## Jog by a relative step

| Command | Format | Parameters | Example | Page |
|---|---|---|---|---|
| MoveRelJ | `MoveRelJ,nRbtID,nAxisId,nDirection,dDistance,;` | nAxisId 0–5 = J1–J6; nDirection 0 = negative, 1 = positive. The unit (°) appears only in the example's comment, and the format line lacks the last comma [UNCLEAR] | `MoveRelJ,0,1,1,10,;` (J2 +10°) | p.129 |
| MoveRelL | `MoveRelL,nRbtID,nAxisId,nDirection,dDistance,nToolMotion,;` | nAxisId 0–5 = X, Y, Z, Rx, Ry, Rz; nToolMotion 0 = current user frame, 1 = tool frame. Unit (mm) only in the example's comment [UNCLEAR]; rotation unit not stated [MISSING] | `MoveRelL,0,1,1,10,1,;` (Y +10 mm, tool frame) | p.130 |

The lab's scripts jog X, Y and Z with `MoveRelL` [MATLAB robotlib/move_robot.m:7–13]. [E03]

## Wait for a move to finish [p.189–191]

Poll `ReadRobotState` every 10 ms. nMovingState = 1 means moving and 0 means done. On an error, send `GrpReset`, and send the next motion only after the reset succeeds.

- The lab's scripts wait for nBlendingDone = 1 instead, polling every 10 ms [MATLAB robotlib/get_state.m:11–12, robotlib/waitaction.m:5–7]. [E03] Which flag is more reliable is not stated. [MISSING]
- How soon nMovingState turns 1 after the reply is not stated. [MISSING] A poll straight after sending may see 0 before the move starts, so `elfin_examples.md` first waits for the move to start.

## Stream positions: servo mode [p.172–175]

**StartServo** sets "the fixed update period and the lookahead time" for online control with ServoJ or ServoP.

- Format: `StartServo,nRbtID,dServoTime,dLookaheadTime,;`
- dServoTime: fixed update period, s, > 0. dLookaheadTime: lookahead time, s, > 0.
- Example: `StartServo,0,0.2,0.02,;`, which in parameter order is a 0.2 s period and a 0.02 s lookahead.

**PushServoJ** sends joint positions "at the fixed update time set by StartServo", and "the robot tracks the joint position command in real time".

- Format: `PushServoJ,nRbtID,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,;` in degrees.
- Example: `PushServoJ,0,0,0,0,0,0,0,;`. The failure example shows code 40014, whose meaning is not given.

**PushServoP** sends a TCP pose, which the controller converts to joints.

- Format: `PushServoP,nRbtID,dX,dY,dZ,dRx,dRy,dRz,dUcs_X,…,dUcs_Rz,dTcp_X,…,dTcp_Rz,;` in mm and °. The parameter table lists the tool frame before the user frame. [UNCLEAR]
- Example: `PushServoP,0,420,0,445,180,0,180,0,0,0,0,0,0,0,0,0,0,0,0,;`

Not documented [MISSING]:

- the minimum or recommended period, and what the lookahead does to the motion (smoothing, added delay);
- how to leave servo mode (there is no StopServo command);
- what happens when pushes arrive late, early or stop;
- whether joint limits, speed limits and collision detection apply while streaming.

By the one-message rule [p.1], each push must get its reply before the next is sent. Measure the reply time and the tracking delay on the E03-Pro before closing a control loop through this path. [UNTESTED]

## Read state

**ReadRobotState** `ReadRobotState,0,;` [p.61–62]. Example reply: `ReadRobotState,OK,1,1,0,0,0,0,0,0,0,1,1,0,0,;`

| # | Field | Meaning |
|---|---|---|
| 1 | nMovingState | 1 = moving |
| 2 | nEnableState | 1 = enabled |
| 3 | nErrorState | 1 = error |
| 4 | nErrorCode | error code |
| 5 | nErrorAxis | axis index, 0–5 |
| 6 | nBreaking | brakes: 0 = engaged, 1 = released |
| 7 | nPause | 1 = paused |
| 8 | nEmergencyStop | 1 = e-stop active |
| 9 | nSaftyGuard | 1 = light curtain triggered |
| 10 | nElectrify | 1 = powered on |
| 11 | nIsConnectToBox | 1 = cabinet connected |
| 12 | nBlendingDone | 0 = waypoint motion running, 1 = waypoint motion done |
| 13 | nInPos | 1 = actual position has reached the command |

Other reads, all sent as `Name,0,;`:

| Command | Returns | Page |
|---|---|---|
| ReadActPos | 24 values: J1–J6 (°); X, Y, Z (mm) and Rx, Ry, Rz (°); then the current tool frame (6) and user frame (6). Example `ReadActPos,OK,10.759,0.000,37.828,…,;` | p.66–67 |
| ReadCmdPos | 12 values: commanded J1–J6, then X–Rz | p.68 |
| ReadActJointVel, ReadCmdJointVel | 6 values, °/s | p.71, p.70 |
| ReadActTcpVel, ReadCmdTcpVel | X, Y, Z in mm/s; Rx, Ry, Rz in °/s | p.73, p.72 |
| ReadActJointCur, ReadCmdJointCur | 6 values, A | p.75, p.74 |
| ReadEmergencyInfo | nESTO_Error, nESTO, nSafetyGuard_Error, nSafetyGuard (0/1). "_Error" = the two redundant channels disagree | p.60 |
| ReadAxisErrorCode | group error code, then J1–J6 codes (0 = none) | p.64 |
| ReadCurFSM | state-machine number; the table is in the missing ErrorCode document | p.65 |

## State stream: Datasheet [p.7, p.185–188]

Connect to port 10004 (JSON at 20 Hz; 10005 is 10 Hz and 10006 is 5 Hz) and read; the controller pushes the packets. No request message is described. [MISSING]

**Packet** [p.188]: `RBTL` (4 bytes), total length including the header (4 bytes), data length (4 bytes), then the JSON. Byte order and signedness are not stated. [MISSING] Since total = data + 12, the right byte order is the one where that holds.

| Key | Meaning |
|---|---|
| PosAndVel.Actual_Position | 12 values: J1–J6 (°), then the pose in the current user and tool frames (mm, °) |
| PosAndVel.Actual_PCS_Base | Pose in the base frame |
| PosAndVel.Actual_Joint_Velocity, _Acceleration, _Current | Per joint: °/s, °/s², A |
| PosAndVel.Actual_Override | Speed ratio, 0–1 |
| StateAndError.robotState | State-machine number (table missing) |
| StateAndError.robotEnabled, robotPaused, robotMoving, robotBlendingDone, InPos | 0/1, as in ReadRobotState |
| StateAndError.Error_AxisID, Error_Code | Error axis 1–6 (ReadRobotState counts 0–5 [UNCLEAR]) and code |
| StateAndError.BrakeState | Per joint: 0 = engaged, 1 = released |
| HardLoad.EtherCAT_FramesPerSecond, EtherCAT_TotalLostFrame | Controller-to-arm bus rate and its total error count |
| FTData.FTControlState | Force control: 0 = off, 1 = on |
| FTData.FTData, FTData.FTSrcData | Force sensor, calibrated and raw: X, Y, Z in N; Rx, Ry, Rz in N·m |

The manual's JSON example has three quirks [p.185–186]; whether the real controller does the same is [UNTESTED]:

- many numbers are strings (`"0.000"`): everything under PosAndVel, plus Conveyor, the cabinet's analog input and output values, and Script.errorCode;
- a trailing comma after `"nAxisGroupErrorCode":[0],` makes it invalid JSON;
- there is no timestamp, so stamp each packet on arrival.

Also: Actual_PCS_Tool "is no longer provided in new versions" [p.187]; the struct ports 10014–10016 have no documented layout. [MISSING]

## Force sensor [p.109–128]

Whether the E03-Pro has a force sensor is not in the attachments. [MISSING] Example 5 in `elfin_examples.md` checks.

| Command | Format and example | Notes | Page |
|---|---|---|---|
| ReadFTCabData | `ReadFTCabData,0,;` → `…,OK,dX,dY,dZ,dRx,dRy,dRz,;` | Calibrated data; N for X–Z, "NM" (N·m) for Rx–Rz | p.128 |
| SetForceZero | `SetForceZero,0,;` | Re-zero: "recalibrates the sensor on top of the existing parameters" | p.114 |
| SetFTPosition | `SetFTPosition,nRbtID,dX,dY,dZ,dRx,dRy,dRz,;` e.g. `SetFTPosition,0,0,0,110,0,0,60,;` | Sensor pose relative to the flange, mm and ° | p.118 |
| SetForceControlState | `SetForceControlState,0,1,;` | 1 = force control on; "the robot then switches to the motion state" | p.109 |
| ReadFTControlState | `ReadFTControlState,0,;` | 0 = off, 1 = seeking, 2 = seek done, 3 = free drive. The format line also lists nState [UNCLEAR] | p.110 |
| HRSetForceDataLimit | `HRSetForceDataLimit,0,500,500,500,500,500,500,-500,-500,-500,-500,-500,-500,;` | Maxima then minima, N and N·m. Exceeding them powers the controller off | p.124–125 |

The Datasheet carries the same sensor data as `FTData.FTData` (calibrated) and `FTData.FTSrcData` (raw) [p.186, p.188]. Other force-control settings (search speed, freedoms, strategy, PID, mass, damping, stiffness, goal force, deformation range, free drive) are on p.115–127.

## Errors in the protocol document [UNCLEAR]

| Item | Problem | Page |
|---|---|---|
| OSCmd | Example lacks the comma before `;` | p.8 |
| ConnectToBox | Format has no parameter; the example sends 0 | p.9 |
| SetLinearMaxAcc | Format names the parameter dMaxVel; the table says dMaxAcc | p.52 |
| SetMaxAcsRange | Format lists dMaxJ6 twice; the last is presumably dMinJ6 | p.53 |
| ReadLinearMaxVel | Format reply lists dJ1–dJ6; the table and example have 3 values | p.59 |
| ReadCmdPos | Failure example is labelled ReadActPos | p.69 |
| ReadFTControlState | Format includes nState; the example does not | p.110 |
| MoveRelJ | Format ends `dDistance;` without the comma | p.129 |
| PushServoP | Frame order differs between format and table | p.174 |
| Datasheet JSON | String numbers and a trailing comma | p.185–186 |
| Error axis | ReadRobotState counts 0–5, Datasheet 1–6 | p.61, p.187 |
