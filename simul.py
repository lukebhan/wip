import numpy as np
import math
import pandas as pd
from motor import Motor
from airframe import Airframe
from allocation import Allocator
from altitudeController import AltitudeController
from positionController import PositionController
from attitudeController import AttitudeController

# Initialize Motors
MotorParams = {"torqueConst": 0.0265, "equivResistance": 0.2700, "currentSat": 38, "staticFric": 0, "damping": 0, "J": 5.0e-5, "thrustCoeff": 0.065}
icMotor = 0
sampleTime = 0.01
motorArray = []
for i in range(8):
    m = Motor(MotorParams, icMotor, sampleTime)
    motorArray.append(m)

# Initialize Airframe
TarotParams = {"g": 9.80, "m": 10.66, "l": 0.6350, "b": 9.8419e-05, "d": 1.8503e-06, "minAng": math.cos(math.pi/8), "maxAng": math.cos(3*math.pi/8), "Ixx": 0.2506, "Iyy": 0.2506, "Izz": 0.4538, "maxSpeed": 670, "voltageSat": 0.0325}
tarot = Airframe(TarotParams, sampleTime)

# Initialize Control Allocation
allocator = Allocator(TarotParams)

# Initial controllers
AltitudeControllerParams = {'m': 10.66, 'g':9.8, 'kdz': -1, 'kpz': -0.5}
altitudeController = AltitudeController(AltitudeControllerParams)
PositionControllerParams = { 'kpx': 0.1, 'kdx': 0, 'kpy': 0.1, 'kdy': 0, 'min_angle': -12*math.pi/180, 'max_angle': 12*math.pi/180 }
positionController = PositionController(PositionControllerParams)
AttitudeControllerParams = {"kdphi": 1, "kpphi": 3, "kdpsi": 1, "kppsi": 3, "kdtheta": 1, "kptheta": 3}
attitudeController = AttitudeController(AttitudeControllerParams)

# Build Trajectories
traj = "e"
if traj == '8':
    xrefarr = pd.read_csv("xref8traj.csv", header=None).iloc[:, 1]
    yrefarr = pd.read_csv("yref8traj.csv", header=None).iloc[:, 1]
elif traj == 'e':
    xrefarr = pd.read_csv("xrefEtraj.csv", header=None).iloc[:, 1]
    yrefarr = pd.read_csv("yrefEtraj.csv", header=None).iloc[:, 1]
elif traj == "zig":
    yrefarr = pd.read_csv("yrefZigtraj.csv", header=None).iloc[:, 1]
    xrefarr = pd.read_csv("xrefZigtraj.csv", header=None).iloc[:, 1]

zref = 3
# Psi ref = 0
psiRef = 0

# Simulate 
# time is in 1/100 of second
time = 7000
arr1 = []
arr2 = []
arr3 = []
xref = xrefarr[0]
yref = yrefarr[0]
np.savetxt('xref', xrefarr)
np.savetxt('yref', yrefarr)
state = tarot.getState()
count = 0
for i in range(time):
    if i % 100 == 0:
        count += 1
        count = min(len(xrefarr)-1, count)
        xref = xrefarr[count]
        yref = yrefarr[count]
    arr1.append(state[0])
    arr2.append(state[1])
    fz = altitudeController.output(state, zref)
    thetaRef, phiRef = positionController.output(state, [xref, yref])
    roll, pitch, yaw = attitudeController.output(state, [phiRef, thetaRef, psiRef]) 
    uDesired = [fz, roll, pitch, yaw]
    refVoltage = allocator.getRefVoltage(uDesired)
    # iterate over each motor
    # introduce resistance fault in motor 4
    if i == 4000:
        motorArray[3].setRes(0.32)
    rpm = np.zeros(8, dtype=np.float32)
    for idx, motor in enumerate(motorArray):
        rpm[idx] = motor.getAngularSpeed(refVoltage[idx])
    state = tarot.update(rpm)
    matrix = tarot.getRotMatrix()
    err = state[0:3] - np.array([xref, yref, zref])
    data = np.concatenate([state, matrix, err])
    arr3.append(data)
    # Add noise 
    if i  % 100 == 0:
        state[0] += np.random.normal(0, 0.1)
        state[1] += np.random.normal(0, 0.1)
np.savetxt('x', arr1)
np.savetxt('y', arr2)
np.savetxt('data5', arr3)
