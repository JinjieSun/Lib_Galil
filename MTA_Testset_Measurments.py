from GalilRobot import GalilRobot
from Aurora import Aurora
import numpy as np
import time

rob_exist = False
tracker_exist = True
trajectory_random = True

n_measurements = 1
m2mm = 1000.
rad2grad = 180./np.pi

# -------------- initialization: measurement system --------------
if tracker_exist:
    print '-------------- initialization: measurement system --------------'
    tracker = Aurora(baud_rat=9600)

    if not tracker._isConnected:
        tracker.connect()
        time.sleep(0.1)

    tracker.init()
    time.sleep(0.1)
    tracker.portHandles_detectAndAssign_FlowChart(printFeedback=True)
    time.sleep(0.1)
    tracker.portHandles_updateStatusAll()

# -------------- initialization: robot --------------
if rob_exist:
    print '-------------- initialization: robot --------------'
    rob = GalilRobot("GalilConfig/ConfigGalil_MiniTubuActu.xml")
    rob.connect()
    rob.motorsOn()
    rob.setHome()
    rob.printInfo(detailed=True)
    time.sleep(.5)
    print 'home', rob.getJointPositions()

# -------------- define trajectory --------------
tic = time.time()
if rob_exist:
    if rob_exist:
        print '-------------- define trajectory --------------'
        rob.setMotorConstraints('MaxSpeed', 275000)
        rob.setMotorConstraints('MaxAcc',  1000000)
        rob.setMotorConstraints('MaxDec',  1000000)

    # pfad = '/home/rgrassmann/PycharmProjects/MiniTubuActu/CTR_Datenerzeugung/Daten/q/daten_MiniTubuActu_Joints_0_test.csv'
    pfad = '/home/rgrassmann/PycharmProjects/MiniTubuActu/CTR_Datenerzeugung/Daten/q/daten_MiniTubuActu_Joints_20_hr.csv'

    if trajectory_random:
        with open(pfad) as data_csv:
            data_list = list(data_csv)
            nb_row = len(data_list)

            viaPoints = []
            idx = 0
            for element in data_list:
                viaPoints.append([
                    float(element.split(',')[6])*rad2grad,
                    float(element.split(',')[7])*m2mm,
                    float(element.split(',')[8])*rad2grad,
                    float(element.split(',')[9])*m2mm,
                    float(element.split(',')[10])*rad2grad,
                    float(element.split(',')[11])*m2mm
                ])
                idx += 1
    else:
        viaPoints = np.array([
            [90., -10., 90., -10., 90., -10.],
            [90., -10., 90., -10., 90., -10.],
            [-180., 15., -180., 20., -180., 20.]
        ])

    viaPoints = np.array(viaPoints)
    viaPointGoHome = np.array(-viaPoints.sum(axis=0))
    # viaPointGoHome = np.array([
    #         [0, 0, 0, 0, 0, -10],
    #         [0, 0, 0, 0, 0, 10]
    #     ])
    n_measurements = len(viaPoints)

    # print viaPoints
    # print viaPointGoHome

# -------------- start tracing --------------
if tracker_exist:
    print '-------------- start tracing --------------'
    tracker.trackingStart()
    time.sleep(3)  # yes, it is a long pause...
    tracker.sensorData_write_FileName('/home/rgrassmann/PycharmProjects/Messungen/', 'data_measurements_20_hr.txt')
    tracker.sensorData_write_FileIni()


# -------------- move and measure sequentially --------------
if tracker_exist:
    q_ref = np.array([0.66797637,	0.03087653,	0.74339460,	-0.01478756])
    t_ref = np.array([-174.08583679,	1.60529678,	-331.67276611])
    tracker.portHandle_setRefFrame(q_ref, t_ref, ith_port_handel=0)


for ith in xrange(n_measurements):
    print '-------------- Step:', ith, '--------------'

    if rob_exist:
        print '-------------- Motion:', ith, '--------------'
        rob.jointPTPLinearMotionSinglePoint(viaPoints[ith], sKurve=True, sKurveValue=0.004)
        time.sleep(.5)

    if tracker_exist:
        print '-------------- Measurement:', ith, '--------------'
        tracker.sensorData_collectData(n_times=5)
        tracker.transformation_measurement2ref()
        tracker.sensorData_write(ith)

# -------------- stop tracing --------------
if rob_exist:
    rob.jointPTPLinearMotionSinglePoint(viaPointGoHome, sKurve=True, sKurveValue=0.004)
    time.sleep(.5)

if tracker_exist:
    print '-------------- stop tracing --------------'
    tracker.sensorData_write_FileClose()

# -------------- close robot and measurement system --------------
print '-------------- close robot and measurement system --------------'
toc = time.time()
print 'duration for ', n_measurements, ' is ', toc - tic, ' sec'

# robot
if rob_exist:
    print 'close robot'
    rob.motorsOff()
    rob.disconnect()

# measurement system
if tracker_exist:
    print 'close measurement system'
    tracker.disconnect()
