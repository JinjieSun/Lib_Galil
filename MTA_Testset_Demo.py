from GalilRobot import GalilRobot
# from Aurora import Aurora
import numpy as np
import time

rob_exist = True
tracker_exist = False
n_measurements = 10
break_time_measurement = 0.1

up_bool = True
in_bool = True

# -------------- initialization: robot --------------
if rob_exist:
    print('-------------- initialization: robot --------------')
    rob = GalilRobot("GalilConfig/ConfigGalil_MiniTubuActu.xml")
    rob.connect()
    rob.motorsOn()
    rob.setHome()
    time.sleep(1)
    rob.printInfo(detailed=True)
else:
    up_bool = False
    in_bool = False

# -------------- initialization: measurement system --------------
if tracker_exist:
    pass
    # print '-------------- initialization: measurement system --------------'
    
    # tracker = Aurora(baud_rat=9600)

    # if not tracker._isConnected:
    #     tracker.connect()
    #     time.sleep(0.1)

    # tracker.init()
    # time.sleep(0.1)
    # tracker.portHandles_detectAndAssign_FlowChart(printFeedback=True)
    # time.sleep(0.1)
    # tracker.portHandles_updateStatusAll()

# -------------- Motion: up --------------
if in_bool:
    viaPoints = np.array([
        [0, 0, 0, -10, 0, -10],
        [0, 0, 0, 10, 0, 10]
    ])
    rob.setMotorConstraints('MaxSpeed', 150000)
    rob.setMotorConstraints('MaxAcc', 1000000)
    rob.setMotorConstraints('MaxDec', 1000000)

# -------------- Motion: up --------------
if up_bool:
    viaPoints = np.array([
        [0, 0, 0, 0, 180, 0],
        [0, 0, -0, 0, -180, 0]
    ])
    rob.setMotorConstraints('MaxSpeed', 300000)
    rob.setMotorConstraints('MaxAcc', 1000000)
    rob.setMotorConstraints('MaxDec', 1000000)

if up_bool or in_bool:
    print('-------------- Motion: up/in --------------')
    #tracker._BEEP(1)
    rob.jointPTPLinearMotionSinglePoint(viaPoints[0], sKurve=True, sKurveValue=0.004)
    time.sleep(.5)

# -------------- start tracing (Aurora) --------------
if tracker_exist:
    # print '-------------- start tracing --------------'
    # tracker.trackingStart()
    # time.sleep(3)  # yes, it is a long pause...
    # tracker.sensorData_write_FileName('/home/rgrassmann/PycharmProjects/Messungen/', 'data_ref.txt')
    # tracker.sensorData_write_FileIni()

    # tracker.sensorData_collectData(n_times=n_measurements)
    # t_ref = np.array([
    #     tracker._port_handles[0]._trans[0],
    #     tracker._port_handles[0]._trans[1],
    #     tracker._port_handles[0]._trans[2]
    # ])
    # q_ref = np.array([
    #     tracker._port_handles[0]._quaternion[0],
    #     tracker._port_handles[0]._quaternion[1],
    #     tracker._port_handles[0]._quaternion[2],
    #     tracker._port_handles[0]._quaternion[3]
    # ])
    # tracker.portHandle_setRefFrame(q_ref, t_ref, ith_port_handel=0)

    # for ith in xrange(n_measurements):
    #     time.sleep(break_time_measurement)
    #     tracker.sensorData_collectData(n_times=10)
    #     tracker.sensorData_write(ith, data_wrt_Ref=True)

    # print '-------------- stop tracing --------------'
    # tracker.sensorData_write_FileClose()

# -------------- start tracing (MTA joints) --------------
# if rob_exist:
#     beta1 = np.zeros(n_measurements)
#     beta2 = np.zeros(n_measurements)
#     beta3 = np.zeros(n_measurements)
#     alpha1 = np.zeros(n_measurements)
#     alpha2 = np.zeros(n_measurements)
#     alpha3 = np.zeros(n_measurements)
#     for ith in xrange(n_measurements):
#         time.sleep(break_time_measurement)
#         alpha1[ith], beta1[ith], alpha2[ith], beta2[ith], alpha3[ith], beta3[ith] = rob.getJointPositions()
#
#     print 'mean beta1', np.mean(beta1), ' und std ', np.std(beta1)
#     print 'mean beta2', np.mean(beta2), ' und std ', np.std(beta2)
#     print 'mean beta3', np.mean(beta3), ' und std ', np.std(beta3)
#     print 'mean alpha1', np.mean(alpha1), ' und std ', np.std(alpha1)
#     print 'mean alpha2', np.mean(alpha2), ' und std ', np.std(alpha2)
#     print 'mean alpha3', np.mean(alpha3), ' und std ', np.std(alpha3)
#time.sleep(20)
    pass
# -------------- Motion: down --------------
if up_bool or in_bool:
    print('-------------- Motion: down/out --------------')
    # tracker._BEEP(1)
    rob.jointPTPLinearMotionSinglePoint(viaPoints[1], sKurve=True, sKurveValue=0.004)
    time.sleep(.5)

# -------------- close robot and measurement system --------------
print('-------------- close robot and measurement system --------------')
# robot
if rob_exist:
    rob.motorsOff()
    rob.disconnect()

# measurement system
# if tracker_exist:
#     # tracker.disconnect()
