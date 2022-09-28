#import gclibDummy as gclib
import gclib
import warnings
import xml.etree.ElementTree as ET
import collections
import math
from enum import IntEnum
import numpy as np
import time


class RobotAxisID(IntEnum):
    """
    Defines robot axis (joint space convention):

    q = [alpha1, beta1, alpha2, beta2, alpha3, beta3, etc. ]

    (only CTR with three tubes is defined)
    """
    alpha1 = 0
    beta1 = 1

    alpha2 = 2
    beta2 = 3

    alpha3 = 4
    beta3 = 5

class GalilRobot:
    # Named Tuples to store informations
    GalilInfo = collections.namedtuple('GalilInfo', 'name ipAddr')
    GalilChannel = collections.namedtuple('GalilChannel', 'ChannelID RobotAxisID PIDParam DynParam')
    PIDParameter = collections.namedtuple('PIDParameter', 'Kp Ki Kd')
    DynamicParameter = collections.namedtuple('DynamicParameter', 'maxSpeed maxAcc')

    # Map galil channel to position in command string
    _dictChanToIdx = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6, 'H': 7}  # fixed convention

    def __init__(self, xmlpath, numActuator=6):
        self._galilBoard = gclib.py()
        GalilInfo = collections.namedtuple('GalilInfo', 'name ipAddr')
        self._info = GalilInfo('', '')
        self._isConnected = False
        self._xmlpath = xmlpath
        self._dictChanToRobotAxisID = {}  # mapping RobotAxisID -> ChannelID
        self._dictRobotAxisIDToChan = {}  # mapping ChannelID   -> RobotAxisID

        # store parameters for every robot axis -> access over index with RobotAxisID
        self._numChan = len(self._dictChanToIdx)
        self._numActuator = numActuator
        self._listKp = [6.] * self._numActuator              # controller gain: min=0, max=4095875, default= 6
        self._listKi = [0.] * self._numActuator              # controller gain: min=0, max= 255999, default= 0
        self._listKd = [64.] * self._numActuator             # controller gain: min=0, max=4095875, default=66
        self._listMaxSpeed = [25000.] * self._numActuator
        self._listMaxAcc = [256000.] * self._numActuator
        self._listMaxDec = [256000.] * self._numActuator
        self._listErrorLimit = [16384.] * self._numActuator
        self._listUnits = ['-'] * self._numActuator
        self._listUnitToTick = [1] * self._numActuator  # Ticks = joint * _listUnitToTick

        # It is assumed that the first channels are for motors.
        self._numActuatorStr = ''.join(chr(ord('A')+x) for x in xrange(self._numActuator))

        # Load robot
        self._parse_config_file(xmlpath)

        # position in joint space np.array([a1, b1, a2, b2, a3, b3])
        self._listJointpositions = [0.0] * self._numActuator
        self._listJointpositionsMin = np.array([0.0, -144.0, 0.0, -115.0, 0.0, -81.0])  # <- hard gecodet...
        self._listJointpositionsMax = np.array([2*np.pi, 0.0, 2*np.pi, 0.0, 2*np.pi, 0.0])  # <- hard gecodet...


    def __del__(self):
        """Closes connection to GalilBoard."""
        self.disconnect()
        print("\033[93m" +
              'Closes connection to ' + str(self._info[0]) + ' (' + str(self._info[1]) + ') ' +
              " \033[0m")

    def printInfo(self, detailed=False):
        """
        Prints human readable info
        """

        # list which holds the robot axis enum associated to the galil channel
        GalilList = ['-'] * self._numChan
        axisList = ['-'] * self._numChan
        KpList = ['-'] * self._numChan
        KiList = ['-'] * self._numChan
        KdList = ['-'] * self._numChan
        SpeedList = ['-'] * self._numChan
        AccList = ['-'] * self._numChan
        DecList = ['-'] * self._numChan
        ErrorLimitsList = ['-'] * self._numChan
        UnitsList = ['-'] * self._numChan
        UnitToTickList = ['-'] * self._numChan
        JointList = ['-'] * self._numChan
        JointPosition = ['-'] * self._numChan
        JointPositionMin = ['-'] * self._numChan
        JointPositionMax = ['-'] * self._numChan

        for chan, idx in self._dictChanToIdx.items():
            GalilList[idx] = chan
            if chan in self._dictChanToRobotAxisID:
                axisList[idx] = self._dictChanToRobotAxisID[chan].name
                KpList[idx] = str(self._listKp[self._dictChanToRobotAxisID[chan]])
                KiList[idx] = str(self._listKi[self._dictChanToRobotAxisID[chan]])
                KdList[idx] = str(self._listKd[self._dictChanToRobotAxisID[chan]])
                SpeedList[idx] = str(self._listMaxSpeed[self._dictChanToRobotAxisID[chan]])
                AccList[idx] = str(self._listMaxAcc[self._dictChanToRobotAxisID[chan]])
                DecList[idx] = str(self._listMaxDec[self._dictChanToRobotAxisID[chan]])
                ErrorLimitsList[idx] = str(self._listErrorLimit[self._dictChanToRobotAxisID[chan]])
                UnitsList[idx] = str(self._listUnits[self._dictChanToRobotAxisID[chan]])
                UnitToTickList[idx] = str(self._listUnitToTick[self._dictChanToRobotAxisID[chan]])
                JointList[idx] = str(self._dictChanToRobotAxisID[chan].value)
                JointPosition[idx] = str(self._listJointpositions[self._dictChanToRobotAxisID[chan]])
                JointPositionMin[idx] = str(self._listJointpositionsMin[self._dictChanToRobotAxisID[chan]])
                JointPositionMax[idx] = str(self._listJointpositionsMax[self._dictChanToRobotAxisID[chan]])

        infoStr = [
            '>GalilChannels: ' + ' '.join(GalilList),
            '>RobotAxis: ' + ' '.join(axisList),
            '>Units: ' + ' '.join(UnitsList),
            '>UnitsToTicks: ' + ' '.join(UnitToTickList)
        ]

        if detailed:
            infoStr.append('>JointIndex: ' + ' '.join(JointList))
            infoStr.append('>Kp: ' + ' '.join(KpList))
            infoStr.append('>Ki: ' + ' '.join(KiList))
            infoStr.append('>Kd: ' + ' '.join(KdList))
            infoStr.append('>maxAcc: ' + ' '.join(AccList))
            infoStr.append('>maxDec: ' + ' '.join(DecList))
            infoStr.append('>maxSpeed: ' + ' '.join(SpeedList))
            infoStr.append('>ErrorLimit: ' + ' '.join(ErrorLimitsList))
            infoStr.append('>Jointspace: ' + ' '.join(JointPosition))
            infoStr.append('>JointspaceMin: ' + ' '.join(JointPositionMin))
            infoStr.append('>JointspaceMax: ' + ' '.join(JointPositionMax))

        rows = [line.strip().split(' ') for line in infoStr]
        # Reorganize data by columns
        cols = zip(*rows)

        # Compute column widths by taking maximum length of values per column
        col_widths = [max(len(value) for value in col) for col in cols]

        # Create a suitable format string
        rowFormat = '\t'.join(['%%-%ds' % width for width in col_widths])  # align left

        toprule = '-'*40 + ' ' + str(self._info[0]) + ' (' + str(self._info[1]) + ') ' + '-'*40
        bottomrule = '-'*len(toprule)
        print('\033[94m' + toprule)

        # Print each row using the computed format
        for row in rows:
            print(rowFormat % tuple(row))

        print(bottomrule + '\033[0m')

    def _parse_config_file(self, xmlpath):
        """Parses XML-File with configuration data

        :param xmlpath: Path to XML file
        """

        # parse xml-file
        tree = ET.parse(xmlpath)
        galil_conf = tree.getroot()

        self._info = GalilRobot.GalilInfo(galil_conf.attrib['configID'], galil_conf.attrib['ipAddr'])
        # Iterate over all channels
        for galilChan in galil_conf:
            robotAxisID = RobotAxisID[galilChan.attrib['RobotAxisID']]
            chanIdx = galilChan.attrib['ChannelID']

            self._dictRobotAxisIDToChan.update({robotAxisID: chanIdx})
            self._dictChanToRobotAxisID.update({chanIdx: robotAxisID})

            self._listUnits[robotAxisID] = galilChan.attrib['Unit']
            self._listUnitToTick[robotAxisID] = float(galilChan.attrib['UnitToTick'])

            # Get parameters for PID controller, max speed and max acceleration
            for param in galilChan:
                if param.tag == 'PIDParameter':
                    self._listKp[robotAxisID] = float(param.attrib['Kp'])
                    self._listKi[robotAxisID] = float(param.attrib['Ki'])
                    self._listKd[robotAxisID] = float(param.attrib['Kd'])
                elif param.tag == 'DynamicParameter':
                    self._listMaxSpeed[robotAxisID] = float(param.attrib['maxSpeed'])
                    self._listMaxAcc[robotAxisID] = float(param.attrib['maxAcc'])
                    self._listMaxDec[robotAxisID] = float(param.attrib['maxAcc'])

    def _send_cmd(self, cmd):
        """ Sends command to GalilController

        :param cmd: Command to send
        """
        return self._galilBoard.GCommand(cmd)

    # def setDictChanToIdx(self, dictChanToIdx):
    #     # _dictChanToIdx = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6, 'H': 7}
    #     for val in dictChanToIdx.itervalues():
    #         if not val >= 0 and not val < len(self._dictChanToIdx):
    #             print("\033[93m" +
    #                   "Warning in setDictChanToIdx(): Fail to set objects " + str(val) + " ." +
    #                   " \033[0m")
    #             return None
    #     for key in dictChanToIdx.iterkeys():
    #         if not key >= ord('A') and not val < ord('A') + len(self._dictChanToIdx):
    #             print("\033[93m" +
    #                   "Warning in setDictChanToIdx(): Fail to set key " + key + " ." +
    #                   " \033[0m")
    #             return None
    #
    #     self._dictChanToIdx = dictChanToIdx
    #     return None

    def connect(self):
        """Connects to the GalilBoard with the ipAdress from ConfigFile"""
        self._galilBoard.GClose()
        self._galilBoard.GOpen(self._info.ipAddr + ' --direct')
        self._isConnected = True

        # send default config to galilboards
        initCommands = [
            'KP ' + ','.join(map(str, self._listKp)),
            'KI ' + ','.join(map(str, self._listKi)),
            'KD ' + ','.join(map(str, self._listKd)),
            'AC ' + ','.join(map(str, self._listMaxAcc)),   # max acceleration
            'DC ' + ','.join(map(str, self._listMaxDec)),   # max deceleration
            'SP ' + ','.join(map(str, self._listMaxSpeed))  # max speed
        ]

        for cmd in initCommands:
            self._send_cmd(cmd)

        if self._info[0] == 'MiniTubuActu':
            # Additional init commands for MiniTubuActu
            self._send_cmd("BR")
            self._send_cmd("AG=0")
            self._send_cmd("AU*=0.5")

            self.setErrorLimit(3000)
        else:
            self._listKp = [6.] * self._numActuator   # controller gain: min=0, max=4095875, default= 6
            self._listKi = [0.] * self._numActuator   # controller gain: min=0, max= 255999, default= 0
            self._listKd = [64.] * self._numActuator  # controller gain: min=0, max=4095875, default=66
            self._listMaxSpeed = [25000.] * self._numActuator
            self._listMaxAcc = [256000.] * self._numActuator
            self._listMaxDec = [256000.] * self._numActuator
            self._listErrorLimit = [16384.] * self._numActuator

        print("\033[93m" +
              "Warning in connect(): joint constraints and collision detection are hard coded." +
              " \033[0m")

    def disconnect(self):
        """Disconnects from GalilBoard"""
        self.motorsOff()
        self._galilBoard.GClose()
        self._isConnected = False

    def motorsOn(self):
        self._send_cmd('SH ' + self._numActuatorStr)
        # PT  startet die Motoren direkt, dass will ich nicht.

    def motorsOff(self):
        time.sleep(0.1)
        self._send_cmd('ST')  # stop motors controlled
        time.sleep(0.1)
        self._send_cmd('MO')  # Motors off
        time.sleep(0.1)

    def stop(self):
        self._send_cmd('ST')  # stop motors controlled

    def stopEmergency(self):
        """Emergency stop"""
        self._send_cmd('AB')  # stop motors without controlled deceleration

    def setHome(self):
        # vorher setCurrPosToZero
        self._send_cmd('DP ' + ','.join(['0']*self._numActuator))

    def go2home(self):

        homePosition = np.zeros(self._numActuator)

        self.jointPTPLinearMotion(homePosition)

    def setMotorGain(self, gainType, gainValue, axisID=None):
        if str.upper(gainType) == 'KP':
            # controller gain: min=0, max=4095875, default=6
            if gainValue > 4095875.:
                print("\033[93m" +
                      "Warning in setMotorGain(): maximum value of Kp is reached." +
                      " \033[0m")
                gainValue = 4095875.
            elif gainValue < 0.:
                print("\033[93m" +
                      "Warning in setMotorGain(): minimum value of Kp is reached." +
                      " \033[0m")
                gainValue = 0.
            if axisID is not None:
                self._listKp[axisID] = gainValue
                self._send_cmd("KP" + self._dictRobotAxisIDToChan[axisID] + " " + str(gainValue))
            else:
                self._listKp = [gainValue] * len(self._listKp)
                self._send_cmd("KP " + ','.join(map(str, self._listKp)))
        elif str.upper(gainType) == 'KI':
            # controller gain: min=0, max=255999, default=0
            if gainValue > 255999.:
                print("\033[93m" +
                      "Warning in setMotorGain(): maximum value of Ki is reached." +
                      " \033[0m")
                gainValue = 255999.
            elif gainValue < 0.:
                print("\033[93m" +
                      "Warning in setMotorGain(): minimum value of Ki is reached." +
                      " \033[0m")
                gainValue = 0.
            if axisID is not None:
                self._listKi[axisID] = gainValue
                self._send_cmd("KI" + self._dictRobotAxisIDToChan[axisID] + " " + str(gainValue))
            else:
                self._listKi = [gainValue] * len(self._listKi)
                self._send_cmd("KI " + ','.join(map(str, self._listKi)))
        elif str.upper(gainType) == 'KD':
            # controller gain: min=0, max=4095875, default=66
            if gainValue > 4095875.:
                print("\033[93m" +
                      "Warning in setMotorGain(): maximum value of Kd is reached." +
                      " \033[0m")
                gainValue = 4095875.
            elif gainValue < 0.:
                print("\033[93m" +
                      "Warning in setMotorGain(): minimum value of Kd is reached." +
                      " \033[0m")
                gainValue = 0.
            if axisID is not None:
                self._listKd[axisID] = gainValue
                self._send_cmd("KD" + self._dictRobotAxisIDToChan[axisID] + " " + str(gainValue))
            else:
                self._listKd = [gainValue] * len(self._listKd)
                self._send_cmd("KD " + ','.join(map(str, self._listKd)))
        else:
            print("\033[93m" +
                  "Warning in setMotorGain(): Wrong controller gain type." +
                  " \033[0m")

    def getMotorGain(self, gainType, axisID=None):
        if str.upper(gainType) == 'KP':
            if axisID is not None:
                return self._listKp[axisID]
            else:
                return self._listKp
        elif str.upper(gainType) == 'KI':
            if axisID is not None:
                return self._listKi[axisID]
            else:
                return self._listKi
        elif str.upper(gainType) == 'KD':
            if axisID is not None:
                return self._listKd[axisID]
            else:
                return self._listKd

    def setUnitToTick(self, ratioValue, axisID=None):
        """


        :param ratioValue: float number
        :param axisID:
        :return:
        """
        if axisID is not None:
            self._listUnitToTick[axisID] = ratioValue
        else:
            self._listUnitToTick = [ratioValue] * len(self._listUnitToTick)

    def getUnitToTick(self, axisID=None):
        if axisID is not None:
            return self._listUnitToTick[axisID]
        else:
            return self._listUnitToTick

    def recoverMotorGain(self, axisID, gainType='all'):  #<----- Verbessern, da kein axisID=None

        if isinstance(axisID, int):
            axisID = [axisID]

        # parse xml-file
        tree = ET.parse(self._xmlpath)
        galil_conf = tree.getroot()

        self._info = GalilRobot.GalilInfo(galil_conf.attrib['configID'], galil_conf.attrib['ipAddr'])
        # Iterate over all channels
        for galilChan in galil_conf:
            pidparams = None
            for param in galilChan:
                if param.tag == 'PIDParameter':
                    pidparams = GalilRobot.PIDParameter(param.attrib['Kp'], param.attrib['Ki'], param.attrib['Kd'])

            self._dictRobotAxisIDToChan.update({RobotAxisID[galilChan.attrib['RobotAxisID']]: galilChan.attrib['ChannelID']})
            chanIdx = self._dictChanToIdx[galilChan.attrib['ChannelID']]  # get index in command string

            if axisID is not None:
                if chanIdx in axisID:
                    if str.upper(gainType) == 'KP' or gainType == 'all':
                        self._listKp[chanIdx] = float(pidparams.Kp)
                    if str.upper(gainType) == 'KI' or gainType == 'all':
                        self._listKi[chanIdx] = float(pidparams.Ki)
                    if str.upper(gainType) == 'KD' or gainType == 'all':
                        self._listKd[chanIdx] = float(pidparams.Kd)
            else:
                self._listKp[chanIdx] = float(pidparams.Kp)
                self._listKi[chanIdx] = float(pidparams.Ki)
                self._listKd[chanIdx] = float(pidparams.Kd)

    def setMotorConstraints(self, constraintType, constraintValue, axisID=None):  # <-------- Verbessern! Kein self._send_cmd
        """Motion Constraints are maximum (minimum) velocity and maximum (minimum) acceleration"""
        if str.upper(constraintType) == 'MAXSPEED':
            if axisID is not None:
                self._listMaxSpeed[axisID] = constraintValue
            else:
                self._listMaxSpeed = [constraintValue] * len(self._listMaxSpeed)
            # self._send_cmd('VS ' + ','.join(map(str, self._listMaxSpeed)))
        elif str.upper(constraintType) == 'MAXACC':
            if axisID is not None:
                self._listMaxAcc[axisID] = constraintValue
            else:
                self._listMaxAcc = [constraintValue] * len(self._listMaxAcc)
            # self._send_cmd('VA ' + ','.join(map(str, self._listMaxAcc)))
        elif str.upper(constraintType) == 'MAXDEC':
            if axisID is not None:
                self._listMaxDec[axisID] = constraintValue
            else:
                self._listMaxDec = [constraintValue] * len(self._listMaxDec)
            # self._send_cmd('VD ' + ','.join(map(str, self._listMaxDec)))
        else:
            print("\033[93m" +
                  "Warning in setMotorConstraints(): Wrong motion constraint type." +
                  " \033[0m")

    def getMotorConstraints(self, gainType, axisID=None):
        if str.upper(gainType) == 'MAXSPEED':
            if axisID is not None:
                return self._listMaxSpeed[axisID]
            else:
                return self._listMaxSpeed
        elif str.upper(gainType) == 'MAXACC':
            if axisID is not None:
                return self._listMaxAcc[axisID]
            else:
                return self._listMaxAcc

    def echoEnable(self, enable=True):
        if enable:
            self._send_cmd("EO 1")
        else:
            self._send_cmd("EO 0")
            print("\033[93m" +
                  "Warning in echoEnable(): echo is off. "
                  "If the echo is off, characters input over the bus will not be echoed back." +
                  " \033[0m")

    def setErrorLimit(self, limitValue, axisID=None):
        """
        The ER command sets the magnitude of the position errors for each axis that will trigger an error condition.
        When the limit is exceeded, the Error output will go low (true) and the controller's red light
        will be turned on. If the Off On Error (OE1) command is active, the motors will be disabled
        """
        if axisID is not None:
            self._listErrorLimit[axisID] = int(limitValue)
            self._send_cmd("ER" + self._dictRobotAxisIDToChan[axisID] + "= " + str(self._listErrorLimit[axisID]))
        else:
            self._listErrorLimit = [int(limitValue)] * len(self._listErrorLimit)
            self._send_cmd("ER " + ','.join(map(str, self._listErrorLimit)))

    # def _createActiveChanCmd(self, galilCmd):
    #     activeAxis = self._dictRobotAxisIDToChan.values()
    #     return galilCmd + ','.join(activeAxis)

    def jointPTPLinearMotion(self, axisRelativeDistances, sKurve=False, sKurveValue=0.5):  #S
        """
        Synchronized linear trajectory generator. It generates a trajectory in Joint space.

        Delta_q = [Delta_alpha1, Delta_beta1, Delta_alpha2, Delta_beta2, Delta_alpha3, Delta_beta3]

        :param axisRelativeDistances: Must contain values for all joints. Order according to order of enum RobotAxisID.
        :param sKurve:
        :param sKurveValue:
        :return:
        """

        self._send_cmd('LM ' + self._numActuatorStr)
        self._send_cmd('CS S')  # Clear Sequence on coordinate S

        for viaPointIdx in xrange(len(axisRelativeDistances)):
            viaPoint = axisRelativeDistances[viaPointIdx]
            viaPointTicks = [0] * self._numActuator
            for idx in xrange(self._numActuator):
                # viaPointTicks is sorted alphabetically (Galil-Channel via self._numActuatorStr[idx])
                viaPointTicks[idx] = viaPoint[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value] * \
                                     self._listUnitToTick[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value]
                viaPointTicks[idx] = np.round(viaPointTicks[idx])

            if not all(v == 0 for v in viaPointTicks):
                self._send_cmd('LI ' + ','.join(map(str, viaPointTicks)))

        self._send_cmd('LE')
        self._send_cmd('VA ' + ','.join(map(str, self._listMaxAcc)))
        self._send_cmd('VD ' + ','.join(map(str, self._listMaxDec)))
        self._send_cmd('VS ' + ','.join(map(str, self._listMaxSpeed)))
        self._send_cmd('BGS')  # Begin Motion in coordinate S
        if sKurve:
            self._send_cmd('IT ' + str(sKurveValue))

        # GMotionComplete instead of self._send_cmd('AM') which is not supported
        # self._galilBoard.GMotionComplete(self._numActuatorStr)
        self._galilBoard.GMotionComplete('S')

        self.updateJointPositions()

        # because of the exception and PEP 8.0 guideline
        return None

    def jointPTPLinearMotionSinglePoint(self, axisRelativeDistances, sKurve=False, sKurveValue=0.5):  #S
        """
        Synchronized linear trajectory generator. It generates a trajectory in Joint space.

        Delta_q = [Delta_alpha1, Delta_beta1, Delta_alpha2, Delta_beta2, Delta_alpha3, Delta_beta3]

        :param axisRelativeDistances: Must contain values for all joints. Order according to order of enum RobotAxisID.
        :param sKurve:
        :param sKurveValue:
        :return:
        """

        self._send_cmd('LM ' + self._numActuatorStr)
        self._send_cmd('CS S')  # Clear Sequence on coordinate S

        viaPoint = axisRelativeDistances
        viaPointTicks = [0] * self._numActuator
        for idx in xrange(self._numActuator):
            # viaPointTicks is sorted alphabetically (Galil-Channel via self._numActuatorStr[idx])
            viaPointTicks[idx] = viaPoint[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value] * \
                                 self._listUnitToTick[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value]
            viaPointTicks[idx] = np.round(viaPointTicks[idx])

        if not all(v == 0 for v in viaPointTicks):
            self._send_cmd('LI ' + ','.join(map(str, viaPointTicks)))

        self._send_cmd('LE')
        self._send_cmd('VA ' + ','.join(map(str, self._listMaxAcc)))
        self._send_cmd('VD ' + ','.join(map(str, self._listMaxDec)))
        self._send_cmd('VS ' + ','.join(map(str, self._listMaxSpeed)))
        #self._send_cmd('BG ' + self._numActuatorStr)
        #self._send_cmd('AM ' + self._numActuatorStr)
        #time.sleep(1)
        self._send_cmd('BGS')  # Begin Motion in coordinate S
        if sKurve:
            self._send_cmd('IT ' + str(sKurveValue))

        # GMotionComplete instead of self._send_cmd('AM') which is not supported
        # self._galilBoard.GMotionComplete(self._numActuatorStr)
        self._galilBoard.GMotionComplete('S')

        self.updateJointPositions()

        # because of the exception and PEP 8.0 guideline
        return None

    def updateJointPositions(self):
        self._send_cmd('PF 10.4')

        for chan, idx in self._dictChanToIdx.items():
            if chan in self._dictChanToRobotAxisID:
                self._listJointpositions[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value] = \
                        float(self._send_cmd('TP ' + chan)) / \
                        self._listUnitToTick[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value]

    def getJointPositions(self, getDesired=False):
        JointPosition = np.zeros(self._numActuator)

        if getDesired:
            for chan, idx in self._dictChanToIdx.items():
                if chan in self._dictChanToRobotAxisID:
                    JointPosition[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value] = \
                        self._listJointpositions[self._dictChanToRobotAxisID[chan]]
        else:
            self._send_cmd('PF 10.4')
            for chan, idx in self._dictChanToIdx.items():
                if chan in self._dictChanToRobotAxisID:
                    JointPosition[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value] = \
                        float(self._send_cmd('TP ' + chan)) / \
                        self._listUnitToTick[self._dictChanToRobotAxisID[self._numActuatorStr[idx]].value]

        return JointPosition

    def collisionCarrier(self, nextPosition):
        # ---------------------------------------- hard coded ----------------------------------------
        JointPosition = self.getJointPositions()

        # position in joint space np.array([a1, b1, a2, b2, a3, b3])

        beta1_new = nextPosition[3] + JointPosition[3]
        beta2_new = nextPosition[4] + JointPosition[4]
        beta3_new = nextPosition[5] + JointPosition[5]

        if beta1_new < beta2_new:
            return True
        elif beta2_new < beta3_new:
            return True
        else:
            return False

    def differentialJoint(self):
        pass
