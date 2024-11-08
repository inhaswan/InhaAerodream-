"""
Python implementation of SIYI SDK
"""
import socket
from siyi_message import *
from time import sleep, time
import logging
from utils import  toInt
import threading
from simple_pid import PID

class SIYISDK:
    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug= debug # print debug messages
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT=' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)
        
        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)        

        self._server_ip = server_ip
        self._port = port

        self._BUFF_SIZE=1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rcv_wait_t = 2 # Receiving wait time
        self._socket.settimeout(self._rcv_wait_t)

        self._connected = False

        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg=ManualZoomMsg()
        self._manualFocus_msg=ManualFocusMsg()
        self._gimbalSpeed_msg=GimbalSpeedMsg()
        self._gimbalRot_msg=GimbalRotateMsg()
        self._center_msg=CenterMsg()
        self._record_msg=RecordingMsg()
        self._mountDir_msg=MountDirMsg()
        self._motionMode_msg=MotionModeMsg()
        self._funcFeedback_msg=FuncFeedbackInfoMsg()
        self._att_msg=AttitdueMsg()
        self._last_att_seq=-1

        # Stop threads
        self._stop = False # used to stop the above thread
        
        self._recv_thread = threading.Thread(target=self.recvLoop)

        # Connection thread
        self._last_fw_seq=0 # used to check on connection liveness
        self._conn_loop_rate = 1 # seconds
        self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,))

        # Gimbal info thread @ 1Hz
        self._gimbal_info_loop_rate = 1
        self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop,
                                                args=(self._gimbal_info_loop_rate,))

        # Gimbal attitude thread @ 10Hz
        self._gimbal_att_loop_rate = 0.1
        self._g_att_thread = threading.Thread(target=self.gimbalAttLoop,
                                                args=(self._gimbal_att_loop_rate,))
        
        self._pid_angle_thread = threading.Thread(target=self.PID_contol,
                                                args=())
        # PID 
        self.yaw_pid = PID(1.3, 0.1, 0.1, setpoint=0)
        self.yaw_pid.output_limits = (-100, 100)

        self.pitch_pid = PID(1.3, 0.1, 0.1, setpoint=0)
        self.pitch_pid.output_limits = (-100, 100)
        self._goal_yaw = 0.
        self._goal_pitch = 0.
        self.is_pause_pid = False

    def resetVars(self):
        """
        Resets variables to their initial values. For example, to prepare for a fresh connection
        """
        self._connected = False
        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg=ManualZoomMsg()
        self._manualFocus_msg=ManualFocusMsg()
        self._gimbalSpeed_msg=GimbalSpeedMsg()
        self._gimbalRot_msg=GimbalRotateMsg()

        self._center_msg=CenterMsg()
        self._record_msg=RecordingMsg()
        self._mountDir_msg=MountDirMsg()
        self._motionMode_msg=MotionModeMsg()
        self._funcFeedback_msg=FuncFeedbackInfoMsg()
        self._att_msg=AttitdueMsg()


        return True

    def connect(self, maxWaitTime=3.0):
        """
        Makes sure there is conenction with the camera before doing anything.
        It requests Frimware version for some time before it gives up

        Params
        --
        maxWaitTime [int] Maximum time to wait before giving up on connection
        """
        self._recv_thread.start()
        self._conn_thread.start()
        t0 = time()
        while(True):
            if self._connected:
                self._g_info_thread.start()
                self._g_att_thread.start()
                self._pid_angle_thread.start()
                return True
            if (time() - t0)>maxWaitTime and not self._connected:
                self.disconnect()
                self._logger.error("Failed to connect to camera")
                return False

    def disconnect(self):
        self._logger.info("Stopping all threads")
        self._stop = True # stop the connection checking thread
        self.resetVars()
        self._socket.close()


    def checkConnection(self):
        """
        checks if there is live connection to the camera by requesting the Firmware version.
        This function is to be run in a thread at a defined frequency
        """
        self._connected = True

        # self.requestFirmwareVersion()
        # sleep(1)
        # if self._fw_msg.seq!=self._last_fw_seq and len(self._fw_msg.gimbal_firmware_ver)>0:
        #     self._connected = True
        #     self._last_fw_seq=self._fw_msg.seq
        # else:
        #     self._connected = False

    def connectionLoop(self, t):
        """
        This function is used in a thread to check connection status periodically

        Params
        --
        t [float] message frequency, secnod(s)
        """
        while(True):
            if self._stop:
                self._connected=False
                self.resetVars()
                self._logger.warning("Connection checking loop is stopped. Check your connection!")
                break
            self.checkConnection()
            sleep(t)

    def isConnected(self):
        return self._connected

    def gimbalInfoLoop(self, t):
        """
        This function is used in a thread to get gimbal info periodically

        Params
        --
        t [float] message frequency, secnod(s) 
        """
        while(True):
            if not self._connected:
                self._logger.warning("Gimbal info thread is stopped. Check connection")
                sleep(1)
                break
            self.requestGimbalInfo()
            sleep(t)

    def gimbalAttLoop(self, t):
        """
        This function is used in a thread to get gimbal attitude periodically

        Params
        --
        t [float] message frequency, secnod(s) 
        """
        while(True):
            if not self._connected:
                self._logger.warning("Gimbal attitude thread is stopped. Check connection")
                break
            self.requestGimbalAttitude()
            sleep(t)

    def sendMsg(self, msg):
        """
        Sends a message to the camera

        Params
        --
        msg [str] Message to send
        """
        b = bytes.fromhex(msg)
        try:
            self._socket.sendto(b, (self._server_ip, self._port))
            return True
        except Exception as e:
            self._logger.error("Could not send bytes")
            return False

    def rcvMsg(self):
        data=None
        try:
            data,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            self._logger.warning("%s. Did not receive message within %s second(s)", e, self._rcv_wait_t)
        return data

    def recvLoop(self):
        self._logger.debug("Started data receiving thread")
        while( not self._stop):
            self.bufferCallback()
        self._logger.debug("Exiting data receiving thread")

    
    def bufferCallback(self):
        """
        Receives messages and parses its content
        """
        buff,addr = self._socket.recvfrom(self._BUFF_SIZE)

        buff_str = buff.hex()
        self._logger.debug("Buffer: %s", buff_str)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2

        HEADER='5566'
        # Go through the buffer
        while(len(buff_str)>=MINIMUM_DATA_LENGTH):
            if buff_str[0:4]!=HEADER:
                # Remove the 1st element and continue 
                tmp=buff_str[1:]
                buff_str=tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            # Data length, bytes are reversed, according to SIYI SDK
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len = high_b+low_b
            data_len = int('0x'+data_len, base=16)
            char_len = data_len*2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH+char_len)):
                # No useful data
                buff_str=''
                break
            
            packet = buff_str[0:MINIMUM_DATA_LENGTH+char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH+char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]

            if cmd_id==COMMAND.ACQUIRE_FW_VER:
                self.parseFirmwareMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_HW_ID:
                self.parseHardwareIDMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_INFO:
                self.parseGimbalInfoMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_ATT:
                self.parseAttitudeMsg(data, seq)
            elif cmd_id==COMMAND.FUNC_FEEDBACK_INFO:
                self.parseFunctionFeedbackMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_SPEED:
                self.parseGimbalSpeedMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_ROT:
                self.parseGimbalRotateMsg(data, seq)
            elif cmd_id==COMMAND.AUTO_FOCUS:
                self.parseAutoFocusMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_FOCUS:
                self.parseManualFocusMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_ZOOM:
                self.parseZoomMsg(data, seq)
            elif cmd_id==COMMAND.CENTER:
                self.parseGimbalCenterMsg(data, seq)
            else:
                self._logger.warning("CMD ID is not recognized")
        
        return
    
    ##################################################
    #               Request functions                #
    ##################################################    
    def requestFirmwareVersion(self):
        """
        Sends request for firmware version

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.firmwareVerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestHardwareID(self):
        """
        Sends request for Hardware ID

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.hwIdMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalAttitude(self):
        """
        Sends request for gimbal attitude

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalAttMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalInfo(self):
        """
        Sends request for gimbal information

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalInfoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFunctionFeedback(self):
        """
        Sends request for function feedback msg

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.funcFeedbackMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestAutoFocus(self):
        """
        Sends request for auto focus

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.autoFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomIn(self):
        """
        Sends request for zoom in

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomInMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomOut(self):
        """
        Sends request for zoom out

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomOutMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomHold(self):
        """
        Sends request for stopping zoom

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopZoomMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestLongFocus(self):
        """
        Sends request for manual focus, long shot

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.longFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestCloseFocus(self):
        """
        Sends request for manual focus, close shot

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.closeFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFocusHold(self):
        """
        Sends request for manual focus, stop

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestCenterGimbal(self):
        """
        Sends request for gimbal centering

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.centerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalSpeed(self, yaw_speed:int, pitch_speed:int):
        """
        Sends request for gimbal centering

        Params
        --
        yaw_speed [int] -100~0~100. away from zero -> fast, close to zero -> slow. Sign is for direction
        pitch_speed [int] Same as yaw_speed
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestGimbalRot(self, yaw_angle, pitch_angle):
        """
        Sends request for gimbal rotation

        Params
        --        
        Returns
        --
        """
        print("set yaw, pitch", yaw_angle, pitch_angle)
        msg = self._out_msg.gimbalRotateMsg(yaw_angle, pitch_angle)
        if not self.sendMsg(msg):
            return False
        return True
    
    

    def requestPhoto(self):
        """
        Sends request for taking photo
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.takePhotoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestRecording(self):
        """
        Sends request for toglling video recording
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.recordMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFPVMode(self):
        """
        Sends request for setting FPV mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.fpvModeMsg()
        self.is_pause_pid = True

        if not self.sendMsg(msg):
            return False
        return True

    def requestLockMode(self):
        """
        Sends request for setting Lock mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.lockModeMsg()
        self.is_pause_pid = True

        if not self.sendMsg(msg):
            return False
        return True

    def requestFollowMode(self):
        """
        Sends request for setting Follow mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.followModeMsg()
        self.is_pause_pid = True
        if not self.sendMsg(msg):
            return False
        return True

    ####################################################
    #                Parsing functions                 #
    ####################################################
    def parseFirmwareMsg(self, msg:str, seq:int):
        try:
            self._fw_msg.gimbal_firmware_ver= msg[8:16]
            self._fw_msg.seq=seq
            
            self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseHardwareIDMsg(self, msg:str, seq:int):
        try:
            self._hw_msg.seq=seq
            self._hw_msg.id = msg
            self._logger.debug("Hardware ID: %s", self._hw_msg.id)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAttitudeMsg(self, msg:str, seq:int):
        
        try:
            self._att_msg.seq=seq
            self._att_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._att_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._att_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
            self._att_msg.yaw_speed = toInt(msg[14:16]+msg[12:14]) /10.
            self._att_msg.pitch_speed = toInt(msg[18:20]+msg[16:18]) /10.
            self._att_msg.roll_speed = toInt(msg[22:24]+msg[20:22]) /10.

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)", 
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            self._logger.debug("(yaw_speed, pitch_speed, roll_speed= (%s, %s, %s)", 
                                    self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalInfoMsg(self, msg:str, seq:int):
        try:
            self._record_msg.seq=seq
            self._mountDir_msg.seq=seq
            self._motionMode_msg.seq=seq
            
            self._record_msg.state = int('0x'+msg[6:8], base=16)
            self._motionMode_msg.mode = int('0x'+msg[8:10], base=16)
            self._mountDir_msg.dir = int('0x'+msg[10:12], base=16)

            self._logger.debug("Recording state %s", self._record_msg.state)
            self._logger.debug("Mounting direction %s", self._mountDir_msg.dir)
            self._logger.debug("Gimbal motion mode %s", self._motionMode_msg.mode)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAutoFocusMsg(self, msg:str, seq:int):
        
        try:
            self._autoFocus_msg.seq=seq
            self._autoFocus_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Auto focus success: %s", self._autoFocus_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseZoomMsg(self, msg:str, seq:int):
        
        try:
            self._manualZoom_msg.seq=seq
            self._manualZoom_msg.level = int('0x'+msg[2:4]+msg[0:2], base=16) /10.

            
            self._logger.debug("Zoom level %s", self._manualZoom_msg.level)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseManualFocusMsg(self, msg:str, seq:int):
        
        try:
            self._manualFocus_msg.seq=seq
            self._manualFocus_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Manual  focus success: %s", self._manualFocus_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg:str, seq:int):
        
        try:
            self._gimbalSpeed_msg.seq=seq
            self._gimbalSpeed_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalRotateMsg(self, msg:str, seq:int):
        pass

        try:
            self._gimbalRot_msg.seq=seq

            self._gimbalRot_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._gimbalRot_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._gimbalRot_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.

            self._logger.debug("Gimbal rotate success: %s", self._gimbalRot_msg)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False


    def parseGimbalCenterMsg(self, msg:str, seq:int):
        
        try:
            self._center_msg.seq=seq
            self._center_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Gimbal center success: %s", self._center_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseFunctionFeedbackMsg(self, msg:str, seq:int):
        
        try:
            self._funcFeedback_msg.seq=seq
            self._funcFeedback_msg.info_type = int('0x'+msg, base=16)

            
            self._logger.debug("Function Feedback Code: %s", self._funcFeedback_msg.info_type)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    ##################################################
    #                   Get functions                #
    ##################################################
    def getAttitude(self):
        return(self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)

    def getAttitudeSpeed(self):
        return(self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)

    def getFirmwareVersion(self):
        return(self._fw_msg.gimbal_firmware_ver)

    def getHardwareID(self):
        return(self._hw_msg.id)

    def getRecordingState(self):
        return(self._record_msg.state)

    def getMotionMode(self):
        return(self._motionMode_msg.mode)

    def getMountingDirection(self):
        return(self._mountDir_msg.dir)

    def getFunctionFeedback(self):
        return(self._funcFeedback_msg.info_type)

    def getZoomLevel(self):
        return(self._manualZoom_msg.level)

    #################################################
    #                 Set functions                 #
    #################################################

    def set_zoom(self, level):
        # Set zoom min-max: 1-30x
        
        if level > 30:
            level = 30
        elif level < 1:
            level = 1

        self.requestZoomHold()
        val = self.getZoomLevel()

        print("Init Zoom level: ", val, " goal:", level)

        if val > level:
            while val > level:
                    self.requestZoomOut()
                    sleep(0.1)
                    val = self.getZoomLevel()
                    print("Zoom level: ", val)
            self.requestZoomHold()
            return True
        else:    
            while val < level:
                    self.requestZoomIn()
                    sleep(0.1)
                    val = self.getZoomLevel()
                    print("Zoom level: ", val)
            self.requestZoomHold()
            return True


    def PID_contol(self):
        start_time = time()
        last_time = start_time

        
        while True:
            if not self._connected:
                self._logger.warning("Gimbal PID thread is stopped. Check connection")
                break
            current_time = time()
            dt = current_time - last_time
            # get error
            self.requestGimbalAttitude()
            if self._att_msg.seq==self._last_att_seq:
                # self._logger.info("Did not get new attitude msg")
                self.requestGimbalSpeed(0,0)
                continue
            if self._motionMode_msg.mode != MotionModeMsg.FOLLOW:
                print("PID on pause")
                sleep(1)
                continue
            else:
                self.is_pause_pid = False

            if self.is_pause_pid:
                print("Pause PID")
                continue

            self._last_att_seq = self._att_msg.seq

            yaw_err = -self._goal_yaw + self._att_msg.yaw # NOTE for some reason it's reversed!!
            pitch_err = self._att_msg.pitch - self._goal_pitch
            # print("yaw_err=", yaw_err, "pitch_err=", pitch_err)
            # print("goal_pitch= %s", self._goal_pitch)
            # print("Attitude (yaw,pitch,roll) eg:", self.getAttitude())
            if abs(yaw_err) > 0.5:
                yaw_output = self.yaw_pid(yaw_err)
            else:
                yaw_output = self.yaw_pid(0)
            if abs(pitch_err) > 0.5:
                pitch_output = self.pitch_pid(pitch_err)
            else:
                pitch_output = self.pitch_pid(0)
            self.requestGimbalSpeed(int(-yaw_output), int(pitch_output))
            sleep(0.05)

    def setYawAngle(self, value):
        self._goal_yaw = value

    def setPitchAngle(self, value):
        self._goal_pitch = value
    def setRotation(self,_yaw,_pitch):
        if _yaw > 170:
            _yaw = 170
        if _yaw < -170:
            _yaw = -170
        if _pitch > 20:
            _pitch = 20
        if _pitch < -90:
            _pitch = 90

        self._goal_yaw = _yaw
        self._goal_pitch = _pitch
        # self.requestGimbalRot(int(self._goal_yaw), int(self._goal_pitch))
