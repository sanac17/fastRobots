from enum import Enum

class CMD(Enum):
    # PING = 0
    # SEND_TWO_INTS = 1
    # SEND_THREE_FLOATS = 2
    # ECHO = 3
    # DANCE = 4
    # SET_VEL = 5
    # GET_TIME_MILLIS = 6
    # GET_TIME_LOOP = 7
    # SEND_TIME_DATA = 8
    # GET_TEMP_READINGS = 9
    # PERFORMANCE = 10 
    # GET_PITCH_DATA = 11
    # GET_ROLL_DATA = 12
    # GET_YAW_G_DATA = 13
    # GET_ROLL_G_DATA = 14
    # GET_PITCH_G_DATA = 15
    # SEND_ALL_DATA = 16
    START_CAR = 0
    START_PID = 1
    STOP_PID = 2
    CHANGE_GAIN = 3
    GET_PID_DATA = 4
    GET_PITCH_DATA = 5
    GET_ROLL_DATA = 6
    GET_YAW_DATA = 7
    GET_IMU_DATA = 8
    GET_TOF_DATA = 9
    START_PID_ORI = 10 
    STOP_PID_ORI = 11 
    START_MAP = 12
    GET_PID_DATA_ORI = 13
    FIND_STEADY_STATE = 14 
    GET_KF_DATA = 15
    STUNT = 16
    NAVIGATION = 17
    
    