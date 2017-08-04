# coding=utf-8

ADDR_PETRONE5815 = '50:8c:b1:5f:16:b7'
CAM_RTSP_URL = 'rtsp://192.168.100.1:554/cam1/mpeg4/'


class PETRONE_SERVICE:
    DRONE_SERVICE = 'C320DF00-7891-11E5-8BCF-FEFF819CDC9F'


class PETRONE_CHARACTERISTIC:
    DRONE_DATA = 'C320DF01-7891-11E5-8BCF-FEFF819CDC9F'
    DRONE_CONF = 'C320DF02-7891-11E5-8BCF-FEFF819CDC9F'


class PETRONE_DATATYPE:
    DNone = 0x0 # 시스템 정보
    Ping = 0x1  # 통신 확인(reserve)
    Ack = 0x2
    Error = 0x3
    Request = 0x4
    Passcode = 0x5

    Control = 0x10
    Command = 0x11
    Command2 = 0x12
    Command3 = 0x13

    LedMode = 0x20
    LedMode2 = 0x21
    LedModeCommand = 0x22

    # ...
    State = 0x31
    Attitude = 0x32
    GyroBias = 0x33
    TrimAll = 0x34
    TrimFlight = 0x35
    TrimDrive = 0x36

    IrMessage = 0x40

    ImuRawAndAngle = 0x50
    Pressure = 0x51
    ImageFlow = 0x52
    Battery = 0x54
    Motor = 0x55
    Temperature = 0x56
    Range = 0x57


class PETRONE_COMMAND:
    CNone = 0x0
    ModePetrone = 0x10  # 동작 모드 전환
    Coordinate = 0x20   # 방위 기준 변경
    Trim = 0x21         # 트림 변경
    FlightEvent = 0x22  # 비행 이벤트 실행
    Stop = 0x24         # 정지
    ResetHeading = 0x50
    ClearGyroBiasAndTrim = 0x51

    PairingActivate = 0x80
    PairingDeactivate = 0x81
    TerminateConnection = 0x82

    Request = 0x90


class PETRONE_FLIGHT_EVENT:
    FNone = 0
    Ready = 1
    TakeOff = 2
    Flight = 3
    Flip = 4
    Stop = 5
    Landing = 6
    Reverse = 7
    Accident = 8
    Error = 9


class PETRONE_LED_MODE:
    LNone = 0x0

    EyeNone = 0x10
    EyeHold = 0x11
    EyeMix = 0x12
    EyeFlicker = 0x13
    EyeFlickerDouble = 0x14
    EyeDimming = 0x15

    ArmNone = 0x40
    ArmHold = 0x41
    ArmMix = 0x42
    ArmFlicker = 0x43
    ArmFlickerDouble = 0x44
    ArmFlickerDimming = 0x45

    ArmFlow = 0x46
    ArmFlowReverse = 0x47


class PETRONE_MODE:
    MNone = 0
    Flight = 0x10
    FlightNoGuard = 0x11
    FlightFPV = 0x12


class PETRONE_TRIMTYPE:
    RollIncrease = 1
    ThrottleIncrease = 7
    ThrottleDecrease = 8


class PETRONE_LED_COLOR:
    c = '''AliceBlue,AntiqueWhite,Aqua,Aquamarine,Azure,Beige,Bisque,Black,BlanchedAlmond,Blue,BlueViolet,
    'Brown,BurlyWood,CadetBlue,Chartreuse,Chocolate,Coral,CornflowerBlue,Cornsilk,Crimson,Cyan,DarkBlue,
    'DarkCyan,DarkGoldenRod,DarkGray,DarkGreen,DarkKhaki,DarkMagenta,DarkOliveGreen,DarkOrange,DarkOrchid,
    'DarkRed,DarkSalmon,DarkSeaGreen,DarkSlateBlue,DarkSlateGray,DarkTurquoise,DarkViolet,DeepPink,
    'DeepSkyBlue,DimGray,DodgerBlue,FireBrick,FloralWhite,ForestGreen,Fuchsia,Gainsboro,GhostWhite,
    'Gold,GoldenRod,Gray,Green,GreenYellow,HoneyDew,HotPink,IndianRed,Indigo,Ivory,Khaki,Lavender,
    'LavenderBlush,LawnGreen,LemonChiffon,LightBlue,LightCoral,LightCyan,LightGoldenRodYellow,LightGray,
    'LightGreen,LightPink,LightSalmon,LightSeaGreen,LightSkyBlue,LightSlateGray,LightSteelBlue,LightYellow,
    'Lime,LimeGreen,Linen,Magenta,Maroon,MediumAquaMarine,MediumBlue,MediumOrchid,MediumPurple,
    'MediumSeaGreen,MediumSlateBlue,MediumSpringGreen,MediumTurquoise,MediumVioletRed,MidnightBlue,
    'MintCream,MistyRose,Moccasin,NavajoWhite,Navy,OldLace,Olive,OliveDrab,Orange,OrangeRed,Orchid,
    'PaleGoldenRod,PaleGreen,PaleTurquoise,PaleVioletRed,PapayaWhip,PeachPuff,Peru,Pink,Plum,PowderBlue,
    'Purple,RebeccaPurple,Red,RosyBrown,RoyalBlue,SaddleBrown,Salmon,SandyBrown,SeaGreen,SeaShell,Sienna,
    'Silver,SkyBlue,SlateBlue,SlateGray,Snow,SpringGreen,SteelBlue,Tan,Teal,Thistle,Tomato,Turquoise,
    'Violet,Wheat,White,WhiteSmoke,Yellow,YellowGreen'''.split(',')

    @staticmethod
    def get_value(color_name):
        return PETRONE_LED_COLOR.c.index(color_name)


