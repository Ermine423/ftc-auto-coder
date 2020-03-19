import linecache

def read(r, fType):
    with open(str(r)+"."+str(fType), 'r') as reader:
        read = reader.read().splitlines()
    return read
def write(w, inList, fType):
    with open(str(w)+"."+str(fType), "wt") as writer:
        for rep in inList:
            writer.write(str(rep) + "\n")
            
    return "Written to " + str(w)+"."+str(fType) + "."
test_list = [""]
def idInit(fileId):
    global test_list
    test_list = read(str(fileId), "java")
def replacer2(subs1, subs2):
    global test_list
    kill = [i for i in test_list if subs1 in i]
    i = 0
    while i <= len(kill)-1:
        kill[i] = kill[i].replace(subs1, "")
        kill[i] = kill[i].replace(subs2, "")
        i += 1
    i = 0
    return kill
def replacer3(subs1, subs2, varSub, t1, t2):
    global test_list
    kill = [i for i in test_list if subs1 in i]
    i = 0
    while i <= len(kill)-1:
        kill[i] = kill[i].replace(subs1, "")
        kill[i] = kill[i].replace(subs2, "")
        if kill[i] == str(globals()[varSub + str(i+1)] + str(t1)):
            kill[i] = str(t1)
        elif kill[i] == str(globals()[varSub + str(i+1)] + str(t2)):
            kill[i] = str(t2)
        i += 1
    i = 0
    return kill
def NSreplacer3(subs1, subs2, varSub):
    global test_list
    kill = [i for i in test_list if subs1 in i]
    i = 0
    while i <= len(kill)-1:
        kill[i] = kill[i].replace(subs1, "")
        kill[i] = kill[i].replace(subs2, "")
        kill[i] = kill[i].replace(str(globals()[varSub + str(i+1)]), "")
        i += 1
    i = 0
    return kill
def Vreplacer2(subs1, subs2, l):
    global test_list
    kill = [i for i in l if subs1 in i]
    i = 0
    while i <= len(kill)-1:
        kill[i] = kill[i].replace(subs1, "")
        kill[i] = kill[i].replace(subs2, "")
        i += 1
    i = 0
    return kill
def assign(varName, source):
    i = 1
    while i <= len(source):
        globals()[varName + str(i)] = source[i-1]
        print(varName + str(i) + " = " + str(globals()[varName + str(i)]))
        i += 1
    return i
#WHAT ACTUALLY IS RUNNING
def motoId():
    motoAmount = assign("motorName", replacer2("public DcMotor ", " = null;"))
    assign("motorDirect", replacer3(".setDirection(DcMotor.Direction.", ");", "motorName", "FORWARD", "REVERSE"))
    assign("motorEncode", replacer3(".setMode(DcMotor.RunMode.", ");", "motorName", "RUN_USING_ENCODER", "RUN_WITHOUT_ENCODER"))
    motoAmount -= 1
    return motoAmount
def servoId():
    servoAmount = assign("servoName", replacer2("public Servo ", " = null;"))
    assign("servoPos", NSreplacer3(".setPosition(", ");", "servoName"))
    servoAmount -= 1
    return servoAmount
def crservoId():
    crservoAmount = assign("crservoName", replacer2("public CRServo ", " = null;"))
    assign("crservoDirect", replacer3(".setDirection(CRServo.Direction.", ");", "crservoName", "FORWARD", "REVERSE"))
    crservoAmount -= 1
    return crservoAmount
def intgyroId():
    intgyroAmount = assign("intgyroName", replacer2("public IntegratingGyroscope ", " = null;"))
    intgyroAmount -= 1
    return intgyroAmount
def mrgyroId():
    mrgyroAmount = assign("mrgyroName", replacer2("public ModernRoboticsI2cGyro ", " = null;"))
    mrgyroAmount -= 1
    return mrgyroAmount
def mrrangeId():
    mrrangeAmount = assign("mrrangeName", replacer2("public ModernRoboticsI2cRangeSensor ", " = null;"))
    mrrangeAmount -= 1
    return mrrangeAmount
def distId():
    distAmount = assign("distName", replacer2("public DistanceSensor ", " = null;"))
    distAmount -= 1
    return distAmount
def touchId():
    touchAmount = assign("touchName", replacer2("public TouchSensor ", " = null;"))
    touchAmount -= 1
    return touchAmount
def colorId():
    colorAmount = assign("colorName", replacer2("public ColorSensor ", " = null;"))
    colorAmount -= 1
    return colorAmount
def odsId():
    odsAmount = assign("odsName", replacer2("public OpticalDistanceSensor ", " = null;"))
    odsAmount -= 1
    return odsAmount
def imuId():
    imuAmount = assign("imuName", replacer2("public BNO055IMU ", " = null;"))
    imuAmount -= 1
    return imuAmount
def compassId():
    compassAmount = assign("compassName", replacer2("public ModernRoboticsI2cCompassSensor ", " = null;"))
    compassAmount -= 1
    return compassAmount
def sensorId():
    sensorAmount = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    i = 0
    sensorAmount[i] = intgyroId()
    i += 1
    sensorAmount[i] = mrgyroId()
    i += 1
    sensorAmount[i] = mrrangeId()
    i += 1
    sensorAmount[i] = distId()
    i += 1
    sensorAmount[i] = touchId()
    i += 1
    sensorAmount[i] = colorId()
    i += 1
    sensorAmount[i] = odsId()
    i += 1
    sensorAmount[i] = imuId()
    i += 1
    sensorAmount[i] = compassId()
    return sensorAmount
