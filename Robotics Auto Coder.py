#LOG:
# V.1: Created a hardware creator for either 2 or 4 drive motors, with custom names.
# V.1.1: Created either two or four other motors on bot. Need to add customizable numbers instead of just 2 or 4.
# V.1.2: Allowed odd numbers of drive and unique motors.
##
# V.2: Allowed both Continuous Rotation and 180-degree servos. Allowed for having 0 of motors or servos, instead of creating a motor named "0".
##
# V.3: Added all kinds of sensors that I know how to program.
#   (IntegratingGyroscope, ModernRoboticsI2cGyro, ModernRoboticsI2cRangeSensor, DistanceSensor, TouchSensor, ColorSensor,
#     ModernRoboticsOpticalDistanceSensor, BNO055IMU, ModernRoboticsI2cCompassSensor)
# V.3.1: Solved for errors (program does not crash), added imports for sensors. Changed file naming conventions, and tested within the Android Studio app until working properly.
# V.4: Now can read hardware files. This will be the base of the test, autonomous, and teleop files.
import linecache
import RAC_Identifier
from RAC_Identifier import read as read
from RAC_Identifier import motoId as motoId
from RAC_Identifier import idInit as idInit
from RAC_Identifier import servoId as servoId
from RAC_Identifier import crservoId as crservoId
from RAC_Identifier import sensorId as sensorId
from time import sleep as sleep
fStyle = 0
motoType = 0
sampList = [0]
driveNames = [0]
normNames = [0]
servoNames = [0]
sensorNames = [0]
def write(w, inList, fType):
    with open(str(w)+"."+str(fType), "wt") as writer:
        for rep in inList:
            writer.write(str(rep) + "\n")
            
    return "Written to " + str(w)+"."+str(fType) + "."
def listSetup(lines):
    global sampList
    sampList = [""]
    iterate = 0
    #Iterates for "lines-1" times, but has sampList[0] equal to 0 already. Thus, lines is equal to the amount of lines in the final product.
    while iterate < int(lines - 1):
        sampList.append("")
        iterate += 1
def setLine(num, txt):
    global sampList
    sampList[num - 1] = str(txt)
def appendLine(num, txt):
    global sampList
    sampList[num - 1] += str(txt)
tName = input("Enter your team number.")
#Ask if a hardware file, a test file, or an autonomous file.
listSetup(15)
#15 Sections of code.
while True:
    try:
        print("Press 0 for hardware file, 1 for test, 2 for autonomous, or 3 for teleop.")
        fStyle = int(input("Is this a hardware file, test file, autonomous file, or teleop file?"))
        if fStyle < 0 or fStyle > 3:
            print("Invalid answer. Try again.")
        else:
            break
    except ValueError:
        print("Invalid answer. Try again.")
fileName = "Sharp" + str(tName)
if fStyle == 0:
    fileName = fileName + "Hardware"
    setLine(1, " package org.firstinspires.ftc.teamcode; \n import com.qualcomm.robotcore.hardware.CRServo; \n import com.qualcomm.robotcore.hardware.DcMotor; \n import com.qualcomm.robotcore.hardware.DcMotorSimple; \n import com.qualcomm.robotcore.hardware.HardwareMap; \n import com.qualcomm.robotcore.hardware.Servo; \n import com.qualcomm.robotcore.util.ElapsedTime;")
    appendLine(1, "\nimport com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;")
    appendLine(1, "\nimport com.qualcomm.robotcore.hardware.IntegratingGyroscope;")
    appendLine(1, "\nimport com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;")
    appendLine(1, "\nimport com.qualcomm.robotcore.hardware.DistanceSensor;")
    appendLine(1, "\nimport com.qualcomm.robotcore.hardware.TouchSensor;")
    appendLine(1, "\nimport com.qualcomm.robotcore.hardware.ColorSensor;")
    appendLine(1, "\nimport com.qualcomm.robotcore.hardware.OpticalDistanceSensor;")
    appendLine(1, "\nimport com.qualcomm.hardware.bosch.BNO055IMU;")
    appendLine(1, "\nimport com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;")
    appendLine(1, "\nimport com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;")
    setLine(2, "//Code created by Jackson O'Connor of Team 8411 SHARP, via the Auto Coder.")
    setLine(3, "public class " + fileName +" {")
    setLine(8, "HardwareMap hwMap = null;\nprivate ElapsedTime period = new ElapsedTime();")
    setLine(9, "public " + fileName + "(){\n\n}")
    setLine(10, "public void init(HardwareMap ahwMap){\nhwMap = ahwMap;")
    setLine(15, "}\npublic void waitForTick(long periodMs){\n\nlong remaining = periodMs - (long)period.milliseconds();\nif(remaining > 0) { \n try { \n Thread.sleep(remaining); \n }catch (InterruptedException e) { \n Thread.currentThread().interrupt();\n}\n}\nperiod.reset();\n}\n}")
    ##DRIVE MOTORS
    while True:
        try:
            dMotoType = int(input("How many motors are in your drive train?"))
            break
        except ValueError:
            print("Invalid answer. Try again.")
    if dMotoType != 0:
        iterate = 1
        while iterate <= dMotoType:
            while True:
                try:
                    dMotoName = str(input("What is the name of drive motor " + str(iterate) + "?"))
                    if dMotoName != "":
                        break
                    else:
                        print("Invalid answer. Try again.")
                except ValueError:
                    print("Invalid answer. Try again.")
            if iterate == 1:
                driveNames[0] = dMotoName
            else:
                driveNames.append(dMotoName)
            iterate += 1
            
        iterate = 0
        while iterate < len(driveNames):
            print(driveNames[iterate])
            while True:
                try:
                    globals()["dMotor" + str(iterate + 1)] = [driveNames[iterate], int(input("Is this motor forward or reverse? Enter \"1\" for forward, or \"0\" for reverse.")), int(input("Does this motor use an encoder? Enter \"0\" for no encoder, or \"1\" for an encoder."))]
                    break
                except ValueError:
                    print("Invalid answer. Try again.")
            if globals()["dMotor" + str(iterate + 1)][2] == 0:
                globals()["dMotor" + str(iterate + 1)][2] = "RUN_WITHOUT_ENCODER"
            else:
                globals()["dMotor" + str(iterate + 1)][2] = "RUN_USING_ENCODER"
            if globals()["dMotor" + str(iterate + 1)][1] == 0:
                globals()["dMotor" + str(iterate + 1)][1] = "REVERSE"
            else:
                globals()["dMotor" + str(iterate + 1)][1] = "FORWARD"
            iterate += 1
        lnSpliceS4 = "//S4\n"
        lnSpliceS11NAME = "\n//NAMES\n"
        lnSpliceS11DIRECT = "\n//DIRECTIONS\n"
        lnSpliceS11PWR = "\n//POWER\n"
        lnSpliceS11ENCODE = "\n//ENCODERS\n"
        iterate = 0
        while iterate < dMotoType:
            lnSpliceS4 += "\npublic DcMotor " + globals()["dMotor" + str(iterate + 1)][0] + " = null;"
            lnSpliceS11NAME += "\n" + globals()["dMotor" + str(iterate + 1)][0] + " = hwMap.dcMotor.get(\"" + globals()["dMotor" + str(iterate + 1)][0] + "\");"
            lnSpliceS11DIRECT += "\n" + globals()["dMotor" + str(iterate + 1)][0] + ".setDirection(DcMotor.Direction." + globals()["dMotor" + str(iterate + 1)][1] + ");"
            lnSpliceS11PWR += "\n" + globals()["dMotor" + str(iterate + 1)][0] + ".setPower(0);"
            lnSpliceS11ENCODE += "\n" + globals()["dMotor" + str(iterate + 1)][0] + ".setMode(DcMotor.RunMode." + globals()["dMotor" + str(iterate + 1)][2] + ");\n"
            iterate += 1
        setLine(4, lnSpliceS4)
        setLine(11, lnSpliceS11NAME + lnSpliceS11DIRECT + lnSpliceS11PWR + lnSpliceS11ENCODE)
    ##OTHER MOTORS
    while True:
        try:
            uMotoType = int(input("How many other motors are on your robot?"))
            break
        except ValueError:
            print("Invalid answer. Try again.")
    if uMotoType != 0:
        iterate = 1
        while iterate <= uMotoType:
            while True:
                try:
                    uMotoName = str(input("What is the name of motor " + str(iterate) + "?"))
                    if uMotoType != "":
                        break
                    else:
                        print("Invalid answer. Try again.")
                except ValueError:
                    print("Invalid answer. Try again.")
            if iterate == 1:
                normNames[0] = uMotoName
            else:
                normNames.append(uMotoName)
            iterate += 1
        iterate = 0
        while iterate < len(normNames):
            print(normNames[iterate])
            while True:
                try:
                    globals()["uMotor" + str(iterate + 1)] = [normNames[iterate], int(input("Is this motor forward or reverse? Enter \"1\" for forward, or \"0\" for reverse.")), int(input("Does this motor use an encoder? Enter \"0\" for no encoder, or \"1\" for an encoder."))]
                    break
                except ValueError:
                    print("Invalid answer. Try again.")
            if globals()["uMotor" + str(iterate + 1)][2] == 0:
                globals()["uMotor" + str(iterate + 1)][2] = "RUN_WITHOUT_ENCODER"
            else:
                globals()["uMotor" + str(iterate + 1)][2] = "RUN_USING_ENCODER"
            if globals()["uMotor" + str(iterate + 1)][1] == 0:
                globals()["uMotor" + str(iterate + 1)][1] = "REVERSE"
            else:
                globals()["uMotor" + str(iterate + 1)][1] = "FORWARD"
            iterate += 1
        lnSpliceS5 = "//S5\n"
        lnSpliceS12NAME = "\n//NAMES\n"
        lnSpliceS12DIRECT = "\n//DIRECTIONS\n"
        lnSpliceS12PWR = "\n//POWER\n"
        lnSpliceS12ENCODE = "\n//ENCODERS\n"
        iterate = 0
        while iterate < uMotoType:
            lnSpliceS5 += "\npublic DcMotor " + globals()["uMotor" + str(iterate + 1)][0] + " = null;"
            lnSpliceS12NAME += "\n" + globals()["uMotor" + str(iterate + 1)][0] + " = hwMap.dcMotor.get(\"" + globals()["uMotor" + str(iterate + 1)][0] + "\");"
            lnSpliceS12DIRECT += "\n" + globals()["uMotor" + str(iterate + 1)][0] + ".setDirection(DcMotor.Direction." + globals()["uMotor" + str(iterate + 1)][1] + ");"
            lnSpliceS12PWR += "\n" + globals()["uMotor" + str(iterate + 1)][0] + ".setPower(0);"
            lnSpliceS12ENCODE += "\n" + globals()["uMotor" + str(iterate + 1)][0] + ".setMode(DcMotor.RunMode." + globals()["uMotor" + str(iterate + 1)][2] + ");\n"
            iterate += 1
        setLine(5, lnSpliceS5)
        setLine(12, lnSpliceS12NAME + lnSpliceS12DIRECT + lnSpliceS12PWR + lnSpliceS12ENCODE)
    ##SERVOS
    while True:
        try:
            servoType = int(input("How many servos are on your robot?"))
            break
        except ValueError:
            print("Invalid answer. Try again.")
    if servoType != 0:
        iterate = 1
        while iterate <= servoType:
            while True:
                try:
                    servoName = str(input("What is the name of servo " + str(iterate) + "?"))
                    if servoName != "":
                        break
                    else:
                        print("Invalid answer. Try again.")
                except ValueError:
                    print("Invalid answer. Try again.")
            if iterate == 1:
                servoNames[0] = servoName
            else:
                servoNames.append(servoName)
            iterate += 1
        iterate = 0
        while iterate < len(servoNames):
            print(servoNames[iterate])
            while True:
                try:
                    globals()["servo" + str(iterate + 1)] = [servoNames[iterate], int(input("Is this servo continuous or 180 degree? \n Enter \"0\" for continuous or \"1\" for 180 degree."))]
                    break
                except ValueError:
                    print("Invalid answer. Try again.")
            if globals()["servo" + str(iterate + 1)][1] == 1:
                while True:
                    try:
                        globals()["servo" + str(iterate + 1)].append(float(input("What position should this start at? (Range 0 to 1)")))
                        break
                    except ValueError:
                        print("Invalid answer. Try again.")
                globals()["servo" + str(iterate + 1)][1] = "Servo"
                globals()["servo" + str(iterate + 1)].append("180")
            if globals()["servo" + str(iterate + 1)][1] == 0:
                while True:
                    try:
                        globals()["servo" + str(iterate + 1)].append(float(input("What power should this servo have upon initialization? (Range -1 to 1)")))
                        break
                    except ValueError:
                        print("Invalid answer. Try again.")
                while True:
                    try:
                        globals()["servo" + str(iterate + 1)].append(int(input("What direction should this servo go? \nEnter \"0\" for REVERSE, or \"1\" for FORWARD.")))
                        break
                    except ValueError:
                        print("Invalid answer. Try again.")
                globals()["servo" + str(iterate + 1)][1] = "CRServo"
                if globals()["servo" + str(iterate + 1)][3] == 0:
                    globals()["servo" + str(iterate + 1)][3] = "REVERSE"
                if globals()["servo" + str(iterate + 1)][3] == 1:
                    globals()["servo" + str(iterate + 1)][3] = "FORWARD"
            iterate += 1
        lnSplice6 = "\n//S6\n"
        lnSpliceS13NAME = "\n//NAMES\n"
        lnSpliceS13DIRECT = "\n//CONTINOUS ROTATION SERVO DIRECTIONS\n"
        lnSpliceS13VAL = "\n//180 DEGREE SERVO POSITIONS\n//AND CONTINUOUS ROTATION SERVO POWERS\n"
        iterate = 0
        while iterate < servoType:
            lnSplice6 += "\npublic " + globals()["servo" + str(iterate + 1)][1] + " " + globals()["servo" + str(iterate + 1)][0] + " = null;"
            lnSpliceS13NAME += "\n" + globals()["servo" + str(iterate + 1)][0] + " = hwMap." + globals()["servo" + str(iterate + 1)][1].lower() + ".get(\"" + globals()["servo" + str(iterate + 1)][0] + "\");"
            if globals()["servo" + str(iterate + 1)][1] == "CRServo":
                lnSpliceS13DIRECT += "\n" + globals()["servo" + str(iterate + 1)][0] + ".setDirection(CRServo.Direction." + globals()["servo" + str(iterate + 1)][3] + ");"
                lnSpliceS13VAL += "\n" + globals()["servo" + str(iterate + 1)][0] + ".setPower(" + str(globals()["servo" + str(iterate + 1)][2]) + ");"
            if globals()["servo" + str(iterate + 1)][1] == "Servo":
                lnSpliceS13VAL += "\n" + globals()["servo" + str(iterate + 1)][0] + ".setPosition(" + str(globals()["servo" + str(iterate + 1)][2]) + ");"
            iterate += 1
        setLine(6, lnSplice6)
        setLine(13, lnSpliceS13NAME + lnSpliceS13DIRECT + lnSpliceS13VAL)
    ##SENSORS
    while True:
        try:
            sensorType = int(input("How many sensors are on your robot?"))
            break
        except ValueError:
            print("Invalid answer. Try again.")
    if sensorType != 0:
        iterate = 1
        while iterate <= sensorType:
            while True:
                try:
                    sensorName = str(input("What is the name of sensor " + str(iterate) + "?"))
                    if sensorName != "":
                        break
                    else:
                        print("Invalid answer. Try again.")
                except ValueError:
                    print("Invalid answer. Try again.")
            if iterate == 1:
                sensorNames[0] = sensorName
            else:
                sensorNames.append(sensorName)
            iterate += 1
        iterate = 0
        while iterate < len(sensorNames):
            print("\n" + sensorNames[iterate] + "\n")
            while True:
                try:
                    globals()["sensor" + str(iterate + 1)] = [sensorNames[iterate], int(input("What type of sensor is this? \n Enter \"0\" for a gyroscope. \n Enter \"1\" for a range sensor. \n Enter \"2\" for a touch sensor. \n Enter \"3\" for a color sensor. \n Enter \"4\" for an optical distance sensor. \n Enter \"5\" for the REV IMU sensor. \n Enter \"6\" for a compass."))]
                    break
                except ValueError:
                    print("Invalid answer. Try again.")
            if globals()["sensor" + str(iterate + 1)][1] == 0:
                #Gyro
                while True:
                    try:
                        globals()["sensor" + str(iterate + 1)].append(int(input("Is this a Modern Robotics Gyroscope? \nEnter \"1\" for yes, or \"0\" for another brand.\n")))
                        break
                    except ValueError:
                        print("Invalid answer. Try again.")
                if globals()["sensor" + str(iterate + 1)][2] == 0:
                    globals()["sensor" + str(iterate + 1)][2] = "IntegratingGyroscope"
                if globals()["sensor" + str(iterate + 1)][2] == 1:
                    globals()["sensor" + str(iterate + 1)][2] = "ModernRoboticsI2cGyro"
            if globals()["sensor" + str(iterate + 1)][1] == 1:
                #Range
                while True:
                    try:
                        globals()["sensor" + str(iterate + 1)].append(int(input("Is this a Modern Robotics Range Sensor or a regular distance sensor? \nEnter \"0\" for Modern Robotics, or \"1\" for the regular sensor.\n")))
                        break
                    except ValueError:
                        print("Invalid answer. Try again.")
                if globals()["sensor" + str(iterate + 1)][2] == 0:
                    globals()["sensor" + str(iterate + 1)][2] = "ModernRoboticsI2cRangeSensor"
                if globals()["sensor" + str(iterate + 1)][2] == 1:
                    globals()["sensor" + str(iterate + 1)][2] = "DistanceSensor"
            if globals()["sensor" + str(iterate + 1)][1] == 2:
                #Touch
                globals()["sensor" + str(iterate + 1)].append("TouchSensor")
            if globals()["sensor" + str(iterate + 1)][1] == 3:
                #Color
                globals()["sensor" + str(iterate + 1)].append("ColorSensor")
            if globals()["sensor" + str(iterate + 1)][1] == 4:
                #ODS
                globals()["sensor" + str(iterate + 1)].append("OpticalDistanceSensor")
            if globals()["sensor" + str(iterate + 1)][1] == 5:
                #IMU
                globals()["sensor" + str(iterate + 1)].append("BNO055IMU")
            if globals()["sensor" + str(iterate + 1)][1] == 6:
                #Compass
                globals()["sensor" + str(iterate + 1)].append("ModernRoboticsI2cCompassSensor")
            iterate += 1
        iterate = 0
        lnSplice7 = "\n//SENSORS\n"
        lnSplice14NAMES = "\n//NAMES\n"
        while iterate < sensorType:
            lnSplice7 += "\n public " + globals()["sensor" + str(iterate + 1)][2] + " " + globals()["sensor" + str(iterate + 1)][0] + " = null;"
            if globals()["sensor" + str(iterate + 1)][1] == 0:
                #Gyro
                if globals()["sensor" + str(iterate + 1)][2] == "IntegratingGyroscope":
                    #IntegratingGyroscope
                    lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(IntegratingGyroscope.class, \""+ globals()["sensor" + str(iterate + 1)][0] + "\");"
                if globals()["sensor" + str(iterate + 1)][2] == "ModernRoboticsI2cGyro":
                    #ModernRoboticsI2cGyro
                    lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(ModernRoboticsI2cGyro.class, \""+ globals()["sensor" + str(iterate + 1)][0] + "\");"
            if globals()["sensor" + str(iterate + 1)][1] == 1:
                #Range
                if globals()["sensor" + str(iterate + 1)][2] == "ModernRoboticsI2cRangeSensor":
                    #ModernRoboticsI2cRangeSensor
                    lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(ModernRoboticsI2cRangeSensor.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");"
                if globals()["sensor" + str(iterate + 1)][2] == "DistanceSensor":
                    #DistanceSensor
                    lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(DistanceSensor.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");"
            if globals()["sensor" + str(iterate + 1)][1] == 2:
                #TouchSensor
                lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.touchSensor.get(\"" + globals()["sensor" + str(iterate + 1)][0] + "\");"
            if globals()["sensor" + str(iterate + 1)][1] == 3:
                #ColorSensor
                lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(ColorSensor.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");\n" + globals()["sensor" + str(iterate + 1)][0] + ".enableLed(true);"
            if globals()["sensor" + str(iterate + 1)][1] == 4:
                #OpticalDistanceSensor
                lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(OpticalDistanceSensor.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");"
            if globals()["sensor" + str(iterate + 1)][1] == 5:
                #BNO055IMU
                lnSplice14NAMES += "\n BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();\nparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;\nparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;\nparameters.calibrationDataFile = \"BNO055IMUCalibration.json\"; // see the calibration sample opmode\nparameters.loggingEnabled      = true;\nparameters.loggingTag          = \"IMU\";\nparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();"
                lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(BNO055IMU.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");\n" + globals()["sensor" + str(iterate + 1)][0] + ".initialize(parameters);"
            if globals()["sensor" + str(iterate + 1)][1] == 6:
                #ModernRoboticsI2cCompassSensor
                lnSplice14NAMES += "\n" + globals()["sensor" + str(iterate + 1)][0] + " = hwMap.get(ModernRoboticsI2cCompassSensor.class, \"" + globals()["sensor" + str(iterate + 1)][0] + "\");"
            iterate += 1
        setLine(7, lnSplice7)
        setLine(14, lnSplice14NAMES)
    print(write(fileName, sampList, "java"))
if fStyle == 1:
    #TEST FILE
    idInit("Sharp" + str(tName) + "Hardware")
    motoAmount = motoId()
    print(motoAmount)
    servoAmount = servoId()
    print(servoAmount)
    crservoAmount = crservoId()
    print(crservoAmount)
    sensorAmount = sensorId()
    print(sensorAmount)
