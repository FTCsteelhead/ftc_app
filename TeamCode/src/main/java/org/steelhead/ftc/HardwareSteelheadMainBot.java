package org.steelhead.ftc;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Alec Matthews on 9/18/16.
 * Added this class so we don't have to
 * edit a sample one
 **/

public class HardwareSteelheadMainBot {
    public DcMotor leftMotor_1      = null;
    public DcMotor leftMotor_2      = null;
    public DcMotor rightMotor_1     = null;
    public DcMotor rightMotor_2     = null;
    public Servo pusherRight        = null;
    public Servo pusherLeft         = null;
    public ColorSensor colorSensor  = null;
    public TouchSensor touchSensor  = null;


    private String leftMotorName_1  = "leftMotor1";
    private String leftMotorName_2  = "leftMotor2";
    private String rightMotorName_1 = "rightMotor1";
    private String rightMotorName_2 = "rightMotor2";
    private String pusherRightName  = "pusherRight";
    private String pusherLeftName   = "pusherLeft";
    private String touchSensorName  = "touch";
    private String colorSensorName  = "color";

    public void init(HardwareMap aHwMap) {

        leftMotor_1 = aHwMap.dcMotor.get(leftMotorName_1);
        leftMotor_2 = aHwMap.dcMotor.get(leftMotorName_2);

        rightMotor_1 = aHwMap.dcMotor.get(rightMotorName_1);
        rightMotor_2 = aHwMap.dcMotor.get(rightMotorName_2);

        //sets the robot direction to backward
        leftMotor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_2.setDirection(DcMotorSimple.Direction.FORWARD );


        robotLeftPower(0);
        robotRightPower(0);

        leftMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pusherRight = aHwMap.servo.get(pusherRightName);
        pusherLeft = aHwMap.servo.get(pusherLeftName);

        pusherRight.setPosition(0.8);
        pusherLeft.setPosition(0.2);

        //initialize sensors
        touchSensor = aHwMap.touchSensor.get(touchSensorName);

        //nasty trick to get the color sensor to work
        colorSensor = aHwMap.colorSensor.get(colorSensorName);
        colorSensor.enableLed(true);
        colorSensor.enableLed(false);
        colorSensor.enableLed(true);
        colorSensor.getManufacturer();
    }

    public void setLeftMotorName(String newName) {

        leftMotorName_1 = newName + "1";
        leftMotorName_2 = newName + "2";
    }

    public void setRightMotorName(String newName) {

        rightMotorName_1 = newName + "1";
        rightMotorName_2 = newName + "2";
    }

    public void robotLeftPower(double power) {
        leftMotor_1.setPower(power);
        leftMotor_2.setPower(power);
    }

    public void robotRightPower(double power) {
        rightMotor_1.setPower(power);
        rightMotor_2.setPower(power);
    }

    public void robotForward() {
        leftMotor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void robotBackward() {
        leftMotor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor_2.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void robotSetZeroPowerBehavior (DcMotor.ZeroPowerBehavior behavior) {
        rightMotor_1.setZeroPowerBehavior(behavior);
        rightMotor_2.setZeroPowerBehavior(behavior);
        leftMotor_1.setZeroPowerBehavior(behavior);
        leftMotor_2.setZeroPowerBehavior(behavior);
    }

    public void enableEncoders(boolean enabled) {
        if (enabled) {
            leftMotor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopAndClearEncoders() {
        leftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
