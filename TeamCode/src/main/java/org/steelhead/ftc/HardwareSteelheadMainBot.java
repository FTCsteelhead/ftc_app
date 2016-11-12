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
    public DcMotor leftMotor                = null;
    public DcMotor rightMotor               = null;
    public Servo pusherRight                = null;
    public Servo pusherLeft                 = null;
    public ColorSensor colorSensor          = null;
    public TouchSensor touchSensor          = null;
    public Adafruit_ColorSensor beaconColor = null;


    private String leftMotorName_1  = "leftMotor1";
    private String rightMotorName_1 = "rightMotor1";
    private String pusherRightName  = "pusherRight";
    private String pusherLeftName   = "pusherLeft";
    private String touchSensorName  = "touch";
    private String colorSensorName  = "color";
    private String beaconColorName  = "BColor";

    public void init(HardwareMap aHwMap) {

        leftMotor = aHwMap.dcMotor.get(leftMotorName_1);

        rightMotor = aHwMap.dcMotor.get(rightMotorName_1);

        //sets the robot direction to backward
        //TODO: check the directions once the electronics are set up
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        robotLeftPower(0);
        robotRightPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

        //Adafruit Color sensor
        beaconColor = new Adafruit_ColorSensor(aHwMap, beaconColorName);
        beaconColor.setLed(false);
    }

    public void setLeftMotorName(String newName) {

        leftMotorName_1 = newName + "1";
    }

    public void setRightMotorName(String newName) {

        rightMotorName_1 = newName + "1";
    }

    public void robotLeftPower(double power) {
        leftMotor.setPower(power);
    }

    public void robotRightPower(double power) {
        rightMotor.setPower(power);
    }

    public void robotForward() {
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void robotBackward() {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void robotSetZeroPowerBehavior (DcMotor.ZeroPowerBehavior behavior) {
        rightMotor.setZeroPowerBehavior(behavior);
        leftMotor.setZeroPowerBehavior(behavior);
    }

    public void enableEncoders(boolean enabled) {
        if (enabled) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopAndClearEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
