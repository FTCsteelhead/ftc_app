package org.steelhead.ftc;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.R;


/**
 * Created by Alec Matthews on 9/18/16.
 * Added this class so we don't have to
 * edit a sample one
 **/

public class HardwareSteelheadMainBot {
    public DcMotor leftMotor                    = null;
    public DcMotor rightMotor                   = null;
    public DcMotor sweeperMotor                 = null;
    public DcMotor rightShooterMotor            = null;
    public DcMotor leftShooterMotor             = null;
    public DcMotor lifterMotor                  = null;
    public Servo pusherRight                    = null;
    public Servo pusherLeft                     = null;
    public Servo shooterServo                   = null;
    public ColorSensor color                    = null;
    public TouchSensor touchSensor              = null;
    public ModernRoboticsI2cGyro gyro           = null;
    public Adafruit_ColorSensor beaconColor     = null;
    public VoltageSensor batVolt                = null;
    public Servo pusherRight2                   = null;
    public Servo pusherLeft2                    = null;
    public Servo trap                           = null;

    public SharedPreferences sharedPref         = null;

    private String leftMotorName_1          = "leftMotor1";
    private String rightMotorName_1         = "rightMotor1";
    private String sweeperMotorName         = "sweeper";
    private String rightShooterMotorName    = "rightShooter";
    private String leftShooterMotorName     = "leftShooter";
    private String lifterMotorName          = "lifter";
    private String pusherRightName          = "pusherRight";
    private String pusherLeftName           = "pusherLeft";
    private String touchSensorName          = "touch";
    private String colorSensorName          = "color";
    private String gyroSensorName           = "gyro";
    private String beaconColorName          = "BColor";
    private String shooterServoName         = "shooter";
    private String pusherRight2Name         = "pusherRight2";
    private String pusherLeft2Name          = "pusherLeft2";
    private String trapName                 = "trap";

    private boolean isRobotBackward = false;
    private boolean isRobotForward  = false;

    private static final String TAG = "ROBOT";
    private Context appContext = null;

    public void init(HardwareMap aHwMap) {
        appContext = aHwMap.appContext;

        leftMotor = aHwMap.dcMotor.get(leftMotorName_1);
        rightMotor = aHwMap.dcMotor.get(rightMotorName_1);
        lifterMotor = aHwMap.dcMotor.get(lifterMotorName);
        sweeperMotor = aHwMap.dcMotor.get(sweeperMotorName);

        leftShooterMotor = aHwMap.dcMotor.get(leftShooterMotorName);
        rightShooterMotor = aHwMap.dcMotor.get(rightShooterMotorName);
        //TODO: check the directions once the electronics are set up
        robotForward();

        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sweeperMotor.setPower(0);
        lifterMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);

        robotLeftPower(0);
        robotRightPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pusherRight = aHwMap.servo.get(pusherRightName);
        pusherLeft = aHwMap.servo.get(pusherLeftName);
        shooterServo = aHwMap.servo.get(shooterServoName);
        pusherRight2 = aHwMap.servo.get(pusherRight2Name);
        pusherLeft2 = aHwMap.servo.get(pusherLeft2Name);
        trap = aHwMap.servo.get(trapName);

        pusherRight2.setPosition(0.8);
        pusherLeft2.setPosition(0.2);
        trap.setPosition(0.5);

        pusherRight.setPosition(0.7);
        pusherLeft.setPosition(0.3);
        shooterServoDown(true);

        //initialize sensors
        batVolt = aHwMap.voltageSensor.get("Right Aux Motor Controller");
        touchSensor = aHwMap.touchSensor.get(touchSensorName);
        Log.i(TAG, touchSensor.getManufacturer().toString());

        //nasty trick to get the color sensor to work
        color = aHwMap.colorSensor.get(colorSensorName);
        color.enableLed(true);
        color.enableLed(false);
        color.enableLed(true);
        Log.i(TAG, color.getManufacturer().toString());

        gyro = (ModernRoboticsI2cGyro)aHwMap.gyroSensor.get(gyroSensorName);
        Log.i(TAG, gyro.getManufacturer().toString());

        //Adafruit Color sensor
        beaconColor = new Adafruit_ColorSensor(aHwMap, beaconColorName);
        beaconColor.setLed(false);

        //Setup SharedPreferences, this is used for getting the threshold values
        sharedPref = appContext.getSharedPreferences(appContext.getString(R.string.AutoPreferences),
                Context.MODE_PRIVATE);
    }

    public void robotLeftPower(double power) {
        leftMotor.setPower(power);
    }

    public void robotRightPower(double power) {
        rightMotor.setPower(power);
    }

    public void shooterPower(double power){
        leftShooterMotor.setPower(power);
        rightShooterMotor.setPower(power);
    }

    public void shooterServoDown(boolean state) {
        if (state) {
            shooterServo.setPosition(0.59);
        } else {
            shooterServo.setPosition(0.40);
        }
    }

    public double shooterMotorOn(boolean state) {
        if (state) {
            double batVoltage = batVolt.getVoltage();
            double motorPercent = (5.5/batVoltage);
            if (motorPercent > 1) {
                shooterPower(1);
            } else {
                shooterPower(motorPercent);
            }
          //  Log.i(TAG, String.format("%f2.2, %f2.2", batVoltage, motorPercent));
            return motorPercent;
        } else {
            shooterPower(0);
            return 0;
        }
    }

    public void robotForward() {
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        isRobotBackward = false;
        isRobotForward = true;
    }

    public void robotBackward() {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        isRobotBackward = true;
        isRobotForward = false;
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

    public boolean isRobotBackward() {
        return isRobotBackward;
    }
    public boolean isRobotForward() {
        return isRobotForward;
    }

    public void setSidePusherPosition(double position){
        pusherRight2.setPosition(1-position);
        pusherLeft2.setPosition(position);
    }

    public void close() {

        pusherLeft.close();
        pusherRight.close();
        shooterServo.close();
        pusherLeft2.close();
        pusherRight2.close();

        color.close();
        beaconColor.close();
        touchSensor.close();
        gyro.close();

        Log.i(TAG, "Everything closed except batVolt");
    }
}
