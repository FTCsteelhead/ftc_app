package org.steelhead.ftc;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Alec Matthews on 11/6/2016.
 * This class is made to simplify the robots autonomous programs.
 */

public class AutoRobotFunctions {

    private double navKP;
    private double navKI;
    private double navKD;

    private double colorKP;
    private double colorKI;
    private double colorKD;

    private HardwareSteelheadMainBot robot;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private AHRS navXDevice;

    private LinearOpMode currentOpMode;
    private ColorSensor colorSensor;
    private Adafruit_ColorSensor beaconColor;
    private TouchSensor touchSensor;

    public enum StopConditions {COLOR, ENCODER, BUTTON}
    public enum Team {RED, BLUE}

    AutoRobotFunctions(byte navXDevicePortNumber, HardwareMap hardwareMap,
                       LinearOpMode currentOpMode, HardwareSteelheadMainBot robot) {
        boolean calibrationComplete = false;
        //TODO: change the motors when the hardware gets modified
        this.leftMotor = robot.leftMotor_1;
        this.rightMotor = robot.rightMotor_1;
        this.currentOpMode = currentOpMode;
        this.touchSensor = robot.touchSensor;
        this.colorSensor = robot.colorSensor;
        this.beaconColor = robot.beaconColor;

        navXDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                navXDevicePortNumber, AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while (!calibrationComplete && currentOpMode.opModeIsActive()) {
            calibrationComplete = !navXDevice.isCalibrating();
            currentOpMode.telemetry.addData("CAL: ", "NavX device calibrating");
            currentOpMode.telemetry.update();
        }
        navXDevice.zeroYaw();
    }

    public void navxRotateToDegree(double degree, double tolerance,
                                   double minMotorOutput, double maxMotorOutput) {
        boolean rotationComplete = false;

        navXPIDController yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(degree);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(minMotorOutput, maxMotorOutput);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance);
        yawPIDController.setPID(navKP, navKI, navKD);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while (!rotationComplete && currentOpMode.opModeIsActive()
                && !Thread.currentThread().isInterrupted()) {
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    rotationComplete = true;
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(output);
                    //TODO: set the correct motor side to negative so it turns the correct direction
                    rightMotor.setPower(-output);
                }
            }
        }
        yawPIDController.close();

    }

    public void navXDriveStraight(double degree, double tolerance,
                                  double minMotorOutput, double maxMotorOutput,
                                  double forwardDriveSpeed, int encoderDistance,
                                  double motorSpeedMul, double minPower,
                                         StopConditions stopCondition, int stopVal) {
        double workingForwardSpeed = forwardDriveSpeed;
        //enable and clear the encoders
        robot.enableEncoders(true);
        robot.stopAndClearEncoders();

        navXPIDController yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(degree);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(minMotorOutput, maxMotorOutput);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance);
        yawPIDController.setPID(navKP, navKI, navKD);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while (currentOpMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            if (stopCondition == StopConditions.COLOR && colorSensor.alpha() > stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
            } else if (stopCondition == StopConditions.ENCODER) {
                if (rightMotor.getCurrentPosition() >= stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
            } else if (stopCondition == StopConditions.BUTTON && touchSensor.isPressed()) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
            }
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(workingForwardSpeed);
                    rightMotor.setPower(workingForwardSpeed);
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(workingForwardSpeed - output);
                    //TODO: set the correct motor side to negative so it turns the correct direction
                    rightMotor.setPower(workingForwardSpeed + output);
                }
            }

            /*
            Slow the robot as it gets close to the line so it does not overshoot,
            This is basically a P controller.
            If the multiplier is equal to -1 turn off the speed reduction
            */

            if (motorSpeedMul != -1) {
                if (rightMotor.getCurrentPosition() >= (encoderDistance - 500)) {
                    int error = (encoderDistance - rightMotor.getCurrentPosition()) - 500;
                    workingForwardSpeed = forwardDriveSpeed - error * motorSpeedMul;
                    if (workingForwardSpeed < minPower) {
                        workingForwardSpeed = minPower;
                    }

                }
            }
        }
        robot.stopAndClearEncoders();
        robot.enableEncoders(false);
        yawPIDController.close();
    }

    public void PIDLineFollow(int threshHoldLow, int threshHoldHigh,
                              double driveSpeed, double minOutputVal,
                              double maxOutputVal, double tolerance, StopConditions stopConditions) {
        ColorPIDController pidController = new ColorPIDController(colorSensor, threshHoldLow, threshHoldHigh);
        pidController.setPID(colorKP, colorKI, colorKD);
        pidController.setTolerance(tolerance);
        pidController.enable();

        while (currentOpMode.opModeIsActive()) {
            if (stopConditions == StopConditions.BUTTON && touchSensor.isPressed()) {
                break;
            }
            double output = pidController.getOutput();
            //TODO: Check sides of the line follower
            leftMotor.setPower(limit((driveSpeed - output), minOutputVal, maxOutputVal));
            rightMotor.setPower(limit((driveSpeed + output), minOutputVal, maxOutputVal));
        }
        pidController.disable();
    }

    public void runWithEncoders(int targetPosition, double leftMotorPower, double rightMotorPower) {
        robot.enableEncoders(true);
        robot.stopAndClearEncoders();

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        while (currentOpMode.opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            currentOpMode.telemetry.addData("ENC left: ", leftMotor.getCurrentPosition());
            currentOpMode.telemetry.addData("ENC right: ", rightMotor.getCurrentPosition());
            currentOpMode.telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        robot.enableEncoders(false);
    }

    public void pushButton(Team team) {

        //TODO: fix color sensor by adding tha adafruit one
        try {
            Thread.sleep(500);
            if (team == Team.RED) {
                if (beaconColor.blueColor() > 60) {
                    robot.pusherRight.setPosition(0.2);
                } else {
                    robot.pusherLeft.setPosition(0.8);
                }
            } else {
                if (beaconColor.blueColor() > 60) {
                    robot.pusherRight.setPosition(0.8);
                } else {
                    robot.pusherLeft.setPosition(0.2);
                }
            }

            Thread.sleep(1000);
            robot.pusherRight.setPosition(0.8);
            robot.pusherLeft.setPosition(0.2);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

    public void setNavXPID(double Kp, double Ki, double Kd) {
        this.navKP = Kp;
        this.navKI = Ki;
        this.navKD = Kd;
    }
    public void setColorPID(double Kp, double Ki, double Kd) {
        this.colorKP = Kp;
        this.colorKI = Ki;
        this.colorKD = Kd;
    }
    private double limit(double a, double minOutputVal, double maxOutputVal) {
        return Math.min(Math.max(a, minOutputVal), maxOutputVal);
    }
}
