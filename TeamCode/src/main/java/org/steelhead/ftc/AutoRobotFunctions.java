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

    private HardwareMap hardwareMap;
    private LinearOpMode currentOpMode;
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;

    public enum StopConditions {COLOR, ENCODER, BUTTON}

    AutoRobotFunctions(byte navXDevicePortNumber, HardwareMap hardwareMap,
                       LinearOpMode currentOpMode, HardwareSteelheadMainBot robot) {
        boolean calibrationComplete = false;
        //TODO: change the motors when the hardware gets modified
        this.leftMotor = robot.leftMotor_1;
        this.rightMotor = robot.rightMotor_1;
        this.hardwareMap = hardwareMap;
        this.currentOpMode = currentOpMode;
        this.touchSensor = robot.touchSensor;
        this.colorSensor = robot.colorSensor;

        navXDevice = AHRS.getInstance(this.hardwareMap.deviceInterfaceModule.get("dim"),
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
                    rightMotor.setPower(output);
                }
            }
        }
        yawPIDController.close();

    }

    public void navXDriveStraightToColor(double degree, double tolerance,
                                  double minMotorOutput, double maxMotorOutput,
                                  double forwardDriveSpeed, int encoderDistance,
                                  double motorSpeedMul, double minPower,
                                         StopConditions stopConditions, int stopVal) {
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
            if (stopConditions == StopConditions.COLOR && colorSensor.alpha() > stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
            } else if (stopConditions == StopConditions.ENCODER) {
                //TODO: maybe stop each motor separately?? Probably wont work
                if (leftMotor.getCurrentPosition() >= stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
            } else if (stopConditions == StopConditions.BUTTON && touchSensor.isPressed()) {
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
                    leftMotor.setPower(workingForwardSpeed + output);
                    //TODO: set the correct motor side to negative so it turns the correct direction
                    rightMotor.setPower(workingForwardSpeed + output);
                }
            }

            /*
            Slow the robot as it gets close to the line so it does not overshoot,
            This is basically a P controller.
            If the multiplier is equal to -1 turn off the speed reduction
            */
            //TODO: Play with adding left and right controllers separate might not work good tho
            if (motorSpeedMul != -1) {
                if (leftMotor.getCurrentPosition() >= (encoderDistance - 500)) {
                    int error = (encoderDistance - leftMotor.getCurrentPosition()) - 500;
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

    public void PIDLineFollow(int threshHoldLow, int threshHoldHigh, double driveSpeed,
                              double minOutputVal, double maxOutputVal, double tolerance, StopConditions stopConditions) {
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
