package org.steelhead.ftc;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public enum StopConditions {COLOR, ENCODER, BUTTON}

    AutoRobotFunctions(byte navXDevicePortNumber, HardwareMap hardwareMap, DcMotor leftMotor,
                       DcMotor rightMotor, HardwareSteelheadMainBot robot, LinearOpMode currentOpMode) {
        //TODO: get left motor and right motor from hardware class not variables
        boolean calibrationComplete = false;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.currentOpMode = currentOpMode;

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
    }

    public void navXDriveStraightToColor(double degree, double tolerance,
                                  double minMotorOutput, double maxMotorOutput,
                                  double forwardDriveSpeed, int encoderDistance,
                                  double motorSpeedMul, double minPower,
                                         StopConditions stopConditions, int stopVal) {
        double workingForwardSpeed = forwardDriveSpeed;

        if (motorSpeedMul != -1 || stopConditions == StopConditions.ENCODER) {
            robot.enableEncoders(true);
            robot.stopAndClearEncoders();
        }

        navXPIDController yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(degree);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(minMotorOutput, maxMotorOutput);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance);
        yawPIDController.setPID(navKP, navKI, navKD);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while (currentOpMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            //Stop at either an encoder value or light value
            if (stopConditions == StopConditions.COLOR) {
                if (colorSensor.alpha() > stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
            } else if (stopConditions == StopConditions.ENCODER) {
                if (leftMotor.getCurrentPosition() >= stopVal) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
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
            This is basically a P controller
            */
            //TODO: Play with adding left and right controllers separate might not work good tho
            //If the multiplier is equal to -1 turn off the speed reduction
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
        //clean up after ourselves after driving straight
        robot.stopAndClearEncoders();
        robot.enableEncoders(false);
        yawPIDController.close();
    }

    public void PIDLineFollow(int threshHoldLow, int threshHoldHigh, double driveSpeed,
                              double tolerance, double maxMotorInput, double minMotorInput) {
        ColorPIDController pidController = new ColorPIDController(colorSensor, threshHoldLow, threshHoldHigh);
        pidController.setPID(colorKP, colorKI, colorKD);
        pidController.setTolerance(tolerance);

        pidController.enable();
        while (currentOpMode.opModeIsActive() /*TODO: add push button support for stops*/) {
            double output = pidController.getOutput();
            leftMotor.setPower(limit((driveSpeed - output), minMotorInput, maxMotorInput));
            rightMotor.setPower(limit((driveSpeed + output), minMotorInput, maxMotorInput));
    }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        pidController.disable();
        //TODO: clean up after ourselves
    }

    public void setNavXPID(double Kp, double Ki, double Kd) {
        this.navKP = Kp;
        this.navKI = Ki;
        this.navKD = Kd;
    }
    public void setColorPID(double Kp, double Ki, double Kd) {
        this.colorKP = Kp;
        this.colorKI = Ki;
        this.colorKD= Kd;
    }
    private double limit(double a, double minOutputVal, double maxOutputVal) {
        return Math.min(Math.max(a, minOutputVal), maxOutputVal);
    }
}
