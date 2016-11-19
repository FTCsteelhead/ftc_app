package org.steelhead.ftc;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Alec Matthews on 11/6/2016.
 * This class is made to simplify the robots autonomous programs.
 */

public class AutoRobotFunctions {

    //PID Values for the navX sensor
    private double navKPdrive;
    private double navKIdrive;
    private double navKDdrive;

    private double navKPturn;
    private double navKIturn;
    private double navKDturn;

    //PID Values for the Modern Robotics Sensor
    private double colorKP;
    private double colorKI;
    private double colorKD;

    //Movement components of the robot including a robot hardware class
    private HardwareSteelheadMainBot robot;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    //NavX Sensor
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ   = 50;
    private final int  DEVICE_TIMEOUT_MS            = 500;
    private AHRS navXDevice;

    //Rest of the sensors
    private LinearOpMode currentOpMode;
    private ColorSensor color;
    private Adafruit_ColorSensor beaconColor;
    private TouchSensor touchSensor;
    private DigitalChannel policeLED;

    public enum StopConditions {COLOR, ENCODER, BUTTON}

    public enum Team {RED, BLUE}

    public enum LineSide {LEFT, RIGHT}

    public AutoRobotFunctions(byte navXDevicePortNumber, HardwareMap hardwareMap,
                              LinearOpMode currentOpMode, HardwareSteelheadMainBot robot) {
        boolean calibrationComplete = false;
        this.robot = robot;
        this.currentOpMode = currentOpMode;
        this.leftMotor = robot.leftMotor;
        this.rightMotor = robot.rightMotor;
        this.touchSensor = robot.touchSensor;
        this.color = robot.color;
        this.beaconColor = robot.beaconColor;
        this.policeLED = robot.policeLED;

        //Setup the navX sensor and wait for calibration to complete
        navXDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                navXDevicePortNumber, AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while (!calibrationComplete && currentOpMode.opModeIsActive()) {
            calibrationComplete = !navXDevice.isCalibrating();
            currentOpMode.telemetry.addData("CAL: ", "NavX device calibrating");
            currentOpMode.telemetry.update();
        }

        if (!navXDevice.isConnected()) {
            currentOpMode.telemetry.addData("ERROR: ", "NavX not connected");
            currentOpMode.telemetry.update();
        }
        navXDevice.zeroYaw();
    }

    //PID controller for rotation to a degree
    public void navxRotateToDegree(double degree, double tolerance,
                                   double minMotorOutput, double maxMotorOutput) {
        boolean rotationComplete = false;
        robot.robotSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        navXPIDController yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(degree);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(minMotorOutput, maxMotorOutput);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance);
        yawPIDController.setPID(navKPturn, navKIturn, navKDturn);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        yawPIDController.enable(true);

        try {
            while (!rotationComplete && currentOpMode.opModeIsActive()
                    && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
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
                currentOpMode.telemetry.addData("YAW: ", navXDevice.getYaw());
                currentOpMode.telemetry.update();
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        yawPIDController.enable(false);
        yawPIDController.close();
        robot.robotSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Drive straight with a PID controller
    public void navXDriveStraight(double degree, double tolerance,
                                  double minOutputRage, double maxOutputRange,
                                  double driveSpeed, int encoderDistance,
                                  double motorSpeedMul, double minEndPower,
                                  StopConditions stopCondition, int stopVal) {
        double workingForwardSpeed = driveSpeed;
        //double rampMul = driveSpeed / 500;
        boolean rampComplete = true;
        ElapsedTime rampTime = new ElapsedTime();
        //Enable and clear the encoders
        robot.stopAndClearEncoders();
        robot.enableEncoders(true);

        //Setup the yaw PID controller
        navXPIDController yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(degree);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(minOutputRage, maxOutputRange);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance);
        yawPIDController.setPID(navKPdrive, navKIdrive, navKDdrive);
        yawPIDController.enable(true);
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        rampTime.reset();
        try {
            while (currentOpMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {

               currentOpMode.telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
                currentOpMode.telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
                //ramp the motor up to prevent damage and jerk

                /*if (!rampComplete && rampTime.milliseconds() <= 500) {
                    int error = (int) rampTime.milliseconds();
                    workingForwardSpeed = error * rampMul;
                    if (workingForwardSpeed > driveSpeed) {
                        workingForwardSpeed = driveSpeed;
                        rampComplete = true;
                    }
                }*/
                if (stopCondition == StopConditions.COLOR && color.alpha() > stopVal) {
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
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftMotor.setPower(workingForwardSpeed);
                        rightMotor.setPower(workingForwardSpeed);
                        currentOpMode.telemetry.addData("Target: ", "On Target");
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftMotor.setPower(limit((workingForwardSpeed + output), minOutputRage, maxOutputRange));
                        //TODO: set the correct motor side to negative so it turns the correct direction
                        rightMotor.setPower(limit((workingForwardSpeed - output), minOutputRage, maxOutputRange));
                        currentOpMode.telemetry.addData("Output: ", output);
                        currentOpMode.telemetry.addData("Target: ", navXDevice.getYaw());
                    }
                } else {
                    currentOpMode.telemetry.addData("navx: ", "DEVICE TIME OUT!");
                }
            /*
            Slow the robot as it gets close to the line so it does not overshoot,
            This is basically a P controller.
            If the multiplier is equal to -1 turn off the speed reduction
            */

                if (motorSpeedMul != -1) {
                    if (rampComplete && rightMotor.getCurrentPosition() >= (encoderDistance - 500)) {
                        int error = (encoderDistance - rightMotor.getCurrentPosition()) - 500;
                        workingForwardSpeed = driveSpeed + error * motorSpeedMul;
                        if (workingForwardSpeed < minEndPower) {
                            workingForwardSpeed = minEndPower;
                        }

                    }
                }
                currentOpMode.telemetry.addData("Light Val: ", color.alpha());
                currentOpMode.telemetry.addData("Working Speed: ", workingForwardSpeed);
                currentOpMode.telemetry.update();
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            yawPIDController.enable(false);
            robot.stopAndClearEncoders();
            robot.enableEncoders(false);
            yawPIDController.close();
        }
    }

    //PID controller for following a line
    public void PIDLineFollow(int threshHoldLow, int threshHoldHigh,
                              double driveSpeed, double minOutputVal,
                              double maxOutputVal, double tolerance,
                              StopConditions stopConditions, LineSide lineSide,
                              double rotationLimitDegree, boolean degreeLimitOn) {
        double currentAngle;
        ColorPIDController pidController = new ColorPIDController(this.color,
                threshHoldLow, threshHoldHigh);
        pidController.setPID(colorKP, colorKI, colorKD);
        pidController.setTolerance(tolerance);
        pidController.enable();

        try {
            //Sleep to allow the pid controller to calculate a first value
            Thread.sleep(100);
            while (currentOpMode.opModeIsActive()) {
                currentAngle = navXDevice.getYaw();
                if (stopConditions == StopConditions.BUTTON && touchSensor.isPressed()) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
                double output = pidController.getOutput();

                if (lineSide == LineSide.LEFT) {
                    leftMotor.setPower(limit((driveSpeed - output), minOutputVal, maxOutputVal));
                    rightMotor.setPower(limit((driveSpeed + output), minOutputVal, maxOutputVal));
                    //TODO: test this with these enabled and disabled to see if they break it or make it better
                    //if you do a rotate first this function should work. with some more tweaking it can work without the rotate
                    if (degreeLimitOn && currentAngle <= rotationLimitDegree - 45.0) {
                        lineSide = LineSide.RIGHT;
                    }
                } else {
                    leftMotor.setPower(limit((driveSpeed + output), minOutputVal, maxOutputVal));
                    rightMotor.setPower(limit((driveSpeed - output), minOutputVal, maxOutputVal));
                    if (degreeLimitOn && currentAngle >= rotationLimitDegree + 45.0) {
                        lineSide = LineSide.LEFT;
                    }
                }

            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            pidController.disable();
        }
    }

    //Drive to an encoder limit
    public void runWithEncoders(int targetPosition, double motorPower) {
        boolean rampComplete = false;
        ElapsedTime rampTime = new ElapsedTime();
        double rampUpMul = motorPower / 500;
        double rampDownMul = motorPower / 30;
        double workingForwardSpeed;

        robot.stopAndClearEncoders();
        robot.enableEncoders(true);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        rampTime.reset();
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
        while (currentOpMode.opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            //Ramp the motor to start with
            if (!rampComplete && rampTime.milliseconds() <= 30) {
                int error = (int) rampTime.milliseconds();
                workingForwardSpeed = error * rampUpMul;
                if (workingForwardSpeed >= motorPower) {
                    workingForwardSpeed = motorPower;
                    rampComplete = true;
                }
                leftMotor.setPower(workingForwardSpeed);
                rightMotor.setPower(workingForwardSpeed);
            }
            //Ramp down as the robot approaches the target.
            if (rampComplete && rightMotor.getCurrentPosition() >= (targetPosition - 30)) {
                int error = (targetPosition - rightMotor.getCurrentPosition()) - 30;
                workingForwardSpeed = motorPower - error * rampDownMul;
                if (workingForwardSpeed < 0.1) {
                    workingForwardSpeed = 0.1;
                }
                leftMotor.setPower(workingForwardSpeed);
                rightMotor.setPower(workingForwardSpeed);
            }
            currentOpMode.telemetry.addData("ENC left: ", leftMotor.getCurrentPosition());
            currentOpMode.telemetry.addData("ENC right: ", rightMotor.getCurrentPosition());
            currentOpMode.telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        robot.stopAndClearEncoders();
        robot.enableEncoders(false);
    }

    //Push the proper color button for the team the robot is on
    public void pushButton(Team team) {

        try {
            Thread.sleep(100);
            if (team == Team.RED) {
                if (beaconColor.blueColor() > 60) {
                    robot.pusherRight.setPosition(0.1);
                } else {
                    robot.pusherLeft.setPosition(0.9);
                }
            } else {
                if (beaconColor.blueColor() > 60) {
                    robot.pusherLeft.setPosition(0.9);
                } else {
                    robot.pusherRight.setPosition(0.1);
                }
            }

            Thread.sleep(600);
            robot.pusherRight.setPosition(0.8);
            robot.pusherLeft.setPosition(0.2);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void pusherActive(boolean state) {
        if (state) {
            robot.pusherRight.setPosition(0.2);
            robot.pusherLeft.setPosition(0.8);
        } else {
            robot.pusherRight.setPosition(0.8);
            robot.pusherLeft.setPosition(0.2);
        }
    }

    public void close() {
        navXDevice.close();
    }

    //Set the PID values for the NavX sensor
    public void setNavXPIDDriveStraight(double Kp, double Ki, double Kd) {
        this.navKPdrive = Kp;
        this.navKIdrive = Ki;
        this.navKDdrive = Kd;
    }

    public void setNavXPIDTurn(double Kp, double Ki, double Kd) {
        this.navKPturn = Kp;
        this.navKIturn = Ki;
        this.navKDturn = Kd;
    }

    //Set the PID values for Color sensor
    public void setColorPID(double Kp, double Ki, double Kd) {
        this.colorKP = Kp;
        this.colorKI = Ki;
        this.colorKD = Kd;
    }

    //Limit function
    private double limit(double a, double minOutputVal, double maxOutputVal) {
        return Math.min(Math.max(a, minOutputVal), maxOutputVal);
    }
}
