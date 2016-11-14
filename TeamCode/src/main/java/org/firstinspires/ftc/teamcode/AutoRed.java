package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.Adafruit_ColorSensor;
import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.ColorPIDController;
import org.steelhead.ftc.HardwareSteelheadMainBot;

import java.text.DecimalFormat;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher - Red", group = "Button")
//@Disabled
public class AutoRed extends LinearOpMode {

    private final byte NAVX_DIM_I2C_PORT = 0;
    private final double TOLERANCE_DEGREES = 2.0;

    private double MAX_OUTPUT_DRIVE = 0.5;
    private double MIN_OUTPUT_DRIVE = -0.5;
    private double MAX_OUTPUT_ROTATE = 0.25;
    private double MIN_OUTPUT_ROTATE = -0.25;
    private double MAX_OUTPUT_LINE = 0.25;
    private double MIN_OUTPUT_LINE = -0.25;

    private AutoRobotFunctions autoRobotFunctions;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

        autoRobotFunctions = new AutoRobotFunctions(NAVX_DIM_I2C_PORT, hardwareMap, this, robot);

        //Setup the PID values for the NavX sensor
        autoRobotFunctions.setNavXPID(0.06, 0.0012, 0.85);
        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);

        //Initialize the hardware
        robot.init(hardwareMap);

        telemetry.addData("STATUS:", "init complete");
        telemetry.update();

        //wait for start of the match
        waitForStart();

        robot.robotBackward();
        autoRobotFunctions.runWithEncoders(500, 0.25);

        autoRobotFunctions.navxRotateToDegree(-45.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        //TODO: check this degree it could work with -45.0 degrees
        autoRobotFunctions.navXDriveStraight(135.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, 0.50, 500, 0.005, 0.05,
                AutoRobotFunctions.StopConditions.COLOR, 8);

        robot.robotForward();

        autoRobotFunctions.runWithEncoders(80, 0.25);

        robot.robotBackward();

        autoRobotFunctions.PIDLineFollow(3, 22, 0.10, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON);

        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        robot.robotForward();

        autoRobotFunctions.runWithEncoders(700, 0.25);

        robot.robotBackward();

        autoRobotFunctions.navxRotateToDegree(0.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        autoRobotFunctions.runWithEncoders(10, 0.20);

        autoRobotFunctions.navxRotateToDegree(0.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        autoRobotFunctions.navXDriveStraight(0.0, TOLERANCE_DEGREES, MIN_OUTPUT_DRIVE,
                MAX_OUTPUT_DRIVE, 0.25, 500, 0.005, 0.05,
                AutoRobotFunctions.StopConditions.COLOR, 8);
        robot.robotForward();
        autoRobotFunctions.runWithEncoders(80, 0.25);
        robot.robotBackward();
        autoRobotFunctions.PIDLineFollow(3, 22, 0.10, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON);
        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}

