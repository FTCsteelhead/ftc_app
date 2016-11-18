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

    private final byte NAVX_DIM_I2C_PORT = 1;
    private final double TOLERANCE_DEGREES = 2.0;

    private double MAX_OUTPUT_DRIVE = 0.65;
    private double MIN_OUTPUT_DRIVE = -0.15;
    private double MAX_OUTPUT_ROTATE = 0.15;
    private double MIN_OUTPUT_ROTATE = -0.15;
    private double MAX_OUTPUT_LINE = 0.25;
    private double MIN_OUTPUT_LINE = -0.15;

    private AutoRobotFunctions autoRobotFunctions;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

        robot.init(hardwareMap);

        autoRobotFunctions = new AutoRobotFunctions(NAVX_DIM_I2C_PORT, hardwareMap, this, robot);

        //Setup the PID values for the NavX sensor
        //TODO: adjust values for drive straight
        autoRobotFunctions.setNavXPIDDriveStraight(0.33, 0.0008, 0.95);
        autoRobotFunctions.setNavXPIDTurn(0.33, 0.0008, 0.95);

        //TODO: also this needs work
        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


        telemetry.addData("STATUS:", "init complete");
        telemetry.update();

        //wait for start of the match

        waitForStart();
        robot.setPoliceLED(true);

        robot.robotForward();
        //autoRobotFunctions.pusherActive(true);
        autoRobotFunctions.runWithEncoders(500, 0.25);

        autoRobotFunctions.navxRotateToDegree(-45.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        autoRobotFunctions.navXDriveStraight(-45.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, 0.50, 3900, 0.0005, 0.1,
                AutoRobotFunctions.StopConditions.COLOR, 20);

        //autoRobotFunctions.pusherActive(false);
        autoRobotFunctions.PIDLineFollow(7, 55, 0.15, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT);

        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        robot.robotBackward();
        autoRobotFunctions.runWithEncoders(1000, 0.35);

        robot.robotForward();
        //autoRobotFunctions.pusherActive(true);
        autoRobotFunctions.navxRotateToDegree(0.0, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        autoRobotFunctions.runWithEncoders(500, 0.35);

        autoRobotFunctions.navXDriveStraight(0.0, TOLERANCE_DEGREES, MIN_OUTPUT_DRIVE,
                MAX_OUTPUT_DRIVE, 0.5, 3100, 0.0005, 0.08,
                AutoRobotFunctions.StopConditions.COLOR, 20);
        /*robot.robotBackward();
        autoRobotFunctions.runWithEncoders(85, 0.2);

        robot.robotForward();*/
        //autoRobotFunctions.pusherActive(false);
        autoRobotFunctions.PIDLineFollow(7, 55, 0.15, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT);
        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        autoRobotFunctions.close();

        robot.setPoliceLED(false);
        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}

