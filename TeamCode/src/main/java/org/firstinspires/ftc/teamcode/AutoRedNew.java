package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher - Red", group = "Button")
//@Disabled
public class AutoRedNew extends LinearOpMode {

    private final byte NAVX_DIM_I2C_PORT = 1;
    private final int TOLERANCE_DEGREES = 2;

    private double MAX_OUTPUT_DRIVE = 1.0;
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

        autoRobotFunctions.setGyroDrivePID(0.33, 0.0008, 0.95);
        autoRobotFunctions.setGyroRotatePID(0.33, 0.0008, 0.95);

        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


        telemetry.addData("STATUS:", "init complete");
        telemetry.update();

        robot.shooterServo.setPosition(1.0);

        //wait for start of the match

        waitForStart();
        robot.setPoliceLED(true);

        robot.robotForward();
        //autoRobotFunctions.pusherActive(true);
        autoRobotFunctions.runWithEncoders(500, 1.0);

        autoRobotFunctions.MRRotate(-40, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);


        autoRobotFunctions.MRDriveStraight(-40, .75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES,
                AutoRobotFunctions.StopConditions.COLOR, 20);

        //int degree, double driveSpeed, double minOutputVal,
       // double maxOutputVal, int tolerance,
     //   AutoRobotFunctions.StopConditions stopCondition, int stopVal

        //autoRobotFunctions.pusherActive(false);
        autoRobotFunctions.PIDLineFollow(7, 55, 0.15, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT, 90, true);

        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        robot.robotBackward();
     //   autoRobotFunctions.runWithEncoders(1000, 1.0);


        //drive back more and shoot the ball
        autoRobotFunctions.MRDriveStraight(-90, .75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES,
                AutoRobotFunctions.StopConditions.ENCODER, 2500);

        robot.shooterPower(1.0);

        robot.shooterServo.setPosition(0.8);

        sleep(500);

        robot.shooterServo.setPosition(1.0);

        robot.shooterPower(0.0);


        robot.robotForward();
        //autoRobotFunctions.pusherActive(true);
        autoRobotFunctions.MRRotate(-20, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        autoRobotFunctions.runWithEncoders(500, 1.0);

        autoRobotFunctions.MRDriveStraight(0, .75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES,
                AutoRobotFunctions.StopConditions.COLOR, 20);
        /*robot.robotBackward();
        autoRobotFunctions.runWithEncoders(85, 0.2);

        robot.robotForward();*/
        //autoRobotFunctions.pusherActive(false);
        autoRobotFunctions.PIDLineFollow(7, 55, 0.15, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT, 90, true);
        autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED);

        autoRobotFunctions.close();

        robot.setPoliceLED(false);
        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}

