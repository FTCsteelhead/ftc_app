package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.Adafruit_GFX;
import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher - Blue 3", group = "Button")
//@Disabled
public class AutoBlue3 extends LinearOpMode {

    private final int TOLERANCE_DEGREES = 2;

    private double MAX_OUTPUT_DRIVE = 1.0;
    private double MIN_OUTPUT_DRIVE = 0.5;
    private double MAX_OUTPUT_ROTATE = 0.25;
    private double MIN_OUTPUT_ROTATE = -0.25;
    private double MAX_OUTPUT_LINE = 0.25;
    private double MIN_OUTPUT_LINE = -0.25;

    private AutoRobotFunctions autoRobotFunctions;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

        robot.init(hardwareMap);
        autoRobotFunctions = new AutoRobotFunctions(this, robot);

        autoRobotFunctions.setGyroDrivePID(0.018, 0.0001, 0.008);
        autoRobotFunctions.setGyroRotatePID(0.0327, 0.0005, 0.0008);

        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


        telemetry.addData("STATUS:", "init complete–check state of gyro");
        telemetry.update();

        //wait for start of the match
        robot.setPoliceLED(true);
        waitForStart();

        robot.robotForward();
        autoRobotFunctions.runWithEncoders(500, 1.0);

        autoRobotFunctions.MRRotate(-40, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        //check to see if we miss the line
        if (autoRobotFunctions.MRDriveStraight(-40, .75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 4000, 0.15,
                AutoRobotFunctions.StopConditions.COLOR, 20, 5500)) {


            autoRobotFunctions.PIDLineFollow(5, 45, 0.20, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                    AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT);

            autoRobotFunctions.pushButton(AutoRobotFunctions.Team.BLUE);

            robot.robotBackward();

            //shoot ball
            robot.shooterPower(0.7);
            robot.sweeperMotor.setPower(-1.0);

            autoRobotFunctions.runWithEncoders(2450, 1.0);

            robot.shooterServoDown(false);
            Thread.sleep(500);
            robot.shooterServoDown(true);
            Thread.sleep(800);
            robot.shooterServoDown(false);
            Thread.sleep(500);
            robot.shooterServoDown(true);
            robot.shooterPower(0.0);
            robot.sweeperMotor.setPower(0.0);

            robot.robotForward();
            autoRobotFunctions.MRRotate(-25, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

            autoRobotFunctions.MRDriveStraight(-25, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 3400, 0.15,
                    AutoRobotFunctions.StopConditions.COLOR, 20, -1);
            autoRobotFunctions.PIDLineFollow(5, 45, 0.20, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                    AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.LEFT);
            autoRobotFunctions.pushButton(AutoRobotFunctions.Team.BLUE);
            //if the robot misses the line do this

            robot.robotBackward();

            autoRobotFunctions.runWithEncoders(500, 1.0);

            robot.robotForward();

            autoRobotFunctions.MRRotate(-45, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

            robot.robotBackward();

            autoRobotFunctions.runWithEncoders(3000, 1.0);

        } else {
            telemetry.addData("You Missed the line!!", "<");
        }

        autoRobotFunctions.close();

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                Adafruit_GFX gfx = new Adafruit_GFX(hardwareMap, "matrix", 8, 8);
                while (opModeIsActive()) {
                    gfx.animateBmp(R.drawable.firework, 19, 130, false);
                }
                gfx.close();
            }
        });
        thread.start();

        robot.setPoliceLED(false);
        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}

