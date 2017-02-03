package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.Adafruit_GFX;
import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher - 2", group = "Button")
//@Disabled
public class Auto2 extends LinearOpMode {

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

        robot.shooterServo.setPosition(1.0);

        //wait for start of the match
        robot.setPoliceLED(true);
        waitForStart();

        robot.robotForward();

        robot.shooterPower(0.7);

        autoRobotFunctions.runWithEncoders(1000, 1.0);

        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        Thread.sleep(800);

        robot.sweeperMotor.setPower(-1.0);

        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        robot.shooterPower(0.0);
        robot.sweeperMotor.setPower(0.0);

        autoRobotFunctions.MRDriveStraight(0, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 3500, 0.15,
                AutoRobotFunctions.StopConditions.COLOR, 10, -1);

        autoRobotFunctions.runWithEncoders(500, 1.0);

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

