/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.HardwareSteelheadMainBot;
import org.steelhead.ftc.MenuCallable;
import org.steelhead.ftc.MenuItem;
import org.steelhead.ftc.TelemetryMenu;
import org.steelhead.ftc.TelemetryMenuSystem;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Cal: AutoConfig", group = "cal")
//@Disabled
public class ThresholdAutoConfig extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

    private TelemetryMenuSystem menuSystem;
    private TelemetryMenu startMenu;
    private TelemetryMenu whiteMenu;
    private TelemetryMenu blueMenu;
    private TelemetryMenu blackMenu;

    private MenuItem goBack;

    private Context appContext = null;
    private OpMode currentOpMode = this;
    private SharedPreferences sharedPref = null;
    private SharedPreferences.Editor editor = null;

    private final static String TAG = "CALIBRATION";

    @Override
    public void init() {
        //initialize the robot
        robot.init(hardwareMap);

        //Open SharedPreferences for editing
        sharedPref = robot.sharedPref;
        editor = sharedPref.edit();

        //get the app context for SharedPreferences
        appContext = hardwareMap.appContext;

        menuSystem = new TelemetryMenuSystem("Main Menu");
        startMenu   = new TelemetryMenu(this, gamepad1, "Choose a sensor to calibrate");
        whiteMenu = new TelemetryMenu(currentOpMode, gamepad1, "White Line Calibrate");
        blueMenu = new TelemetryMenu(currentOpMode, gamepad1, "Blue Color Calibrate");
        blackMenu = new TelemetryMenu(currentOpMode, gamepad1, "Black Color Calibrate");

        goBack = new MenuItem("Go Back", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                goBack.setSelected(false);
                menuSystem.goToMenu(0);
            }
        });

        //Set up the white menu
        whiteMenu.addItem(new MenuItem("Write Value", true, new MenuCallable() {
            @Override
            public void menuCallable() {
                int colorValue = (robot.color.red() + robot.color.green() + robot.color.blue()/3);
                editor.putInt(appContext.getString(R.string.White_Threshold), colorValue);
                editor.commit();
                Log.i(TAG, String.format("White color saved %d", colorValue));
                //menuSystem.goToMenu(0);
            }
        }));
        whiteMenu.addItem(new MenuItem("Clear Value", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                editor.remove(appContext.getString(R.string.White_Threshold));
                editor.commit();
                //menuSystem.goToMenu(0);
            }
        }));
        whiteMenu.addItem(goBack);

        //Set up the blue menu
        blueMenu.addItem(new MenuItem("Write Value", true, new MenuCallable() {
            @Override
            public void menuCallable() {
                int colorValue = robot.beaconColor.blueColor();
                editor.putInt(appContext.getString(R.string.Blue_Color), colorValue);
                editor.commit();
                Log.i(TAG, String.format("Beacon blue saved %d", colorValue));
                //menuSystem.goToMenu(0);
            }
        }));
        blueMenu.addItem(new MenuItem("Clear Value", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                editor.remove(appContext.getString(R.string.Blue_Color));
                editor.commit();
                //menuSystem.goToMenu(0);
            }
        }));
        blueMenu.addItem(goBack);

        //Set up the black menu
        blackMenu.addItem(new MenuItem("Write Value", true, new MenuCallable() {
            @Override
            public void menuCallable() {
                int colorValue = (robot.color.red() + robot.color.green() + robot.color.blue()/3);
                editor.putInt(appContext.getString(R.string.Black_Threshold), colorValue);
                editor.commit();
                Log.i(TAG, String.format("Black color saved %d", colorValue));
                //menuSystem.goToMenu(0);
            }
        }));
        blackMenu.addItem(new MenuItem("Clear Value", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                editor.remove(appContext.getString(R.string.Blue_Color));
                editor.commit();
                //menuSystem.goToMenu(0);
            }
        }));
        blackMenu.addItem(goBack);

        //Set up the start menu
        startMenu.addItem(new MenuItem("White Line", true, new MenuCallable() {
            @Override
            public void menuCallable() {
                menuSystem.goToMenu(1);
            }
        }));
        startMenu.addItem(new MenuItem("Blue Beacon Color", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                menuSystem.goToMenu(2);
            }
        }));
        startMenu.addItem(new MenuItem("Black Color", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                menuSystem.goToMenu(3);
            }
        }));

        //Add all of the menus to the menu system
        menuSystem.addMenu(startMenu);
        menuSystem.addMenu(whiteMenu);
        menuSystem.addMenu(blueMenu);
        menuSystem.addMenu(blackMenu);

        telemetry.addData("Status", "Initialized");
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        menuSystem.updateMenuSystem();
        telemetry.addData("Brightness", (robot.color.red() + robot.color.green() + robot.color.blue())/3);
        telemetry.addData("Blue Color", robot.beaconColor.blueColor());
    }
}
