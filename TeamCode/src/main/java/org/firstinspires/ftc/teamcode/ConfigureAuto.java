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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.MenuCallable;
import org.steelhead.ftc.MenuItem;
import org.steelhead.ftc.TelemetryMenu;
import org.steelhead.ftc.TelemetryMenuSystem;

/**
 * Displays a telemetry menu
 */
@Autonomous(name = "Configure: Auto", group = "Configure")
//@Disabled
public class ConfigureAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();


    private TelemetryMenuSystem menuSystem;
    private TelemetryMenu firstMenu;
    private TelemetryMenu displayMenu;
    private OpMode opMode = this;

    @Override
    public void init() {

        menuSystem = new TelemetryMenuSystem("System");

        firstMenu = new TelemetryMenu(this, gamepad1, "Choose Team");

        firstMenu.addItem(new MenuItem("Red Team", true, new MenuCallable() {
            @Override
            public void menuCallable() {

                Context context = hardwareMap.appContext;

                SharedPreferences sharedPref = context.getSharedPreferences(
                        context.getString(R.string.AutoPreferences), Context.MODE_PRIVATE);
                SharedPreferences.Editor editor = sharedPref.edit();
                editor.putString(context.getString(R.string.Team), "Red");
                editor.commit();

                sharedPref.getString(context.getString(R.string.Team), "No Team");

                //menu-seption Menu inside a menu
                displayMenu = new TelemetryMenu(opMode, gamepad1,
                        sharedPref.getString(context.getString(R.string.Team), "No Team"));

                displayMenu.addItem(new MenuItem("Done", true, new MenuCallable() {
                    @Override
                    public void menuCallable() {
                        requestOpModeStop();
                    }
                }));
                menuSystem.addMenu(displayMenu);
                menuSystem.advanceMenu();
            }
        }));

        firstMenu.addItem(new MenuItem("Blue Team", false, new MenuCallable() {
            @Override
            public void menuCallable() {
                Context context = hardwareMap.appContext;
                SharedPreferences sharedPref = context.getSharedPreferences(
                        context.getString(R.string.AutoPreferences), Context.MODE_PRIVATE);
                SharedPreferences.Editor editor = sharedPref.edit();
                editor.putString(context.getString(R.string.Team), "Blue");
                editor.commit();


                sharedPref.getString(context.getString(R.string.Team), "No Team");

                displayMenu = new TelemetryMenu(opMode, gamepad1,
                        sharedPref.getString(context.getString(R.string.Team), "No Team"));

                displayMenu.addItem(new MenuItem("Done", true, new MenuCallable() {
                    @Override
                    public void menuCallable() {
                        requestOpModeStop();
                    }
                }));
                menuSystem.addMenu(displayMenu);
                menuSystem.advanceMenu();
            }
        }));

        menuSystem.addMenu(firstMenu);
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

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        menuSystem.updateMenuSystem();
    }
}
