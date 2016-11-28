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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.Adafruit_GFX;
import org.steelhead.ftc.Adafruit_LedMatrix;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Concept: LED Matrix", group = "Concept")
//@Disabled
public class MatrixTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Adafruit_LedMatrix ledMatrix;
    private Adafruit_GFX adafruitGfx;
    private Thread t;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        ledMatrix = new Adafruit_LedMatrix(hardwareMap, "matrix");
        adafruitGfx = new Adafruit_GFX(ledMatrix, (byte)8, (byte)8);
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {

        runtime.reset();
        adafruitGfx.drawChar((byte)1, (byte)1,'@', Adafruit_LedMatrix.Color.RED );
        ledMatrix.updateDisplay();
        adafruitGfx.setTextColor(Adafruit_LedMatrix.Color.RED);

         t = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    adafruitGfx.setTextColor(Adafruit_LedMatrix.Color.RED);
                    for (int i = 7; i >= -45; i--) {
                        ledMatrix.clearDisplay();
                        adafruitGfx.setCursor(i, 1);
                        adafruitGfx.print("Steelhead");
                        ledMatrix.updateDisplay();
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    adafruitGfx.setTextColor(Adafruit_LedMatrix.Color.YELLOW);
                    for (int i = 7; i >= -20; i--) {
                        ledMatrix.clearDisplay();
                        adafruitGfx.setCursor(i, 1);
                        adafruitGfx.print("8176");
                        ledMatrix.updateDisplay();
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    adafruitGfx.setTextColor(Adafruit_LedMatrix.Color.GREEN);

                    for (int i = 7; i >= -55; i--) {
                        ledMatrix.clearDisplay();
                        adafruitGfx.setCursor(i, 1);
                        adafruitGfx.print("MIG Sucks! ");
                        ledMatrix.updateDisplay();
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            }
        });
        t.start();
    }

    @Override
    public void loop() {

    }

}
