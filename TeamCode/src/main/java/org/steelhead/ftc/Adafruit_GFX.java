package org.steelhead.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Alec Matthews on 11/24/2016.
 * This class adds some graphics drawing routines for the led matrix mostly ported from the
 * Adafruit_GFX library.
 */

public class Adafruit_GFX {
    private Adafruit_LedMatrix ledMatrix;
    private byte width, height;

    public Adafruit_GFX(Adafruit_LedMatrix ledMatrix, byte width, byte height) {
        this.ledMatrix = ledMatrix;
        this.width = width;
        this.height = height;
    }

    public void drawRect(byte x, byte y, byte w, byte h, Adafruit_LedMatrix.Color color) {
        drawFastHLine(x, y, w, color);
        drawFastHLine(x,(byte)(y+h-1), w, color);
        drawFastVLine(x, y, h, color);
        drawFastVLine((byte)(x+w-1), y, h, color);
    }
    public void fillRect(byte x, byte y, byte w, byte h, Adafruit_LedMatrix.Color color) {
        for (byte i=x; i<(x+w); i++) {
            drawFastVLine(i, y, h, color);
        }
    }
    public void fillScreen(Adafruit_LedMatrix.Color color) {
        fillRect((byte)0,(byte)0, width, height, color);
    }

    // Bresenham's algorithm I still need to figure out how this works
    public void drawLine(byte x0, byte y0, byte x1, byte y1, Adafruit_LedMatrix.Color color) {
        boolean steep = Math.abs(y1-y0) > Math.abs(x1-x0);
        if (steep) {
            byte t = x0;
            x0 = y0;
            y0 = t;

            t=x1;
            x1 = y1;
            y1 = t;
        }
        if (x0 > x1) {
            byte t = x0;
            x0 = x1;
            x1 = t;

            t=y0;
            y1 = y0;
            y0 = t;
        }
        int dx, dy;
        dx = x1 - x0;
        dy = Math.abs(y1-y0);

        int err = dx/2;
        byte yStep;

        if (y0 < y1) {
            yStep = 1;
        } else {
            yStep = -1;
        }

        for (; x0<=x1; x0++) {
            if (steep) {
                ledMatrix.drawPixel(y0, x0, color);
            } else {
                ledMatrix.drawPixel(x0, y0, color);
            }
            err -= dy;
            if (err < 0) {
                y0 += yStep;
                err += dx;
            }
        }
    }
    public void drawFastHLine(byte x, byte y, byte w, Adafruit_LedMatrix.Color color) {
        drawLine(x, y, (byte)(x+w-1), y, color);
    }
    public void drawFastVLine(byte x, byte y, byte h, Adafruit_LedMatrix.Color color) {
        drawLine(x, y, x,(byte)(y+h-1), color);
    }
}
