package org.steelhead.ftc;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Alec Matthews on 11/24/2016.
 * This class adds some graphics drawing routines for the led matrix mostly ported from the
 * Adafruit_GFX library.
 */

public class Adafruit_GFX {
    private Adafruit_LedMatrix ledMatrix;
    private HardwareMap hardwareMap;
    private int width, height;
    private int cursorX, cursorY;
    private Adafruit_LedMatrix.Color textColor = Adafruit_LedMatrix.Color.GREEN;
    private final char [] font;

    private boolean animationRunning = false;

    public Adafruit_GFX(HardwareMap hardwareMap, String ledMatrixName, int width, int height) {
        this.hardwareMap = hardwareMap;
        this.ledMatrix = new Adafruit_LedMatrix(hardwareMap, ledMatrixName);
        this.width = width;
        this.height = height;
        Font f = new Font();
        font = f.font;
    }

    public void clearDisplay() {
        ledMatrix.clearDisplay();
    }

    public void updateDisplay() {
        ledMatrix.updateDisplay();
    }

    public void close() {
        clearDisplay();
        updateDisplay();
        ledMatrix.close();
    }

    public void write(char c) {
        if (c == '\n') {
            cursorY += 8;
            cursorX = 0;
        } else if (c=='\r') {
            //Do nothing
        }else {
            drawChar((byte)cursorX,(byte) cursorY, c, textColor);
            cursorX += 6;
        }
    }

    public void print(String s) {
        int length = s.length();
        for (int i = 0; i < length; i++) {
            write(s.charAt(i));
        }
    }
    public void scrollText(String s) {
        try {
            for (int i = 7; i >= -(s.length() * 7); i--) {
                ledMatrix.clearDisplay();
                setCursor(i, 1);
                print(s);
                ledMatrix.updateDisplay();
                Thread.sleep(100);
            }
        }catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public void drawChar(byte x, byte y, char c, Adafruit_LedMatrix.Color color) {
        if ((x >= width) || (y >= height) || (x+6 < 0) || (y+8 < 0))
            return;

        for (int i = 0; i<6; i++) {
            char line;
            if (i < 5) {
                line = font[(5*(c-32))+i];
            }
            else {
                line = 0x0;
            }
            for (int j = 0; j<8; j++, line >>=1) {
                if ((line & 0x1) == 1) {
                    ledMatrix.drawPixel((byte)(x+i), (byte)(y+j), color);
                }
            }
        }
    }
    public void setCursor(int x, int y) {
        cursorX = x;
        cursorY = y;
    }
    public void setTextColor(Adafruit_LedMatrix.Color color) {
        this.textColor = color;
    }
    public void drawRect(byte x, byte y, byte w, byte h, Adafruit_LedMatrix.Color color) {
        drawFastHLine(x, y, w, color);
        drawFastHLine(x,(byte)(y+h-1), w, color);
        drawFastVLine(x, y, h, color);
        drawFastVLine((byte)(x+w-1), y, h, color);
    }
    public void animateBmp(int id, int numberOfFrames, long sleepTime, boolean isLoop) {
        int frame = 0;
        animationRunning = true;
        while (animationRunning) {
            drawBmpFromResource(id, frame);
            ledMatrix.updateDisplay();

            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            if (frame == numberOfFrames-1) {
                frame = 0;

                if (!isLoop) {
                    animationRunning = false;
                }
            } else {
                frame++;
            }
        }
    }
    public void drawBmp(byte  pic[], Adafruit_LedMatrix.Color color) {
        for (int y = 0; y < 8; y++) {
            byte line = pic[y];
            for (int x = 0; x < 8; x++, line>>=1) {
                if ((line & 0x01) == 1) {
                    ledMatrix.drawPixel(x, y, color);
                }
            }
        }
    }

    public void drawBmpFromResource(int id, int imagePos) {
        Context context = hardwareMap.appContext;
        BitmapFactory.Options options= new BitmapFactory.Options();
        options.inScaled = false;
        Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(), id, options);

        for(int y=0; y < 8; y++) {
            for (int x=imagePos*8; x < (imagePos*8)+8; x++) {
                int c = bitmap.getPixel(x, y);
                switch (c) {
                    case Color.WHITE:
                        ledMatrix.drawPixel(x-(8*imagePos), y, Adafruit_LedMatrix.Color.OFF);
                        break;
                    case Color.RED:
                        ledMatrix.drawPixel(x-(8*imagePos), y, Adafruit_LedMatrix.Color.RED);
                        break;
                    case Color.GREEN:
                        ledMatrix.drawPixel(x-(8*imagePos), y, Adafruit_LedMatrix.Color.GREEN);
                        break;
                    case Color.YELLOW:
                        ledMatrix.drawPixel(x-(8*imagePos), y, Adafruit_LedMatrix.Color.YELLOW);
                        break;
                }
            }
        }
    }
    public void stopAnimation() {
        animationRunning = false;
    }
    public void fillRect(int x, int y, int w, int h, Adafruit_LedMatrix.Color color) {
        for (int i=x; i<(x+w); i++) {
            drawFastVLine(i, y, h, color);
        }
    }
    public void fillScreen(Adafruit_LedMatrix.Color color) {
        fillRect((byte)0,(byte)0, width, height, color);
    }

    // Bresenham's algorithm I still need to figure out how this works
    public void drawLine(int x0, int y0, int x1, int y1, Adafruit_LedMatrix.Color color) {
        boolean steep = Math.abs(y1-y0) > Math.abs(x1-x0);
        if (steep) {
            int t = x0;
            x0 = y0;
            y0 = t;

            t=x1;
            x1 = y1;
            y1 = t;
        }
        if (x0 > x1) {
            int t = x0;
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
    public void drawFastHLine(int x, int y, int w, Adafruit_LedMatrix.Color color) {
        drawLine(x, y, x+w-1, y, color);
    }
    public void drawFastVLine(int x, int y, int h, Adafruit_LedMatrix.Color color) {
        drawLine(x, y, x, y+h-1, color);
    }
}
