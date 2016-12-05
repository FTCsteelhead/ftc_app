package org.steelhead.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.concurrent.locks.Lock;

/**
 * Created by Alec Matthews on 11/23/2016.
 * This is a driver for the adafruit 8x8 bi-color led matrix. It uses the Holtek HT16K33 as an i2c
 * to multiplexer converter. It supports different blink speeds and a low power sleep mode.
 */

//// TODO: 11/25/16 Add methods to turn off the display and put it into sleep mode

public class Adafruit_LedMatrix implements I2cController.I2cPortReadyCallback {

    static final short
            ADDRESS = 0xe0, //Device address

            SYSTEM_SETUP = 0x20, //System setup command
            SYSTEM_ON = 0x01,
            SYSTEM_OFF = 0x00,

            DISPLAY_SETUP = 0x80, //Display setup command
            DISPLAY_ON = 0x01,
            DISPLAY_OFF = 0x00,
            BLINK_OFF = 0x00,
            BLINK_2HZ = 0x02,
            BLINK_1HZ = 0x04,
            BLINK_HALF_HZ = 0x06,

            BRIGHTNESS_COMMAND = 0xe0, //Brightness command
            SCREEN_RAM_START  = 0x00;

    public enum BlinkRate {
        BLINK_OFF,
        BLINK_2HZ,
        BLINK_1HZ,
        BLINK_HALF_HZ
    }

    public enum Color {
        RED,
        GREEN,
        YELLOW,
        OFF
    }

    static final byte
            READ_MODE   = 0x00 - 128,
            WRITE_MODE  = 0x00;

    static final int
            CACHE_MODE  = 0,
            DEV_ADDR    = 1,
            REG_ADDR    = 2,
            REG_COUNT   = 3,
            DATA_OFFSET = 4,
            ACTION_FLAG = 31;

    private ArrayQueue<I2cTransferHT16K33> transferQueue;
    private I2cDevice               dev;
    private short                   devAddr;
    private byte[]                  rCache;
    private byte[]                  wCache;
    private Lock                    rLock;
    private Lock                    wLock;
    private int[]                 ramBuffer;

    public Adafruit_LedMatrix(HardwareMap hardwareMap, String deviceName) {
        transferQueue = new ArrayQueue<>();
        dev = hardwareMap.i2cDevice.get(deviceName);
        devAddr = ADDRESS;

        rCache = dev.getI2cReadCache();
        wCache = dev.getI2cWriteCache();
        rLock = dev.getI2cReadCacheLock();
        wLock = dev.getI2cWriteCacheLock();

        ramBuffer = new int[8];

        addCommandRequest((byte)(SYSTEM_SETUP | SYSTEM_ON));
        addCommandRequest((byte)(DISPLAY_SETUP | DISPLAY_ON));
        executeCommands();
        dev.registerForI2cPortReadyCallback(this);
    }

    public void setBrightness(int brightness) {
        if (brightness > 15)
            brightness = 15;
        addCommandRequest((byte)(BRIGHTNESS_COMMAND | brightness));
    }
    public void setBlinkRate(BlinkRate blinkRate) {
        switch (blinkRate) {
            case BLINK_OFF:
                addCommandRequest((byte)(DISPLAY_SETUP | DISPLAY_ON));
                break;
            case BLINK_1HZ:
                addCommandRequest((byte)(DISPLAY_SETUP | DISPLAY_ON | BLINK_1HZ));
                break;
            case BLINK_2HZ:
                addCommandRequest((byte)(DISPLAY_SETUP | DISPLAY_ON | BLINK_2HZ));
                break;
            case BLINK_HALF_HZ:
                addCommandRequest((byte)(DISPLAY_SETUP | DISPLAY_ON | BLINK_HALF_HZ));
                break;
        }
    }

    public void clearDisplay() {
        for (int i=0; i<8; i++) {
            ramBuffer[i] = 0;
        }
    }

    public void drawPixel(int x, int y, Color color) {
        if ((y < 0) || (x >= 8)) return;
        if ((x < 0) || (x >= 8)) return;

        //// TODO: 11/23/2016 Look into rotation
        switch (color) {
            case GREEN:
                //set green LED
                ramBuffer[y] |= 1<<x;
                //Turn off red LED
                ramBuffer[y] &= ~(1<<x+8);
                break;
            case RED:
                //Turn on Red LED
                ramBuffer[y] |= 1 << (x+8);
                //Turn off Green LED
                ramBuffer[y] &= ~(1 << x);
                break;
            case YELLOW:
                //Turn Green and red on
                ramBuffer[y] |= (1 << (x+8) | (1 << x));
                break;
            case OFF:
                ramBuffer[y] &= ~(1 << x) & ~(1 << (x+8));
                break;
        }
    }

    public void updateDisplay() {
        addUpdateScreenRequest();
    }

    public void close() {
        transferQueue.close();
        dev.deregisterForPortReadyCallback();
        dev.close();
    }
    @Override
    public void portIsReady(int port) {
        try {
            rLock.lock();
            if (rCache[0] == wCache[0] && rCache[1] == wCache[1] && rCache[2] == wCache[2] &&
                    rCache[3] == wCache[3]) {
                rCache[DEV_ADDR] = 0;

                executeCommands();
            } else {
                dev.readI2cCacheFromController();
            }
        } finally {
            rLock.unlock();
        }
    }

    private void executeCommands() {
        boolean isWrite = false;
        boolean isCommandTransfer = false;
        short regAddr = 0;

        try {
            wLock.lock();
            if (!transferQueue.isEmpty()) {
                I2cTransferHT16K33 element = transferQueue.remove();
                isWrite = (element.mode == WRITE_MODE);
                regAddr = element.regAddr;
                isCommandTransfer = element.isCommandTransfer;
                element = null;
            }
        } finally {
            wLock.unlock();
        }
        if (isWrite) {
            if (isCommandTransfer) {
                writeCommand(regAddr);
            }else {
                writeDisplay(ramBuffer);
            }
        }
    }

    private void writeCommand(short regAddr) {
        try {
            wLock.lock();
            wCache[CACHE_MODE] = WRITE_MODE;
            wCache[DEV_ADDR] = (byte)(devAddr & 0xFF);
            wCache[REG_ADDR] = (byte)(regAddr & 0xFF);
            wCache[REG_COUNT] = 1;

            wCache[ACTION_FLAG] = -1;
        } finally {
            wLock.unlock();
        }
        dev.writeI2cCacheToController();
    }
    private void writeDisplay(int[] ramBuffer) {
        try {
            wLock.lock();
            wCache[CACHE_MODE] = WRITE_MODE;
            wCache[DEV_ADDR] = (byte)(devAddr & 0xFF);
            wCache[REG_ADDR] = (byte)(SCREEN_RAM_START & 0xFF);
            wCache[REG_COUNT] = (byte)16;
            int offsetCounter = 0;
            for (int i = 0; i<8; i++) {
                wCache[DATA_OFFSET + offsetCounter] = (byte)(ramBuffer[i] & 0xFF);
                offsetCounter++;
                wCache[DATA_OFFSET + offsetCounter] = (byte)((ramBuffer[i]>>8) & 0xFF);
                offsetCounter++;
            }
            wCache[ACTION_FLAG] = -1;
        } finally {
            wLock.unlock();
        }
        dev.writeI2cCacheToController();
    }

    private void addCommandRequest(byte regAddr) {
        addRequest(new I2cTransferHT16K33(regAddr, true));
    }

    private void addUpdateScreenRequest() {

        addRequest(new I2cTransferHT16K33(false));
    }

    private void addRequest(I2cTransferHT16K33 element) {
        try {
            rLock.lock();
            try {
                wLock.lock();
                transferQueue.add(element);
            } finally {
                wLock.unlock();
            }
        } finally {
            rLock.unlock();
        }
    }

}
