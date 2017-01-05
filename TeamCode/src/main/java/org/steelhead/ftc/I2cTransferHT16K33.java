package org.steelhead.ftc;

/**
 * Created by Alec Matthews on 11/23/2016.
 * This add a structure for the two different types of transfers that need to be done for the led
 * matrix driver chip
 */

public class I2cTransferHT16K33 {
    static final byte
            READ_MODE = 0x00-128,
            WRITE_MODE = 0x00;

    public byte mode;
    public byte regAddr;
    public boolean isCommandTransfer;

    public I2cTransferHT16K33 (byte regAddr, boolean isCommandTransfer) {
        this.regAddr = regAddr;
        this.isCommandTransfer = isCommandTransfer;
        this.mode = WRITE_MODE;
    }

    public I2cTransferHT16K33 (boolean isCommandTransfer) {
        this.regAddr = 0x00;
        this.isCommandTransfer = isCommandTransfer;
        this.mode = WRITE_MODE;
    }
}
