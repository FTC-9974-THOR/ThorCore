package org.ftc9974.thorcore.internal;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;

public abstract class LynxI2cDeviceSynchEx extends LynxI2cDeviceSynchV2 {

    public LynxI2cDeviceSynchEx(final Context context, final LynxModule module, int bus) {
        super(context, module, bus);
    }

    public void writeSingleByte(byte b) {
        final Supplier<LynxI2cWriteSingleByteCommand> supplier = () -> new LynxI2cWriteSingleByteCommand(getModule(), bus, i2cAddr, b);
        try {
            acquireI2cLockWhile(() -> {
                sendI2cTransaction(supplier);
                waitForWriteCompletions(I2cWaitControl.ATOMIC);
                return null;
            });
        } catch (LynxNackException | InterruptedException | RobotCoreException e) {
            // not our problem, let the SDK handle it
            handleException(e);
        }
    }

    public void writeMultipleBytes(byte[] data) {
        final Supplier<LynxI2cWriteMultipleBytesCommand> supplier = () -> new LynxI2cWriteMultipleBytesCommand(getModule(), bus, i2cAddr, data);
        try {
            acquireI2cLockWhile(() -> {
                sendI2cTransaction(supplier);
                waitForWriteCompletions(I2cWaitControl.ATOMIC);
                return null;
            });
        } catch (LynxNackException | InterruptedException | RobotCoreException e) {
            // not our problem, let the SDK handle it
            handleException(e);
        }
    }

    public byte readSingleByte() {
        LynxCommand<?> command = new LynxI2cReadSingleByteCommand(getModule(), bus, i2cAddr);
        try {
            command.send();

            return pollForReadResult(i2cAddr, 0, 1).data[0];
        } catch (InterruptedException | LynxNackException e) {
            handleException(e);
        }
        // return placeholder
        return (byte) 0;
    }

    public byte[] readMultipleBytes(int numBytes) {
        LynxCommand<?> command = new LynxI2cReadMultipleBytesCommand(getModule(), bus, i2cAddr, numBytes);
        try {
            command.send();

            readTimeStampedPlaceholder.reset();
            return pollForReadResult(i2cAddr, 0, numBytes).data;
        } catch (InterruptedException | LynxNackException e) {
            handleException(e);
        }
        return null;
    }

    @Override
    public synchronized TimestampedData readTimeStamped(final int ireg, final int creg)
    {
        LynxI2cDeviceSynchEx deviceHavingProblems = null;

        try {
            return acquireI2cLockWhile(() -> {
                LynxCommand<?> tx = new LynxI2cWriteReadMultipleBytesCommand(getModule(), bus, i2cAddr, ireg, creg);
                tx.send();

                readTimeStampedPlaceholder.reset();
                return pollForReadResult(i2cAddr, ireg, creg);
            });
        } catch (InterruptedException|RobotCoreException|RuntimeException e) {
            handleException(e);
        } catch (LynxNackException e) {
            /*
             * This is a possible device problem, go ahead and tell makeFakeData to warn.
             */
            deviceHavingProblems = this;
            handleException(e);
        }
        return readTimeStampedPlaceholder.log(TimestampedI2cData.makeFakeData(getI2cAddress(), ireg, creg));
    }
}
