package org.ftc9974.thorcore.control;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.util.MathUtilities;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Implementation of a PIDF (Proportional, Integral, Derivative, Feed-forward) controller.
 *
 * To use, construct a new instance with {@link #PIDF(double, double, double, double)}.
 *
 * If you are using a phone with an Android API level higher than 24, you can bind functions as input
 * and output, much like WPILib.
 *
 * Input function:
 * {@code pidf.setInputFunction(() -> [input])}
 * ~or~
 * {@code pidf.setInputFunction(this::[function that returns input])}
 *
 * Output function is set the same way:
 * {@code pidf.setOutputFunction((output) -> [do something with output])}
 * ~or~
 * {@code pidf.setOutputFunction(this::[function that accepts output as a parameter])}
 *
 * For example, if you wanted to use a potentiometer to control a motor:
 * <code>
 *     AnalogInput pot = hardwareMap.analogInput.get("pot");
 *     DcMotor motor = hardwareMap.dcMotor.get("motor");
 *     PIDF pidf = new PIDF(1, 0, 0, 0);
 *     pidf.setInputFunction(pot::getVoltage);
 *     pidf.setOutputFunction(motor::setPower);
 * </code>
 *
 * To run the PIDF, you can use one of two functions:
 * <ul>
 *     <li>{@link #update()}, if you are using bindings</li>
 *     <li>{@link #update(double)} if you are not</li>
 * </ul>
 */
@SuppressWarnings({"WeakerAccess", "unused"})
public final class PIDF {

    private static final String TAG = "org.ftc9974.thorcore.control.PIDF";

    private double kP, kI, kD, kF;
    private double setpoint;
    private double runningIntegral;
    private double lastError;
    private double integralMin, integralMax;
    private double errorThreshold;
    private double lastInput;
    private double lastOutput;

    private DoubleSupplier inputFunc;
    private DoubleConsumer outputFunc;

    private long lastTime = -1;
    private long lastDTime = -1;

    private boolean continuous;
    private double contLow, contDiff, contHigh;
    private double direction;

    private boolean invertedPhase;

    private double peakOutputForward, peakOutputReverse;
    private double nominalOutputForward, nominalOutputReverse;

    private double period;
    private boolean periodAppliesOnlyToDTerm;
    private double lastDTerm;

    private double deadzone;

    /**
     * Construct a new PIDF controller.
     * @param coefficients PIDF coefficients
     */
    public PIDF(PIDFCoefficients coefficients) {
        this(coefficients, Double.MIN_VALUE, Double.MAX_VALUE);
    }

    /**
     * Construct a new PIDF controller.
     * @param coefficients PIDF coefficients
     * @param intMin minimum bound of the integral term
     * @param intMax maximum bound of the integral term
     */
    public PIDF(PIDFCoefficients coefficients, double intMin, double intMax) {
        this(coefficients.p, coefficients.i, coefficients.d, coefficients.f, intMin, intMax);
    }

    /**
     * Construct a new PIDF controller.
     * @param p p tuning
     * @param i i tuning
     * @param d d tuning
     * @param f f tuning
     */
    public PIDF(double p, double i, double d, double f) {
        this(p, i, d, f, Double.MIN_VALUE, Double.MAX_VALUE);
    }

    /**
     * Construct a new PIDF controller, with a bounded integral. The integral term will never
     * go outside the specified bounds.
     * @param p p tuning
     * @param i i tuning
     * @param d d tuning
     * @param f f tuning
     * @param intMin minimum bound of the integral
     * @param intMax maximum bound of the integral
     */
    public PIDF(double p, double i, double d, double f, double intMin, double intMax) {
        setTunings(p, i, d, f);
        integralMin = intMin;
        integralMax = intMax;
        contLow = -1;
        contDiff = 2;
        contHigh = 1;
        period = -1;
        periodAppliesOnlyToDTerm = false;
    }

    /**
     * Updates the PIDF tunings.
     * @param p p tuning
     * @param i i tuning
     * @param d d tuning
     * @param f f tuning
     */
    public void setTunings(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    /**
     * Sets the update frequency of the PIDF loop.
     * Set to any value less than or equal to zero
     * to allow the PIDF to update every time update()
     * is called;
     * @param period update frequency, in seconds
     */
    public void setPeriod(double period) {
        this.period = period;
    }

    public double getPeriod() {
        return period;
    }

    public void setAtTargetThreshold(double threshold) {
        errorThreshold = threshold;
    }

    public double getAtTargetThreshold() {
        return errorThreshold;
    }

    // this doesn't work i have no clue why it doesn't
    public boolean atTarget() {
        return Math.abs(lastError) < errorThreshold && lastTime != -1;
    }

    /**
     * Binds an input function. This function must take no parameters, and return input as a double.
     * @param input input function
     */
    @TargetApi(Build.VERSION_CODES.N)
    public void setInputFunction(DoubleSupplier input) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
            RobotLog.ee(TAG, "IO bindings require API level 24+");
            return;
        }
        inputFunc = input;
    }

    /**
     * Binds an output function. This function must take a single double parameter (the PIDF's
     * output) and return nothing.
     * @param output output function
     */
    @TargetApi(Build.VERSION_CODES.N)
    public void setOutputFunction(DoubleConsumer output) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
            RobotLog.ee(TAG, "IO bindings require API level 24+");
            return;
        }
        outputFunc = output;
    }

    public void setContinuous(boolean isContinuous) {
        this.continuous = isContinuous;
    }

    public void setContinuityRange(double low, double high) {
        contLow = low;
        contHigh = high;
        contDiff = contHigh - contLow;
    }

    private double getContinuousError(double error) {
        if (continuous && contDiff > 0) {
            error %= contDiff;
            if (Math.abs(error) > contDiff / 2) {
                if (error > 0) {
                    return error - contDiff;
                } else {
                    return error + contDiff;
                }
            }
        }

        return error;
    }

    /**
     * Sets the setpoint of the PIDF.
     * @param setpoint setpoint
     */
    public void setSetpoint(double setpoint) {
        if (continuous) {
            this.setpoint = MathUtilities.wraparound(setpoint, contLow, contHigh);
        } else {
            this.setpoint = setpoint;
        }
        // This is important! This updates the atTarget()
        // logic. Without it, atTarget() can return false
        // positives, due to lastError not being updated.
        lastError = getContinuousError(setpoint - lastInput);
    }

    /**
     * Runs the PIDF using bindings.
     * @return output of the PIDF
     */
    public double update() {
        if (inputFunc == null) {
            RobotLog.ww(TAG, "Input function is not set!");
            return 0;
        }
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
            RobotLog.ee(TAG, "SDK level must be >= 24 to use IO bindings");
            return 0;
        } else {
            double ret = update(inputFunc.getAsDouble());
            if (outputFunc != null) {
                outputFunc.accept(ret);
            }
            return ret;
        }
    }

    /**
     * Runs the PIDF, without bindings.
     * @param input input to the PIDF (aka process variable)
     * @return output of the PIDF
     */
    public double update(double input) {
        double error = getContinuousError(((invertedPhase) ? -1 : 1) * (setpoint - input));
        if (Math.abs(error) < errorThreshold) {
            lastError = error;
            return 0;
        }

        long currentTime = System.nanoTime();
        if (lastTime < 0) {
            lastTime = currentTime;
        }
        double deltaTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (deltaTime >= period) {
            double pComponent = 0, iComponent = 0, dComponent = 0;

            pComponent = kP * error;

            if (kI != 0) {
                runningIntegral += error * deltaTime;
                runningIntegral = Range.clip(runningIntegral, integralMin, integralMax);
                iComponent = kI * runningIntegral;
            }

            if (kD != 0 && deltaTime != 0) {
                double d = (error - lastError) / deltaTime;
                dComponent = kD * d;
            }

            double fComponent = kF;

            double output = pComponent + iComponent + dComponent + fComponent;

            if (output > 0) {
                // forward
                if (peakOutputForward != 0 && output > peakOutputForward) {
                    output = peakOutputForward;
                } else if (output < nominalOutputForward) {
                    output = nominalOutputForward;
                }
            } else if (output < 0) {
                // reverse
                if (peakOutputReverse != 0 && output < peakOutputReverse) {
                    output = peakOutputReverse;
                } else if (output > nominalOutputReverse) {
                    output = nominalOutputReverse;
                }
            }

            lastError = error;
            //lastTime = SystemClock.uptimeMillis();
            lastInput = input;
            if (Math.abs(error) < errorThreshold) {
                lastOutput = 0;
            } else {
                lastOutput = output;
            }
        }
        return lastOutput;
    }

    /**
     * Resets the integral term.
     *
     * If you stop using the PIDF, then need it again, call this method. As time goes on, the
     * integral can grow indefinitely - something called integral windup. If the integral winds up
     * while the PIDF is not in use, it will overpower all other terms when the PIDF is used again.
     * Call this method to remove the integral windup.
     */
    public void resetControl() {
        runningIntegral = 0;
        lastTime = SystemClock.uptimeMillis();
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getLastError() {
        return lastError;
    }

    /**
     * Sets the phasing of the PIDF.
     * Use this if a positive output does not correspond to the input signal increasing.
     * @param phase true if output and input are not in phase, false otherwise
     */
    public void setPhase(boolean phase) {
        invertedPhase = phase;
    }

    public double getPeakOutputForward() {
        return peakOutputForward;
    }

    public void setPeakOutputForward(double peakOutputForward) {
        this.peakOutputForward = peakOutputForward;
    }

    public double getPeakOutputReverse() {
        return peakOutputReverse;
    }

    public void setPeakOutputReverse(double peakOutputReverse) {
        this.peakOutputReverse = peakOutputReverse;
    }

    public double getNominalOutputForward() {
        return nominalOutputForward;
    }

    public void setNominalOutputForward(double nominalOutputForward) {
        this.nominalOutputForward = nominalOutputForward;
    }

    public double getNominalOutputReverse() {
        return nominalOutputReverse;
    }

    public void setNominalOutputReverse(double nominalOutputReverse) {
        this.nominalOutputReverse = nominalOutputReverse;
    }
}
