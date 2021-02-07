package org.ftc9974.thorcore.control;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.ftc9974.thorcore.util.MathUtilities;

import java.util.List;

/**
 * Allows use of the robot's battery voltage sensor to output a (relatively) constant voltage on a
 * motor port.
 *
 * This class can be used wherever motors need to stay at the same speed regardless of battery
 * voltage, such as the driving during autonomous. It can even be used to control the speed of a
 * flywheel without oscillations.
 */
public class VoltageRegulator {

    private final VoltageSensor voltageSensor;
    private double targetVoltage;

    /**
     * Constructs a new VoltageRegulator that will use the REV Hub's internal voltage sensor to
     * measure battery voltage.
     *
     * @param hw the hardware map
     */
    public VoltageRegulator(HardwareMap hw) {
        List<LynxVoltageSensor> voltageSensors = hw.getAll(LynxVoltageSensor.class);
        if (voltageSensors.isEmpty()) {
            // this shouldn't be able to happen on a REV Hub, since the LynxVoltageSensor is built
            // into the Hub. the Modern Robotics Motor Controllers and the NXT Motor Controllers
            // have a similar sensor, but they aren't legal for competition anymore.
            throw new RuntimeException("Could not automatically find a battery voltage sensor. Please specify one in the constructor.");
        }
        // get the first sensor. on dual-hub bots it would theoretically be better to use the one
        // on the same hub as the motor port this VoltageRegulator will be used with, to ensure you
        // get the right battery voltage, but the hubs are powered off of the same bus. If the
        // battery voltage on one hub is different from the voltage on the other hub, you have much
        // bigger problems to worry about than this class not working correctly.
        voltageSensor = voltageSensors.get(0);
    }

    /**
     * Constructs a new VoltageRegulator that will use the specified voltage sensor to measure
     * battery voltage.
     *
     * @param voltageSensor battery voltage sensor
     */
    public VoltageRegulator(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    /**
     * Sets the target voltage this regulator will attempt to regulate to.
     *
     * In theory, this can be any number, but in practice you shouldn't go above 15V or below -15V.
     * This is because FTC batteries are 12V nominal, 14.9V peak, and the motor controllers can't
     * output a voltage higher than the battery voltage.
     *
     * @param targetVoltage voltage, in volts.
     */
    public void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
    }

    /**
     * Gets the target voltage of this regulator.
     *
     * @return voltage, in volts.
     */
    public double getTargetVoltage() {
        return targetVoltage;
    }

    /**
     * Returns a regulated power that, when given to a motor, produces the target voltage at the
     * motor port.
     *
     * @return power, in the range [-1, 1]
     */
    public double getRegulatedOutput() {
        return MathUtilities.constrain(targetVoltage / voltageSensor.getVoltage(), -1, 1);
    }

    /**
     * Returns a regulated power that, when given to a motor, produces the specified target voltage
     * at the motor port, and stores the provided target voltage as the new target voltage.
     *
     * Choosing between this method and {@link #getRegulatedOutput()} is largely a choice of
     * semantics, as there's no difference between this method and calling {@link #setTargetVoltage(double)}
     * before calling getRegulatedOutput().
     *
     * @param targetVoltage voltage to regulate to
     * @return power, in the range [-1, 1]
     */
    public double getRegulatedOutput(double targetVoltage) {
        setTargetVoltage(targetVoltage);
        return getRegulatedOutput();
    }

    /**
     * Gets the voltage of the robot battery.
     *
     * @return voltage, in volts.
     */
    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }
}
