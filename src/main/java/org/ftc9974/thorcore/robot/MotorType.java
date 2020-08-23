package org.ftc9974.thorcore.robot;

public enum MotorType {
    NEVEREST_3_7(103), // 7 pulses per magnet * 3.7 gear ratio * 4 edges per pulse
    NEVEREST_20(7 * 20 * 4), // 7 pulses per magnet * 20 gear ratio * 4 edges per pulse
    NEVEREST_40(7 * 40 * 4), // 7 pulses per magnet * 40 gear ratio * 4 edges per pulse
    NEVEREST_60(7 * 60 * 4), // 7 pulses per magnet * 60 gear ratio * 4 edges per pulse

    TORQUENADO(1440),
    USDIGITAL(1440), // encoder for TETRIX MAX motors. Not sure on this value, will double check it
                           // later

    CORE_HEX(288),
    HD_HEX_20(7 * 20 * 4), // Ticks per revolution is actually not listed on REV's website, so I have no clue
                                 // what the actual value is. I assume it's the same as the Neverest motors.
    HD_HEX_40(7 * 40 * 4),

    YELLOWJACKET_3_7((int) (7 * 4 * 3.7)),   // 1620 RPM
    YELLOWJACKET_5_2((int) (7 * 4 * 5.2)),   // 1150 RPM
    YELLOWJACKET_13_7((int) (7 * 4 * 13.7)), //  435 RPM
    YELLOWJACKET_19_2((int) (7 * 4 * 19.2)), //  312 RPM
    YELLOWJACKET_26_9((int) (7 * 4 * 26.9)), //  223 RPM
    YELLOWJACKET_50_9((int) (7 * 4 * 50.9)), //  117 RPM
    YELLOWJACKET_71_2((int) (7 * 4 * 71.2)), //   84 RPM
    YELLOWJACKET_99_5((int) (7 * 4 * 99.5)), //   60 RPM
    YELLOWJACKET_139((int) (7 * 4 * 139)),   //   43 RPM
    YELLOWJACKET_188((int) (7 * 4 * 188));   //   30 RPM

    public int ticksPerRevolution;

    MotorType(int ticks) {
        ticksPerRevolution = ticks;
    }
}
