package org.ftc9974.thorcore.seasonal.powerplay;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.vision.NEONVision;
import org.ftc9974.thorcore.vision.Seeker;

import java.io.IOException;

public class PowerPlaySeeker extends Seeker {

    public final Signature blueConeAndLine = new Signature(
            NEONVision.yuvColorLong(0xff, 0xff, 0x80),
            NEONVision.yuvColorLong(0x00, 0x8f, 0x00),
            25000 / (1080.0 * 720.0),
            2
    );
    public final Signature redConeAndLine = new Signature(
            NEONVision.yuvColorLong(0xff, 0x7b, 0xff),
            NEONVision.yuvColorLong(0x2a, 0x4f, 0x99),
            25000 / (1080.0 * 720.0),
            2
    );
    public final Signature pole = new Signature(
            NEONVision.yuvColorLong(0xff, 0x4f, 0xff),
            NEONVision.yuvColorLong(0x80, 0x00, 0x2f),
            10000 / (1080.0 * 720.0),
            2
    );

    @RealizableFactory
    public PowerPlaySeeker(String name, HardwareMap hw) throws IOException {
        super(name, hw);

        registerSignatures(blueConeAndLine, redConeAndLine, pole);
    }
}
