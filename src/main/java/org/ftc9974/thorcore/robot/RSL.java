package org.ftc9974.thorcore.robot;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.internal.RealizableFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class RSL {

    private final List<LynxModule> displayModules;
    private boolean showingEnabled;

    @RealizableFactory
    public RSL(HardwareMap hw, String name) {
        displayModules = new ArrayList<>();
        displayModules.add(hw.get(LynxModule.class, name));
    }

    public RSL(Collection<? extends LynxModule> modules) {
        displayModules = new ArrayList<>(modules);
    }

    public RSL(LynxModule... modules) {
        displayModules = Arrays.asList(modules);
    }

    public void showEnabled() {
        for (LynxModule displayModule : displayModules) {
            displayModule.pushPattern(Arrays.asList(
                    new Blinker.Step(Color.rgb(255, 0, 0), 500, TimeUnit.MILLISECONDS),
                    new Blinker.Step(0, 500, TimeUnit.MILLISECONDS)
            ));
        }
        showingEnabled = true;
    }

    public void showDisabled() {
        if (showingEnabled) {
            for (LynxModule displayModule : displayModules) {
                displayModule.popPattern();
            }
            showingEnabled = false;
        }
    }
}
