package org.ftc9974.thorcore.util;


import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public final class OpModeUtilities {

    public static FtcRobotControllerActivity getRootActivity() {
        return (FtcRobotControllerActivity) AppUtil.getInstance().getRootActivity();
    }

    public static OpModeManagerImpl getOpModeManager() {
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
        if (manager == null) {
            manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        }
        if (manager == null) {
            throw new RuntimeException("Unable to obtain an instance of OpModeManagerImpl. This is an internal error.");
        }
        return manager;
    }

    public static void registerListener(OpModeManagerImpl.Notifications listener) {
        getOpModeManager().registerListener(listener);
    }

    public static void unregisterListener(OpModeManagerImpl.Notifications listener) {
        getOpModeManager().unregisterListener(listener);
    }

    /*public static void changeConfiguration(String name) {
        RobotConfigFileManager manager = getRootActivity().getRobotConfigFileManager();
        List<RobotConfigFile> configFiles = manager.getXMLFiles();
        RobotConfigFile targetFile = null;
        for (RobotConfigFile configFile : configFiles) {
            if (configFile.getName().equals(name)) {
                targetFile = configFile;
                break;
            }
        }
        if (targetFile == null) {
            throw new IllegalArgumentException(String.format(Locale.getDefault(), "No configuration found with the name \"%s\"", name));
        }
        manager.setActiveConfig(targetFile);
    }*/
}
