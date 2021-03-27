package org.ftc9974.thorcore.meta;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.meta.annotation.EEVal;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.meta.annotation.Namespace;
import org.ftc9974.thorcore.meta.annotation.Realized;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Locale;

/**
 * Used to "realize" fields marked with {@code @Hardware}/{@code @EEVal}.
 */
public final class Realizer {

    private Realizer() {}

    private static final String TAG = "org.ftc9974.thorcore.meta.Realizer";

    /**
     * Realizes an instance object.
     *
     * Every field marked with {@code @Hardware} is initialized from the hardwareMap, following the
     * ThorCore naming convention.
     *
     * Every field marked with {@code @EEVal} is initialized from EEVals - that is, they are loaded
     * from persistent storage.
     *
     * The ThorCore naming convention is as follows:
     * [All capital letters and numbers in class name]-[variable name]
     * Some examples:
     * <table border="1" summary="Naming convention">
     *     <tr>
     *         <td><b>Class name</b></td><td><b>Variable name</b></td><td><b>Name</b></td>
     *     </tr>
     *     <tr>
     *         <td>TeleOp</td><td>intakeMotor</td><td>TO-intake</td>
     *     </tr>
     *     <tr>
     *         <td>LandAuto1</td><td>colorSensor</td><td>LA1-colorSensor</td>
     *     </tr>
     *     <tr>
     *         <td>GarretsRedTripleGlyphTest</td><td>frontLeftDriveMotor</td><td>GRTGT-frontLeftDriveMotor</td>
     *     </tr>
     * </table>
     * @param obj object to realize
     * @param hardwareMap hardwareMap
     */
    @SuppressWarnings({"unchecked", "deprecation"})
    public static void realize(Object obj, HardwareMap hardwareMap) {
        EEVals.init(hardwareMap.appContext);
        Class clazz = obj.getClass();
        Field[] fields = clazz.getDeclaredFields();
        boolean realizedMarked = clazz.isAnnotationPresent(Realized.class);
        for (Field field : fields) {
            if (field.isAnnotationPresent(Hardware.class) || realizedMarked) {
                try {
                    String formattedName = getFormattedName(field);
                    RobotLog.dd(TAG, String.format(Locale.getDefault(), "Attempting to find factory method or constructor for type %s", field.getType().getSimpleName()));
                    Method factory = findRealizableFactoryMethod(field.getType());
                    if (factory != null) {
                        RobotLog.dd(TAG, "Factory method found");
                        field.set(obj, factory.invoke(null, formattedName, hardwareMap));
                    } else {
                        Constructor constructor = findRealizableFactoryConstructor(field.getType());
                        if (constructor != null) {
                            RobotLog.vv(TAG, "Factory constructor found");
                            field.set(obj, constructor.newInstance(formattedName, hardwareMap));
                        } else {
                            RobotLog.dd(TAG, String.format(Locale.getDefault(), "No factory method found. Using HardwareMap, looking for \"%s\" with type %s", formattedName, field.getType().getSimpleName()));
                            field.set(obj, hardwareMap.get(field.getType(), formattedName));
                        }
                    }
                } catch (IllegalAccessException e) {
                    RobotLog.ee(TAG, e, "Cannot access annotated field \"%s\". Please make sure the field is not private.", getFormattedName(field));
                    RobotLog.setGlobalErrorMsg("Cannot access annotated field \"%s\". Please make sure the field is not private.", getFormattedName(field));
                } catch (IllegalArgumentException e) {
                    RobotLog.ee(TAG, "Field \"%s\" is not in HardwareMap", getFormattedName(field));
                    RobotLog.setGlobalErrorMsg(String.format(Locale.getDefault(), "Could not find hardware with name \"%s\". Check your configuration file.", getFormattedName(field)));
                    throw e;
                } catch (InvocationTargetException e) {
                    // don't fail silently
                    //RobotLog.ee(TAG, "An internal error has ocurred: error invoking constructor for \"%s\"", field.getType().getSimpleName());
                    throw new RuntimeException(String.format(Locale.getDefault(), "Error invoking constructor for %s: %s", field.getType().getSimpleName(), e.getTargetException().toString()));
                } catch (InstantiationException e) {
                    // don't fail silently
                    //RobotLog.ee(TAG, "An internal error has ocurred: error invoking constructor for \"%s\"", field.getType().getSimpleName());
                    throw new RuntimeException(String.format(Locale.getDefault(), "Error invoking constructor for %s: %s", field.getType().getSimpleName(), e.toString()));
                }
            }
            if (field.isAnnotationPresent(EEVal.class) || realizedMarked) {
                String prefix = "";
                if (clazz.isAnnotationPresent(Namespace.class)) {
                    prefix = ((Namespace) clazz.getAnnotation(Namespace.class)).value() + "/";
                }
                try {
                    field.set(obj, EEVals.get(prefix + getFormattedName(field), field.getType()));
                } catch (IllegalAccessException e) {
                    RobotLog.ee(TAG, e, "Cannot access annotated field \"%s\"", getFormattedName(field));
                } catch (IllegalArgumentException e) {
                    RobotLog.ii(TAG, "Field \"%s\" is not in EEVals, skipping", getFormattedName(field));
                }
            }
        }
    }

    private static Method findRealizableFactoryMethod(Class type) {
        RobotLog.vv(TAG, String.format(Locale.getDefault(), "[findRealizableFactoryMethod] scanning type %s", type.getSimpleName()));
        for (Method m : type.getDeclaredMethods()) {
            RobotLog.vv(TAG, String.format(Locale.getDefault(), "Looking at method %s", m.getName()));
            if (m.isAnnotationPresent(RealizableFactory.class)) {
                return m;
            }
        }
        return null;
    }

    private static Constructor findRealizableFactoryConstructor(Class type) {
        RobotLog.vv(TAG, String.format(Locale.getDefault(), "[findRealizableFactoryConstructor] scanning type %s", type.getSimpleName()));
        for (Constructor c : type.getDeclaredConstructors()) {
            RobotLog.vv(TAG, String.format(Locale.getDefault(), "Looking at constructor %s", c.getName()));
            if (c.isAnnotationPresent(RealizableFactory.class)) {
                return c;
            }
        }
        return null;
    }

    static String getFormattedName(Field field) {
        if (field.isAnnotationPresent(Hardware.class)) {
            Hardware annotation = field.getAnnotation(Hardware.class);
            if (!annotation.name().equals("")) {
                return annotation.name();
            }
        }
        return String.format("%s-%s",
                field.getDeclaringClass().getSimpleName().replaceAll("[^A-Z0-9]", ""),
                field.getName());
    }
}
