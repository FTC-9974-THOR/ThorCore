package org.ftc9974.thorcore.meta;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.junit.Assert;
import org.junit.Test;

import java.lang.reflect.Field;

public class RealizerTests {

    @Hardware
    DcMotor motor;

    @Hardware(name = "RT-motor2")
    DcMotor anotherMotor;

    int anInt;
    double aDouble;

    private class Class1 {
        double aDouble;
    }

    @Test
    public void nameFormatting_hardware_annotated_defaultName() throws Exception {
        Field field = this.getClass().getDeclaredField("motor");
        Assert.assertEquals("RT-motor", Realizer.getFormattedName(field));
    }

    @Test
    public void nameFormatting_hardware_annotated_customName() throws Exception {
        Field field = this.getClass().getDeclaredField("anotherMotor");
        Assert.assertEquals("RT-motor2", Realizer.getFormattedName(field));
    }

    @Test
    public void nameFormatting_primitiveInt() throws Exception {
        Field field = this.getClass().getDeclaredField("anInt");
        Assert.assertEquals("RT-anInt", Realizer.getFormattedName(field));
    }

    @Test
    public void nameFormatting_primitiveDouble() throws Exception {
        Field field = this.getClass().getDeclaredField("aDouble");
        Assert.assertEquals("RT-aDouble", Realizer.getFormattedName(field));
    }

    @Test
    public void nameFormatting_numbers() throws Exception {
        Class1 class1 = new Class1();
        Field field = class1.getClass().getDeclaredField("aDouble");
        Assert.assertEquals("C1-aDouble", Realizer.getFormattedName(field));
    }
}
