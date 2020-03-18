package org.ftc9974.thorcore.internal;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotation used to mark a method as a Realizable factory method. If you have a custom hardware
 * type that you want to be able to use {@link org.ftc9974.thorcore.meta.Realizer} to create, stick
 * this annotation above a method with the following signature:
 * {@code public [type of custom hardware class] [some name](String name, HardwareMap hardwareMap)}
 * Or, you can put it on a constructor that takes the same arguments.
 *
 * For an example of how this works, see {@link org.ftc9974.thorcore.robot.Motor}
 *
 * This is not the... ideal way to do this, but it's the only way I've found. :(
 *
 * @see org.ftc9974.thorcore.robot.Motor
 */
@Retention(RetentionPolicy.RUNTIME)
@Documented
@Target({ElementType.METHOD, ElementType.CONSTRUCTOR})
public @interface RealizableFactory {

}
