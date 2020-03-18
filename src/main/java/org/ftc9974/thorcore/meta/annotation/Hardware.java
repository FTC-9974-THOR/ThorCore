package org.ftc9974.thorcore.meta.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Use this annotation to mark something as realized hardware. Instead of using
 * {@code hardwareMap.get()}, use this annotation.
 *
 * @see org.ftc9974.thorcore.meta.Realizer
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@Documented
public @interface Hardware {

    String name() default "";
}
