package org.ftc9974.thorcore.meta.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * Use this to mark all hardware in a class as {@code @Hardware}.
 */
@Retention(RetentionPolicy.RUNTIME)
@Documented
public @interface Realized {
}
