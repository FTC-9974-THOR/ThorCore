package org.ftc9974.thorcore.meta.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Use this annotation to mark a variable as a EEVal. EEVals are stored on the phone's persistent
 * storage, and are editable on the phone. Instead of having magic numbers hard-coded, stick them
 * in an EEVal. Then, you can edit it on the phone without recompiling the app.
 *
 * <b>Note: the EEVals editor is currently not working, so these are kind of useless. :(</b>
 *
 * @deprecated As far as we can tell, these are inferior to Instant Run.
 */
@Retention(RetentionPolicy.RUNTIME)
@Documented
@Target({ElementType.FIELD})
@Deprecated
public @interface EEVal {

    String name() default "";
}
