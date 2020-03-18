package org.ftc9974.thorcore.internal;

import java.util.Locale;

public final class CommunicationException extends RuntimeException {

    public CommunicationException(String deviceName, String problem) {
        super(String.format(Locale.getDefault(), "Communication problem with \"%s\": %s", deviceName, problem));
    }

    public CommunicationException(String deviceName, String problem, Throwable cause) {
        super(String.format(Locale.getDefault(), "Communication problem with \"%s\": %s", deviceName, problem), cause);
    }
}
