//
// Created by user on 2/6/21.
//

#include <jni.h>
#include "libpixyusb2.h"
#include "JNIUtilities.h"

#include <unordered_map>
#include <string>
// weird include path to placate code inspection
#include <fmt/include/fmt/core.h>

static std::unordered_map<jint, Pixy2> pixies;
static bool initialized;

extern "C" {
    // initializing the pixy registry in static scope could produce an uncatchable error, so do it
    // manually from Java
    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_initNativeLayer(
            JNIEnv* env, jclass cls) {
        if (!initialized) {
            try {
                // Most teams are only going to use 1 Pixy. 2 is the most I would expect to be in common use.
                pixies.reserve(2);
            } catch (const std::bad_alloc &err) {
                throwOutOfMemoryError(env, fmt::format("bad_alloc creating pixy registry: {%s}",
                                                       err.what()));
            }
            pixies.clear();
            initialized = true;
        }
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeOpen(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        Pixy2 newPixy;
        jint retCode = newPixy.init(fileDescriptor);
        if (retCode < 0) {
            // indicates open failure
            throwIOException(env, fmt::format("unable to open pixy: return code {%d}", retCode));
        }

        // use a move insertion to pass ownership of newPixy to the pixy registry. this makes it so
        // pixies.clear() will properly free the memory used by newPixy.
        pixies.insert(std::make_pair(fileDescriptor, newPixy));
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeClose(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        pixies.erase(fileDescriptor);
    }

    JNIEXPORT jobject JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeGetVersion(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        jclass retClass = tryFindClass(env, "org/ftc9974/thorcore/robot/sensors/Pixy2USB$Version");
        jmethodID ctor = env->GetMethodID(retClass, "<init>", "(ISSILjava/lang/String)V");

        try {
            auto &referencedPixy = pixies[fileDescriptor];
            if (referencedPixy.version == nullptr) {
                int retCode = referencedPixy.getVersion();
                if (retCode < 0) {
                    throwIOException(env, fmt::format("Could not get Pixy version: return code {%d}", retCode));
                    return nullptr;
                }
            }

            jstring firmwareType = env->NewStringUTF(referencedPixy.version->firmwareType);
            return env->NewObject(retClass, ctor,
                                  (jint) referencedPixy.version->hardware,
                                  (jshort) referencedPixy.version->firmwareMajor,
                                  (jshort) referencedPixy.version->firmwareMinor,
                                  (jint) referencedPixy.version->firmwareBuild,
                                  firmwareType);
        } catch (const std::out_of_range& err) {
            throwNoSuchElementException(env, fmt::format("No Pixy is connected on FD{%d}", fileDescriptor));
        }

        return nullptr;
    }
}
