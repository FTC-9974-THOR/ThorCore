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
#include <libusb.h>
#include <android/log.h>

#define TAG "JNIPixy2USB"

// borrowed from libpixyusb2 examples
int demosaic(uint16_t width, uint16_t height, const uint8_t *bayerImage, uint8_t *image)
{
    uint32_t x, y, xx, yy, r, g, b;
    uint8_t *pixel0, *pixel;

    for (y=0; y<height; y++)
    {
        yy = y;
        if (yy==0)
            yy++;
        else if (yy==height-1)
            yy--;
        pixel0 = (uint8_t *)bayerImage + yy*width;
        for (x=0; x<width; x++, image += 4)
        {
            xx = x;
            if (xx==0)
                xx++;
            else if (xx==width-1)
                xx--;
            pixel = pixel0 + xx;
            if (yy&1)
            {
                if (xx&1)
                {
                    r = *pixel;
                    g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
                    b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
                }
                else
                {
                    r = (*(pixel-1)+*(pixel+1))>>1;
                    g = *pixel;
                    b = (*(pixel-width)+*(pixel+width))>>1;
                }
            }
            else
            {
                if (xx&1)
                {
                    r = (*(pixel-width)+*(pixel+width))>>1;
                    g = *pixel;
                    b = (*(pixel-1)+*(pixel+1))>>1;
                }
                else
                {
                    r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
                    g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
                    b = *pixel;
                }
            }
            //*image = (b<<16) | (g<<8) | r;
            // put into android ARGB_8888 format, with full alpha (0xff)
            *image = r;
            *(image + 1) = g;
            *(image + 2) = b;
            *(image + 3) = 0xff;
        }
    }
    return 0;
}

enum PixyProgram {
    COLOR_CONNECTED_COMPONENTS,
    LINE_TRACKING,
    VIDEO
};

// storing a registry in both Java and native code feels redundant. todo find a better solution
// Pixy2 is pretty slick in that it automatically handles libusb contexts, so i can instantiate as
// many of them as i want without worrying about them affecting each other.
static std::unordered_map<jint, Pixy2*> pixies;

Pixy2* findPixyOrThrow(JNIEnv* env, int fd) {
    auto f = pixies.find(fd);
    if (f == pixies.end()) {
        throwIOException(env, fmt::format("No Pixy is connected on FD {}", fd));
        return nullptr;
    }
    return f->second;
}

extern "C" {
    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeOpen(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        auto* newPixy = new Pixy2();
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Attempting to open Pixy on FD %d", fileDescriptor);
        jint retCode = newPixy->init(fileDescriptor);
        switch (retCode) {
            case LIBUSB_ERROR_NO_MEM:
                throwOutOfMemoryError(env, "LIBUSB_ERROR_NO_MEM while wrapping file descriptor");
                return;
            case LIBUSB_ERROR_ACCESS:
                throwIllegalAccessError(env, "insufficient permissions: LIBUSB_ERROR_ACCESS");
                return;
            case LIBUSB_ERROR_BUSY:
                throwIOException(env, "file descriptor already in use: LIBUSB_ERROR_BUSY");
                return;
            case LIBUSB_ERROR_NO_DEVICE:
                throwIOException(env, "connection lost: LIBUSB_ERROR_NO_DEVICE");
                return;
            default:
                break;
        }
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Successfully opened Pixy on FD %d", fileDescriptor);

        pixies.insert(std::make_pair(fileDescriptor, newPixy));
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeClose(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        auto f = pixies.find(fileDescriptor);
        if (f != pixies.end()) {
            delete f->second;
            pixies.erase(fileDescriptor);
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Successfully closed Pixy on FD %d", fileDescriptor);
        }
    }

    JNIEXPORT jobject JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeGetVersion(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        jclass retClass = tryFindClass(env, "org/ftc9974/thorcore/robot/sensors/Pixy2USB$Version");
        jmethodID ctor = env->GetMethodID(retClass, "<init>", "(ISSILjava/lang/String;)V");

        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return nullptr;
        }

        int retCode = referencedPixy->getVersion();
        if (retCode < PIXY_RESULT_OK) {
            throwIOException(env, fmt::format("Could not get Pixy version: return code {}", retCode));
            return nullptr;
        }

        jstring firmwareType = env->NewStringUTF(referencedPixy->version->firmwareType);
        return env->NewObject(retClass, ctor,
                              (jint) referencedPixy->version->hardware,
                              (jshort) referencedPixy->version->firmwareMajor,
                              (jshort) referencedPixy->version->firmwareMinor,
                              (jint) referencedPixy->version->firmwareBuild,
                              firmwareType);
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeSetProgram(
            JNIEnv* env, jclass cls, jint fileDescriptor, jint programIndex) {
        std::string programName;
        switch ((PixyProgram) programIndex) {
            case COLOR_CONNECTED_COMPONENTS:
                programName = "color_connected_components";
                break;
            case LINE_TRACKING:
                programName = "line_tracking";
                break;
            case VIDEO:
                programName = "video";
                break;
        }
        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return;
        }
        int retCode = referencedPixy->changeProg(programName.c_str());
        if (retCode == PIXY_RESULT_ERROR) {
            throwIOException(env, "could not set program");
        }
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeSetLamps(
            JNIEnv* env, jclass cls, jint fileDescriptor, jboolean upper, jboolean lower) {
        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return;
        }

        int retCode = referencedPixy->setLamp(upper, lower);
        if (retCode < PIXY_RESULT_OK) {
            throwIOException(env, fmt::format("unable to set lamp states: return code {}", retCode));
            return;
        }
    }

    JNIEXPORT jobject JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeGetResolution(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        jclass size = env->FindClass("android/util/Size");
        jmethodID ctor = env->GetMethodID(size, "<init>", "(II)V");

        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return nullptr;
        }

        // Pixy2 auto-fills resolution fields
        return env->NewObject(size, ctor, referencedPixy->frameWidth, referencedPixy->frameHeight);
    }

    JNIEXPORT jobjectArray JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeGetCCCBlocks(
            JNIEnv* env, jclass cls, jint fileDescriptor) {
        jclass block = env->FindClass("org/ftc9974/thorcore/robot/sensors/Pixy2USB$Block");
        jmethodID ctor = env->GetMethodID(block, "<init>", "(IIIIISSS)V");

        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return nullptr;
        }

        Pixy2CCC<Link2USB>* ccc = &referencedPixy->ccc;
        int retCode = ccc->getBlocks();
        if (retCode < PIXY_RESULT_OK) {
            throwIOException(env, fmt::format("pixy2->ccc.getBlocks() failed: return code {}", retCode));
            return nullptr;
        }

        jobjectArray arr = env->NewObjectArray(ccc->numBlocks, block, nullptr);
        if (arr == nullptr) {
            return nullptr;
        }

        Block* blockPtr = ccc->blocks;
        for (int i = 0, n = ccc->numBlocks; i < n; i++) {
            jobject jBlock = env->NewObject(block, ctor,
                    blockPtr->m_signature,
                    blockPtr->m_x,
                    blockPtr->m_y,
                    blockPtr->m_width,
                    blockPtr->m_height,
                    blockPtr->m_angle,
                    blockPtr->m_index,
                    blockPtr->m_age);
            env->SetObjectArrayElement(arr, i, jBlock);
            blockPtr += sizeof(Block);
        }

        return arr;
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeSetLED(
            JNIEnv* env, jclass cls, jint fileDescriptor, jint r, jint g, jint b) {
        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return;
        }

        int retCode = referencedPixy->setLED(r, g, b);
        if (retCode != PIXY_RESULT_OK) {
            throwIOException(env, fmt::format("pixy2->setLED failed: return code {}", retCode));
            return;
        }
    }

    JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_robot_sensors_Pixy2USB_nativeGetFrame(
            JNIEnv* env, jclass cls, jint fileDescriptor, jobject outputBuffer) {
        Pixy2* referencedPixy = findPixyOrThrow(env, fileDescriptor);
        if (referencedPixy == nullptr) {
            return;
        }

        // that buffer best be in little endian
        auto* outputBufPtr = (uint8_t*) env->GetDirectBufferAddress(outputBuffer);
        uint32_t outputBufLen = env->GetDirectBufferCapacity(outputBuffer);

        // image size, in pixels
        int imageSize = referencedPixy->frameWidth * referencedPixy->frameHeight;

        if (outputBufLen != imageSize * 4) {
            throwIndexOutOfBoundsException(env, fmt::format("Buffer size mismatch: expected {}, got {}", imageSize * 4, outputBufLen));
            return;
        }

        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Stopping pixy for frame grab");

        referencedPixy->changeProg("video");
        // apparently you need to call stop() before getRawFrame()? i should have read the docs closer
        referencedPixy->m_link.stop();

        auto* bayerFrame = (uint8_t*) malloc(imageSize);
        if (bayerFrame != nullptr) {
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Grabbing frame");
            referencedPixy->m_link.getRawFrame(&bayerFrame);
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Frame grabbed, demosaicing");
            demosaic(referencedPixy->frameWidth, referencedPixy->frameHeight, bayerFrame,
                     outputBufPtr);
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Demosaic done");
            free(bayerFrame);
        }

        referencedPixy->m_link.resume();
        referencedPixy->changeProg("color_connected_components");
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Done");
    }
}
