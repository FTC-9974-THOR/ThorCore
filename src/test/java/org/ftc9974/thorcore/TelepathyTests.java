package org.ftc9974.thorcore;

import org.junit.Assert;
import org.junit.Test;

import java.util.Locale;

public class TelepathyTests {

    @Test
    public void test_serializeString() {
        String key = "Key";
        String message = "Message";
        byte[] serialized = Telepathy.prepareForTransmit(key, Telepathy.Type.STRING, message.getBytes());
        System.out.println(String.format(Locale.getDefault(), "Key: %s\nMessage: %s", key, message));
        System.out.println("Key bytes:");
        for (byte b : key.getBytes()) {
            System.out.print(b);
            System.out.print(' ');
        }
        System.out.println("\nMessage bytes:");
        for (byte b : message.getBytes()) {
            System.out.print(b);
            System.out.print(' ');
        }
        System.out.println("\nContents of serialized packet:");
        for (byte b : serialized) {
            System.out.print(b);
            System.out.print(String.format(Locale.getDefault(), "\t%8s", Integer.toBinaryString(b)).replace(' ', '0'));
            if (b > 31 && b < 127) {
                System.out.println("\t" + (char) b);
            } else {
                System.out.println();
            }
        }
        Assert.assertArrayEquals(new byte[] {
                (byte) 75,
                (byte) 101,
                (byte) 121,
                (byte) 0,
                (byte) 0,
                (byte) 0,
                (byte) 0,
                (byte) 7,
                (byte) 77,
                (byte) 101,
                (byte) 115,
                (byte) 115,
                (byte) 97,
                (byte) 103,
                (byte) 101
        }, serialized);
    }

    @Test
    public void test_deserializeString() {
        String key = "Key";
        String message = "Message";
        byte[] serialized = Telepathy.prepareForTransmit(key, Telepathy.Type.STRING, message.getBytes());
        System.out.println(String.format(Locale.getDefault(), "Key: %s\nMessage: %s", key, message));
        System.out.println("Key bytes:");
        for (byte b : key.getBytes()) {
            System.out.print(b);
            System.out.print(' ');
        }
        System.out.println("\nMessage bytes:");
        for (byte b : message.getBytes()) {
            System.out.print(b);
            System.out.print(' ');
        }
        System.out.println("\nContents of serialized packet:");
        for (byte b : serialized) {
            System.out.print(b);
            System.out.print(String.format(Locale.getDefault(), "\t%8s", Integer.toBinaryString(b)).replace(' ', '0'));
            if (b > 31 && b < 127) {
                System.out.println("\t" + (char) b);
            } else {
                System.out.println();
            }
        }
        Telepathy.Message deserialized = Telepathy.deserializeMessage(serialized);
        System.out.println(String.format(Locale.getDefault(), "Deserialized key: %s\nDeserialized type: %s\nDeserialized message: %s",
                deserialized.key,
                deserialized.type.toString(),
                deserialized.message.toString()));
        Assert.assertEquals(key, deserialized.key);
        Assert.assertEquals(Telepathy.Type.STRING, deserialized.type);
        Assert.assertEquals(message, deserialized.message);
    }
}
