package com.hhbgk.example.androidyuvencoder;

public class CodecWrapper {
    private String tag = getClass().getSimpleName();
    static {
        System.loadLibrary("codec");
    }
    public static native void init();

    public static native int encode(byte[] array, int srcW, int srcH, int dstW, int dstH);

    public static native void destroy();
}
