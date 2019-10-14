package com.inspiration.inspcv;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Created by guinea on 2/21/18.
 * -------------------------------------------------------------------------------------
 * Credit to the OpenRC team for coming up with this general solution.
 */

public class OpenCVLoader {
    // This path may need to be changed for Samsung S5 phones; 
    // its hardcoded because idk how else to get the path without a Context.
    // Feel free to change the value to fit the phone you are using.
    private static String filesDir = "/data/user/0/com.qualcomm.ftcrobotcontroller/files";
    public static void setFilesDir(String path) {
        filesDir = path;
    }
    public static void loadOpenCV() {

        File protectedStorageLib = new File(filesDir + "/extra/libopencv_java3.so");
        File protectedExtraFolder = new File(filesDir + "/extra/");
        File internalStorageLib = new File(Environment.getExternalStorageDirectory() + "/EnderCV/libopencv_java3.so");
        if (!protectedStorageLib.exists() && internalStorageLib.exists()) {
            if (!protectedExtraFolder.exists())
                protectedExtraFolder.mkdir();

        }

        try {
            /*
             * Copy the file with a 1MiB buffer
             */
            InputStream is = new FileInputStream(internalStorageLib);
            OutputStream os = new FileOutputStream(protectedStorageLib);
            byte[] buff = new byte[1024];
            int len;
            while ((len = is.read(buff)) > 0) {
                os.write(buff, 0, len);
            }
            is.close();
            os.close();

            System.load(protectedStorageLib.getAbsolutePath());
        } catch (Exception e) {
            Log.e("EnderCV", "OpenCV Load Error: ", e);
        }

    }
}
