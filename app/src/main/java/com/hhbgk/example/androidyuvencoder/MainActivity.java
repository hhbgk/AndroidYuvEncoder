package com.hhbgk.example.androidyuvencoder;

import android.Manifest;
import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ConfigurationInfo;
import android.content.pm.PackageManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;

public class MainActivity extends Activity implements View.OnClickListener{
    String tag = getClass().getSimpleName();
    Button mButton;
    private static final int MY_PERMISSION_REQUEST_CODE = 10000;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Log.e(tag, "on create .......................");
        mButton = findViewById(R.id.sample_button);

        /**
         * 第 1 步: 检查是否有相应的权限
         */
        boolean isAllGranted = checkPermissionAllGranted(new String[] {
                        Manifest.permission.WRITE_EXTERNAL_STORAGE
                }
        );

        // 如果这3个权限全都拥有, 则直接执行备份代码
        if (isAllGranted) {
            doSomething();
        } else {
            /**
             * 第 2 步: 请求权限
             */
            // 一次请求多个权限, 如果其他有权限是已经授予的将会自动忽略掉
            ActivityCompat.requestPermissions(
                    this,
                    new String[] {
                            Manifest.permission.WRITE_EXTERNAL_STORAGE
                    },
                    MY_PERMISSION_REQUEST_CODE
            );
        }
    }

    /**
     * 检查是否拥有指定的所有权限
     */
    private boolean checkPermissionAllGranted(String[] permissions) {
        for (String permission : permissions) {
            if (ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED) {
                // 只要有一个权限没有被授予, 则直接返回 false
                return false;
            }
        }
        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == MY_PERMISSION_REQUEST_CODE) {
            boolean isAllGranted = true;

            // 判断是否所有的权限都已经授予了
            for (int grant : grantResults) {
                if (grant != PackageManager.PERMISSION_GRANTED) {
                    isAllGranted = false;
                    break;
                }
            }

            if (isAllGranted) {
                // 如果所有的权限都授予了, 则执行备份代码
                doSomething();
            } else {
                // 弹出对话框告诉用户需要权限的原因, 并引导用户去应用权限管理中手动打开权限按钮
                Toast.makeText(this, "You have to open permissions in the settings.", Toast.LENGTH_LONG).show();
            }
        }
    }

    private void doSomething() {
        mButton.setOnClickListener(this);
        CodecWrapper.init();
    }

    @Override
    public void onClick(View view) {
        InputStream inputStream = getResources().openRawResource(R.raw.src_vga);
        byte[] b = null;
        try {
            b = new byte[inputStream.available()];
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (b != null){
            try {
                inputStream.read(b);
                CodecWrapper.encode(b, 640,480,1280,720);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        try {
            inputStream.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        Log.e(tag, "onStart ");
    }

    @Override
    protected void onRestart() {
        super.onRestart();
        finish();
        Log.e(tag, "onRestart ");
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.e(tag, "onResume ");
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.e(tag, " onPause");
    }

    @Override
    protected void onStop() {
        super.onStop();
        Log.e(tag, " onStop");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.e(tag, " onDestroy");
        CodecWrapper.destroy();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        Log.i(tag, "requestCode=" + requestCode +", resultCode=" + resultCode);
        if (1000 == requestCode) {
            Log.i(tag, "onActivityResult ===========");
        }
    }
}
