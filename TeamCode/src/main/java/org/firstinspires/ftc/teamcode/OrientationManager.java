package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * Created by ???
 * Modified by Cameron Rhodes on 9/30/2015. (https://github.com/1885FTCAustralia/FTC-AU-Robot/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/OrientOp.java)
 * Modified by ycao on 2017-02-25.
 */

class OrientationManager implements SensorEventListener {
    private SensorManager mSensorManager;
    private Telemetry telemetry;

    private boolean azimuthInitialized;
    private float initAzimuth;

    // orientation values
    float azimuth = 0.0f;       // value in radians
    private float pitch = 0.0f;        // value in radians
    private float roll = 0.0f;         // value in radians

    private float[] mGravity;       // latest sensor values
    private float[] mGeomagnetic;   // latest sensor values

    /*
     * Constructor
     */
    OrientationManager(HardwareMap ahwMap, Telemetry tm) {
        mSensorManager = (SensorManager) ahwMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        telemetry = tm;
    }

    public void start() {
        try {
            // needed FtcConfig to get context for getSystemService
            // but this required change to FtcRobotControllerActivity to set the context for us
            //FtcConfig.context.getSystemService(Context.SENSOR_SERVICE);
            Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            Sensor magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            // delay value is SENSOR_DELAY_UI which is ok for telemetry, maybe not for actual robot use
            mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
            mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);

            azimuthInitialized = false;
        } catch (Exception e) {
            telemetry.addData("Error", e.getStackTrace());
            DbgLog.msg(Arrays.toString(e.getStackTrace()));
        }
    }

    public void loop() {
        telemetry.addData("1 Start", "Orient started");
        if (mGravity == null) {
            telemetry.addData("2 Gravity", "Gravity sensor values null ");
            telemetry.addData("3 Geomagnetic", "Geomagnetic sensor values null ");
        } else {
            telemetry.addData("2 Gravity", "Gravity sensor returning values ");
            telemetry.addData("3 Geomagnetic", "Geomagnetic sensor returning values ");
        }
        telemetry.addData("4 yaw", String.format("yaw= %0 6.3f",Math.toDegrees(azimuth)));
        telemetry.addData("5 pitch", String.format("pitch = %0 6.3f",Math.toDegrees(pitch)));
        telemetry.addData("6 roll", String.format("roll = %0 6.3f",Math.toDegrees(roll)));
    }

    public void stop() {
        mSensorManager.unregisterListener(this);
    }


    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not sure if needed, placeholder just in case
    }

    public void onSensorChanged(SensorEvent event) {
        // we need both sensor values to calculate orientation
        // only one value will have changed when this method called, we assume we can still use the other value.
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mGravity = event.values;
        }
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            mGeomagnetic = event.values;
        }
        if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                // orientation contains: azimuth, pitch and roll
                // raw values ranges from -pi to pi
                if(!azimuthInitialized){
                    initAzimuth = (float)(orientation[0] + Math.PI);
                    azimuthInitialized = true;
                }
                azimuth = (float)(orientation[0] + Math.PI) - initAzimuth;
                if(azimuth > Math.PI){
                    azimuth -= 2 * Math.PI;
                }
                else if(azimuth < -Math.PI){
                    azimuth += 2 * Math.PI;
                }
                pitch = orientation[1];
                roll = orientation[2];
            }
        }
    }
}
