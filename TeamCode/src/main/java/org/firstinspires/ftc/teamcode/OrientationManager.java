package org.firstinspires.ftc.teamcode;

import com.hoan.dsensor_master.DProcessedSensor;
import com.hoan.dsensor_master.DSensorManager;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;

import static java.lang.Math.PI;

/**
 * Created by ycao on 2017-02-25.
 */

class OrientationManager {
    boolean azimuthInitialized;
    private double initAzimuth;

    // orientation values
    double azimuth = 0.0f;       // value in degrees
    /*
     * Constructor
     */
    OrientationManager() {

    }

    public void start(HardwareMap ahwMap) {
        try {
            // needed FtcConfig to get context for getSystemService
            // but this required change to FtcRobotControllerActivity to set the context for us
            //FtcConfig.context.getSystemService(Context.SENSOR_SERVICE);
            DSensorManager.startDProcessedSensor(ahwMap.appContext, DProcessedSensor.TYPE_3D_COMPASS,
                    dSensorEvent -> {
                        // update UI
                        // dSensorEvent.values[0] is the azimuth.
                        azimuth = dSensorEvent.values[0];
                        azimuth = azimuth / PI * 180;
                        if(!azimuthInitialized){
                            initAzimuth = azimuth;
                            azimuthInitialized = true;
                        }
                        azimuth -= initAzimuth;
                        while (azimuth > 180)  azimuth -= 360;
                        while (azimuth <= -180) azimuth += 360;
                    });
        } catch (Exception e) {
            DbgLog.msg(Arrays.toString(e.getStackTrace()));
        }
    }

    public void stop() {
        DSensorManager.stopDSensor();
    }
}
