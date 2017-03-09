package org.firstinspires.ftc.teamcode;

import com.hoan.dsensor_master.DProcessedSensor;
import com.hoan.dsensor_master.DSensorEvent;
import com.hoan.dsensor_master.DSensorManager;
import com.hoan.dsensor_master.interfaces.DProcessedEventListener;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;

/**
 * Created by ycao on 2017-02-25.
 */

class OrientationManager {
    boolean azimuthInitialized;
    private float initAzimuth;

    // orientation values
    float azimuth = 0.0f;       // value in radians
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
                    new DProcessedEventListener() {
                        @Override
                        public void onProcessedValueChanged(DSensorEvent dSensorEvent) {
                            // update UI
                            // dSensorEvent.values[0] is the azimuth.
                            azimuth = dSensorEvent.values[0];
                            if(!azimuthInitialized){
                                initAzimuth = azimuth;
                                azimuthInitialized = true;
                            }
                            azimuth -= initAzimuth;
                        }
                    });
        } catch (Exception e) {
            DbgLog.msg(Arrays.toString(e.getStackTrace()));
        }
    }

    public void stop() {
        DSensorManager.stopDSensor();
    }
}
