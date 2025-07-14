package com.pedropathing.localization.localizers;

import static com.pedropathing.localization.constants.TwoWheelConstants.NAVX_DEVICE_UPDATE_RATE_HZ;
import static com.pedropathing.localization.constants.TwoWheelConstants.NAVX_HardwareMapName;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class NavxImu implements IMU {
    private AHRS navx_device;
    private boolean calibration_complete = false;

    public NavxImu(HardwareMap map, Parameters parameters) {
        navx_device = AHRS.getInstance(map.get(NavxMicroNavigationSensor.class, NAVX_HardwareMapName),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        initialize(parameters);
    }

    public boolean initialize(Parameters parameters) {
        if (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (calibration_complete)  {
                navx_device.zeroYaw();
            } else {
                Log.d("navX-Micro", "Startup Calibration in Progress");
            }
        }

        return calibration_complete;
    }

    public void resetYaw() {
        navx_device.zeroYaw();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        // TODO: Test if you need to flip a sign here
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                navx_device.getYaw() * (-1),
                navx_device.getPitch(),
                navx_device.getRoll(),
                0);
    }

    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return null;
    }

    public Quaternion getRobotOrientationAsQuaternion() {
        return null;
    }

    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "navx";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

