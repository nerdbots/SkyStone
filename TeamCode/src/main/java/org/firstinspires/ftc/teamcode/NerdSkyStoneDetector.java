package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.network.NetworkDiscoveryManager;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Locale;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
//@TeleOp(name = "Skystone Detector OpMode", group="DogeCV")

public class NerdSkyStoneDetector {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    public NerdSkyStoneDetector(LinearOpMode opmode){
        this.hardwareMap = opmode.hardwareMap;
        this.opMode=opmode;
    }


    public double[] detectSkyStone() {

        double[] skyStoneXYP = new double[3];
        double stonePosition=0;

        if(skyStoneDetector.getScreenPosition().x < 75){
            stonePosition=1.0;


        }else if(skyStoneDetector.getScreenPosition().y > 150){
            stonePosition=3.0;
        }
        else {
            stonePosition=2.0;
        }

        skyStoneXYP[0]=skyStoneDetector.getScreenPosition().x;
        skyStoneXYP[1]=skyStoneDetector.getScreenPosition().y;
        skyStoneXYP[2]=stonePosition;


        /*
         * Send some stats to the telemetry
        */
            this.opMode.telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            this.opMode.telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            this.opMode.telemetry.addData("Stone Position", stonePosition);
            this.opMode.telemetry.update();


            return skyStoneXYP;

    }

    public void  initNerdStoneDetector(){

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        this.opMode.telemetry.addData(">", "Robot Ready.");    //
        this.opMode.telemetry.update();
    }

}
