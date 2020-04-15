/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class NerdHardwareWorld {



    private LinearOpMode opmode;

    private HardwareMap hardwareMap;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotorB;
    private DcMotor rightMotorB;

    private TouchSensor touchLeft;
    private TouchSensor touchRight;
    private TouchSensor touchBack;


    private Servo servoPitch ;
    private Servo servoAngle ;
    private DcMotor tapeMotor ;

    private BNO055IMU imu = null;   // Gyro device



    public NerdHardwareWorld(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    public void initializeHardware(){

        //Initialize Motors

        this.leftMotor = this.hardwareMap.dcMotor.get("Front_Left_Motor");
        this.rightMotor = this.hardwareMap.dcMotor.get("Front_Right_Motor");
        this.leftMotorB = this.hardwareMap.dcMotor.get("Rear_Left_Motor");
        this.rightMotorB = this.hardwareMap.dcMotor.get("Rear_Right_Motor");

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftMotorB.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        this.imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        this.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.touchLeft = this.hardwareMap.touchSensor.get("touchL");
        this.touchRight = this.hardwareMap.touchSensor.get("touchR");
        this.touchBack = this.hardwareMap.touchSensor.get("touchB");

        this.servoPitch = this.hardwareMap.get(Servo.class, "TurretPitch");
        this.servoAngle = this.hardwareMap.get(Servo.class, "TurretAngle");
        this.tapeMotor = this.hardwareMap.get(DcMotor.class, "TapeMotor");

    }

    public void resetMotorsAndRunWithoutEncoders() {
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

