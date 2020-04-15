/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Timer;
import java.util.TimerTask;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="getVelocitiesFUNC", group="Final")
//@Disabled
public class getVelocitiesFUNC extends LinearOpMode {

    boolean debugFlag = false;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotorB;
    private DcMotor rightMotorB;
    private double maxSpeed = 1.0;
    private double currentMaxSpeed = 0;
    private double rampUpDownConstant = 0.9; //less than 1. Higher the value, smaller steps.
    private final double GEAR_RATIO = 15.0 / 20.0;  // Gear ratio
    private final double ticksPerRotation = 560.0; //For omni wheels we are using
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    public Timer timer = new Timer();

    ElapsedTime runtime = new ElapsedTime();

    double prevRuntime = 0.0;
    double frontLeftVelocity = 0, frontRightVelocity = 0, rearLeftVelocity = 0, rearRightVelocity = 0;
    int leftCurrent, rightCurrent, rightBCurrent, leftBCurrent;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    double leftInches, rightInches, leftBInches, rightBInches;
    double currentAcceleration = 0.0;
    double avgVelocity = 0.0;
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;
    //int prevLeftTicks = 0, prevRightTicks = 0, prevLeftBTicks = 0, prevRightBTicks = 0;
    int leftDeltaTicks = 0, rightDeltaTicks = 0, leftBDeltaTicks = 0, rightBDeltaTicks = 0, avgDeltaTicks;
    double [] Velocities = {frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity};
    double loopTime=0.025;
    @Override
    public void runOpMode() {
        //Initialize Hardware
        initializeHardware();

        telemetry.addData("Init", "Completed");
        telemetry.update();
        waitForStart();



    }

    public double[] measureVelocities() {

        runtime.reset();

        motorsResetAndRunUsingEncoders();
        RobotLog.d("VelocityFUNC3Scheduled, frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity, avgVelocity, currentAcceleration, maxAcceleration, maxVelocity, runtime.seconds, leftDeltaTicks, rightDeltaTicks, leftBDeltaTicks, rightBDeltaTicks, avgDeltaTicks");

        if (opModeIsActive()) {
            timer.scheduleAtFixedRate(new TimerTask() {
                @Override
                public void run() {

                        leftCurrent = leftMotor.getCurrentPosition();
                        rightCurrent = rightMotor.getCurrentPosition();
                        leftBCurrent = leftMotorB.getCurrentPosition();
                        rightBCurrent = rightMotorB.getCurrentPosition();

                        leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
                        rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
                        leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
                        rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);

                        frontLeftVelocity = leftInches / loopTime;
                        frontRightVelocity = rightInches / loopTime;
                        rearLeftVelocity = leftBInches / loopTime;
                        rearRightVelocity = rightBInches / loopTime;

                        avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity)/4;

                        currentAcceleration = avgVelocity/ loopTime;

                        if (currentAcceleration > maxAcceleration) {
                            maxAcceleration = currentAcceleration;
                        }

                        if (avgVelocity > maxVelocity){
                            maxVelocity = avgVelocity;
                        }

                        leftDeltaTicks = leftCurrent - prevLeft;
                        rightDeltaTicks = rightCurrent - prevRight;
                        leftBDeltaTicks = leftBCurrent - prevLeftB;
                        rightBDeltaTicks = rightBCurrent - prevRightB;
                        avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks)/4;

                        prevLeft = leftCurrent;
                        prevRight = rightCurrent;
                        prevLeftB = leftBCurrent;
                        prevRightB = rightBCurrent;


                        //RobotLog.d("VelocityFUNC3Scheduled, %f, %f,%f,%f,%f,%f,%f,%f,%f, %d, %d,%d,%d,%d,", frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity, avgVelocity, currentAcceleration, maxAcceleration, maxVelocity, runtime.seconds(), leftDeltaTicks, rightDeltaTicks, leftBDeltaTicks, rightBDeltaTicks, avgDeltaTicks);

                        telemetry.addData("frontLeftVelocity:", frontLeftVelocity);
                        telemetry.addData("frontRightVelocity:", frontRightVelocity);
                        telemetry.addData("rearLeftVelocity:", rearLeftVelocity);
                        telemetry.addData("rearRightVelocity:", rearRightVelocity);
                        telemetry.addData("avgVelocity:", avgVelocity);
                        telemetry.addData("Acceleration", currentAcceleration);
                        telemetry.addData("maxAcceleration", maxAcceleration);
                        telemetry.addData("maxVelocity", maxVelocity);
                        telemetry.addData("loopRuntime", runtime.seconds());
                        telemetry.addData("leftDeltaTicks", leftDeltaTicks);
                        telemetry.addData("rightDeltaTicks", rightDeltaTicks);
                        telemetry.addData("leftBDeltaTicks", leftBDeltaTicks);
                        telemetry.addData("rightBDeltaTicks",rightBDeltaTicks);
                        telemetry.addData("avgDeltaTicks", avgDeltaTicks);
                        telemetry.update();

                        if(runtime.seconds()>=5){
                            timer.cancel();
                       }
                    }
            },(long)0,(long)(loopTime*1000));

        }
while (opModeIsActive() && runtime.seconds()<=5.0) {
    leftMotor.setPower(1);
    rightMotor.setPower(1);
    leftMotorB.setPower(1);
    rightMotorB.setPower(1);
}
        return Velocities;
    }



    public void initializeHardware() {

        //Initialize Motors

        this.leftMotor = this.hardwareMap.dcMotor.get("Front_Left_Motor");
        this.rightMotor = this.hardwareMap.dcMotor.get("Front_Right_Motor");
        this.leftMotorB = this.hardwareMap.dcMotor.get("Rear_Left_Motor");
        this.rightMotorB = this.hardwareMap.dcMotor.get("Rear_Right_Motor");

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double rampUpSpeeds() {

        double rampUpMaxSpeed;
        rampUpMaxSpeed = this.maxSpeed - (this.maxSpeed - currentMaxSpeed) * this.rampUpDownConstant;
        currentMaxSpeed = rampUpMaxSpeed;

        return rampUpMaxSpeed;
    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * 3.14;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
        double straightDistanceToTravel = wheelDistanceToTravel / (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return straightDistanceToTravel;
    }

    public void motorsRunUsingEncoders(){

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorB.setPower(0);
        rightMotorB.setPower(0);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void motorsResetAndRunUsingEncoders(){

        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
public double[] getVelocityForCurrentLoop(double loopTime){
    leftCurrent = leftMotor.getCurrentPosition();
    rightCurrent = rightMotor.getCurrentPosition();
    leftBCurrent = leftMotorB.getCurrentPosition();
    rightBCurrent = rightMotorB.getCurrentPosition();

    leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
    rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
    leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
    rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);

    frontLeftVelocity = leftInches / loopTime;
    frontRightVelocity = rightInches / loopTime;
    rearLeftVelocity = leftBInches / loopTime;
    rearRightVelocity = rightBInches / loopTime;

    avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity)/4;

    currentAcceleration = avgVelocity/ loopTime;

    if (currentAcceleration > maxAcceleration) {
        maxAcceleration = currentAcceleration;
    }

    if (avgVelocity > maxVelocity){
        maxVelocity = avgVelocity;
    }

    leftDeltaTicks = leftCurrent - prevLeft;
    rightDeltaTicks = rightCurrent - prevRight;
    leftBDeltaTicks = leftBCurrent - prevLeftB;
    rightBDeltaTicks = rightBCurrent - prevRightB;
    avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks)/4;

    prevLeft = leftCurrent;
    prevRight = rightCurrent;
    prevLeftB = leftBCurrent;
    prevRightB = rightBCurrent;
return Velocities;
}

}