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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.ArrayList;
import java.util.Timer;
import java.util.*;
import java.util.TimerTask;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Queue;
import java.util.List;


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
public class NerdPIDCalc_MotionProfiling2 {
    private LinearOpMode opmode;
    private HardwareMap hardwareMap;
    private ElapsedTime moveTime = new ElapsedTime();
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor leftMotorB;
    public DcMotor rightMotorB;
    private ElapsedTime PIDTime = new ElapsedTime();
    private double FLPrevError = 0;
    private double FLTotalError = 0;
    private double FLVeloc = 0;
    private double FLMaxVeloc = 10;
    private double FLP = 0;
    private double FLI = 0;
    private double FLD = 0;
    private double FLkP;
    private double FLkI;
    private double FLkD;
    private ElapsedTime FLPIDTime = new ElapsedTime();
    private double FRPrevError = 0;
    private double FRTotalError = 0;
    private double FRVeloc = 0;
    private double FRMaxVeloc = 10;
    private double FRP = 0;
    private double FRI = 0;
    private double FRD = 0;
    private double FRkP;
    private double FRkI;
    private double FRkD;
    private ElapsedTime FRPIDTime = new ElapsedTime();
    private double RLPrevError = 0;
    private double RLTotalError = 0;
    private double RLVeloc = 0;
    private double RLMaxVeloc = 10;
    private double RLP = 0;
    private double RLI = 0;
    private double RLD = 0;
    private double RLkP;
    private double RLkI;
    private double RLkD;
    private ElapsedTime RLPIDTime = new ElapsedTime();
    private double RRPrevError = 0;
    private double RRTotalError = 0;
    private double RRVeloc = 0;
    private double RRMaxVeloc = 10;
    private double RRP = 0;
    private double RRI = 0;
    private double RRD = 0;
    private double RRkP;
    private double RRkI;
    private double RRkD;
    private ElapsedTime RRPIDTime = new ElapsedTime();

    public ElapsedTime driveTime = new ElapsedTime();

    public ElapsedTime tickTime = new ElapsedTime();

    public double[] Velocities = new double[4];

    private double deltaTickTime = 0;

    private double prevTickTime = 0;

    private double tTime = 0;

    public boolean debugFlag = false;
    public boolean debugFlag2 = false;

    public int ACDStage = 0;

    private double prevDriveTime;

    public int LWM = -1;

    public int RWM = 1;

    public boolean ramp = true;

    public int rampUpRate = 10;
    public int rampDownRate = -10;
    public int rampMaxVeloc = 60;
    public int rampMinVeloc = 0;
    public double coastTime = 1;
    public double rampUpTime = 0.1;
    public double rampDownTime = 0.1;

    ArrayList<Double> rAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> FLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> FRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> RLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> RRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));

    double FLInVeloc = 0;
    double FRInVeloc = 0;
    double RLInVeloc = 0;
    double RRInVeloc = 0;

    private double sum = 0;

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
    //double[] Velocities = {frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity};
    double loopTime = 0.05;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 15.0 / 20.0;  // Gear ratio
    private final double ticksPerRotation = 560.0; //For omni wheels we are using
    public Timer timer = new Timer();
    public double MaxVeloc_20_80 = 0;
    ElapsedTime maxVelocTime = new ElapsedTime();

    public double maxPower = 0;

    public NerdPIDCalc_MotionProfiling2(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    public void setFLGains(double kPV, double kIV, double kDV) {
        FLkP = kPV;
        FLkI = kIV;
        FLkD = kDV;
    }

    public void setFRGains(double kPV, double kIV, double kDV) {
        FRkP = kPV;
        FRkI = kIV;
        FRkD = kDV;
    }

    public void setRLGains(double kPV, double kIV, double kDV) {
        RLkP = kPV;
        RLkI = kIV;
        RLkD = kDV;
    }

    public void setRRGains(double kPV, double kIV, double kDV) {
        RRkP = kPV;
        RRkI = kIV;
        RRkD = kDV;
        moveTime.reset();
    }

    public void resetFL() {
        FLPrevError = 0;
        FLTotalError = 0;
        FLVeloc = 0;
        FLPIDTime.reset();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetFR() {
        FRPrevError = 0;
        FRTotalError = 0;
        FRVeloc = 0;
        FRPIDTime.reset();
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRL() {
        RLPrevError = 0;
        RLTotalError = 0;
        RLVeloc = 0;
        RLPIDTime.reset();
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRR() {
        RRPrevError = 0;
        RRTotalError = 0;
        RRVeloc = 0;
        RRPIDTime.reset();
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void PIDVeloc(double CurrVeloc, double TVeloc, String Motor) {
        double DError = 0;
        int DBanMin = -1;
        int DBanMax = 1;
        //   int MaxError = 10;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxVeloc = 0;
        double kP = 0, kI = 0, kD = 0;
        switch (Motor) {
            case "FL":
                TotalError = FLTotalError;
                PrevError = FLPrevError;
                MaxVeloc = FLMaxVeloc;
                PIDTime = FLPIDTime;
                kP = FLkP;
                kI = FLkI;
                kD = FLkD;
                break;
            case "FR":
                TotalError = FRTotalError;
                PrevError = FRPrevError;
                MaxVeloc = FRMaxVeloc;
                PIDTime = FRPIDTime;
                kP = FRkP;
                kI = FRkI;
                kD = FRkD;
                break;
            case "RL":
                TotalError = RLTotalError;
                PrevError = RLPrevError;
                MaxVeloc = RLMaxVeloc;
                PIDTime = RLPIDTime;
                kP = RLkP;
                kI = RLkI;
                kD = RLkD;
                break;
            case "RR":
                TotalError = RRTotalError;
                PrevError = RRPrevError;
                MaxVeloc = RRMaxVeloc;
                PIDTime = RRPIDTime;
                kP = RRkP;
                kI = RRkI;
                kD = RRkD;
                break;
        }

        //calculate error (Proportional)
        error = TVeloc - CurrVeloc;
        //Calculate Total error (Integral)
        TotalError = (error * PIDTime.seconds()) + TotalError;
        //  PIDTime.reset();
        //do deadban
        if (DBanMax > error && error > DBanMin) {
            error = 0;
            //TotalError = 0;
        }
        //calculate delta error (Derivative)
        DError = (error - PrevError) / PIDTime.seconds();
        //reset elapsed timer
        PIDTime.reset();
        //Max total error
        //  if (Math.abs(TotalError) > MaxError) {
        //        if (TotalError > 0) {
        //           TotalError = MaxError;
        //      } else {
        //            TotalError = -MaxError;
        //       }
        //  }
        //Calculate final speed
        speed = (error * kP) + (TotalError * kI) + (DError * kD);

        //Make sure speed is no larger than MaxSpeed
//        if (Math.abs(speed) > MaxVeloc) {
//            if (speed > 0) {
//                speed = MaxVeloc;
//            } else {
//                speed = -MaxVeloc;
//            }
//        }
        PrevError = error;

        // opmode.telemetry.addData("speed", speed);


        switch (Motor) {
            case "FL":
                FLTotalError = TotalError;
                FLPrevError = PrevError;
                FLVeloc = speed;
                FLP = error;
                FLI = TotalError;
                FLD = DError;
                opmode.telemetry.addData("errorFL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d("Time - %f, FL error - %f, FL Target Veloc - %f, FL Input Veloc - %f, FL Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "FR":
                FRTotalError = TotalError;
                FRPrevError = PrevError;
                FRVeloc = speed;
                FRP = error;
                FRI = TotalError;
                FRD = DError;
                opmode.telemetry.addData("errorFR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d("Time - %f, FR error - %f, FR Target Veloc - %f, FR Input Veloc - %f, FR Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "RL":
                RLTotalError = TotalError;
                RLPrevError = PrevError;
                RLVeloc = speed;
                RLP = error;
                RLI = TotalError;
                RLD = DError;
                opmode.telemetry.addData("errorRL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d("Time - %f, RL error - %f, RL Target Veloc - %f, RL Input Veloc - %f, RL Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "RR":
                RRTotalError = TotalError;
                RRPrevError = PrevError;
                RRVeloc = speed;
                RRP = error;
                RRI = TotalError;
                RRD = DError;
                opmode.telemetry.addData("errorRR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d("Time - %f, RR error - %f, RR Target Veloc - %f, RR Input Veloc - %f, RR Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed);
                break;

        }

        if(Math.abs(FLVeloc) > Math.abs(FRVeloc)){
            maxPower = Math.abs(FLVeloc);
        } else {
            maxPower = Math.abs(FRVeloc);
        }
        if(Math.abs(RLVeloc) > maxPower){
            maxPower = Math.abs(RLVeloc);
        }
        if(Math.abs(RRVeloc) > maxPower) {
            maxPower = Math.abs(RRVeloc);
        }

        if(maxPower > 1) {
            FLVeloc /= maxPower;
            FRVeloc /= maxPower;
            RLVeloc /= maxPower;
            RRVeloc /= maxPower;
        }

    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * 3.14;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
        double straightDistanceToTravel = wheelDistanceToTravel / (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return straightDistanceToTravel;
    }

    public double[] getVelocityForCurrentLoop() {
        leftCurrent = leftMotor.getCurrentPosition();
        rightCurrent = rightMotor.getCurrentPosition();
        leftBCurrent = leftMotorB.getCurrentPosition();
        rightBCurrent = rightMotorB.getCurrentPosition();

        tTime = tickTime.seconds();

        deltaTickTime = tTime - prevTickTime;

        leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
        rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
        leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
        rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);
        frontLeftVelocity = leftInches / deltaTickTime;
        frontRightVelocity = rightInches / deltaTickTime;
        rearLeftVelocity = leftBInches / deltaTickTime;
        rearRightVelocity = rightBInches / deltaTickTime;
        avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity) / 4;
        currentAcceleration = avgVelocity / deltaTickTime;
        if (currentAcceleration > maxAcceleration) {
            maxAcceleration = currentAcceleration;
        }
        if (avgVelocity > maxVelocity) {
            maxVelocity = avgVelocity;
        }
        leftDeltaTicks = leftCurrent - prevLeft;
        rightDeltaTicks = rightCurrent - prevRight;
        leftBDeltaTicks = leftBCurrent - prevLeftB;
        rightBDeltaTicks = rightBCurrent - prevRightB;
        avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks) / 4;
        prevLeft = leftCurrent;
        prevRight = rightCurrent;
        prevLeftB = leftBCurrent;
        prevRightB = rightBCurrent;
        Velocities[0] = frontLeftVelocity;
        Velocities[1] = frontRightVelocity;
        Velocities[2] = rearLeftVelocity;
        Velocities[3] = rearRightVelocity;

        prevTickTime = tTime;

        return Velocities;
    }


    public void runMotors() {
/*
        maxVelocTime.reset();
        resetFL();
        resetFR();
        resetRL();
        resetRR();






//todo: if run once, uncomment

        tickTime.reset();

        opmode.telemetry.addLine("Inside runMotors");
        opmode.telemetry.update();
*/

        //   opmode.sleep(1000);
        if (this.opmode.opModeIsActive()) {
            opmode.telemetry.addLine("Inside opmodeisactive");
            // opmode.telemetry.update();


            // opmode.sleep(1000);
            leftMotor.setPower(LWM * 1);
            rightMotor.setPower(RWM * 1);
            leftMotorB.setPower(LWM * 1);
            rightMotorB.setPower(RWM * 1);

            driveTime.reset();
            prevDriveTime = 0;

            moveTime.reset();

            loop:
            while (opmode.opModeIsActive()  /*&& driveTime.seconds() <= 2*/) {

                opmode.telemetry.addData("stage", ACDStage);
                // opmode.telemetry.update();
//
//                switch (ACDStage) {
//                    case (1):
//                        if(driveTime.seconds() - prevDriveTime >= rampUpTime) {
//                            MaxVeloc_20_80 += rampUpRate;
//                            prevDriveTime = driveTime.seconds();
//
//                        }
//                        if(MaxVeloc_20_80 == rampMaxVeloc - rampUpRate)
//                            break loop;
//
//                        break;
//
//                    case (2):
//                        MaxVeloc_20_80 = rampMaxVeloc;
//
//                        if (driveTime.seconds() >= coastTime)
//                            break loop;
//
//                        break;
//
//                    case (3):
//                        if(driveTime.seconds() - prevDriveTime >= rampDownTime) {
//                            MaxVeloc_20_80 -= rampDownRate;
//                            prevDriveTime = driveTime.seconds();
//
//
//                        }
//                        if(MaxVeloc_20_80 == rampMinVeloc)
//                            break loop;
//
//                        break;
//                }
//
//


                opmode.telemetry.addLine("run started");
                // opmode.telemetry.update();
                //  opmode.sleep(1000);

                opmode.telemetry.addLine("runtime reset");
                // opmode.telemetry.update();
                //   opmode.sleep(1000);


                getVelocityForCurrentLoop();


                runtime.reset();
                opmode.telemetry.addLine("Velocity gotten");
                //    opmode.telemetry.update();
                //     opmode.sleep(1000);


                MaxVeloc_20_80 = 60;
                PIDVeloc(runningAverage(Velocities[0], "FL"), LWM * MaxVeloc_20_80, "FL");
                PIDVeloc(runningAverage(Velocities[1], "FR"), RWM * MaxVeloc_20_80, "FR");
                PIDVeloc(runningAverage(Velocities[2], "RL"), LWM * MaxVeloc_20_80, "RL");
                PIDVeloc(runningAverage(Velocities[3], "RR"), RWM * MaxVeloc_20_80, "RR");

                opmode.telemetry.addLine("PID done");
                //  opmode.telemetry.update();
                //      opmode.sleep(1000);

                leftMotor.setPower(FLVeloc);
                rightMotor.setPower(FRVeloc);
                leftMotorB.setPower(RLVeloc);
                rightMotorB.setPower(RRVeloc);

//                leftMotor.setPower(-1);
//                rightMotor.setPower(1);
//                leftMotorB.setPower(-1);
//                rightMotorB.setPower(1);

                /*
                leftMotor.setPower(Velocities[0] * 0.07);
                rightMotor.setPower(Velocities[1] * 0.07);
                leftMotorB.setPower(Velocities[2] * 0.07);
                rightMotorB.setPower(Velocities[3] * 0.07);
                */

                opmode.telemetry.addData("FLPower", FLVeloc);
                opmode.telemetry.addData("FRPower", FRVeloc);
                opmode.telemetry.addData("RLPower", RLVeloc);
                opmode.telemetry.addData("RRPower", RRVeloc);

                opmode.telemetry.addLine("power set");
                // opmode.telemetry.update();
                //opmode.sleep(1000);
                opmode.telemetry.addLine("looptime gotten");
                // opmode.telemetry.update();
                // opmode.sleep(1000);


                // opmode.telemetry.addData("Motor speeeeeedd", FLVeloc);
                opmode.telemetry.addData("VelocityFL", Velocities[0]);
                opmode.telemetry.addData("VelocityFR", Velocities[1]);
                opmode.telemetry.addData("VelocityRL", Velocities[2]);
                opmode.telemetry.addData("VelocityRR", Velocities[3]);
                opmode.telemetry.addData("Average motor POWAAAH", ((leftMotor.getPower() + leftMotorB.getPower() + rightMotor.getPower() +
                        rightMotorB.getPower()) / 4));
                opmode.telemetry.update();

                // opmode.sleep(40);

                  //  RobotLog.d("Runtime - %f, FL Power - %f, FR Power - %f, RL Power - %f, RR Power - %f", moveTime.seconds(), FLVeloc, FRVeloc, RLVeloc, RRVeloc);
                if(debugFlag2){
                  RobotLog.d("Runtime - %f, Target Veloc - %f, FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f, FLP - %f, FLI - %f, RRP - %f, RRI - %f", moveTime.seconds(), MaxVeloc_20_80, -Velocities[0], Velocities[1], -Velocities[2], Velocities[3], -FLP, -FLI, RRP, RRI);
                }
//                if(debugFlag2)
           //       RobotLog.d("Runtime - %f, Target Veloc - %f, FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f", moveTime.seconds(), MaxVeloc_20_80, Velocities[0], Velocities[1], Velocities[2], Velocities[3]);
                //   RobotLog.d("Runtime - %f, FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f, FL Power - %f, FR Power - %f, RL Power - %f, RR Power - %f, FLOut - %f, FROut - %f, RLOut - %f, RROut - %f", moveTime.seconds(), Velocities[0], Velocities[1], Velocities[2], Velocities[3], leftMotor.getPower(), rightMotor.getPower(), leftMotorB.getPower(), rightMotorB.getPower(), FLVeloc, FRVeloc, RLVeloc, RRVeloc);
//                RobotLog.d("Runtime - %f, Target Veloc - %f, FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f, delta time - %f, FL delta ticks - %d, FR delta ticks - %d, RL delta ticks - %d, RR delta ticks - %d, Ave Delta Ticks - %d",
//                           moveTime.seconds(), MaxVeloc_20_80, Velocities[0], Velocities[1], Velocities[2], Velocities[3], deltaTickTime, leftDeltaTicks, rightDeltaTicks, leftBDeltaTicks, rightBDeltaTicks, avgDeltaTicks);
            //    RobotLog.d("Runtime - %f, FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f, FL Power - %f, FR Power - %f, RL Power - %f, RR Power - %f", moveTime.seconds(), Velocities[0], Velocities[1], Velocities[2], Velocities[3],  FLVeloc, FRVeloc, RLVeloc, RRVeloc);

            }
        }
        opmode.telemetry.addLine("run ended");
        //opmode.telemetry.update();
        //  opmode.sleep(1000);
    }

    public void initializeHardware() {
        //Initialize Motors
        this.leftMotor = this.hardwareMap.dcMotor.get("Front_Left_Motor");
        this.rightMotor = this.hardwareMap.dcMotor.get("Front_Right_Motor");
        this.leftMotorB = this.hardwareMap.dcMotor.get("Rear_Left_Motor");
        this.rightMotorB = this.hardwareMap.dcMotor.get("Rear_Right_Motor");
        //this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //this.leftMotorB.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public double runningAverage(double inputValue, String motor) {

        switch (motor) {
            case ("FL"):
                rAverage = FLrAverage;
                break;
            case ("FR"):
                rAverage = FRrAverage;
                break;
            case ("RL"):
                rAverage = RLrAverage;
                break;
            case ("RR"):
                rAverage = RRrAverage;
                break;
        }


        rAverage.add(inputValue);

        //if (rAverage.size() > 4)
        rAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += rAverage.get(i);

        opmode.telemetry.addData("sum", sum);



        switch (motor) {
            case ("FL"):
                FLrAverage = rAverage;
                break;
            case ("FR"):
                FRrAverage = rAverage;
                break;
            case ("RL"):
                RLrAverage = rAverage;
                break;
            case ("RR"):
                RRrAverage = rAverage;
                break;
        }

        return inputValue;
        //return sum/4;


    }
}