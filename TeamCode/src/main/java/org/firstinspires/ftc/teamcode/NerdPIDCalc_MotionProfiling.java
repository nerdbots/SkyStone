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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Timer;
import java.util.TimerTask;
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
public class NerdPIDCalc_MotionProfiling {
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
    private double FLkP;
    private double FLkI;
    private double FLkD;
    private ElapsedTime FLPIDTime = new ElapsedTime();
    private double FRPrevError = 0;
    private double FRTotalError = 0;
    private double FRVeloc = 0;
    private double FRMaxVeloc = 10;
    private double FRkP;
    private double FRkI;
    private double FRkD;
    private ElapsedTime FRPIDTime = new ElapsedTime();
    private double RLPrevError = 0;
    private double RLTotalError = 0;
    private double RLVeloc = 0;
    private double RLMaxVeloc = 10;
    private double RLkP;
    private double RLkI;
    private double RLkD;
    private ElapsedTime RLPIDTime = new ElapsedTime();
    private double RRPrevError = 0;
    private double RRTotalError = 0;
    private double RRVeloc = 0;
    private double RRMaxVeloc = 10;
    private double RRkP;
    private double RRkI;
    private double RRkD;
    private ElapsedTime RRPIDTime = new ElapsedTime();

    private ElapsedTime driveTime = new ElapsedTime();

    private ElapsedTime tickTime = new ElapsedTime();

    public double [] Velocities = new double[4];

    private double deltaTickTime = 0;

    private double prevTickTime = 0;

    private double tTime = 0;

    private boolean debugFlag = true;


    //wwd
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
    public double MaxVeloc_20_80 = 20;
    ElapsedTime maxVelocTime = new ElapsedTime();
    public NerdPIDCalc_MotionProfiling(LinearOpMode opmode) {
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
        int MaxError = 10;
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
        if (Math.abs(speed) > MaxVeloc) {
            if (speed > 0) {
                speed = MaxVeloc;
            } else {
                speed = -MaxVeloc;
            }
        }
        PrevError = error;

        // opmode.telemetry.addData("speed", speed);



        switch (Motor) {
            case "FL":
                FLTotalError = TotalError;
                FLPrevError = PrevError;
                FLVeloc = speed;
                opmode.telemetry.addData("errorFL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d ("Time - %f, FL error - %f, FL Target Veloc - %f, FL Input Veloc - %f, FL Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "FR":
                FRTotalError = TotalError;
                FRPrevError = PrevError;
                FRVeloc = speed;
                opmode.telemetry.addData("errorFR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d ("Time - %f, FR error - %f, FR Target Veloc - %f, FR Input Veloc - %f, FR Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "RL":
                RLTotalError = TotalError;
                RLPrevError = PrevError;
                RLVeloc = speed;
                opmode.telemetry.addData("errorRL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d ("Time - %f, RL error - %f, RL Target Veloc - %f, RL Input Veloc - %f, RL Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed, maxVelocTime.seconds());
                break;
            case "RR":
                RRTotalError = TotalError;
                RRPrevError = PrevError;
                RRVeloc = speed;
                opmode.telemetry.addData("errorRR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);
                if (debugFlag)
                    RobotLog.d ("Time - %f, RR error - %f, RR Target Veloc - %f, RR Input Veloc - %f, RR Output Veloc - %f", maxVelocTime.seconds(), error, TVeloc, CurrVeloc, speed);
                break;
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
        moveTime.reset();
        maxVelocTime.reset();
        resetFL();
        resetFR();
        resetRL();
        resetRR();






        tickTime.reset();

        opmode.telemetry.addLine("Inside runMotors");
        opmode.telemetry.update();


        //   opmode.sleep(1000);
        if (this.opmode.opModeIsActive()) {
            opmode.telemetry.addLine("Inside opmodeisactive");
            opmode.telemetry.update();




            // opmode.sleep(1000);
            // leftMotor.setPower(1);
            //rightMotor.setPower(1);
            //leftMotorB.setPower(1);
            //rightMotorB.setPower(1);

            driveTime.reset();

            while(opmode.opModeIsActive() && driveTime.seconds() <= 5) {
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
                PIDVeloc(Velocities[0], -100, "FL");
                PIDVeloc(Velocities[1], 100, "FR");
                PIDVeloc(Velocities[2], -100, "RL");
                PIDVeloc(Velocities[3], 100, "RR");

                opmode.telemetry.addLine("PID done");
                //  opmode.telemetry.update();
                //      opmode.sleep(1000);

                leftMotor.setPower(FLVeloc);
                rightMotor.setPower(FRVeloc);
                leftMotorB.setPower(RLVeloc);
                rightMotorB.setPower(RRVeloc);

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
                //opmode.telemetry.update();
                //  opmode.sleep(1000);
               /* if(maxVelocTime.seconds() > 10 && maxVelocTime.seconds() <= 20) {
                    MaxVeloc_20_80 = 80;
                } else {
                    MaxVeloc_20_80 = 20;
                }*/

                // opmode.telemetry.addData("Motor speeeeeedd", FLVeloc);
                opmode.telemetry.addData("VelocityFL", Velocities[0]);
                opmode.telemetry.addData("VelocityFR", Velocities[1]);
                opmode.telemetry.addData("VelocityRL", Velocities[2]);
                opmode.telemetry.addData("VelocityRR", Velocities[3]);
                opmode.telemetry.update();
                // opmode.sleep(40);


            }
        }
        opmode.telemetry.addLine("run ended");
        //opmode.telemetry.update();
        //  opmode.sleep(1000);
    }
    public void initializeHardware(){
        //Initialize Motors
        this.leftMotor = this.hardwareMap.dcMotor.get("Front_Left_Motor");
        this.rightMotor = this.hardwareMap.dcMotor.get("Front_Right_Motor");
        this.leftMotorB = this.hardwareMap.dcMotor.get("Rear_Left_Motor");
        this.rightMotorB = this.hardwareMap.dcMotor.get("Rear_Right_Motor");
        //this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //this.leftMotorB.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
