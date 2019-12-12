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
import com.qualcomm.robotcore.util.RobotLog;

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
@Autonomous(name="NerdFullAuton_Red", group="Final")
//@Disabled
public class
NerdFullAuton_Red extends LinearOpMode {
    private NerdBOT myNerdBOT ;
    private NerdArmMove Arm;
    private double Skystone_Position = 0;
    private double position_run3_x = 82.0;
    private double offset_x_run3 = 0;
    private  double run3_x = 0;
    private  double drop_2_offset = 0;
    boolean debugFlag = false;
   // static private VuforiaFindCase2Blue VFC;
    static private OpenCVSkyStone VFC;

    private double x_offset_2 = 0;

    private final int X_DIRECTION = -1; // 1 For Red Alliance, -1 for Blue
    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myNerdBOT = new NerdBOT(this);
        Arm = new NerdArmMove(this);
        myNerdBOT.setDebug(debugFlag);
        //VFC = new VuforiaFindCase2Blue(this);
        VFC = new OpenCVSkyStone(this);


        //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
        VFC.initOpenCVSkyStone();
        //Initialize the PID Calculators
        myNerdBOT.initializeXPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.0, 0.0,debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.015, 0.000, 0.0,debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.0075, 0.000, 0.0,debugFlag);
        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0,0.5);


        telemetry.addData("Init", "Completed");
        telemetry.update();


        waitForStart();
        myNerdBOT.resetAngle();
        //made change here

        //UNITS ARE IN INCHES
        if (debugFlag);
        RobotLog.d("NerdSampleOpMode - Run1");

      //  myNerdBOT.nerdPidDrive(X_DIRECTION*0.0, 12.5, 0.0);
        Skystone_Position = VFC.findPosition();
//        telemetry.addData("Position Case",Skystone_Position );
//        telemetry.update();
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run2");


        Arm.ArmLoop(-210,7, 0.8, 0.5); // -160, 0.5

        myNerdBOT.setMinMaxSpeeds(0.0,0.4);


        if (Skystone_Position == 1) {
            myNerdBOT.nerdPidDrive(X_DIRECTION*5.0, 26.0, 0.0, false, false);
            offset_x_run3 = 8.0;
            drop_2_offset = -36.0;//CHANGED THIS
        }
        else if (Skystone_Position == 2 || Skystone_Position == 4) {
            myNerdBOT.nerdPidDrive(3.0, 26.0, 0.0, false, false);
            offset_x_run3 = -1.5; // was going wrong direction
            drop_2_offset = -5.0;
        }
        else if (Skystone_Position == 3) {
            myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 10.0, 0.0, false, false); //13.5

            myNerdBOT.nerdPidDrive( X_DIRECTION*-10.0, 16.0, 0.0, false, false); //13.5
            offset_x_run3 = -11.0;//-7
            drop_2_offset = -15.0; // -5, -14
        }
        else
        {
//            myNerdBOT.nerdPidDrive(speed, X_DIRECTION*0.0, 13.5, 0.0, false, false);
//            offset_x_run3 = 0.0;
//            drop_2_offset = 2.0;
//            sleep(2000);

            myNerdBOT.nerdPidDrive(1.5, 26.0, 0.0, false, false);
            offset_x_run3 = -1.5; // was going wrong direction
            drop_2_offset = -2.0;

        }



        Arm.ArmLoop(-210,160, 0.5, 1.0); // grab 1
        //sleep(500);
        Arm.ArmLoop(210,-10, 1.0, 0.2); // home
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run3");


        myNerdBOT.nerdPidDrive( 0.0, -5.0, 0); // move ack to miss bridge

        myNerdBOT.setMinMaxSpeeds(0.0,0.85); // Go faster when going longer distance.
        run3_x = (position_run3_x +offset_x_run3);
        myNerdBOT.nerdPidDrive(  X_DIRECTION*-run3_x, 0.0, 0.0, true, false); // go to foundation

        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run4");

        myNerdBOT.setMinMaxSpeeds(0.0,0.3);// go slower for more precise tasks


        myNerdBOT.nerdPidDrive(  X_DIRECTION*0.0, 9.0, 0.0, true, false); // approach foundation

        myNerdBOT.setMinMaxSpeeds(0.0,0.4);


   //     Arm.ArmLoop(-60,135, 0.2, 0.6); // half-drop
        Arm.ArmLoop(-210,160, 0.5, 1.0);// put down the block
        Arm.ArmLoop(-210,-10, 0.5, 0.5);  // home front arm



        myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, -2, 0); // back up to miss nub

        Arm.UseTheForce(); // put arm down at half force

        // sleep(500);

        myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, -34.0, 0); // pull foundation

        Arm.KillTheForce();

        Arm.ArmLoop(10,-10, 0.8, 0.5); // home arms



        myNerdBOT.setMinMaxSpeeds(0.0,0.7); // go at faster speed for long distances

        myNerdBOT.nerdPidDrive( X_DIRECTION*22.0, 0.5, 0); // move to get away from the foundation
        myNerdBOT.nerdPidDrive( X_DIRECTION*21.0, 16.5, 0); // strafe to miss [parked] opponent CHANGED THIS


        myNerdBOT.nerdPidDrive( X_DIRECTION*(61 + drop_2_offset), 0.0, 0); // go to other side of the field

        Arm.ArmLoop(-210,-10,0.8,0.5); // drop one arm

        myNerdBOT.setMinMaxSpeeds(0.0,0.3); // go slower for more precise tasks

        myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 8.5, 0); //changed this

        myNerdBOT.setMinMaxSpeeds(0.0,0.5);

//        Arm.ArmLoop(-160,143, 0.5, 0.8);// put down the block
//        Arm.ArmLoop(-10,7, 0.8, 0.2);  // squeeze foundation and return front arm up

        Arm.ArmLoop(-210,160, 0.5, 1.0);// drop second arm
        Arm.ArmLoop(210,-10, 1.0, 0.2);  // return both arms up


        myNerdBOT.setMinMaxSpeeds(0.0,0.85); //
        myNerdBOT.nerdPidDrive( X_DIRECTION*-75 - X_DIRECTION*drop_2_offset, -6.5, 0); //CHANGED THIS
        //myNerdBOT.setMinMaxSpeeds(0.0,0.7);

        myNerdBOT.nerdPidTurn( X_DIRECTION*90);

        myNerdBOT.nerdPidDrive( X_DIRECTION*-4, 22, X_DIRECTION*90, false, false); // 19 CHANGED THIS; 21

        //   Arm.ArmLoop(-60,135, 0.2, 0.6); // half-drop
        Arm.ArmLoop(-210,160, 0.5, 1.0);// put down the block
        Arm.ArmLoop(-160,-10, 0.5, 0.5);  // return rear arm up
        Arm.ArmLoop(10,-10, 0.5, 0.5);  // return both arm up

        myNerdBOT.nerdPidDrive( X_DIRECTION*5, -27, X_DIRECTION*90); //park //CHANGED THIS; -27


        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Completed");
    }
}