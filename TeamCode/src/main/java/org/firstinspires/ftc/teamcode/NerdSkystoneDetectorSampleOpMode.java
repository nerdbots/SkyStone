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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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
@Autonomous(name="NerdSkystoneDetectorSampleOpMode", group="Linear Opmode")
@Disabled
public class NerdSkystoneDetectorSampleOpMode extends LinearOpMode {
    private NerdBOT myNerdBOT;
    private NerdSkyStoneDetector nerdSkyStoneDetector;
    private  OpenCVSkyStone openCVSkyStoneDetector;
    private NerdArmMove Arm;
    private int X_DIRECTION = 1;

    private boolean debugFlag = false;


    @Override
    public void runOpMode() {

        int skyStonePosition;
        myNerdBOT = new NerdBOT(this);
        Arm = new NerdArmMove(this);
       // nerdSkyStoneDetector = new NerdSkyStoneDetector(this);
        openCVSkyStoneDetector = new OpenCVSkyStone(this);
        //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
       // nerdSkyStoneDetector.initNerdStoneDetector();
        openCVSkyStoneDetector.initOpenCVSkyStone();
        //Initialize the PID Calculators

        myNerdBOT.initializeXPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.0, 0.0,debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.015, 0.000, 0.0, debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.015, 0.000, 1.4,debugFlag);//0.02535

        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0, 0.4);

        telemetry.addData("Init", "Completed");
        telemetry.update();

        waitForStart();

        //double [] skyStoneXYPValues;

       // myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 16.0, 0.0);

//        skyStoneXYPValues = nerdSkyStoneDetector.detectSkyStone();
//
//        telemetry.addData("OpMode Stone Position X", skyStoneXYPValues[0]);
//        telemetry.addData("OpMode Stone Position Y", skyStoneXYPValues[1]);
//        telemetry.addData("OpMode Stone Position", skyStoneXYPValues[2]);
//        telemetry.update();

        skyStonePosition = openCVSkyStoneDetector.findPosition();

        telemetry.addData("Sky Stone Position", skyStonePosition);
        telemetry.update();

        sleep(50000);
    }




}

