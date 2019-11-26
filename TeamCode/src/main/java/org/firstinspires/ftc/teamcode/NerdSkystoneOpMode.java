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
import java.util.HashMap;

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
@Autonomous(name="NerdSkystoneOpMode", group="Linear Opmode")
//@Disabled
public class NerdSkystoneOpMode extends LinearOpMode {
    private NerdBOT myNerdBOT;
    private NerdArmMove Arm;
    static private VuforiaFindCase2 VFC;
    boolean debugFlag = false;
    private int Skystone_Position = 0;
    private double foundation_distance=82.0;
    private HashMap<Integer, NerdSkystone> skyStonesMap = new HashMap<Integer, NerdSkystone>();
    private final int X_DIRECTION = 1; // 1 For Red Alliance, -1 for Blue
    private final int MAX_BLOCK_DROPS=3 ; // How many blocks will be delivered to the foundation.

    @Override
    public void runOpMode() {

        myNerdBOT = new NerdBOT(this);
        Arm = new NerdArmMove(this);
        VFC = new VuforiaFindCase2(this);
        myNerdBOT.setDebug(debugFlag);

        //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
        VFC.initVuforia();

        //Initialize the PID Calculators
        myNerdBOT.initializeXPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.015, 0.000, 0.0, debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.0075, 0.000, 0.0, debugFlag);

        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0, 0.5);

        telemetry.addData("Init", "Completed");
        telemetry.update();

        waitForStart();
        //Move forward and detect Skystone
        myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 12.5, 0.0);
        Skystone_Position = VFC.vuforia();

        //Based on the Skystone position detected by vuforia, set the pickup order and offsets
        stonesOrder(Skystone_Position);

        NerdSkystone currentSkyStone;
        NerdSkystone nextSkyStone;
        double dropDistance;
        double pickupDistance;

        for (int dropNumber = 1; dropNumber <= MAX_BLOCK_DROPS; dropNumber++ ) {
            currentSkyStone = skyStonesMap.get(dropNumber);

            //Drop the first arm
            Arm.ArmLoop(-170, 7, 0.8, 0.5); // -160, 0.5

            //Move to the block to be picked.
            if(dropNumber == 1) {
                //Only first run we may have to use X and Y based on Skystone position, since we add
                // the offsets gathered from this to the total pickup distance for the next stone
                myNerdBOT.nerdPidDrive(X_DIRECTION * currentSkyStone.getX_offset(), 12.5, 0.0, false, false);
            }
            else{
                myNerdBOT.nerdPidDrive(0, 7.5, 0.0, false, false);

            }

            //Pickup the block
            Arm.ArmLoop(-170,140, 0.5, 0.8); // grab 1
            Arm.ArmLoop(-10,7, 0.6, 0.2); // home

            // Move Back
  //          myNerdBOT.nerdPidDrive( 0.0, -8.0, 0); // move ack to miss bridge

            //We reduce the distance to foundation every run.
            // We drop the first skystone at farthest distance, and then work our way backwards to prevent slipping.
            if (dropNumber > 1) {
                foundation_distance = foundation_distance - 8;
            }
            dropDistance = foundation_distance + currentSkyStone.getX_offset();

            //Move to the foundation
            myNerdBOT.setMinMaxSpeeds(0.0,1); // Go faster when going longer distance.
            myNerdBOT.nerdPidDrive(  X_DIRECTION*-dropDistance, -8, 0.0); // go to foundation myNerdBOT.setMinMaxSpeeds(0.0,0.3);// go slower for more precise tasks

            //Approach slowly to foundation
            myNerdBOT.setMinMaxSpeeds(0.0,0.5);
            myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 9.0, 0.0, true, false); // approach foundation

            //Drop the blocks
            Arm.ArmLoop(-60,135, 0.2, 0.6); // half-drop
            Arm.ArmLoop(-160,143, 0.5, 0.8);// put down the block
            Arm.ArmLoop(-160,7, 0.5, 0.5);  // home front arm
            Arm.ArmLoop(-10,7, 0.5, 0.5); // home arms

            //Move back from Foundation
            //myNerdBOT.nerdPidDrive(X_DIRECTION*0.0, -11.0, 0); // back up from foundation so we can move back to blocks without hitting foundation

            if(dropNumber < MAX_BLOCK_DROPS) {
                //Get the offset from next skystone and calculate the distance to next stone to be picked up.
                nextSkyStone = skyStonesMap.get(dropNumber + 1);
                pickupDistance = foundation_distance + nextSkyStone.getX_offset();

                //Move to the next Stone
                myNerdBOT.setMinMaxSpeeds(0.0, 1); // go at faster speed for long distances
                myNerdBOT.nerdPidDrive(X_DIRECTION * (pickupDistance), -5, 0); // go to other side of the field
            }
            else{
                //If this is last block to be transferred, park.
                myNerdBOT.setMinMaxSpeeds(0.0, 1); // go at faster speed for long distances
                myNerdBOT.nerdPidDrive(X_DIRECTION * (foundation_distance/2.0), -5, 0);

            }
        }
    }

public void stonesOrder(int Skystone_Position) {
        if (Skystone_Position == 1) {
            //Order of priority for Plan B bot intake 1.0 for skystone position 1: Block 1, 4, 2, 3, 5, 6
            skyStonesMap.put(1, new NerdSkystone(1, -8));
            skyStonesMap.put(2, new NerdSkystone(4,16));
            skyStonesMap.put(3, new NerdSkystone(2, 0));
            skyStonesMap.put(4, new NerdSkystone(3, 8));
            skyStonesMap.put(5, new NerdSkystone(5,24));
            skyStonesMap.put(6, new NerdSkystone(6,32));
        }
        else if (Skystone_Position == 2 || Skystone_Position == 4) {
            //Order of priority for Plan B bot intake 1.0 for skystone position 2: Block 2, 5, 1, 3, 4, 6
            skyStonesMap.put(1, new NerdSkystone(2, 0));
            skyStonesMap.put(2, new NerdSkystone(5,24));
            skyStonesMap.put(3, new NerdSkystone(1, -8));
            skyStonesMap.put(4, new NerdSkystone(3, 8));
            skyStonesMap.put(5, new NerdSkystone(4,16));
            skyStonesMap.put(6, new NerdSkystone(6,32));
        }
        else {
            //Order of priority for Plan B bot intake 1.0 for skystone position 3: Block 3, 1, 2, 4, 5, 6
            skyStonesMap.put(1, new NerdSkystone(3, 8));
            skyStonesMap.put(2, new NerdSkystone(1,-8));
            skyStonesMap.put(3, new NerdSkystone(2, 0));
            skyStonesMap.put(4, new NerdSkystone(4, 16));
            skyStonesMap.put(5, new NerdSkystone(5,24));
            skyStonesMap.put(6, new NerdSkystone(6,32));

        }
}


}

