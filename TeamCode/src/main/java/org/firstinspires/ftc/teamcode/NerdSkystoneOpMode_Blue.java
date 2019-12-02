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
@Autonomous(name="NerdSkystoneOpMode_Blue", group="Linear Opmode")
//@Disabled
public class NerdSkystoneOpMode_Blue extends LinearOpMode {
    private NerdBOT myNerdBOT;
    private NerdArmMove Arm;
    private double[] SkystoneLocationArray = new double[3];
    boolean debugFlag = false;
    private double foundation_distance=90.0;
    private HashMap<Integer, NerdSkystone> skyStonesMap = new HashMap<Integer, NerdSkystone>();
    private final int X_DIRECTION = 1; // 1 For Red Alliance, -1 for Blue
    private final int MAX_BLOCK_DROPS=3 ; // How many blocks will be delivered to the foundation.
    private VuforiaFindLocation VFC;
    double Skystone_Position=2;
    private final double FOUNDATION_OFFSET_FOR_LAST_DROP=20.0;
    private final double ARM_OFFSET=0.0;

    @Override
    public void runOpMode() {

        myNerdBOT = new NerdBOT(this);
        Arm = new NerdArmMove(this);
        VFC = new VuforiaFindLocation(this);
        myNerdBOT.setDebug(debugFlag);

        //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
        VFC.initVuforia();

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
        //Move forward and detect Skystone
        myNerdBOT.nerdPidDrive( X_DIRECTION*0.0, 16.0, 0.0);
        SkystoneLocationArray = VFC.vuforia();
        Skystone_Position = SkystoneLocationArray[2];

        //Based on the Skystone position detected by vuforia, set the pickup order and offsets
        stonesOrder(Skystone_Position);

        NerdSkystone currentSkyStone;
        NerdSkystone nextSkyStone;
        double dropDistance;
        double pickupDistance;

        for (int dropNumber = 1; dropNumber <= MAX_BLOCK_DROPS; dropNumber++ ) {
            currentSkyStone = skyStonesMap.get(dropNumber);

            myNerdBOT.setMinMaxSpeeds(0.0, 0.4);

            //Drop the first arm
            Arm.ArmLoop(-170, 7, 0.8, 0.5); // -160, 0.5

            //Move to the block to be picked.
            if(dropNumber == 1) {
                //Only first run we will use X and Y from vuforia output. We add
                // the offsets based on positions to the total pickup distance for the next stone
                myNerdBOT.nerdPidDrive(SkystoneLocationArray[0], SkystoneLocationArray[1], 0.0, false, false);

            }
            else if (dropNumber == MAX_BLOCK_DROPS){
                myNerdBOT.nerdPidDrive(0, 8.0, 0.0, false, false);

            }
            else{
                myNerdBOT.nerdPidDrive(0, 7.0, 0.0, false, false);

            }
            //Pickup the block
            Arm.ArmLoop(-170,160, 0.5, 0.8); // grab 1
            Arm.ArmLoop(-10,7, 0.6, 0.2); // home


            //We reduce the distance to foundation every run.
            // We drop the first skystone at farthest distance, and then work our way backwards to prevent slipping.
            if (dropNumber > 1) {
                foundation_distance = foundation_distance - 8;
            }
            dropDistance = foundation_distance + X_DIRECTION*currentSkyStone.getX_offset();

            //For Last block, foundation has moved, so change the drop distance accordingly
            if(dropNumber == MAX_BLOCK_DROPS)
                dropDistance=dropDistance*0.5 + FOUNDATION_OFFSET_FOR_LAST_DROP;

            //For longer distance in X direction, we change the  gains and speed.
            setPIDGainsForRampUpDown();
            myNerdBOT.setMinMaxSpeeds(0.0,1);
            double ydistance = -8.0;
            if(dropNumber == MAX_BLOCK_DROPS) ydistance=ydistance-1;

            myNerdBOT.nerdPidDriveWithRampUpDown(  X_DIRECTION*-dropDistance, ydistance, 0, false, false); // go to foundation myNerdBOT.setMinMaxSpeeds(0.0,0.3);// go slower for more precise tasks

            //Reset Z PID gains for shorter travel in Y direction

            setPIDGainsForShortDistances();


            if(dropNumber < MAX_BLOCK_DROPS) {
                // If this is not the last block, we move to foundation and then drop
                myNerdBOT.setMinMaxSpeeds(0.0, 0.5);
                myNerdBOT.nerdPidDrive(X_DIRECTION * 0.0, 8.0, 0.0, true, false); // approach foundation

                //Drop the blocks
                Arm.ArmLoop(-60,135, 0.2, 0.6); // half-drop
                Arm.ArmLoop(-160,143, 0.5, 0.8);// put down the block
//                Arm.ArmLoop(-160,7, 0.5, 0.5);  // home front arm
//                Arm.ArmLoop(-10,7, 0.5, 0.5); // home arms

                //Get the offset from next skystone and calculate the distance to next stone to be picked up.
                    nextSkyStone = skyStonesMap.get(dropNumber + 1);
                    pickupDistance = foundation_distance + X_DIRECTION*nextSkyStone.getX_offset()- ARM_OFFSET;

                //For longer distance in X direction, we change the Z gains and speed.
                 setPIDGainsForRampUpDown();
                 myNerdBOT.setMinMaxSpeeds(0.0, 1);

                //For moving arm and robot together
                myNerdBOT.nerdArm.resetArm();
                myNerdBOT.nerdPidDriveWithRampUpDownWithArmAction(X_DIRECTION * (pickupDistance), -8.5, 0, false, false, 4); // go to other side of the field

            setPIDGainsForShortDistances();

            }else{
                //If it is last block, turn and drop and come back to Park

                myNerdBOT.nerdPidTurn(X_DIRECTION*90);

                //Drop the blocks
//                Arm.ArmLoop(-60,135, 0.2, 0.6); // half-drop
                Arm.ArmLoop(-160,143, 0.5, 0.8);// put down the block
//                Arm.ArmLoop(-160,7, 0.5, 0.5);  // home front arm
//                Arm.ArmLoop(-10,7, 0.5, 0.5); // home arms

               // myNerdBOT.nerdPidDrive(4.0,-10,90);
                setPIDGainsForRampUpDown();
                myNerdBOT.nerdPidDriveWithRampUpDownWithArmAction(X_DIRECTION*4.0,-20,X_DIRECTION*90,false,false,4);

                setPIDGainsForShortDistances();
            }

            setPIDGainsForShortDistances();

        }
    }

public void stonesOrder(double Skystone_Position) {
        if (Skystone_Position == 1) {
            //Order of priority for Plan B bot intake 1.0 for skystone position 1: Block 1, 4, 2, 3, 5, 6
            skyStonesMap.put(1, new NerdSkystone(1, -8));
            skyStonesMap.put(2, new NerdSkystone(4,16));
            skyStonesMap.put(3, new NerdSkystone(2, 0));
            skyStonesMap.put(4, new NerdSkystone(3, 8));
            skyStonesMap.put(5, new NerdSkystone(5,24));
            skyStonesMap.put(6, new NerdSkystone(6,32));
        }
        else if (Skystone_Position == 2) {
            //Order of priority for Plan B bot intake 1.0 for skystone position 2: Block 2, 5, 1, 3, 4, 6
            skyStonesMap.put(1, new NerdSkystone(2, 0));
            skyStonesMap.put(2, new NerdSkystone(5,24));
            skyStonesMap.put(3, new NerdSkystone(1, -8));
            skyStonesMap.put(4, new NerdSkystone(3, 8));
            skyStonesMap.put(5, new NerdSkystone(4,16));
            skyStonesMap.put(6, new NerdSkystone(6,32));
        }
        else { // For position 3 OR if not detected.
            //Order of priority for Plan B bot intake 1.0 for skystone position 3: Block 3, 1, 2, 4, 5, 6
            skyStonesMap.put(1, new NerdSkystone(3, 8));
            skyStonesMap.put(2, new NerdSkystone(1,-8));
            skyStonesMap.put(3, new NerdSkystone(2, 0));
            skyStonesMap.put(4, new NerdSkystone(4, 16));
            skyStonesMap.put(5, new NerdSkystone(5,24));
            skyStonesMap.put(6, new NerdSkystone(6,32));

        }


}

    public void setPIDGainsForRampUpDown() {
        myNerdBOT.setZPIDGains(0.3, 0.3, 0.0);
        myNerdBOT.setXPIDGains(0.0025, 0.005, 0.0);
        myNerdBOT.setYPIDGains(0.0025, 0.005, 0.0);
    }
    public void setPIDGainsForShortDistances() {
        myNerdBOT.setXPIDGains(0.0025, 0.0, 0.0);
        myNerdBOT.setYPIDGains(0.0025, 0.0, 0.0);
        myNerdBOT.setZPIDGains(0.015, 0.0, 0.0);
    }
}

