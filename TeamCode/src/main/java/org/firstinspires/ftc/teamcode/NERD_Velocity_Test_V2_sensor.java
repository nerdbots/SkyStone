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
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Autonomous(name="NerdVelocSense", group="a mogh")

public class NERD_Velocity_Test_V2_sensor extends LinearOpMode {
    private NerdPIDCalc_MotionProfiling VPID;

    private ElapsedTime ETime = new ElapsedTime();


    private final int X_DIRECTION = -1; // 1 For Red Alliance, -1 for Blue

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        VPID = new NerdPIDCalc_MotionProfiling(this);

        VPID.setFLGains(0.00732, 0.01, 0.0);
        VPID.setFRGains(0.00732, 0.01, 0.0);
        VPID.setRLGains(0.00732, 0.01, 0.0);
        VPID.setRRGains(0.00732, 0.01, 0.0);

        VPID.initializeHardware();

        waitForStart();



            for (double i = 0.1; i <= 1; i += 0.1) {

                ETime.reset();

                VPID.leftMotor.setPower(i);
                VPID.rightMotor.setPower(i);
                VPID.leftMotorB.setPower(i);
                VPID.rightMotorB.setPower(i);

                while (ETime.seconds() <= 1) {

                    VPID.getVelocityForCurrentLoop();

                    RobotLog.d("FL Veloc - %f, FR Veloc - %f, RL Veloc - %f, RR Veloc - %f", VPID.Velocities[0], VPID.Velocities[1], VPID.Velocities[2], VPID.Velocities[3]);

                    telemetry.addData("FL Velocc", VPID.Velocities[0]);
                    telemetry.addData("FR Velocc", VPID.Velocities[1]);
                    telemetry.addData("RL Velocc", VPID.Velocities[2]);
                    telemetry.addData("RR Velocc", VPID.Velocities[3]);
                    telemetry.update();





            }
        }
    }
}