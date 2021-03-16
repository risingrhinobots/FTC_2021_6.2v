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

package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Motor test", group = "UltimateGame")
//@Disabled
public class Motot_Test_T1 extends LinearOpMode {


    boolean rampUp = true;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor InTakeMotor = null;
    private DcMotor ConveyorMotor = null;
    private DcMotor LeftShooter = null;
    private DcMotor RightShooter = null;
    Servo   gateServo;
    Servo   armServo;
    Servo   gripServo;
    Servo   guideServo;
    private DistanceSensor sensorRange;

    double  armPosition, gripPosition, gatePosition,guidePosition;
    double armMinPosition, armMaxPosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;


    @Override
    public void runOpMode() {

        // define the motors
        BackLeftDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        InTakeMotor = hardwareMap.get(DcMotor.class, "InTake");
        ConveyorMotor = hardwareMap.get(DcMotor.class, "Conveyor");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
        armServo = hardwareMap.get(Servo.class,"arm");
        gripServo = hardwareMap.get(Servo.class,"grip");
        guideServo = hardwareMap.get(Servo.class,"guide");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_ods");
       // gateServo = hardwareMap.get(Servo.class,"gate");
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ConveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for start button.
        armMaxPosition=0.16;
        armMinPosition=0.025;
        armPosition=0.03;
        gripPosition=0.95;
        guidePosition=0.55;
        gatePosition=0.40;

        LeftShooter.setPower(0);
        RightShooter.setPower(0);
        armServo.setPosition(armPosition);
        telemetry.addData("Arm Position", String.format("%.01f in", armPosition));
        telemetry.update();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //This is very important to keep left drive to forward and right drive to REVERSE
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        InTakeMotor.setDirection(DcMotor.Direction.REVERSE);
        ConveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftShooter.setDirection(DcMotor.Direction.REVERSE);
        RightShooter.setDirection(DcMotor.Direction.FORWARD);

/*
       BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        InTakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ConveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double xl;
            double xr;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //Please note it is very important to note that DRIVE is set to positive and

            double drive2 = gamepad1.left_stick_y;
            double turn2 = gamepad1.right_stick_x;

            // Display the current value
            telemetry.addData("game pad2 left stick Position", "%5.2f", drive2);
            telemetry.addData("game pad2 right stick Position", "%5.2f", turn2);
            telemetry.update();

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            leftPower = Range.clip(drive*0.8  + turn*0.7 , -1, 1);
            rightPower = Range.clip(drive*0.8 - turn*0.7 , -1, 1);

            //calculations to make drive motors increase quadratically

            if(leftPower<0){
                xl = -1;
            } else{
                xl = 1;
            }

            if (rightPower<0){
                xr = -1;
            } else{
                xr = 1;
            }

            leftPower = (0.75*xl)*(leftPower*leftPower)+(0.01*leftPower);
            rightPower = (0.75*xr)*(rightPower*rightPower)+(0.01*rightPower);




/*
            if(sensorRange.getDistance(DistanceUnit.INCH) <= 2){
                leftPower = leftPower*0.1;
                rightPower = rightPower*0.1;
            }
*/
            // Send calculated power to wheels
            BackLeftDrive.setPower(leftPower);
            BackRightDrive.setPower(rightPower);
            FrontLeftDrive.setPower(leftPower);
            FrontRightDrive.setPower(rightPower);

            if(gamepad1.dpad_right){
                FrontRightDrive.setPower(1.0);
                telemetry.addLine("Front right");
            }

            if(gamepad1.dpad_left){
                FrontLeftDrive.setPower(1.0);
                telemetry.addLine("Front Left");
            }

            if(gamepad1.dpad_down){
                BackLeftDrive.setPower(1.0);
                telemetry.addLine("Back Left");
            }

            if(gamepad1.dpad_up){
                BackRightDrive.setPower(1.0);
                telemetry.addLine("Back Right");
            }

            if(gamepad1.y){
                FrontLeftDrive.setPower(0);
                FrontRightDrive.setPower(0);
                BackRightDrive.setPower(0);
                BackLeftDrive.setPower(0);
            }

            telemetry.addData("leftpower", leftPower);
            telemetry.addData("rightpower",rightPower);
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

            telemetry.update();

        }  // End of While loop
    }   // end of runopmode
} // end of program


