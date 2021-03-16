/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Blue Right", group = "UltimateGame")
//@Disabled
public class BlueRight extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    /* Declare OpMode members. */
    HardwarePushbot_TC robot   = new HardwarePushbot_TC();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.3;

    double  armPosition, gripPosition, gatePosition,guidePosition;
    double armMinPosition, armMaxPosition;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        armMaxPosition=0.16;
        armMinPosition=0.025;
        armPosition=0.03;
        gripPosition=0.95;
        guidePosition=0.55;
        gatePosition=0.40;

        robot.init(hardwareMap);

        robot.gripServo.setPosition(gripPosition);
        robot.armServo.setPosition(armPosition);
        //robot.guideServo.setPosition(guidePosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        telemetry.update();



        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if(pipeline.getAnalysis() > 155){
                telemetry.addLine("four");
                telemetry.update();
                sleep(5000);

                //Drive forward to the target BOX
                encoderDrive(DRIVE_SPEED, 110,110,110,110, 5);

                // S1: Forward 50 Inches with 5 Sec timeout
                //following is an example of left turn
                // encoderDrive(TURN_SPEED,-10,10,-10,10,4);
                //following is an example of right turn
                //turn right since this is blue corner to drop the wobble from the back of the robot
                encoderDrive(TURN_SPEED,11.5,-11.5,11.5,-11.5,4);  // S2: Turn Right 12 Inches with 4 Sec timeout
                //after turning drive backwords so that the wobble can be dropped in the target
                encoderDrive(DRIVE_SPEED,-20,-20,-20,-20,4);  // S2: Turn Right 12 Inches with 4 Sec timeout

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);

                //move forward to clear away from wobble
                encoderDrive(DRIVE_SPEED, 7,7,7,7, 5);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);

                //turn left towards the shooting target zone
                encoderDrive(TURN_SPEED, -11.5,11.5,-11.5,11.5, 5);
                //back towards out of the launch line

                encoderDrive(0.5*(DRIVE_SPEED), -50,-50,-50,-50, 5);


                //encoderDrive(TURN_SPEED,-2,2,-2,2,4);

                robot.guideServo.setPosition(guidePosition);
                robot.conveyor.setPower(1);
                robot.leftShooter.setPower(1);
                robot.rightShooter.setPower(1);
                robot.intake.setPower(1);
                sleep(6000);

                robot.conveyor.setPower(0);
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
                robot.intake.setPower(0);

                //drive to position on the launch line
                encoderDrive(DRIVE_SPEED, 10,10,10,10, 5);

                break;

            }
            else if(pipeline.getAnalysis() > 140){
                telemetry.addLine("One");
                sleep(5000);
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                // FORWARD DRIVE SAMPLE. reverse drive will be all negative values
                encoderDrive(DRIVE_SPEED, 93,93,93,93, 5);
                // S1: Forward 50 Inches with 5 Sec timeout
                //following is an example of left turn
                // encoderDrive(TURN_SPEED,-10,10,-10,10,4);
                //following is an example of right turn
                encoderDrive(TURN_SPEED,11,-11,11,-11,4);  // S2: Turn Right 12 Inches with 4 Sec timeout

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);
                encoderDrive(DRIVE_SPEED, 5,5,5,5, 5);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);


                encoderDrive(DRIVE_SPEED, -11,11,-11,11, 5);
                //back towards out of the launch line
                encoderDrive(DRIVE_SPEED, -36,-36,-36,-36, 5);
                //turn towards the goal
                encoderDrive(TURN_SPEED,-1.8,1.8,-1.8,1.8,4);

                robot.guideServo.setPosition(guidePosition);
                robot.conveyor.setPower(1);
                robot.leftShooter.setPower(1);
                robot.rightShooter.setPower(1);
                robot.intake.setPower(1);
                sleep(6000);

                robot.conveyor.setPower(0);
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
                robot.intake.setPower(0);

                encoderDrive(DRIVE_SPEED, 5,5,5,5, 5);
                break;
            }
            else {
                telemetry.addLine("none");
                sleep(5000);
                //Drive forward to the target BOX
                encoderDrive(DRIVE_SPEED, 65,65,65,65, 5);


                // S1: Forward 50 Inches with 5 Sec timeout
                //following is an example of left turn
                // encoderDrive(TURN_SPEED,-10,10,-10,10,4);
                //following is an example of right turn
                //turn right since this is blue corner to drop the wobble from the back of the robot
                encoderDrive(TURN_SPEED,11.5,-11.5,11.5,-11.5,4);  // S2: Turn Right 12 Inches with 4 Sec timeout
                //after turning drive backwords so that the wobble can be dropped in the target
                encoderDrive(DRIVE_SPEED,-20,-20,-20,-20,4);  // S2: Turn Right 12 Inches with 4 Sec timeout

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);

                //move forward to clear away from wobble
                encoderDrive(DRIVE_SPEED, 6,6,6,6, 5);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);

                //turn left towards the shooting target zone
                encoderDrive(TURN_SPEED, -12,12,-12,12, 5);
                //back towards out of the launch line
                encoderDrive(DRIVE_SPEED, -11.5,-11.5,-11.5,-11.5, 5);




                //encoderDrive(TURN_SPEED,-2,2,-2,2,4);

                robot.guideServo.setPosition(guidePosition);
                robot.conveyor.setPower(1);
                robot.leftShooter.setPower(1);
                robot.rightShooter.setPower(1);
                robot.intake.setPower(1);
                sleep(6000);

                robot.conveyor.setPower(0);
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
                robot.intake.setPower(0);
                //drive to position on the launch line
                encoderDrive(DRIVE_SPEED, 10,10,10,10, 5);

                break;
            }
            // Don't burn CPU cycles busy-looping in this sample
            //sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(45,135);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches,
                             double backleftInches, double backrightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = (int)(frontrightInches * COUNTS_PER_INCH);
            newbackLeftTarget = (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = (int)(backrightInches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newfrontLeftTarget);
            robot.frontRight.setTargetPosition(newfrontRightTarget);
            robot.backLeft.setTargetPosition(newfrontLeftTarget);
            robot.backRight.setTargetPosition(newfrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}