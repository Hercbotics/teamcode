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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="HardwareDebbuging", group="Iterative Opmode")
public class HardwareDebugging extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor dc_Motor1 = null;
    private DcMotor dc_Motor2 = null;
    private Servo   servo1    = null;
    private Servo   servo2    = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dc_Motor1 = hardwareMap.get(DcMotor.class, "dc_Motor1");
        dc_Motor2 = hardwareMap.get(DcMotor.class, "dc_Motor2");
        servo1    = hardwareMap.get(Servo.class, "servo1");
        servo2    = hardwareMap.get(Servo.class, "servo2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dc_Motor1.setDirection(DcMotor.Direction.FORWARD);
        dc_Motor2.setDirection(DcMotor.Direction.FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each Power1 wheel to save power level for telemetry
        double Motor1Power;
        double Motor2Power;
        double Posi = 0;



        // Choose to Power1 using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to Power2.
        // - This uses basic math to combine motions and is easier to Power1 straight.
        double Power1 = gamepad1.left_stick_y;
        double Power2 = gamepad1.right_stick_x;
        Motor1Power   = Range.clip(Power1, -1.0, 1.0) ;
        Motor2Power   = Range.clip(Power2, -1.0, 1.0) ;

        float Servo_Close = gamepad1.right_trigger;
        float Servo_Open  = gamepad1.left_trigger;
        while (Servo_Close > 0){
            Posi = Range.clip(Posi + 0.1, 0, 1.0);
        }
        while (Servo_Open > 0){
            Posi = Range.clip(Posi - 0.1, 0, 1.0);
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to Power1 forward slowly and keep straight.
        // Motor1Power  = -gamepad1.left_stick_y ;
        // Motor2Power = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        dc_Motor1.setPower(Motor1Power);
        dc_Motor2.setPower(Motor2Power);
        servo1.setPosition(Posi);
        servo2.setPosition(-Posi);
        double Servo1 = servo1.getPosition();
        double Servo2 = servo2.getPosition();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Motor1 (%.2f), Motor2 (%.2f)", Motor1Power, Motor2Power);
        // Will HardwareMap.get( class, device) send an ID Number???
        telemetry.addData("Motor ID", "Motor1:(%.2f), Motor2:(%.2f)", dc_Motor1, dc_Motor2);

        telemetry.addData("Servos", "servo1 ($.2f), servo2 (%.2f)", Servo1 , Servo2);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
