/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.ftc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import gay.zharel.hermes.actions.Action;
import gay.zharel.hermes.actions.SequentialAction;
import gay.zharel.hermes.actions.SleepAction;
import gay.zharel.hermes.actions.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

public class AsyncSamples {
    // sample: actionsRunBlocking
    public static void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            running = action.run(packet);

            dash.sendTelemetryPacket(packet);
        }
    }
    // end sample

    // sample: actionsTeleop
    public class TeleopWithActions extends OpMode {
        private FtcDashboard dash = FtcDashboard.getInstance();
        private List<Action> runningActions = new ArrayList<>();

        @Override
        public void init() {
        }

        @Override
        public void loop() {
            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }
    }
    // end sample

    public class GamepadTrigger extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            List<Action> runningActions = new ArrayList<>();

            Servo servo = hardwareMap.servo.get("servo");

            // sample: actionsGamepadTrigger
            if (gamepad1.a) {
                runningActions.add(new SequentialAction(
                        new SleepAction(Duration.ofSeconds(500)),
                        new InstantAction(() -> servo.setPosition(0.5))
                ));
            }
            // end sample
        }
    }
}
