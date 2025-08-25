import SingleConfig from '@/components/AssignableSingleConfig';
import { GenericButton } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import { useConfigVariable, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { useOpMode } from '@/hooks/useOpMode';
import { initOpMode, startOpMode } from '@/store/actions/opmode';
import { ChangeEvent, ChangeEventHandler, useState } from 'react';

export default function AngularPush() {
    // config and opmode management
    const setConfig = useSetConfigVariable();
    const opMode = useOpMode();

    // track ui state
    const [started, setStarted] = useState(false);
    const [hasRun, setHasRun] = useState("no"); // "no" | "error" | "yes"

    // use to modify inches travelled (TODO)
    const [totalRotations, setTotalRotations] = useState(1.0);
    const [advancedUser, setAdvancedUser] = useState(false);

    const handleRotationsUpdate = (e: { target: { value: string | null; }; }) => {
        const value = e.target.value as number | null;
        if (value != null) {
            setTotalRotations(value);
        }
    };

    // get necessary config vars

    // names of motors: string

    // FINALLY, we have sufficient information to deduce which motor is where :)
    const drivefr = useConfigVariable("AngularPush/DriveFRMotorName"); // motor encoder only
    const drivefl = useConfigVariable("AngularPush/DriveFLMotorName"); // motor encoder only
    const drivebr = useConfigVariable("AngularPush/DriveBRMotorName"); // motor encoder only
    const drivebl = useConfigVariable("AngularPush/DriveBLMotorName"); // motor encoder only

    // movements of motors
    const parPodInches = useConfigVariable("AngularPush/XMotorInches"); // 2 wheel only
    const perpPodInches = useConfigVariable("AngularPush/YMotorInches"); // 2 wheel & 3 wheel

    const drivetrainInches = useConfigVariable("AngularPush/MotorEncoderAverageInches") // motor encoder only (for trackwidth)

    const imuOrientation = useConfigVariable("AngularPush/IMUOrientation");

    const measuredRotations = useConfigVariable("AngularPush/MeasuredRotations");

    // pinpoint: is everything plugged in right? (FORWARD / REVERSED / DUAL / NONE)
    const pinpointStatus = useConfigVariable("AngularPush/PinpointStatus");

    // current localizer choice (should not update during opmode)
    const localizer = useConfigVariable("HermesConfig/Localizer");

    if (localizer == "THREE_WHEEL" || localizer == "SPARKFUN_OTOS") {
        setTotalRotations(5.0);
    }


    const startAngularPush = () => {
        if (opMode.canInitOpMode) {
            opMode.initOpMode('TestSineWaveOpMode');
            opMode.startOpMode();
        }
        setStarted(true);
    };

    const emstop = () => {
        opMode.stopOpMode();
        alert('Opmode Emergency Stopped.');
    };

    const stop = () => {
        // stop the opmode
        setHasRun("error");
        if (opMode.canStopOpMode) {
            setHasRun("yes");
            opMode.stopOpMode();

            // write constants
            switch (localizer) {
                // pod offsets
                case "GOBILDA_PINPOINT":
                case "TWO_WHEEL":
                    setConfig("HermesConfig/XOffsetInches", (parPodInches as number) / (totalRotations * 2.0 * Math.PI));
                    setConfig("HermesConfig/YOffsetInches", (perpPodInches as number) / (totalRotations * 2.0 * Math.PI));
                    break;
                case "THREE_WHEEL":
                    setConfig("HermesConfig/YOffsetInches", (perpPodInches as number) / (totalRotations * 2.0 * Math.PI));
                    setConfig("HermesConfig/AngularScalar", (totalRotations) / (measuredRotations as number));
                    break;

                case "MOTOR_ENCODERS": // YEAHHHH
                    setConfig("HermesConfig/FrontRightMotorName", drivefr);
                    setConfig("HermesConfig/FrontLeftMotorName", drivefl);
                    setConfig("HermesConfig/BackRightMotorName", drivebr);              
                    setConfig("HermesConfig/BackLeftMotorName", drivebl);
                    setConfig("HermesConfig/Trackwidth", (drivetrainInches as number) / (totalRotations * Math.PI)) // multiply by 2 to get full trackwidth, cancelling the 2 in conversion to radians
                    break;
                
                case "SPARKFUN_OTOS":
                    setConfig("HermesConfig/AngularScalar", (totalRotations) / (measuredRotations as number));
                    // TODO
                    break;
            }
        }
        setStarted(false);
    };

    const getOpModeStatus = () => {
        if (opMode.isOpModeRunning) {
            return "Tuning running!";
        } else if (opMode.isOpModeStopped || hasRun == "yes") {
            return <p className="text-green-500">Tuning stopped.</p>;
        } else if (opMode.isOpModeInitialized) {
            return "Tuning intialized!";
        } else if (started) {
            return "Initializing tuning...";
        } else if (hasRun == "error") {
            return <p className="text-red-600">Error running tuning OpMode. Please wait for the tuning to start running before stopping it!</p>;
        } else {
            return <></>;
        }
    }

    return (
        <div>
        <h1 className="text-3xl">Angular Push Tuner</h1>

        <div className="text-left">
            <span>
            <p>
                Place the robot such that the corner of the robot lines up with the corner of one of your tiles. Press the button
                below and then push the robot to roughly the middle of the tile 
                <ImportantInstruction>
                    without rotating it at all.
                </ImportantInstruction>{' '}Then, slowly{' '}
                <ImportantInstruction>
                rotate the robot counterclockwise
                </ImportantInstruction>{' '}
                until you {totalRotations == 1.0 ? "have spun it around exactly once" : "have rotated it " + totalRotations + " times"}, 
                <ImportantInstruction>{' '}
                    moving everything else on the robot as little as possible.
                </ImportantInstruction> You must keep the {localizer == "MOTOR_ENCODERS" ? 'wheels in contact with' : localizer == "SPARKFUN_OTOS" ? 'sensor on top of' : "odometry pods in contact with "} the tile at all times. When you are done, 
                <ImportantInstruction>{' '}
                push the robot back to the corner without rotating it and press the button below to end.
                </ImportantInstruction>
            </p>
            </span>

            <br/>
            
            <div className="flex flex-col text-center items-center">
                <AdvancedInstruction>
                    Advanced users only: you may modify the number of rotations you are performing here. It will save automatically and instantly.
                    Do not modify if you do not know what this does. The default value is {localizer == "THREE_WHEEL" || localizer == "SPARKFUN_OTOS" ? '10' : '1'}.
                </AdvancedInstruction>
                <div className="block">
                    <ImportantInstruction>
                        <label htmlFor="advancedUser">I am an advanced user that understands what this parameter does and would like to modify it </label>
                    </ImportantInstruction>
                    <input type="checkbox" id="advancedUser" checked={advancedUser} onChange={(e) => setAdvancedUser(e.target.checked)}></input>
                </div>
                <input type="number" id="totalRotations" disabled={!advancedUser} onChange={handleRotationsUpdate} value={totalRotations} className={"mx-auto text-center mt-2 rounded-xl " + (advancedUser ? "" : "bg-gray-400/50 text-hermes-cyan-dark/50")}></input>
            </div>
        </div>

        <div className="flex flex-col">
            {started ? (
            <GenericButton className="mt-10 rounded-xl p-4" onClick={stop}>
                Click me when you're done!
            </GenericButton>
            ) : hasRun == "yes" ? 
            (
                <GenericButton
                className="mt-10 rounded-xl p-4"
                onClick={startAngularPush}
                >
                Not satisfied? Press this button to try again!
                </GenericButton>
            )
            :
            (
            <GenericButton
                className="mt-10 rounded-xl p-4"
                onClick={startAngularPush}
            >
                Click me when you're ready to start!
            </GenericButton>
            )}

            <p>
            {
                getOpModeStatus()
            }
            </p>

            <GenericButton
            className="mt-10 rounded-xl bg-red-600 p-4 text-white"
            onClick={emstop}
            >
            Emergency Stop!
            </GenericButton>
        </div>

        
        <GenericButton href="/hermes/angular-push" className={"p-4 rounded-xl mt-10 transition duration-500 " + (hasRun == "yes" ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>

            
        </div>
    );
}
