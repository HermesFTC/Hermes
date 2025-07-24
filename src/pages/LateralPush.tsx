import SingleConfig from '@/components/AssignableSingleConfig';
import { GenericButton } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import { useConfigVariable, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { useOpMode } from '@/hooks/useOpMode';
import { initOpMode, startOpMode } from '@/store/actions/opmode';
import { ChangeEvent, ChangeEventHandler, useState } from 'react';

export default function LateralPush() {
    // config and opmode management
    const setConfig = useSetConfigVariable();
    const opMode = useOpMode();

    // track ui state
    const [started, setStarted] = useState(false);
    const [hasRun, setHasRun] = useState("no"); // "no" | "error" | "yes"

    // use to modify inches travelled (TODO)
    const [inchesTraveled, setInchesTraveled] = useState(24.0);
    const [advancedUser, setAdvancedUser] = useState(false);

    const handleInchesTravelledUpdate = (e: { target: { value: string | null; }; }) => {
        const value = e.target.value as number | null;
        if (value != null) {
            setInchesTraveled(value);
        }
    };

    // get necessary config vars
    const measuredTicks = useConfigVariable("LateralPush/MeasuredTicks");

    // names of motors: string
    const perpPod = useConfigVariable("LateralPush/YMotorName"); // 2 wheel & 3 wheel

    const drivea1 = useConfigVariable("LateralPush/DriveA1MotorName"); // motor encoder only: front right / back left (positive ticks)
    const drivea2 = useConfigVariable("LateralPush/DriveA2MotorName"); // motor encoder only: front right / back left (positive ticks)
    const driveb1 = useConfigVariable("LateralPush/DriveB1MotorName"); // motor encoder only: front left / back right (negative ticks)
    const driveb2 = useConfigVariable("LateralPush/DriveB2MotorName"); // motor encoder only: front left / back right (negative ticks)

    // directions of motors
    const perpPodDir = useConfigVariable("LateralPush/YMotorDirection"); // 2 wheel & 3 wheel

    // pinpoint: is everything plugged in right? (FORWARD / REVERSED / DUAL / NONE)
    const pinpointStatus = useConfigVariable("LateralPush/PinpointStatus");

    // current localizer choice (should not update during opmode)
    const localizer = useConfigVariable("HermesConfig/Localizer");
    const usingKnownOdoPods = useConfigVariable("HermesConfig/IsKnownOdoPod");


    const startLateralPush = () => {
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
    
            // compute and write ticks per inch, mec motor encoders only
            let ticksPerInch = (measuredTicks as number) / inchesTraveled;
            if (localizer == "MOTOR_ENCODERS" && !usingKnownOdoPods) {
                setConfig("HermesConfig/LateralTicksPerInch", ticksPerInch);
            }

            // write motor directions
            switch (localizer) {
                case "GOBILDA_PINPOINT":
                    if (pinpointStatus == "REVERSED" ) { // ensure this accounts for a flipped state by rotating the expected outputs in code by offsets
                        setConfig("HermesConfig/PinpointYDirection", "REVERSE");
                    } else if (pinpointStatus != "FORWARD" ) {
                        alert("Your pinpoint may have wiring issues. Please double check that the pods are plugged in correctly and tracking before continuing.");
                    }
                    break;

                case "TWO_WHEEL":
                case "THREE_WHEEL":
                    setConfig("HermesConfig/YMotorName", perpPod);
                    setConfig("HermesConfig/YMotorDirection", perpPodDir);
                    break;

                case "MOTOR_ENCODERS": // continue narrowing down the possibility space of the motors. this will all be resolved by angular tuner!
                    setConfig("HermesConfig/DriveA1MotorName", drivea1);
                    setConfig("HermesConfig/DriveA2MotorName", drivea2);
                    setConfig("HermesConfig/DriveB1MotorName", driveb1);                  
                    setConfig("HermesConfig/DriveB2MotorName", driveb2);
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
        <h1 className="text-3xl">Lateral Push Tuner</h1>

        <div className="text-left">
            <span>
            <p>
                Place the robot such that the corner of the robot lines up with the corner of one of your tiles. Press the button
                below and then slowly{' '}
                <ImportantInstruction>
                push the robot perfectly straight in the direction you wish to
                consider left
                </ImportantInstruction>{' '}
                until you {inchesTraveled == 24 ? "get to the edge of the next tile (24 inches)" : "have moved forward " + inchesTraveled + " inches"}, 
                <ImportantInstruction>{' '}
                    moving everything else on the robot as little as possible.
                </ImportantInstruction> When you are
                done,{' '}
                <ImportantInstruction>
                do not move the robot and press the button below to end.
                </ImportantInstruction>
            </p>
            </span>

            <br/>
            
            <div className="flex flex-col text-center items-center">
                <AdvancedInstruction>
                    Advanced users only: you may modify the number of inches you are moving here. It will save automatically and instantly.
                    Do not modify if you do not know what this does. The default value is 24.
                </AdvancedInstruction>
                <div className="block">
                    <ImportantInstruction>
                        <label htmlFor="advancedUser">I am an advanced user that understands what this parameter does and would like to modify it </label>
                    </ImportantInstruction>
                    <input type="checkbox" id="advancedUser" checked={advancedUser} onChange={(e) => setAdvancedUser(e.target.checked)}></input>
                </div>
                <input type="number" id="inchesTraveled" disabled={!advancedUser} onChange={handleInchesTravelledUpdate} value={inchesTraveled} className={"mx-auto text-center mt-2 rounded-xl " + (advancedUser ? "" : "bg-gray-400/50 text-hermes-cyan-dark/50")}></input>
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
                onClick={startLateralPush}
                >
                Not satisfied? Press this button to try again!
                </GenericButton>
            )
            :
            (
            <GenericButton
                className="mt-10 rounded-xl p-4"
                onClick={startLateralPush}
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
