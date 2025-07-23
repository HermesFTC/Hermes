import SingleConfig from '@/components/AssignableSingleConfig';
import { GenericButton } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import { useConfigVariable, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { useOpMode } from '@/hooks/useOpMode';
import { initOpMode, startOpMode } from '@/store/actions/opmode';
import { ChangeEvent, ChangeEventHandler, useState } from 'react';

export default function ForwardPush() {
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
    const measuredTicks = useConfigVariable("ForwardPush/MeasuredTicks");

    // names of motors: string
    const parPod1 = useConfigVariable("ForwardPush/X1MotorName"); // 2 wheel & 3 wheel
    const parPod2 = useConfigVariable("ForwardPush/X2MotorName"); // 3 wheel only

    const drive1 = useConfigVariable("ForwardPush/Drive1MotorName"); // motor encoder only
    const drive2 = useConfigVariable("ForwardPush/Drive2MotorName"); // motor encoder only
    const drive3 = useConfigVariable("ForwardPush/Drive3MotorName"); // motor encoder only
    const drive4 = useConfigVariable("ForwardPush/Drive4MotorName"); // motor encoder only

    // directions of motors
    const parPod1Dir = useConfigVariable("ForwardPush/X1MotorDirection"); // 2 wheel & 3 wheel
    const parPod2Dir = useConfigVariable("ForwardPush/X2MotorDirection"); // 3 wheel only

    const drive1Dir = useConfigVariable("ForwardPush/Drive1MotorDirection"); // motor encoder only
    const drive2Dir = useConfigVariable("ForwardPush/Drive2MotorDirection"); // motor encoder only
    const drive3Dir = useConfigVariable("ForwardPush/Drive3MotorDirection"); // motor encoder only
    const drive4Dir = useConfigVariable("ForwardPush/Drive4MotorDirection"); // motor encoder only

    // pinpoint: is everything plugged in right? (FORWARD / REVERSED / FLIPPED / FLIPPED_REVERSE / DUAL / NONE) -> dual = both x and y increase, none = both x and y do nothing
    const pinpointStatus = useConfigVariable("ForwardPush/PinpointStatus");

    // otos: is everything working properly? (FORWARD / REVERSED / OFFSET / OFFSET_REVERSED / DUAL / NONE)
    const otosStatus = useConfigVariable("ForwardPush/OTOSStatus");

    // current localizer choice (should not update during opmode)
    const localizer = useConfigVariable("HermesConfig/Localizer");
    const usingKnownOdoPods = useConfigVariable("HermesConfig/IsKnownOdoPod");


    const startForwardPush = () => {
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
    
            // compute and write ticks per inch
            let ticksPerInch = (measuredTicks as number) / inchesTraveled;
            if (localizer != "SPARKFUN_OTOS" && !usingKnownOdoPods) {
                setConfig("HermesConfig/TicksPerInch", ticksPerInch);
            }

            // write motor directions
            switch (localizer) {
                case "GOBILDA_PINPOINT":
                    if (pinpointStatus == "REVERSED" ) {
                        setConfig("HermesConfig/PinpointXDirection", "REVERSE");

                    } else if (pinpointStatus == "FLIPPED") {
                        setConfig("HermesConfig/PinpointYDirection", "REVERSE"); // make Y negative so it matches up on a rotation matrix LOL
                        setConfig("HermesConfig/PinpointHeadingOffset", Math.PI / 2.0);
                        alert("Your pinpoint pods are reversed. If you would like to use pinpoint outside of Hermes, switch the wires going into the pinpoint and run this tuner again.")
                    
                    } else if (pinpointStatus == "FLIPPED_REVERSE") {
                        setConfig("HermesConfig/PinpointYDirection", "FORWARD"); // make Y negative so it matches up on a rotation matrix LOL
                        setConfig("HermesConfig/PinpointHeadingOffset", Math.PI / 2.0);
                        alert("Your pinpoint pods are reversed. If you would like to use pinpoint outside of Hermes, switch the wires going into the pinpoint and run this tuner again.")
                    
                    } else if (pinpointStatus != "FORWARD" ) {
                        alert("Your pinpoint may have wiring issues. Please double check that the pods are plugged in correctly and tracking before continuing.");
                    } else {
                        setConfig("HermesConfig/PinpointXDirection", "FORWARD");
                    }
                    break;

                case "TWO_WHEEL":
                    setConfig("HermesConfig/XMotorName", parPod1);
                    setConfig("HermesConfig/XMotorDirection", parPod1Dir);
                    break;

                case "THREE_WHEEL":
                    setConfig("HermesConfig/X1MotorName", parPod1);
                    setConfig("HermesConfig/X1MotorDirection", parPod1Dir);

                    setConfig("HermesConfig/X2MotorName", parPod2);
                    setConfig("HermesConfig/X2MotorDirection", parPod2Dir);
                    break;

                case "MOTOR_ENCODERS":
                    setConfig("HermesConfig/Drive1MotorName", drive1);
                    setConfig("HermesConfig/Drive1MotorDirection", drive1Dir);

                    setConfig("HermesConfig/Drive2MotorName", drive2);
                    setConfig("HermesConfig/Drive2MotorDirection", drive2Dir);
                    
                    setConfig("HermesConfig/Drive3MotorName", drive3);
                    setConfig("HermesConfig/Drive3MotorDirection", drive3Dir);
                    
                    setConfig("HermesConfig/Drive4MotorName", drive4);
                    setConfig("HermesConfig/Drive4MotorDirection", drive4Dir);
                    break;

                case "SPARKFUN_OTOS":
                    if (otosStatus == "REVERSED" ) {
                        setConfig("HermesConfig/OTOSHeadingOffset", Math.PI); // flip 180
                        
                    } else if (otosStatus == "OFFSET") { // +y -> correlates with a rotation of +90 deg
                        setConfig("HermesConfig/OTOSHeadingOffset", Math.PI / 2.0);

                    } else if (otosStatus == "OFFSET_REVERSED") { // -y -> correlates with a rotation of -90 deg
                        setConfig("HermesConfig/OTOSHeadingOffset", -Math.PI / 2.0);
                        
                    } else if (otosStatus != "FORWARD" ) {
                        alert("Your OTOS may have installation issues. Please double check that your OTOS is oriented properly before continuing.");
                    }

                    setConfig("HermesConfig/OTOSLinearScalar", inchesTraveled / (measuredTicks as number))
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
        <h1 className="text-3xl">Forward Push Tuner</h1>

        <div className="text-left">
            <span>
            <p>
                Place the robot such that the corner of the robot lines up with the corner of one of your tiles. Press the button
                below and then slowly{' '}
                <ImportantInstruction>
                push the robot perfectly straight in the direction you wish to
                consider forward
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
                onClick={startForwardPush}
                >
                Not satisfied? Press this button to try again!
                </GenericButton>
            )
            :
            (
            <GenericButton
                className="mt-10 rounded-xl p-4"
                onClick={startForwardPush}
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

        
        <GenericButton href={localizer != "SPARKFUN_OTOS" ? "/hermes/lateral-push" : "/hermes/angular-push"} className={"p-4 rounded-xl mt-10 transition duration-500 " + (hasRun == "yes" ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>

            
        </div>
    );
}
