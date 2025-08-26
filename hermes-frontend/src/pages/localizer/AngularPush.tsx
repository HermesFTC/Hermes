import { AdvancedUserConfirmationModal } from '@/components/tuning/AdvancedUserConfirmation';
import { GenericButton } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import TuningOpModeButton from '@/components/tuning/TuningOpModeButton';
import { useConfigVariable, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { RunState } from '@/store/types/opmode';
import { useState } from 'react';
import { BASE_HERMES_URL, BASE_TUNING_CONFIG_PATH, TUNING_OPMODE_PATHS } from '@/constants';

export default function AngularPush() {
    // config management
    const setConfig = useSetConfigVariable();
    const CONFIG_PREFIX = BASE_TUNING_CONFIG_PATH + TUNING_OPMODE_PATHS.FORWARD_PUSH;

    // track ui state
    const [runState, setRunState] = useState(RunState.IDLE)

    // use to modify inches travelled (TODO)
    const [totalRotations, setTotalRotations] = useState(1.0);
    const [advancedUser, setAdvancedUser] = useState(false);

    const handleRotationsUpdate = (e: { target: { value: string | null; }; }) => {
        const value = e.target.value as number | null;
        if (value != null) {
            setTotalRotations(value);
            setConfig(CONFIG_PREFIX + "actualRevolutions", value)
        }
    };

    // current localizer choice (should not update during opmode)
    const localizer = useConfigVariable(BASE_TUNING_CONFIG_PATH + "localizer");

    if (localizer == "THREE_WHEEL" || localizer == "SPARKFUN_OTOS") {
        setTotalRotations(5.0);
        setConfig(CONFIG_PREFIX + "actualRevolutions", 5.0)
    }

    return (
        <div>
        <h1 className="text-3xl">Angular Push Tuner</h1>

        <div className="text-left">
            <span>
            <p>
                Place the robot such that the corner of the robot lines up with the corner of one of your tiles. Press the button
                below and then push the robot to roughly the middle of the tile 
                <ImportantInstruction>{' '}
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
                <AdvancedUserConfirmationModal advancedUser={advancedUser} setAdvancedUser={setAdvancedUser}/>
                <input type="number" id="totalRotations" disabled={!advancedUser} onChange={handleRotationsUpdate} value={totalRotations} className={"mx-auto text-center mt-2 rounded-xl " + (advancedUser ? "" : "bg-gray-400/50 text-hermes-cyan-dark/50")}></input>
            </div>
        </div>

        <TuningOpModeButton runState={runState} setRunState={setRunState} opModeName="AngularPushTest"/>

        
        <GenericButton href={BASE_HERMES_URL + "/forward-ramp"} className={"p-4 rounded-xl mt-10 transition duration-500 " + (runState === RunState.STOPPED ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>

            
        </div>
    );
}
