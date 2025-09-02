import { AdvancedUserConfirmationModal } from '@/components/tuning/AdvancedUserConfirmation';
import { GenericButton } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import TuningOpModeButton from '@/components/tuning/TuningOpModeButton';
import { useConfigVariable, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { RunState } from '@/store/types/opmode';
import { useState } from 'react';
import { BASE_HERMES_URL, BASE_TUNING_CONFIG_PATH, TUNING_OPMODE_PATHS } from '@/constants';

export default function ForwardPush() {
    localStorage.setItem("TuningStage", "Localizer");
    localStorage.setItem("TuningPage", "forward-push");
    localStorage.setItem("TuningProgress", "1");

    // config and opmode management
    const setConfig = useSetConfigVariable();
    const CONFIG_PREFIX = BASE_TUNING_CONFIG_PATH + TUNING_OPMODE_PATHS.FORWARD_PUSH;

    const [runState, setRunState] = useState(RunState.IDLE)
    
    // use to modify inches travelled
    const [inchesTraveled, setInchesTraveled] = useState(24.0);
    const [advancedUser, setAdvancedUser] = useState(false);

    const handleInchesTravelledUpdate = (e: { target: { value: string | null; }; }) => {
        const value = e.target.value as number | null;
        if (value != null) {
            setInchesTraveled(value);
            setConfig(CONFIG_PREFIX + "actualInchesTravelled", value)
        }
    };

    // current localizer choice (should not update during opmode)
    const localizer = useConfigVariable(BASE_TUNING_CONFIG_PATH + "localizer");

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
                <AdvancedUserConfirmationModal advancedUser={advancedUser} setAdvancedUser={setAdvancedUser}/>
                <input type="number" id="inchesTraveled" disabled={!advancedUser} onChange={handleInchesTravelledUpdate} value={inchesTraveled} className={"mx-auto text-center mt-2 rounded-xl " + (advancedUser ? "" : "bg-gray-400/50 text-hermes-cyan-dark/50")}></input>
            </div>
        </div>

        <TuningOpModeButton runState={runState} setRunState={setRunState} opModeName="ForwardPushTest"/>
        
        <GenericButton href={localizer != "SPARKFUN_OTOS" ? BASE_HERMES_URL + "/lateral-push" : BASE_HERMES_URL + "/angular-push"} className={"p-4 rounded-xl mt-10 transition duration-500 " + (runState === RunState.STOPPED ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>
            
        </div>
    );
}
