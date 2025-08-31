import { AdvancedUserConfirmationModal } from '@/components/tuning/AdvancedUserConfirmation';
import { GenericButton, GenericButtonProps } from '@/components/GenericButton';
import { AdvancedInstruction, ImportantInstruction } from '@/components/TextModifications';
import TuningOpModeButton from '@/components/tuning/TuningOpModeButton';
import { useAppendConfigArray, useConfigVariable, useConfigVariableState, useSetConfigVariable } from '@/hooks/useConfigVariables';
import { RunState } from '@/store/types/opmode';
import { useState } from 'react';
import { BASE_HERMES_URL, BASE_TUNING_CONFIG_PATH, TUNING_OPMODE_PATHS } from '@/constants';
import { ConfigVarState, CustomVarState } from '@/store/types/config';
import { map } from 'lodash';

export default function DriveTuner() {
    // config and opmode management
    const setConfig = useSetConfigVariable();
    const CONFIG_PREFIX = BASE_TUNING_CONFIG_PATH + TUNING_OPMODE_PATHS.DRIVETRAIN_CONFIG;

    const [runState, setRunState] = useState(RunState.IDLE);

    // extract all motor names from array
    const motors = ["motor1","motor2","motor3","motor4","motor5","motor6","motor7","motor8",];

    /*const motorNames = Object.values((useConfigVariableState(CONFIG_PREFIX + "motorNames") as CustomVarState).__value as Record<string, ConfigVarState>)

    for (let i = 0; i < motorNames.length; i++) {
        motors.push(motorNames[i].__value as string);
    }*/

    let idx = 0;

    const chosenMotorsMap: Map<string, boolean> = new Map<string, boolean>();

    const chosenMotors = () => {
        let motors = [];
        for (let motorName in chosenMotorsMap) {
            if (chosenMotorsMap.get(motorName)) {
                motors.push(motorName)
            }
        }
        return motors
    }

    const updateConfig = () => {
        const chosenMotorsList = chosenMotors()
        for (let i = 0; i < chosenMotorsList.length; i++) {
            setConfig(CONFIG_PREFIX + "chosenMotors/" + i, chosenMotorsList[i])
        }
        console.log(chosenMotorsMap)
    }

    return (
        <div>
        <h1 className="text-3xl">Drivetrain Configuration Tuner</h1>

        <div className="text-left">
            <span>
            <p>
                This is the Drivetrain Configuration Tuner. It will automatically configure the drivetrain parameters, like motor direction for Mecanum and Tank.
                Before the test begins, select the motors that make up your drivetrain.
            </p>
            </span>
            <div className="mt-4">
                <h1 className="text-xl text-center mb-2">Motor Selection</h1>
                {motors.map((motorName) => 
                    (<SelectionButton onClick={
                        () => {chosenMotorsMap.set(motorName, !chosenMotorsMap.get(motorName)); updateConfig();}
                    }
                    className={
                        "p-2 mr-2 mb-2 rounded-md"
                    }
                    >
                        {motorName}
                    </SelectionButton>)
                )}
            </div>

            <br/>
        </div>

        <TuningOpModeButton runState={runState} setRunState={setRunState} opModeName="DrivetrainConfigTest"/>
        
        <GenericButton href={BASE_HERMES_URL + "forward-ramp"} className={"p-4 rounded-xl mt-10 transition duration-500 " + (runState === RunState.STOPPED ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>
            
        </div>
    );
}

export function SelectionButton(props: GenericButtonProps) {
    const [active, setActive] = useState(false);

    const toggle = () => { props.onClick(); setActive(!active); }

    const classNames = props.className + " " + (active ? "bg-white" : "bg-transparent")
    console.log(classNames)

    return GenericButton({
        ...props,
        className: classNames,
        onClick: toggle,
        
    })
}