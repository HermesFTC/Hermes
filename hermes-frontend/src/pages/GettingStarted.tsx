import { GenericButton } from "@/components/GenericButton"
import { AdvancedInstruction, ImportantInstruction } from "@/components/TextModifications"
import { useConfigVariable, useSetConfigVariable } from "@/hooks/useConfigVariables"
import {  ReactNode } from "react"

export default function GettingStarted() {

    const setConfig = useSetConfigVariable();
    const setLocalizer = (value: string) => setConfig("HermesConfig/Localizer", value)

    const currentLocalizer = useConfigVariable("HermesConfig/Localizer")

    const localizers = [
        {
            name: "GOBILDA_PINPOINT",
            label: "goBILDA Pinpoint"
        },
        {
            name: "TWO_WHEEL",
            label: "Two Wheel Odometry"
        },
        {
            name: "THREE_WHEEL",
            label: "Three Wheel Odometry"
        },
        {
            name: "MOTOR_ENCODERS",
            label: "Motor Encoders (Mecanum)"
        },
        {
            name: "SPARKFUN_OTOS",
            label: "SparkFun OTOS"
        },
    ]



    return (

        <div>
            <h1 className="text-3xl">Welcome to Hermes!</h1>

            <div className="text-left">
                <span><p>This is the start of the Hermes tuning process! <ImportantInstruction>Read every instruction carefully. Extra important instructions will be bolded in this color. </ImportantInstruction><AdvancedInstruction>Sometimes advanced instructions will be included for veteran users. We recommend that you <b>do not pay attention to text in this color</b> during your first time tuning. </AdvancedInstruction> Let's get started!</p></span>
                <br/>
                <p>For Hermes to work best, it needs to be tuned to your robot. You will need access to the robot and 2 field tiles. You can see your progress in the progress bar at the bottom!</p>
                <br/>
                <p>Any time your make a major change to the robot's weight or mechanism, consider redoing this process to get better results for the new robot.</p>
                <br/>
                <span><p>Before we get started, <ImportantInstruction>choose your localization (odometry) option.</ImportantInstruction></p></span>
            </div>

            <div className="mt-4 flex justify-center items-center text-center flex flex-col">
                {localizers.map((localizer) =>
                    <LocalizerSelection value={localizer.name}>{localizer.label}</LocalizerSelection>
                )
                }
            </div>

            <GenericButton href="/hermes/forward-push" className={"p-4 rounded-xl mt-10 duration-500 " + (currentLocalizer == "NONE" ? "none opacity-0" : "opacity-100")}>I'm ready to move on!</GenericButton>

        </div>

    )
}

interface DashButtonProps {
    path: string,
    value: string | boolean | number | null,
    children: ReactNode
}

interface PathedDashButtonProps {
    value: string | boolean | number | null,
    children: ReactNode
}

function DashButton (props: DashButtonProps) {
    const setConfig = useSetConfigVariable();
    const currentValue = useConfigVariable(props.path);

    return (
    <button 
        className={"border-solid border-4 my-2 rounded-xl py-4"
        + " w-1/2 lg:w-full transition duration-500 border-hermes-cyan-dark "
         + (currentValue == props.value ? " bg-hermes-cyan-dark text-white": "")}
        onClick={() => {setConfig(props.path, props.value)}}>{props.children}
    </button>
    );
}

const LocalizerSelection = (props: PathedDashButtonProps) => DashButton({path: "HermesConfig/Localizer", ...props});