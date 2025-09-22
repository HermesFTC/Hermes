import { GenericButton } from "@/components/GenericButton"
import { AdvancedInstruction, ImportantInstruction } from "@/components/TextModifications"
import { BASE_HERMES_URL, BASE_TUNING_CONFIG_PATH } from "@/constants"
import { useConfigVariable, useSetConfigVariable } from "@/hooks/useConfigVariables"
import {  ReactNode } from "react"
import { DashButton, PathedDashButtonProps } from "./GettingStarted"
import { useOpMode } from "@/hooks/useOpMode"

const DRIVE_PATH = BASE_TUNING_CONFIG_PATH + "drive"

export default function DriveSelection() {
    localStorage.setItem("TuningStage", "Drive");
    localStorage.setItem("TuningPage", "drive-selection");
    localStorage.setItem("TuningProgress", "0");

    const setConfig = useSetConfigVariable();
    const opMode = useOpMode();

    const currentDrive = useConfigVariable(DRIVE_PATH)

    const drives = [
        {
            name: "MECANUM",
            label: "Mecanum Drive"
        },
        {
            name: "TANK",
            label: "Tank Drive"
        },
    ]

    const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));

    // run the motor name opmode on drive selection
    const getMotorNames = () => {
        opMode.initOpMode("MotorFetchOpMode")
        opMode.startOpMode()
        sleep(100)
        opMode.stopOpMode()
    }

    return (

        <div>
            <h1 className="text-3xl">Drivetrain Selection</h1>

            <div className="text-center">
                <p>Localizer tuning is now complete!</p><br></br>
                <span><p><ImportantInstruction>Choose your drivetrain option.</ImportantInstruction></p></span>
            </div>

            <div className="mt-4 flex justify-center items-center text-center flex flex-col">
                {drives.map((drive) =>

                    <DriveSelectionButton onClick={getMotorNames} value={drive.name}>{drive.label}</DriveSelectionButton>
                )
                }
            </div>

            <GenericButton href={BASE_HERMES_URL + "/drive-config"} className={"p-4 rounded-xl mt-10 duration-500 " + (currentDrive == "CUSTOM" ? "none opacity-0" : "opacity-100")}>I'm ready to move on!</GenericButton>

        </div>

    )
}

const DriveSelectionButton = (props: PathedDashButtonProps) => DashButton({path: DRIVE_PATH, ...props});