import SingleConfig from "@/components/AssignableSingleConfig"
import { ImportantInstruction } from "@/components/TextModifications"
import BasicVariable from "@/components/views/ConfigView/BasicVariable"
import CustomVariable from "@/components/views/ConfigView/CustomVariable"
import EnumInput from "@/components/views/ConfigView/inputs/EnumInput"
import { useConfigVariable, useSetConfigVariable } from "@/hooks/useConfigVariables"

export default function GettingStarted() {

    const setConfig = useSetConfigVariable()

    const currentLocalizer = useConfigVariable("Hermes/Localizer")



    return (

        <div>

            <h1 className="text-3xl">Welcome to Hermes!</h1>

            <div className="text-left">
                <p>This is the start of the Hermes tuning process! <ImportantInstruction>Read every instruction carefully. Extra important instructions will be bolded in this color.</ImportantInstruction> Let's get started!</p>
                <br/>
                <p>For Hermes to work best, it needs to be tuned to your robot. You will need access to the robot and 2 field tiles. You can see your progress in the progress bar at the bottom!</p>
                <br/>
                <p>Any time your make a major change to the robot's weight or mechanism, consider redoing this process to get better results for the new robot.</p>
                <br/>
                <p>Before we get started, <ImportantInstruction>choose your localization (odometry) option.</ImportantInstruction></p>
            </div>

            <div className="mt-4 flex justify-center items-center text-center flex flex-col">
                {currentLocalizer}
                <button className="border-solid border-hermes-cyan-dark border-4 rounded-xl py-4 w-1/2 lg:w-full" onClick={() => {setConfig("Hermes/Localizer", "TWO_WHEEL");console.log("HBDSUAHD");}}>test</button>
            </div>  

            

        </div>

    )
}