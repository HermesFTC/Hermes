import SlidingButton from "@/components/SlidingButton"
import { BASE_HERMES_URL } from "@/constants"

export default function Home() {

    const saved = localStorage.getItem("TuningPage")

    return (
        <div className="text-center h-[100vh] text-hermes-cyan-dark">
            <div className="flex flex-row m-auto items-center justify-center">
                <img src="src/assets/icons/hermes_logo.svg"/>
                <h1 className="text-9xl">HermesFTC</h1>
            </div>
            <div className="text-center w-full">
                <SlidingButton href={BASE_HERMES_URL + "/getting-started"}>Get Started</SlidingButton>
                {saved === null ? <></> : <SlidingButton className={"mt-4"} delay={300} href={BASE_HERMES_URL + "/" + saved}>Continue From Save</SlidingButton>}
            </div>
        </div>
    )
}