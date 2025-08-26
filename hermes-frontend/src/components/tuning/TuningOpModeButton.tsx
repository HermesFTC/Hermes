import { useOpMode } from "@/hooks/useOpMode";
import { RunState } from "@/store/types/opmode";
import { PropsWithChildren, useState } from "react";
import { GenericButton } from "../GenericButton";

interface TuningOpModeButtonProps extends PropsWithChildren<any> {
    runState: RunState,
    setRunState: (v: RunState) => void,
    opModeName: string
}

export default function TuningOpModeButton({runState, setRunState, opModeName}: TuningOpModeButtonProps) {
    const opMode = useOpMode();

    const startOpMode = () => {
        if (opMode.canInitOpMode) {
            opMode.initOpMode(opModeName);
            opMode.startOpMode();
        }
        setRunState(RunState.STARTED);
    };

    const emstop = () => {
        opMode.stopOpMode();
        alert('Opmode Emergency Stopped.');
    };

    const stop = () => {
        // stop the opmode
        setRunState(RunState.ERROR);

        if (opMode.canStopOpMode) {
            setRunState(RunState.STOPPED);
            opMode.stopOpMode();
        }
    };

    const getOpModeStatus = () => {
        if (opMode.isOpModeRunning) {
            return "Tuning running!";
        } else if (opMode.isOpModeStopped) {
            return <p className="text-green-500">Tuning stopped.</p>;
        } else if (opMode.isOpModeInitialized) {
            return "Tuning intialized!";
        } else if (runState === RunState.STARTED) {
            return "Initializing tuning...";
        } else if (runState === RunState.ERROR) {
            return <p className="text-red-600">Error running tuning OpMode. Please wait for the tuning to start running before stopping it!</p>;
        } else {
            return <></>;
        }
    }

    return <div className="flex flex-col">
        {runState === RunState.STARTED ? (
        <GenericButton className="mt-10 rounded-xl p-4" onClick={stop}>
            Click me when you're done!
        </GenericButton>
        ) : runState === RunState.STOPPED ? 
        (
            <GenericButton
            className="mt-10 rounded-xl p-4"
            onClick={startOpMode}
            >
            Not satisfied? Press this button to try again!
            </GenericButton>
        )
        :
        (
        <GenericButton
            className="mt-10 rounded-xl p-4"
            onClick={startOpMode}
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
    </div>;
}