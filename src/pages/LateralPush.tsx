import { GenericButton } from '@/components/GenericButton';
import { ImportantInstruction } from '@/components/TextModifications';
import { useSetConfigVariable } from '@/hooks/useConfigVariables';
import { useOpMode } from '@/hooks/useOpMode';
import { initOpMode, startOpMode } from '@/store/actions/opmode';
import { useState } from 'react';

export default function LateralPush() {
  const setConfig = useSetConfigVariable();
  const opMode = useOpMode();

  const [started, setStarted] = useState(false);
  const [hasRun, setHasRun] = useState(false);

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
    if (opMode.canStopOpMode) {
      opMode.stopOpMode();
      setHasRun(true);
    }
    setStarted(false);
  };

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
            until you get to the edge of the next tile (24 inches). When you are
            done,{' '}
            <ImportantInstruction>
              do not move the robot and press the button below to end.
            </ImportantInstruction>
          </p>
        </span>
      </div>

      <div className="flex flex-col">
        {started ? (
          <GenericButton className="mt-10 rounded-xl p-4" onClick={stop}>
            Click me when you're done!
          </GenericButton>
        ) : hasRun ? 
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

        <GenericButton
          className="mt-10 rounded-xl bg-red-600 p-4 text-white"
          onClick={emstop}
        >
          Emergency Stop!
        </GenericButton>
      </div>

    
    <GenericButton href="/hermes/angular-push" className={"p-4 rounded-xl mt-10 transition duration-500 " + (hasRun ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>

        
    </div>
  );
}
