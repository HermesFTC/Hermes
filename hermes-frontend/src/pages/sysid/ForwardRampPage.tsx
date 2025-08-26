import { GenericButton } from '@/components/GenericButton';
import { ImportantInstruction } from '@/components/TextModifications';
import { ForwardRampRegression } from '@/components/graph';
import TuningOpModeButton from '@/components/tuning/TuningOpModeButton';
import { BASE_HERMES_URL, BASE_TUNING_CONFIG_PATH, TUNING_OPMODE_PATHS } from '@/constants';
import { useSetConfigVariable } from '@/hooks/useConfigVariables';
import { RunState } from '@/store/types/opmode';
import React, { useEffect, useState } from 'react';

const CONFIG_PREFIX = BASE_TUNING_CONFIG_PATH + TUNING_OPMODE_PATHS.FORWARD_RAMP;

const ForwardRampPage: React.FC = () => {

  const setConfig = useSetConfigVariable();

  const [runState, setRunState] = useState(RunState.IDLE);

  const updateRunState = (runState: RunState) => {
    setRunState(runState)
    switch (runState) {
      // once the test is done once, set the direction to reverse
      case RunState.STOPPED:
        setConfig(CONFIG_PREFIX + "direction", -1.0);
        break;
    }
  }

  return (
    <div className="content p-6 w-full max-w-7xl mx-auto">
      <header className="mb-8">
        <h1 className="text-3xl font-bold mb-2">Forward Ramp Test</h1>
        <p className="text-gray-600 mb-4">Version: {import.meta.env.VITE_APP_VERSION || 'dev'}</p>
      </header>

      <div className="lg:w-2/3">
        <p>
          This is the start of Drivetrain Characterization. This is where the parameters for your drivetrain will be tuned.
          <br></br><br></br>
          This is the Forward Ramp Test. The robot will start to move forward slowly, gradually increasing in speed. The robot will not stop until you stop it. 
          <ImportantInstruction> Stop the OpMode before the robot collides with any obstacles. Then, run the OpMode again. </ImportantInstruction> 
          This time, the robot will go backwards. Again, stop the OpMode before the robot collides with any obstacles.
          Finally, press the "Load Latest" button.
          <br></br><br></br>
          It is recommended to have a full 6 tiles (the length of the field) cleared out of obstacles for the robot to drive across. The longer the robot can drive,
          the more accurate the data will be.
        </p>

      </div>

      <div className="lg:mr-[30%]">
        <TuningOpModeButton runState={runState} setRunState={updateRunState} opModeName="ForwardRampTest"></TuningOpModeButton>
      </div>
      
      <div className="w-full mt-4">
        <ForwardRampRegression />
      </div>

      <GenericButton href={BASE_HERMES_URL + "/forward-step"} className={"p-4 rounded-xl mt-10 transition duration-500 " + (runState === RunState.STOPPED ? "opacity-100" : "none opacity-0") }>I'm ready to move on!</GenericButton>
    </div>
  );
};

export default ForwardRampPage; 