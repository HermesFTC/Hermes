import React, { useState } from 'react';
import LinearRegressionChart from './graph';
import FileLoader from './FileLoader';
import {
  QuasistaticParameters,
  DynamicParameters
} from './utils';
import { useSetConfigVariable } from '@/hooks/useConfigVariables';
import { BASE_ROBOT_CONFIG_PATH } from '@/constants';

interface ForwardRampRegressionProps {
  className?: string;
}

export const ForwardRampRegression: React.FC<ForwardRampRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<QuasistaticParameters | null>(null);

  const setConfig = useSetConfigVariable()
  const CONFIG_PREFIX = BASE_ROBOT_CONFIG_PATH + "feedforward/translational/";

  const setKV = (kV: number) => setConfig(CONFIG_PREFIX + "kV", kV);
  const setKS = (kS: number) => setConfig(CONFIG_PREFIX + "kStatic", kS);

  const chartCallback = (slope: number, intercept: number) => { setKV(slope); setKS(intercept); }

  const handleDataLoaded = async (inputData: QuasistaticParameters): Promise<string[]> => {
    setData(inputData);
    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Forward Ramp Regression</h2>
      
      <FileLoader
        name="forwardRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {data && (
        <LinearRegressionChart
          xs={data.velocities.values}
          ys={data.voltages.values}
          options={{
            title: 'Forward Ramp Regression (kV, kS)',
            slope: 'kV',
            intercept: 'kS',
            xLabel: 'forward velocity [inches/s]',
            yLabel: 'applied voltage [V]'
          }}
          onChange={chartCallback}
        />
      )}
    </div>
  );
};

interface ForwardStepRegressionProps {
  className?: string;
}

export const ForwardStepRegression: React.FC<ForwardStepRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<DynamicParameters | null>(null);

  const setConfig = useSetConfigVariable()
  const CONFIG_PREFIX = BASE_ROBOT_CONFIG_PATH + "feedforward/translational/";

  const setKA = (kA: number) => setConfig(CONFIG_PREFIX + "kA", kA);

  const chartCallback = (slope: number, intercept: number) => { setKA(slope); }

  const handleDataLoaded = async (inputData: DynamicParameters): Promise<string[]> => {
    setData(inputData);
    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Forward Step Regression</h2>
      
      <FileLoader
        name="forwardStep"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {data && (
        <LinearRegressionChart
          xs={data.accelerations.values}
          ys={data.deltaVoltages.values}
          options={{
            title: 'Forward Step Regression (kA)',
            slope: 'kA',
            xLabel: 'forward acceleration [inches/s²]',
            yLabel: 'delta voltage [V]'
          }}
          onChange={chartCallback}
        />
      )}
    </div>
  );
};

interface LateralRampRegressionProps {
  className?: string;
}

export const LateralRampRegression: React.FC<LateralRampRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<QuasistaticParameters | null>(null);

  const setConfig = useSetConfigVariable()
  const CONFIG_PREFIX = BASE_ROBOT_CONFIG_PATH + "feedforward/lateral/";

  const setKV = (kV: number) => setConfig(CONFIG_PREFIX + "kV", kV);
  const setKS = (kS: number) => setConfig(CONFIG_PREFIX + "kStatic", kS);

  const chartCallback = (slope: number, intercept: number) => { setKV(slope); setKS(intercept); }

  const handleDataLoaded = async (inputData: QuasistaticParameters): Promise<string[]> => {
    setData(inputData);
    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Lateral Ramp Regression</h2>
      
      <FileLoader
        name="lateralRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {data && (
        <LinearRegressionChart
          xs={data.velocities.values}
          ys={data.voltages.values}
          options={{
            title: 'Lateral Ramp Regression (kV, kS)',
            slope: 'kV',
            intercept: 'kS',
            xLabel: 'lateral velocity [inches/s]',
            yLabel: 'applied voltage [V]'
          }}
          onChange={chartCallback}
        />
      )}
    </div>
  );
};

interface AngularRampRegressionProps {
  className?: string;
}

export const AngularRampRegression: React.FC<AngularRampRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<QuasistaticParameters | null>(null);

  const setConfig = useSetConfigVariable()
  const CONFIG_PREFIX = BASE_ROBOT_CONFIG_PATH + "feedforward/rotational/";

  const setKV = (kV: number) => setConfig(CONFIG_PREFIX + "kV", kV);
  const setKS = (kS: number) => setConfig(CONFIG_PREFIX + "kStatic", kS);

  const chartCallback = (slope: number, intercept: number) => { setKV(slope); setKS(intercept); }

  const handleDataLoaded = async (inputData: QuasistaticParameters): Promise<string[]> => {
    setData(inputData);
    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Angular Ramp Regression</h2>
      
      <FileLoader
        name="angularRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {data && (
        <LinearRegressionChart
          xs={data.velocities.values}
          ys={data.voltages.values}
          options={{
            title: 'Angular Ramp Regression (kV, kS)',
            slope: 'kV',
            intercept: 'kS',
            xLabel: 'angular velocity [rad/s]',
            yLabel: 'applied voltage [V]'
          }}
          onChange={chartCallback}
        />
      )}
    </div>
  );
};

interface AngularStepRegressionProps {
  className?: string;
}

export const AngularStepRegression: React.FC<AngularStepRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<DynamicParameters | null>(null);

  const setConfig = useSetConfigVariable()
  const CONFIG_PREFIX = BASE_ROBOT_CONFIG_PATH + "feedforward/rotational/";

  const setKA = (kA: number) => setConfig(CONFIG_PREFIX + "kA", kA);

  const chartCallback = (slope: number, intercept: number) => { setKA(slope); }

  const handleDataLoaded = async (inputData: DynamicParameters): Promise<string[]> => {
    setData(inputData);
    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Angular Step Regression</h2>
      
      <FileLoader
        name="angularStep"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {data && (
        <LinearRegressionChart
          xs={data.accelerations.values}
          ys={data.deltaVoltages.values}
          options={{
            title: 'Angular Step Regression (kA)',
            slope: 'kA',
            xLabel: 'angular acceleration [rad/s²]',
            yLabel: 'delta voltage [V]'
          }}
          onChange={chartCallback}
        />
      )}
    </div>
  );
}; 