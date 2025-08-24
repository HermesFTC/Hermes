import React, { useState } from 'react';
import LinearRegressionChart from './graph';
import FileLoader from '../FileLoader';
import {
  InputAngularRampData,
  prepareAngularRampData,
  getPosZAngVelocity,
  QuasistaticParameters,
  ForwardRampData,
  InputLateralRampData,
  prepareLateralRampData
} from './utils';

interface DeadWheelAngularRampRegressionProps {
  className?: string;
}

export const DeadWheelAngularRampRegression: React.FC<DeadWheelAngularRampRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<InputAngularRampData | null>(null);
  const [processedData, setProcessedData] = useState<{
    angVels: number[];
    parEncVels: { title: string; values: number[] }[];
    perpEncVels: { title: string; values: number[] }[];
    trackWidthData: { xs: number[]; ys: number[] };
  } | null>(null);
  const [kvValue, setKvValue] = useState<number>(0.1);
  const [ksValue, setKsValue] = useState<number>(0.1);

  const handleDataLoaded = async (inputData: InputAngularRampData): Promise<string[]> => {
    setData(inputData);
    
    const preparedData = prepareAngularRampData(inputData);
    const angVels = getPosZAngVelocity(preparedData);

    const parEncVels = preparedData.parEncVels.map((vs, i) => ({
      title: `Parallel Wheel ${i} Regression`,
      values: vs.values
    }));

    const perpEncVels = preparedData.perpEncVels.map((vs, i) => ({
      title: `Perpendicular Wheel ${i} Regression`,
      values: vs.values
    }));

    // Calculate track width data
    const allPowers = [...preparedData.leftPowers, ...preparedData.rightPowers];
    const appliedVoltages = preparedData.voltages.values.map((v, i) =>
      allPowers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v);

    const trackWidthYs = appliedVoltages.map((v) =>
      (v - ksValue) / kvValue * (preparedData.type === 'mecanum' ? 2 : 1));

    setProcessedData({
      angVels,
      parEncVels,
      perpEncVels,
      trackWidthData: { xs: angVels, ys: trackWidthYs }
    });

    return [];
  };

  const updateTrackWidthData = () => {
    if (!data || !processedData) return;

    const preparedData = prepareAngularRampData(data);
    const allPowers = [...preparedData.leftPowers, ...preparedData.rightPowers];
    const appliedVoltages = preparedData.voltages.values.map((v, i) =>
      allPowers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v);

    const trackWidthYs = appliedVoltages.map((v) =>
      (v - ksValue) / kvValue * (preparedData.type === 'mecanum' ? 2 : 1));

    setProcessedData(prev => prev ? {
      ...prev,
      trackWidthData: { xs: processedData.angVels, ys: trackWidthYs }
    } : null);
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Dead Wheel Angular Ramp Regression</h2>
      
      <FileLoader
        name="deadWheelAngularRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {processedData && (
        <div className="space-y-6">
          {/* Parallel Wheel Charts */}
          <div>
            <h3 className="text-xl font-semibold mb-3">Parallel Wheels</h3>
            <div className="space-y-4">
              {processedData.parEncVels.map((wheel, i) => (
                <LinearRegressionChart
                  key={`par-${i}`}
                  xs={processedData.angVels}
                  ys={wheel.values}
                  options={{
                    title: wheel.title,
                    slope: 'y-position',
                    xLabel: 'angular velocity [rad/s]',
                    yLabel: 'wheel velocity [ticks/s]'
                  }}
                />
              ))}
            </div>
          </div>

          {/* Perpendicular Wheel Charts */}
          <div>
            <h3 className="text-xl font-semibold mb-3">Perpendicular Wheels</h3>
            <div className="space-y-4">
              {processedData.perpEncVels.map((wheel, i) => (
                <LinearRegressionChart
                  key={`perp-${i}`}
                  xs={processedData.angVels}
                  ys={wheel.values}
                  options={{
                    title: wheel.title,
                    slope: 'x-position',
                    xLabel: 'angular velocity [rad/s]',
                    yLabel: 'wheel velocity [ticks/s]'
                  }}
                />
              ))}
            </div>
          </div>

          {/* Track Width Chart */}
          <div>
            <h3 className="text-xl font-semibold mb-3">Track Width</h3>
            <div className="mb-4 flex gap-4 items-center">
              <label className="flex items-center gap-2">
                kV:
                <input
                  type="number"
                  value={kvValue}
                  onChange={(e) => setKvValue(parseFloat(e.target.value))}
                  step="0.001"
                  className="border rounded px-2 py-1 w-20"
                />
              </label>
              <label className="flex items-center gap-2">
                kS:
                <input
                  type="number"
                  value={ksValue}
                  onChange={(e) => setKsValue(parseFloat(e.target.value))}
                  step="0.001"
                  className="border rounded px-2 py-1 w-20"
                />
              </label>
              <button
                onClick={updateTrackWidthData}
                className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
              >
                Update
              </button>
            </div>
            <LinearRegressionChart
              xs={processedData.trackWidthData.xs}
              ys={processedData.trackWidthData.ys}
              options={{
                title: 'Track Width Regression',
                slope: 'track width',
                xLabel: 'angular velocity [rad/s]',
                yLabel: 'wheel velocity [ticks/s]'
              }}
            />
          </div>
        </div>
      )}
    </div>
  );
};

interface DriveEncoderAngularRampRegressionProps {
  className?: string;
}

export const DriveEncoderAngularRampRegression: React.FC<DriveEncoderAngularRampRegressionProps> = ({
  className = ''
}) => {
  const [processedData, setProcessedData] = useState<{
    feedforwardData: { xs: number[]; ys: number[] };
    trackWidthData: { xs: number[]; ys: number[] };
  } | null>(null);

  const handleDataLoaded = async (inputData: InputAngularRampData): Promise<string[]> => {
    const data = prepareAngularRampData(inputData);

    // Feedforward regression data
    const feedforwardXs = [
      ...data.leftEncVels.flatMap(vs => vs.values.map(v => -v)),
      ...data.rightEncVels.flatMap(vs => vs.values),
    ];
    const feedforwardYs = [
      ...data.leftPowers.flatMap(ps => {
        return ps.values.map((p, i) => -p * data.voltages.values[i]);
      }),
      ...data.rightPowers.flatMap(ps => {
        return ps.values.map((p, i) => p * data.voltages.values[i]);
      }),
    ];

    // Track width regression data
    const angVels = getPosZAngVelocity(data).slice(1, -1);
    const trackWidthYs = angVels.map((_, i) =>
      (data.leftEncVels.reduce((acc, vs) => acc - vs.values[i], 0) / data.leftEncVels.length
        + data.rightEncVels.reduce((acc, vs) => acc + vs.values[i], 0) / data.rightEncVels.length)
      * (data.type === 'mecanum' ? 0.5 : 1)
    );

    setProcessedData({
      feedforwardData: { xs: feedforwardXs, ys: feedforwardYs },
      trackWidthData: { xs: angVels, ys: trackWidthYs }
    });

    return [];
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Drive Encoder Angular Ramp Regression</h2>
      
      <FileLoader
        name="driveEncoderAngularRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {processedData && (
        <div className="space-y-6">
          <LinearRegressionChart
            xs={processedData.feedforwardData.xs}
            ys={processedData.feedforwardData.ys}
            options={{
              title: 'Feedforward Regression',
              slope: 'kV',
              intercept: 'kS',
              xLabel: 'wheel velocity [ticks/s]',
              yLabel: 'applied voltage [V]'
            }}
          />

          <LinearRegressionChart
            xs={processedData.trackWidthData.xs}
            ys={processedData.trackWidthData.ys}
            options={{
              title: 'Track Width Regression',
              slope: 'track width',
              xLabel: 'angular velocity [rad/s]',
              yLabel: 'wheel velocity [ticks/s]'
            }}
          />
        </div>
      )}
    </div>
  );
};

interface ForwardRampRegressionProps {
  className?: string;
}

export const ForwardRampRegression: React.FC<ForwardRampRegressionProps> = ({
  className = ''
}) => {
  const [data, setData] = useState<ForwardRampData | null>(null);

  const handleDataLoaded = async (inputData: QuasistaticParameters): Promise<string[]> => {
    console.log(inputData);
    const processedData: ForwardRampData = {
      forwardVoltage: inputData.voltages,
      forwardVel: inputData.velocities
    };

    console.log(processedData);
    setData(processedData);

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
          xs={data.forwardVel.values}
          ys={data.forwardVoltage.values}
          options={{
            title: 'Forward Ramp Regression',
            slope: 'kV',
            intercept: 'kS',
            xLabel: 'forward velocity [inches/s]',
            yLabel: 'applied voltage [V]'
          }}
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
  const [data, setData] = useState<InputLateralRampData | null>(null);
  const [regressionData, setRegressionData] = useState<{ xs: number[]; ys: number[] } | null>(null);
  const [inPerTick, setInPerTick] = useState<number>(0.001);
  const [kvValue, setKvValue] = useState<number>(0.1);
  const [ksValue, setKsValue] = useState<number>(0.1);
  const [warnings, setWarnings] = useState<string[]>([]);

  const handleDataLoaded = async (inputData: InputLateralRampData): Promise<string[]> => {
    setData(inputData);
    updateRegressionData(inputData, inPerTick, kvValue, ksValue);
    return warnings;
  };

  const updateRegressionData = (
    inputData: InputLateralRampData,
    inPerTickValue: number,
    kv: number,
    ks: number
  ) => {
    const processedData = prepareLateralRampData(inputData);

    const appliedVoltages = processedData.perpEncVels.flatMap(() =>
      processedData.voltages.values.map((v, i) =>
        Math.max(
          -processedData.frontLeftPower.values[i],
          processedData.backLeftPower.values[i],
          processedData.frontRightPower.values[i],
          -processedData.backRightPower.values[i],
          0,
        ) * v));

    const allPerpEncVels = processedData.perpEncVels.flatMap(v => v.values);

    const expectedPerpVelTicks = appliedVoltages.map(voltage => (voltage - ks) / kv);
    const perpEncVelsInches = allPerpEncVels.map(v => v * inPerTickValue);

    setRegressionData({
      xs: expectedPerpVelTicks,
      ys: perpEncVelsInches
    });

    // Check for warnings
    const minVel = allPerpEncVels.reduce((acc, v) => Math.min(acc, v), 0);
    const maxAbsVel = allPerpEncVels.reduce((acc, v) => Math.max(acc, Math.abs(v)), 0);
    const newWarnings = [];
    if (-minVel > 0.2 * maxAbsVel) {
      newWarnings.push('Warning: Lateral velocity should not go negative. Make sure the robot is pushed rightward and the encoders are oriented correctly.');
    }
    setWarnings(newWarnings);
  };

  const handleUpdate = () => {
    if (data) {
      updateRegressionData(data, inPerTick, kvValue, ksValue);
    }
  };

  return (
    <div className={className}>
      <h2 className="text-2xl font-bold mb-4">Lateral Ramp Regression</h2>
      
      <FileLoader
        name="lateralRamp"
        onDataLoaded={handleDataLoaded}
        className="mb-6"
      />

      {regressionData && (
        <div className="space-y-6">
          <div className="flex gap-4 items-center mb-4">
            <label className="flex items-center gap-2">
              In/Tick:
              <input
                type="number"
                value={inPerTick}
                onChange={(e) => setInPerTick(parseFloat(e.target.value))}
                step="0.0001"
                className="border rounded px-2 py-1 w-24"
              />
            </label>
            <label className="flex items-center gap-2">
              kV:
              <input
                type="number"
                value={kvValue}
                onChange={(e) => setKvValue(parseFloat(e.target.value))}
                step="0.001"
                className="border rounded px-2 py-1 w-20"
              />
            </label>
            <label className="flex items-center gap-2">
              kS:
              <input
                type="number"
                value={ksValue}
                onChange={(e) => setKsValue(parseFloat(e.target.value))}
                step="0.001"
                className="border rounded px-2 py-1 w-20"
              />
            </label>
            <button
              onClick={handleUpdate}
              className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
            >
              Update
            </button>
          </div>

          <LinearRegressionChart
            xs={regressionData.xs}
            ys={regressionData.ys}
            options={{
              title: 'Lateral Ramp Regression',
              slope: 'lateral in per tick',
              xLabel: 'expected velocity from feedforward [ticks/s]',
              yLabel: 'actual velocity [in/s]'
            }}
          />

          {warnings.length > 0 && (
            <div className="p-3 bg-yellow-100 border border-yellow-400 text-yellow-700 rounded">
              {warnings.map((warning, i) => (
                <p key={i}>{warning}</p>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
}; 