// Main chart component
export { default as LinearRegressionChart } from './graph';
export type { RegressionOptions, LinearRegressionChartProps, LinearRegressionChartRef } from './graph';

// Demo component
export { default as RegressionDemo } from './RegressionDemo';

// Specific regression analysis components
export {
  DeadWheelAngularRampRegression,
  DriveEncoderAngularRampRegression,
  ForwardRampRegression,
  LateralRampRegression
} from './regressions';

// Utility functions and types
export {
  numDerivOnline,
  numDerivOffline,
  inverseOverflow,
  prepareAngularRampData,
  prepareLateralRampData,
  getPosZAngVelocity
} from './utils';

export type {
  Signal,
  QuasistaticParameters,
  DriveType,
  AngularRampData,
  InputAngularRampData,
  ForwardRampData,
  LateralRampData,
  InputLateralRampData
} from './utils';

// File loader component
export { default as FileLoader } from '../FileLoader';
export type { FileLoaderProps } from '../FileLoader'; 