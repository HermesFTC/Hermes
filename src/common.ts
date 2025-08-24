import regression, { DataPoint } from 'regression';

// Re-export utility functions for backwards compatibility
export {
  numDerivOnline,
  numDerivOffline,
  inverseOverflow,
  prepareAngularRampData,
  prepareLateralRampData,
  getPosZAngVelocity
} from './components/graph/utils';

// Re-export types for backwards compatibility
export type {
  Signal,
  QuasistaticParameters,
  DriveType,
  AngularRampData,
  InputAngularRampData,
  ForwardRampData,
  LateralRampData,
  InputLateralRampData
} from './components/graph/utils';

// https://en.wikipedia.org/wiki/Kahan_summation_algorithm#The_algorithm
function kahanSum(xs: number[]) {
  let sum = 0;
  let c = 0;

  for (let i = 0; i < xs.length; i++) {
    const y = xs[i] - c;
    const t = sum + y;
    c = (t - sum) - y;
    sum = t;
  }

  return sum;
}

// https://en.wikipedia.org/wiki/Simple_linear_regression#Simple_linear_regression_without_the_intercept_term_(single_regressor)
export function fitLinearNoIntercept(xs: number[], ys: number[]) {
  return kahanSum(
      xs.map((x, i) => x * ys[i])
  ) / kahanSum(
      xs.map(x => x * x)
  );
}

export function fitLinearWithScaling(xs: number[], ys: number[]) {
  const xOffset = xs.reduce((a, b) => a + b, 0) / xs.length;
  const yOffset = ys.reduce((a, b) => a + b, 0) / ys.length;

  const xScale = xs.reduce((acc, x) => Math.max(acc, Math.abs(x - xOffset)), 0);
  const yScale = ys.reduce((acc, y) => Math.max(acc, Math.abs(y - yOffset)), 0);

  const data: DataPoint[] = xs.map((x, i) => [(x - xOffset) / xScale, (ys[i] - yOffset) / yScale]);

  const result = regression.linear(data);
  const [m, b] = result.equation;

  return [m * yScale / xScale, b * yScale - m * xOffset * yScale / xScale + yOffset];
}

// Legacy function support - these now use React components instead of DOM manipulation
// Kept for backwards compatibility but will return empty arrays since charts are now handled by React
export async function loadDeadWheelAngularRampRegression(): Promise<string[]> {
  console.warn('loadDeadWheelAngularRampRegression is deprecated. Use DeadWheelAngularRampRegression React component instead.');
  return [];
}

export async function loadDriveEncoderAngularRampRegression(): Promise<string[]> {
  console.warn('loadDriveEncoderAngularRampRegression is deprecated. Use DriveEncoderAngularRampRegression React component instead.');
  return [];
}

export async function loadForwardRampRegression(): Promise<string[]> {
  console.warn('loadForwardRampRegression is deprecated. Use ForwardRampRegression React component instead.');
  return [];
}

export async function loadLateralRampRegression(): Promise<string[]> {
  console.warn('loadLateralRampRegression is deprecated. Use LateralRampRegression React component instead.');
  return [];
}

export function installButtonHandlers<T>(name: string, loadRegression: (data: T) => Promise<string[]>) {
  console.warn('installButtonHandlers is deprecated. Use FileLoader React component instead.');
}
