export type Signal = {
  times: number[];
  values: number[];
};

export type QuasistaticParameters = {
  velocities: Signal,
  voltages: Signal,
}

export type DriveType = 'tank' | 'mecanum';

export type AngularRampData = {
  type: DriveType;
  leftPowers: Signal[];
  rightPowers: Signal[];
  voltages: Signal;
  leftEncPositions: Signal[];
  rightEncPositions: Signal[];
  parEncPositions: Signal[];
  perpEncPositions: Signal[];
  leftEncVels: Signal[];
  rightEncVels: Signal[];
  parEncVels: Signal[];
  perpEncVels: Signal[];
  angVels: Signal[]; // length 3
};

export type InputAngularRampData = AngularRampData & {
  // default to true
  leftEncFixVels: boolean[] | undefined;
  rightEncFixVels: boolean[] | undefined;
  parEncFixVels: boolean[] | undefined;
  perpEncFixVels: boolean[] | undefined;
};

export type ForwardRampData = {
  forwardVoltage: Signal;
  forwardVel: Signal
};

export type LateralRampData = {
  type: DriveType;
  frontLeftPower: Signal;
  backLeftPower: Signal;
  frontRightPower: Signal;
  backRightPower: Signal;
  voltages: Signal;
  perpEncPositions: Signal[];
  perpEncVels: Signal[];
};

export type InputLateralRampData = LateralRampData & {
  perpEncFixVels: boolean[] | undefined;
};

// no output for first pair
export function numDerivOnline(xs: number[], ys: number[]) {
  if (xs.length !== ys.length) {
    throw new Error(`${xs.length} !== ${ys.length}`);
  }

  return ys
      .slice(1)
      .map((y, i) => (y - ys[i]) / (xs[i + 1] - xs[i]));
}

// no output for first or last pair
export function numDerivOffline(xs: number[], ys: number[]) {
  return ys
      .slice(2)
      .map((y, i) => (y - ys[i]) / (xs[i + 2] - xs[i]));
}

const CPS_STEP = 0x10000;

export function inverseOverflow(input: number, estimate: number) {
  // convert to uint16
  let real = input & 0xffff;
  // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
  // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
  real += ((real % 20) / 4) * CPS_STEP;
  // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
  real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
  return real;
}

// no output for first or last pair
function fixVels(ts: number[], xs: number[], vs: number[]) {
  if (ts.length !== xs.length || ts.length !== vs.length) {
    throw new Error(`${ts.length} !== ${xs.length} !== ${vs.length}`);
  }

  return numDerivOffline(ts, xs).map((est, i) => inverseOverflow(vs[i + 1], est));
}

function getPosZAngVelocity(data: AngularRampData) {
  const p = data.angVels.reduce<[number, number, boolean, number[]]>((acc, vsArg, axisIdx) => {
    const vs = vsArg.values;
    const maxV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const minV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const [accMaxV, _axisIdx, _axisRev, _] = acc;
    if (maxV >= -minV) {
      if (maxV >= accMaxV) {
        return [maxV, axisIdx, false, vs];
      }
    } else {
      if (-minV >= accMaxV) {
        return [-minV, axisIdx, true, vs];
      }
    }
    return acc;
  }, [0, -1, false, []]);
  if (p[1] !== 2 || p[2] !== false) {
    throw new Error(`More rotation about the ${p[2] ? '-' : '+'}${['x', 'y', 'z'][p[1]]}-axis than the +z-axis. Fix the IMU orientation and run again.`);
  }
  return p[3];
}

function sliceSignal(s: Signal, start: number, end: number): Signal {
  return {
    times: s.times.slice(start, end),
    values: s.values.slice(start, end),
  };
}

function sliceSignals(xs: Signal[], start: number, end: number): Signal[] {
  return xs.map(s => sliceSignal(s, start, end));
}

// drops the first sample and the last two samples
function prepareEncSignals(pss: Signal[], vss: Signal[], fs: boolean[]): [Signal[], Signal[]] {
  const newPss = [];
  const newVss = [];
  for (let i = 0; i < pss.length; i++) {
    const ps = sliceSignal(pss[i], 0, -1);
    const vs = sliceSignal(vss[i], 0, -1);
    const newPs = sliceSignal(ps, 1, -1);
    newPss.push(newPs);
    if (vs.times.length === 0) {
      newVss.push({
        times: newPs.times,
        values: numDerivOffline(ps.times, ps.values),
      });
    } else if (fs[i]) {
      newVss.push({
        times: newPs.times,
        values: fixVels(vs.times, ps.values, vs.values),
      });
    } else {
      newVss.push(sliceSignal(vs, 1, -1));
    }
  }
  return [newPss, newVss];
}

export function prepareAngularRampData(inputData: InputAngularRampData): AngularRampData {
  const leftEncFixVels = inputData.leftEncFixVels ?? inputData.leftEncPositions.map(() => true);
  const rightEncFixVels = inputData.rightEncFixVels ?? inputData.rightEncPositions.map(() => true);
  const parEncFixVels = inputData.parEncFixVels ?? inputData.parEncPositions.map(() => true);
  const perpEncFixVels = inputData.perpEncFixVels ?? inputData.perpEncPositions.map(() => true);

  const [leftEncPositions, leftEncVels] = prepareEncSignals(inputData.leftEncPositions, inputData.leftEncVels, leftEncFixVels);
  const [rightEncPositions, rightEncVels] = prepareEncSignals(inputData.rightEncPositions, inputData.rightEncVels, rightEncFixVels);
  const [parEncPositions, parEncVels] = prepareEncSignals(inputData.parEncPositions, inputData.parEncVels, parEncFixVels);
  const [perpEncPositions, perpEncVels] = prepareEncSignals(inputData.perpEncPositions, inputData.perpEncVels, perpEncFixVels);

  const data: AngularRampData = {
    type: inputData.type,
    leftPowers: sliceSignals(inputData.leftPowers, 1, -2),
    rightPowers: sliceSignals(inputData.rightPowers, 1, -2),
    voltages: sliceSignal(inputData.voltages, 1, -2),
    leftEncPositions,
    leftEncVels,
    rightEncPositions,
    rightEncVels,
    parEncPositions,
    parEncVels,
    perpEncPositions,
    perpEncVels,
    angVels: sliceSignals(inputData.angVels, 1, -2),
  };

  const lengths = [
    ...data.leftPowers.map(ps => ps.values.length),
    ...data.rightPowers.map(ps => ps.values.length),
    data.voltages.values.length,
    ...data.leftEncPositions.map(ps => ps.values.length),
    ...data.rightEncPositions.map(ps => ps.values.length),
    ...data.parEncPositions.map(ps => ps.values.length),
    ...data.perpEncPositions.map(ps => ps.values.length),
    ...data.leftEncVels.map(ps => ps.values.length),
    ...data.rightEncVels.map(ps => ps.values.length),
    ...data.parEncVels.map(ps => ps.values.length),
    ...data.perpEncVels.map(ps => ps.values.length),
    ...data.angVels.map(ps => ps.values.length),
  ];
  if (lengths.some(l => l !== lengths[0])) {
    throw new Error(`Lengths do not match: ${lengths.join(', ')}`);
  }

  return data;
}

export function prepareLateralRampData(inputData: InputLateralRampData): LateralRampData {
  const perpEncFixVels = inputData.perpEncFixVels ?? inputData.perpEncVels.map(() => true);
  const [perpEncPositions, perpEncVels] = prepareEncSignals(inputData.perpEncPositions, inputData.perpEncVels, perpEncFixVels);

  const data: LateralRampData = {
    type: inputData.type,
    frontLeftPower: sliceSignal(inputData.frontLeftPower, 1, -2),
    backLeftPower: sliceSignal(inputData.backLeftPower, 1, -2),
    frontRightPower: sliceSignal(inputData.frontRightPower, 1, -2),
    backRightPower: sliceSignal(inputData.backRightPower, 1, -2),
    voltages: sliceSignal(inputData.voltages, 1, -2),
    perpEncPositions: perpEncPositions,
    perpEncVels: perpEncVels,
  };

  const lengths = [
    data.frontLeftPower.values.length,
    data.backLeftPower.values.length,
    data.frontRightPower.values.length,
    data.backRightPower.values.length,
    data.voltages.values.length,
    ...perpEncPositions.map(p => p.values.length),
    ...perpEncVels.map(v => v.values.length),
  ];
  if (lengths.some(l => l !== lengths[0])) {
    throw new Error(`Lengths do not match: ${lengths.join(', ')}`);
  }

  return data;
}

export { getPosZAngVelocity }; 