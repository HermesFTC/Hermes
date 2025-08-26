export type Signal = {
  times: number[];
  values: number[];
};

export type QuasistaticParameters = {
  velocities: Signal,
  voltages: Signal,
}

export type DynamicParameters = {
  accelerations: Signal,
  deltaVoltages: Signal,
}

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