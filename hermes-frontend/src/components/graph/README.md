# Graph Components

This directory contains React components for regression analysis and chart visualization, refactored from the original DOM manipulation code in `common.ts`.

## Components

### LinearRegressionChart
A React component that wraps Plotly.js functionality for interactive linear regression analysis.

```tsx
import { LinearRegressionChart, LinearRegressionChartRef } from './components/graph';
import { useRef } from 'react';

const MyComponent = () => {
  const chartRef = useRef<LinearRegressionChartRef>(null);

  const handleGetRegressionValues = () => {
    if (chartRef.current) {
      const { slope, intercept } = chartRef.current.getRegressionValues();
      console.log('Current regression:', slope, intercept);
      
      const { xs, ys } = chartRef.current.getFilteredData();
      console.log('Filtered data points:', xs.length);
    }
  };

  return (
    <div>
      <LinearRegressionChart
        ref={chartRef}
        xs={[1, 2, 3, 4]}
        ys={[2, 4, 6, 8]}
        options={{
          title: 'My Regression',
          slope: 'slope',
          intercept: 'intercept', // optional - omit for no-intercept fit
          xLabel: 'X values',
          yLabel: 'Y values'
        }}
        onChange={(slope, intercept) => console.log(slope, intercept)}
      />
      <button onClick={handleGetRegressionValues}>Get Current Values</button>
    </div>
  );
};
```

#### Ref Methods

The component exposes these methods via ref:

- `getRegressionValues()`: Returns current `{ slope: number, intercept: number }`
- `getFilteredData()`: Returns currently selected data points `{ xs: number[], ys: number[] }`
- `resetSelection()`: Resets all data points to selected state

### RegressionDemo
A demo component that shows how to use the ref methods to access regression values from external components.

```tsx
import { RegressionDemo } from './components/graph';

<RegressionDemo />
```

### Regression Analysis Components

#### DeadWheelAngularRampRegression
For analyzing dead wheel encoder data during angular ramp tests.

```tsx
import { DeadWheelAngularRampRegression } from './components/graph';

<DeadWheelAngularRampRegression />
```

#### DriveEncoderAngularRampRegression
For analyzing drive encoder data during angular ramp tests.

```tsx
import { DriveEncoderAngularRampRegression } from './components/graph';

<DriveEncoderAngularRampRegression />
```

#### ForwardRampRegression
For analyzing forward ramp test data.

```tsx
import { ForwardRampRegression } from './components/graph';

<ForwardRampRegression />
```

#### LateralRampRegression
For analyzing lateral ramp test data.

```tsx
import { LateralRampRegression } from './components/graph';

<LateralRampRegression />
```

### FileLoader
A reusable component for loading data files (JSON) either from server or local upload.

```tsx
import { FileLoader } from './components/graph';

<FileLoader
  name="myData"
  onDataLoaded={async (data) => {
    // Process the loaded data
    console.log(data);
    return []; // Return array of warning messages
  }}
/>
```

## Features

- **Interactive Charts**: Click and drag to select data points, use `i` to include or `e` to exclude selected points
- **File Loading**: Load latest data from server or upload local files
- **Error Handling**: Comprehensive error and warning display
- **TypeScript**: Full type safety with proper TypeScript definitions
- **Responsive**: Charts automatically resize based on window size

## Migration from DOM Manipulation

The original `common.ts` functions have been replaced:

- `newLinearRegressionChart()` → `<LinearRegressionChart />` component  
- `loadDeadWheelAngularRampRegression()` → `<DeadWheelAngularRampRegression />` component
- `loadDriveEncoderAngularRampRegression()` → `<DriveEncoderAngularRampRegression />` component  
- `loadForwardRampRegression()` → `<ForwardRampRegression />` component
- `loadLateralRampRegression()` → `<LateralRampRegression />` component
- `installButtonHandlers()` → `<FileLoader />` component

The old functions are still exported from `common.ts` for backwards compatibility but will log deprecation warnings.

## Styling

Components use Tailwind CSS classes. Make sure Tailwind is configured in your project. The components include:
- Blue buttons for actions
- Red/yellow backgrounds for errors/warnings  
- Responsive layout with proper spacing
- Modern form inputs with proper styling 