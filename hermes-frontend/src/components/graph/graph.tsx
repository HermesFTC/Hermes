import React, { useEffect, useRef, useState, useCallback, forwardRef, useImperativeHandle } from 'react';
import Plotly from 'plotly.js-basic-dist-min';
import regression, { DataPoint } from 'regression';

// Utility functions moved from common.ts
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

function fitLinearNoIntercept(xs: number[], ys: number[]) {
  return kahanSum(
      xs.map((x, i) => x * ys[i])
  ) / kahanSum(
      xs.map(x => x * x)
  );
}

function fitLinearWithScaling(xs: number[], ys: number[]) {
  const xOffset = xs.reduce((a, b) => a + b, 0) / xs.length;
  const yOffset = ys.reduce((a, b) => a + b, 0) / ys.length;

  const xScale = xs.reduce((acc, x) => Math.max(acc, Math.abs(x - xOffset)), 0);
  const yScale = ys.reduce((acc, y) => Math.max(acc, Math.abs(y - yOffset)), 0);

  const data: DataPoint[] = xs.map((x, i) => [(x - xOffset) / xScale, (ys[i] - yOffset) / yScale]);

  const result = regression.linear(data);
  const [m, b] = result.equation;

  return [m * yScale / xScale, b * yScale - m * xOffset * yScale / xScale + yOffset];
}

export interface RegressionOptions {
  title: string;
  slope: string;
  intercept?: string;
  xLabel: string;
  yLabel: string;
}

export interface LinearRegressionChartProps {
  xs: number[];
  ys: number[];
  options: RegressionOptions;
  onChange?: (m: number, b: number) => void;
  className?: string;
}

export interface LinearRegressionChartRef {
  getRegressionValues: () => { slope: number; intercept: number };
  getFilteredData: () => { xs: number[]; ys: number[] };
  resetSelection: () => void;
}

export const LinearRegressionChart = forwardRef<LinearRegressionChartRef, LinearRegressionChartProps>(({
  xs,
  ys,
  options,
  onChange,
  className = ''
}, ref) => {
  const chartRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<Plotly.PlotlyHTMLElement | null>(null);
  const [mask, setMask] = useState<boolean[]>(() => xs.map(() => true));
  const [pendingSelection, setPendingSelection] = useState<Plotly.PlotSelectionEvent | null>(null);
  const [regressionResults, setRegressionResults] = useState<{ slope: number; intercept: number }>({ slope: 0, intercept: 0 });

  const color = '#777';
  const colorLight = '#bbb';

  const fit = useCallback((xData: number[], yData: number[]) => {
    return options.intercept === undefined ? [fitLinearNoIntercept(xData, yData), 0] : fitLinearWithScaling(xData, yData);
  }, [options.intercept]);

  const updateRegression = useCallback(() => {
    const filteredXs = xs.filter((_, i) => mask[i]);
    const filteredYs = ys.filter((_, i) => mask[i]);
    
    if (filteredXs.length === 0) return;
    
    const [m, b] = fit(filteredXs, filteredYs);
    setRegressionResults({ slope: m, intercept: b });
    
    if (onChange) onChange(m, b);

         if (plotRef.current) {
       // Update marker colors
       Plotly.restyle(plotRef.current, {
         'marker.color': [mask.map(selected => selected ? color : colorLight)]
       }, [0]);

      // Update regression line
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      
      Plotly.restyle(plotRef.current, {
        x: [[minX, maxX]],
        y: [[m * minX + b, m * maxX + b]],
      }, [1]);
    }
  }, [xs, ys, mask, fit, onChange]);

  const applySelection = useCallback((include: boolean) => {
    if (!pendingSelection) return false;

    const newMask = [...mask];
    for (const pt of pendingSelection.points) {
      newMask[pt.pointIndex] = include;
    }
    
    setMask(newMask);
    setPendingSelection(null);

    if (plotRef.current) {
      Plotly.restyle(plotRef.current, {
        'selectedpoints': [null],
      }, [0]);
    }

    return true;
  }, [mask, pendingSelection]);

  const handleInclude = useCallback(() => {
    if (applySelection(true)) {
      updateRegression();
    }
  }, [applySelection, updateRegression]);

  const handleExclude = useCallback(() => {
    if (applySelection(false)) {
      updateRegression();
    }
  }, [applySelection, updateRegression]);

  const resetSelection = useCallback(() => {
    setMask(xs.map(() => true));
    updateRegression();
  }, [xs, updateRegression]);

  const getFilteredData = useCallback(() => {
    return {
      xs: xs.filter((_, i) => mask[i]),
      ys: ys.filter((_, i) => mask[i])
    };
  }, [xs, ys, mask]);

  // Expose methods to parent components via ref
  useImperativeHandle(ref, () => ({
    getRegressionValues: () => regressionResults,
    getFilteredData,
    resetSelection
  }), [regressionResults, getFilteredData, resetSelection]);

  // Handle keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'i') {
        if (applySelection(true)) {
          updateRegression();
        }
      } else if (e.key === 'e') {
        if (applySelection(false)) {
          updateRegression();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [applySelection, updateRegression]);

  // Chart resizing logic
  const resizeChart = useCallback(() => {
    if (!chartRef.current || !plotRef.current) return;
    
    const containerWidth = chartRef.current.offsetWidth;
    const width = Math.max(300, containerWidth - 20); // Minimum width of 300px, with padding
    const height = Math.min(width * 9 / 16, 600); // 16:9 aspect ratio, max height 600px
    
    Plotly.relayout(plotRef.current, {
      width,
      height
    });
  }, []);

  // Handle window resize
  useEffect(() => {
    const handleResize = () => {
      // Debounce resize to avoid too many calls
      setTimeout(resizeChart, 100);
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [resizeChart]);

  // ResizeObserver for container size changes
  useEffect(() => {
    if (!chartRef.current) return;

    const resizeObserver = new ResizeObserver(() => {
      setTimeout(resizeChart, 50);
    });

    resizeObserver.observe(chartRef.current);

    return () => {
      resizeObserver.disconnect();
    };
  }, [resizeChart]);

  // Initialize chart
  useEffect(() => {
    if (!chartRef.current || xs.length === 0 || ys.length === 0) return;

    const [m, b] = fit(xs, ys);
    setRegressionResults({ slope: m, intercept: b });
    if (onChange) onChange(m, b);

    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);

    // Get initial dimensions
    const containerWidth = chartRef.current.offsetWidth || 800;
    const width = Math.max(300, containerWidth - 20);
    const height = Math.min(width * 9 / 16, 600);

    Plotly.newPlot(chartRef.current, [{
      type: 'scatter',
      mode: 'markers',
      x: xs,
      y: ys,
      name: 'Samples',
      marker: { color: mask.map(selected => selected ? color : colorLight), size: 5 },
    }, {
      type: 'scatter',
      mode: 'lines',
      x: [minX, maxX],
      y: [m * minX + b, m * maxX + b],
      name: 'Regression Line',
      line: { color: 'red' }
    }], {
      title: {
        text: options.title || ''
      },
      dragmode: 'select',
      showlegend: false,
      hovermode: false,
      width,
      height,
      autosize: true,
      xaxis: {
        title: {
          text: options.xLabel || '',
        }
      },
      yaxis: {
        title: {
          text: options.yLabel || '',
        }
      }
    }, {
      modeBarButtonsToRemove: [],
      displayModeBar: true,
    }).then(plot => {
      plotRef.current = plot;
      
      plot.on('plotly_selected', (eventData) => {
        if (eventData) {
          setPendingSelection(eventData);
        }
      });
      
      // Initial resize to ensure proper fit
      setTimeout(resizeChart, 100);
    });

    return () => {
      if (plotRef.current) {
        Plotly.purge(plotRef.current);
        plotRef.current = null;
      }
    };
  }, [xs, ys, options, resizeChart]);

  // Update chart when data changes
  useEffect(() => {
    if (plotRef.current && xs.length > 0 && ys.length > 0) {
      const newMask = xs.map(() => true);
      setMask(newMask);
      
      Plotly.restyle(plotRef.current, {
        x: [xs],
        y: [ys],
        'marker.color': [newMask.map(selected => selected ? color : colorLight)]
      }, [0]);

      updateRegression();
    }
  }, [xs, ys, updateRegression]);

  const resultsText = options.intercept === undefined 
    ? `${options.slope || 'slope'}: ${regressionResults.slope}`
    : `${options.slope || 'slope'}: ${regressionResults.slope}, ${options.intercept || 'y-intercept'}: ${regressionResults.intercept}`;

  return (
    <div className={`w-full ${className}`}>
      <div className="bar flex items-center gap-4 mb-4">
        <div className="flex gap-2">
          <button 
            onClick={handleInclude}
            className="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
          >
            [i]nclude
          </button>
          <button 
            onClick={handleExclude}
            className="px-3 py-1 bg-red-500 text-white rounded hover:bg-red-600"
          >
            [e]xclude
          </button>
        </div>
        <p className="text-sm font-mono">
          {resultsText}
        </p>
      </div>
      <div ref={chartRef} className="w-full min-h-[300px]" />
    </div>
  );
});

LinearRegressionChart.displayName = 'LinearRegressionChart';

export default LinearRegressionChart;

// Re-export regression components for convenience
export {
  ForwardRampRegression,
  ForwardStepRegression,
  LateralRampRegression,
  AngularRampRegression,
  AngularStepRegression
} from './regressions';

// Re-export utility types
export type {
  Signal,
  QuasistaticParameters,
  DynamicParameters
} from './utils';
