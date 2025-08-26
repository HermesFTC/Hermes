import React, { useRef, useState } from 'react';
import { LinearRegressionChart, LinearRegressionChartRef } from './graph';

const RegressionDemo: React.FC = () => {
  const chartRef = useRef<LinearRegressionChartRef>(null);
  const [externalValues, setExternalValues] = useState<{ slope: number; intercept: number } | null>(null);
  const [filteredCount, setFilteredCount] = useState<number>(0);

  // Sample data
  const sampleXs = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
  const sampleYs = [2.1, 3.9, 6.2, 7.8, 10.1, 11.8, 14.2, 16.1, 17.9, 20.2];

  const handleGetValues = () => {
    if (chartRef.current) {
      const regressionValues = chartRef.current.getRegressionValues();
      const filteredData = chartRef.current.getFilteredData();
      
      setExternalValues(regressionValues);
      setFilteredCount(filteredData.xs.length);
      
      console.log('Current regression:', regressionValues);
      console.log('Filtered data:', filteredData);
    }
  };

  const handleReset = () => {
    if (chartRef.current) {
      chartRef.current.resetSelection();
      setExternalValues(null);
      setFilteredCount(0);
    }
  };

  const handleChartChange = (slope: number, intercept: number) => {
    // This callback is called whenever the regression changes
    console.log('Regression updated:', { slope, intercept });
  };

  return (
    <div className="p-6 max-w-4xl mx-auto">
      <h2 className="text-2xl font-bold mb-4">Regression Chart Demo</h2>
      <p className="text-gray-600 mb-6">
        This demo shows how external components can access regression values from the chart.
        Select data points and click "Get Values" to see the current regression parameters.
      </p>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Chart */}
        <div className="lg:col-span-2">
          <LinearRegressionChart
            ref={chartRef}
            xs={sampleXs}
            ys={sampleYs}
            options={{
              title: 'Sample Linear Regression',
              slope: 'Slope',
              intercept: 'Y-intercept',
              xLabel: 'X Values',
              yLabel: 'Y Values'
            }}
            onChange={handleChartChange}
          />
        </div>

        {/* Controls and Information */}
        <div className="lg:col-span-1 space-y-4">
          <div className="bg-white p-4 rounded-lg shadow border">
            <h3 className="font-semibold mb-3">External Controls</h3>
            <div className="space-y-2">
              <button
                onClick={handleGetValues}
                className="w-full px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
              >
                Get Current Values
              </button>
              <button
                onClick={handleReset}
                className="w-full px-4 py-2 bg-gray-500 text-white rounded hover:bg-gray-600"
              >
                Reset Selection
              </button>
            </div>
          </div>

          {externalValues && (
            <div className="bg-green-50 p-4 rounded-lg border border-green-200">
              <h3 className="font-semibold text-green-800 mb-2">Retrieved Values</h3>
              <div className="text-sm space-y-1">
                <div>
                  <strong>Slope:</strong> {externalValues.slope.toFixed(4)}
                </div>
                <div>
                  <strong>Intercept:</strong> {externalValues.intercept.toFixed(4)}
                </div>
                <div>
                  <strong>Data Points:</strong> {filteredCount} / {sampleXs.length}
                </div>
              </div>
            </div>
          )}

          <div className="bg-blue-50 p-4 rounded-lg border border-blue-200">
            <h3 className="font-semibold text-blue-800 mb-2">Instructions</h3>
            <div className="text-sm text-blue-700 space-y-2">
              <p>1. Select data points by dragging on the chart</p>
              <p>2. Press 'i' to include or 'e' to exclude selected points</p>
              <p>3. Click "Get Current Values" to retrieve regression parameters</p>
              <p>4. Use "Reset Selection" to include all points again</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RegressionDemo; 