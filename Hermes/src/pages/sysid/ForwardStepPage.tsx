import { ForwardStepRegression } from '@/components/graph';
import React from 'react';

const ForwardStepPage: React.FC = () => {
  return (
    <div className="content p-6 w-full max-w-7xl mx-auto">
      <header className="mb-8">
        <h1 className="text-3xl font-bold mb-2">RR Forward Step Regression</h1>
        <p className="text-gray-600 mb-4">Version: {import.meta.env.VITE_APP_VERSION || 'dev'}</p>
      </header>
      
      <div className="w-full">
        <ForwardStepRegression />
      </div>
    </div>
  );
};

export default ForwardStepPage; 