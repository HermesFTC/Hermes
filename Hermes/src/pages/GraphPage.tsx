import React from 'react';
import { Outlet } from 'react-router-dom';

const GraphPage: React.FC = () => {
  return (
    <div className="w-full min-h-screen bg-gray-50">
      <div className="w-full max-w-none">
        <Outlet />
      </div>
    </div>
  );
};

export default GraphPage;