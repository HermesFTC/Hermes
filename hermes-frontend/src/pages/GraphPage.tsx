import React from 'react';
import { Outlet } from 'react-router-dom';

const GraphPage: React.FC = () => {
  return (
    <div className="w-full min-h-screen">
      <div className="w-full max-w-none">
        <Outlet />
      </div>
    </div>
  );
};

export default GraphPage;