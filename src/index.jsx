import { createRoot } from 'react-dom/client';
import { Provider } from 'react-redux';

import ReactDOM from "react-dom/client";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import configureStore from './store/configureStore';
import { ThemeProvider } from '@/hooks/useTheme';

import './index.css';
import { BASE_HERMES_URL } from './constants';
import Home from './pages/Home';
import TuningPage from './pages/TuningPage';
import GettingStarted from './pages/GettingStarted';
import GraphPage from './pages/GraphPage';

import { getLayoutPreset } from "@/store/actions/settings";
import { startSocketWatcher } from "@/store/middleware/socketMiddleware";
import { useEffect } from "react";
import { useDispatch } from "react-redux/es/hooks/useDispatch";
import DefaultLayout from './pages/DefaultLayout';
import ForwardPush from './pages/ForwardPush';
import LateralPush from './pages/LateralPush';
import AngularPush from './pages/AngularPush';
import ForwardRampPage from './pages/ForwardRampPage';
import LateralRampPage from './pages/LateralRampPage';
import DriveEncoderAngularRampPage from './pages/DriveEncoderAngularRampPage';
import DeadWheelAngularRampPage from './pages/DeadWheelAngularRampPage';

export default function App() {

  return (
    
    <Provider store={store}>
      <BrowserRouter basename={BASE_HERMES_URL}>
        <Routes>
          <Route element={<DefaultLayout/>}>
            <Route index element={<Home/>}/>
            <Route element={<TuningPage/>}>
              <Route path="/getting-started" element={<GettingStarted/>}/>
              <Route path="/forward-push" element={<ForwardPush/>}/>
              <Route path="/lateral-push" element={<LateralPush/>}/>
              <Route path="/angular-push" element={<AngularPush/>}/>
            </Route>
          </Route>
          <Route element={<GraphPage/>}>
            <Route path="/forward-ramp" element={<ForwardRampPage/>}/>
            <Route path="/lateral-ramp" element={<LateralRampPage/>}/>
            <Route path="/drive-encoder-angular-ramp" element={<DriveEncoderAngularRampPage/>}/>
            <Route path="/dead-wheel-angular-ramp" element={<DeadWheelAngularRampPage/>}/>
          </Route>
        </Routes>
      </BrowserRouter>
    </Provider>
  );
}

const store = configureStore();
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(<App />);


