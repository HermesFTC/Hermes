
import { Provider } from 'react-redux';

import ReactDOM from "react-dom/client";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import configureStore from './store/configureStore';

import './index.css';
import { BASE_HERMES_URL } from './constants';
import Home from './pages/Home';
import TuningPage from './pages/TuningPage';
import GraphPage from './pages/GraphPage';
import GettingStarted from './pages/GettingStarted';

import DefaultLayout from './pages/DefaultLayout';
import ForwardPush from './pages/ForwardPush';
import LateralPush from './pages/LateralPush';
import AngularPush from './pages/AngularPush';
import ForwardRampPage from './pages/ForwardRampPage';
import ForwardStepPage from './pages/ForwardStepPage';
import LateralRampPage from './pages/LateralRampPage';
import AngularRampPage from './pages/AngularRampPage';
import AngularStepPage from './pages/AngularStepPage';

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
              <Route path="/forward-step" element={<ForwardStepPage/>}/>
              <Route path="/lateral-ramp" element={<LateralRampPage/>}/>
              <Route path="/angular-ramp" element={<AngularRampPage/>}/>
              <Route path="/angular-step" element={<AngularStepPage/>}/>
            </Route>
        </Routes>
      </BrowserRouter>
    </Provider>
  );
}

const store = configureStore();
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(<App />);


