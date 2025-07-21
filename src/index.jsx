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

import { getLayoutPreset } from "@/store/actions/settings";
import { startSocketWatcher } from "@/store/middleware/socketMiddleware";
import { useEffect } from "react";
import { useDispatch } from "react-redux/es/hooks/useDispatch";
import DefaultLayout from './pages/DefaultLayout';

export default function App() {

  return (
    
    <Provider store={store}>
      <BrowserRouter basename={BASE_HERMES_URL}>
        <Routes>
          <Route element={<DefaultLayout/>}>
            <Route index element={<Home/>}/>
            <Route element={<TuningPage/>}>
              <Route path="/getting-started" element={<GettingStarted/>}/>
            </Route>
          </Route>
        </Routes>
      </BrowserRouter>
    </Provider>
  );
}

const store = configureStore();
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(<App />);


