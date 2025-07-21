import { createRoot } from 'react-dom/client';
import { Provider } from 'react-redux';

import ReactDOM from "react-dom/client";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import Home from './pages/home';
import configureStore from './store/configureStore';
import { ThemeProvider } from '@/hooks/useTheme';

import './index.css';
import { BASE_HERMES_URL } from './constants';

export default function App() {
  return (
    <BrowserRouter basename={BASE_HERMES_URL}>
      <Routes>
        <Route index element={<Home />}/>
        <Route path="/getting-started" element={<p>hi!</p>}/>
      </Routes>
    </BrowserRouter>
  );
}

const store = configureStore();
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(<App />);


