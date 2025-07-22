import { Middleware } from 'redux';
import { GET_LAYOUT_PRESET, SAVE_LAYOUT_PRESET } from '@/store/types';
import { receiveLayoutPreset } from '@/store/actions/settings';
import { RootState } from '@/store/reducers';

const LAYOUT_PRESET_KEY = 'layoutPreset';

const storageMiddleware: Middleware<Record<string, unknown>, RootState> =
  (store) => (next) => (action) => {
    switch (action.type) {
      default:
        next(action);

        break;
    }
  };

export default storageMiddleware;
